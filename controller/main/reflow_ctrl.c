#include "reflow_ctrl.h"
#include "reflow_algo.h"
#include "temperature.h"
#include "plug_ctrl.h"
#include "ota_update.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* ── Shell around reflow_algo ────────────────────────────────────
   One task, every ALGO_DT_S: read the sensor, step the control law while a
   run is active, carry out the plug command it returns, print the log line.

   Ending a run (complete, fault or stop) hands the plug to the force-off
   branch, which keeps requesting OFF until the plug confirms it, however long
   that takes: a plug that was unreachable at the end of a run is switched off
   as soon as it answers again. */

#define TICK_MS      ((int)(ALGO_DT_S * 1000))
#define NVS_NS       "ctrl"
#define MAX_EVENTS   8
#define TRACE_MAX    1200

static const char *TAG = "reflow_ctrl";

static SemaphoreHandle_t s_mutex;

/* Guarded by s_mutex */
static algo_params_t    s_prm;
static algo_t           s_algo;
static reflow_profile_t s_profile;      /* the algo points at this during a run */
static bool             s_active;       /* a run owns the plug */
static bool             s_force_off;    /* keep switching off until confirmed (also at boot) */
static int64_t          s_t0_us;
static ctrl_status_t    s_st;
static bool             s_log;
static ctrl_trace_pt_t  s_trace[TRACE_MAX];
static int              s_trace_n;
static int              s_trace_every;  /* ticks per point */
static int              s_trace_tick;

/* Plug results arrive in the Zigbee task; the control task prints them. */
typedef struct {
    int64_t      us;
    bool         ok;
    plug_state_t state;
    const char  *reason;
} plug_event_t;
static portMUX_TYPE  s_ev_mux = portMUX_INITIALIZER_UNLOCKED;
static plug_event_t  s_ev[MAX_EVENTS];
static unsigned      s_ev_head, s_ev_tail;

static long ms_now(void) { return (long)(esp_timer_get_time() / 1000); }

/* ── Parameters ──────────────────────────────────────────────────── */

#define PARAM(n) { #n, offsetof(algo_params_t, n) }
static const struct { const char *name; size_t off; } k_params[] = {
    PARAM(K), PARAM(tau), PARAM(theta), PARAM(loss_quad), PARAM(t_room),
    PARAM(kp), PARAM(ti), PARAM(kd), PARAM(i_max), PARAM(lookahead),
    PARAM(over_weight), PARAM(coast), PARAM(coast_margin), PARAM(lag_band),
    PARAM(lag_span), PARAM(max_stretch), PARAM(max_hold), PARAM(min_dwell), PARAM(min_on), PARAM(sd_hyst), PARAM(pulse_duty), PARAM(pulse_rise), PARAM(plug_timeout),
    PARAM(refresh), PARAM(reassert), PARAM(anomaly_after), PARAM(anomaly_slope),
    PARAM(filt_tau), PARAM(over_margin),
};
#define NUM_PARAMS (sizeof(k_params) / sizeof(k_params[0]))

static float *param_ptr(algo_params_t *p, int i) { return (float *)((char *)p + k_params[i].off); }

/* Each parameter is its own NVS key, so adding one keeps the others. */
static void params_load(void)
{
    algo_params_default(&s_prm);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    int n = 0;
    for (int i = 0; i < (int)NUM_PARAMS; i++) {
        uint32_t bits;
        if (nvs_get_u32(h, k_params[i].name, &bits) == ESP_OK) {
            memcpy(param_ptr(&s_prm, i), &bits, sizeof(bits));
            n++;
        }
    }
    nvs_close(h);
    if (n) ESP_LOGI(TAG, "%d tuning parameter(s) loaded from NVS", n);
}

esp_err_t reflow_ctrl_param_set(const char *name, float value)
{
    for (int i = 0; i < (int)NUM_PARAMS; i++) {
        if (strcmp(name, k_params[i].name) == 0) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            *param_ptr(&s_prm, i) = value;
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

/* One @P line, so a log file records the parameters each run used. */
void reflow_ctrl_params_print(void)
{
    algo_params_t p;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    p = s_prm;
    xSemaphoreGive(s_mutex);
    printf("@P");
    for (int i = 0; i < (int)NUM_PARAMS; i++)
        printf(",%s=%g", k_params[i].name, (double)*param_ptr(&p, i));
    printf("\r\n");
}

esp_err_t reflow_ctrl_params_save(void)
{
    algo_params_t p;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    p = s_prm;
    xSemaphoreGive(s_mutex);
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    for (int i = 0; i < (int)NUM_PARAMS && err == ESP_OK; i++) {
        uint32_t bits;
        memcpy(&bits, param_ptr(&p, i), sizeof(bits));
        err = nvs_set_u32(h, k_params[i].name, bits);
    }
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void reflow_ctrl_params_reset(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    algo_params_default(&s_prm);
    xSemaphoreGive(s_mutex);
}

/* ── Plug events ─────────────────────────────────────────────────── */

static void plug_result_cb(bool ok, plug_state_t state, const char *reason)
{
    taskENTER_CRITICAL(&s_ev_mux);
    if (s_ev_head - s_ev_tail < MAX_EVENTS) {
        s_ev[s_ev_head % MAX_EVENTS] = (plug_event_t){
            .us = esp_timer_get_time(), .ok = ok, .state = state, .reason = reason,
        };
        s_ev_head++;
    }
    taskEXIT_CRITICAL(&s_ev_mux);
}

static void print_plug_events(void)
{
    for (;;) {
        plug_event_t e;
        bool have = false;
        taskENTER_CRITICAL(&s_ev_mux);
        if (s_ev_tail != s_ev_head) { e = s_ev[s_ev_tail % MAX_EVENTS]; s_ev_tail++; have = true; }
        taskEXIT_CRITICAL(&s_ev_mux);
        if (!have) return;
        printf("@E,%ld,plug,%s,%s,%s\r\n", (long)(e.us / 1000), e.ok ? "ok" : "fail",
               plug_state_str(e.state), e.reason ? e.reason : "");
    }
}

/* ── Control task ────────────────────────────────────────────────── */

/* Under s_mutex. */
static void trace_add(const ctrl_trace_pt_t *pt)
{
    if (++s_trace_tick < s_trace_every) return;
    s_trace_tick = 0;
    if (s_trace_n == TRACE_MAX) {
        for (int k = 0; k < TRACE_MAX / 2; k++) s_trace[k] = s_trace[2 * k];
        s_trace_n = TRACE_MAX / 2;
        s_trace_every *= 2;
    }
    s_trace[s_trace_n++] = *pt;
}

int reflow_ctrl_trace_get(int first, ctrl_trace_pt_t *out, int max)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = 0;
    for (int k = first; k < s_trace_n && n < max; k++, n++) out[n] = s_trace[k];
    xSemaphoreGive(s_mutex);
    return n;
}

static ctrl_state_t state_of(algo_phase_t p)
{
    switch (p) {
        case ALGO_RUNNING: return CTRL_STATE_RUNNING;
        case ALGO_COOLING: return CTRL_STATE_COOLING;
        case ALGO_DONE:    return CTRL_STATE_COMPLETE;
        default:           return CTRL_STATE_ERROR;
    }
}

static void control_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    esp_err_t  last_cmd_err = ESP_OK;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(TICK_MS));

        float temp = 0, amb = 0;
        bool  ok     = (temperature_read(&temp) == ESP_OK);
        bool  amb_ok = (temperature_read_ambient(&amb) == ESP_OK);
        plug_status_t ps;
        plug_ctrl_get_status(&ps);
        bool plug_ok = ps.available && ps.state != PLUG_UNKNOWN;

        algo_output_t o;
        bool       stepped = false, ended = false, anomaly = false, hold_capped = false, logging;
        char       phase_label[PROFILE_LABEL_LEN] = "";
        algo_cmd_t cmd = ALGO_CMD_NONE;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (amb_ok) s_st.ambient = amb;
        if (ok != s_st.sensor_ok) ESP_LOGI(TAG, "Sensor %s", ok ? "connected" : "disconnected");
        s_st.sensor_ok = ok;
        if (ok) s_st.temp = temp;

        if (s_active) {
            algo_input_t in = {
                .t            = (float)(esp_timer_get_time() - s_t0_us) / 1e6f,
                .sensor_ok    = ok,
                .temp         = temp,
                .plug_ok      = plug_ok,
                .plug_on      = ps.state == PLUG_ON,
                .plug_pending = ps.pending,
            };
            algo_step(&s_algo, &in, &o);
            stepped        = true;
            cmd            = o.cmd;
            anomaly        = o.anomaly;
            hold_capped    = o.hold_capped;
            s_st.state     = state_of(o.phase);
            s_st.elapsed   = in.t;
            s_st.profile_t = o.profile_t;
            s_st.setpoint  = o.sp;
            s_st.power     = o.u;
            s_st.fault     = algo_fault_str(o.fault);
            if (s_algo.mode == ALGO_MODE_PROFILE)
                strlcpy(s_st.phase, s_profile.waypoints[o.segment].label, sizeof(s_st.phase));
            strlcpy(phase_label, s_st.phase, sizeof(phase_label));
            if (ok) {
                ctrl_trace_pt_t pt = {
                    .t = in.t, .temp = temp, .sp = o.sp,
                    .plug = (int8_t)(plug_ok ? (ps.state == PLUG_ON) : -1), .power = o.u,
                };
                trace_add(&pt);
            }
            if (o.phase == ALGO_DONE || o.phase == ALGO_FAULT) {
                s_active    = false;
                s_force_off = true;
                ended       = true;
            }
        } else if (s_force_off) {
            if (ps.state == PLUG_OFF && !ps.pending) s_force_off = false;
            else if (ps.available && !ps.pending)    cmd = ALGO_CMD_OFF;
        }
        logging = s_log || stepped;
        ctrl_state_t state = s_st.state;
        const char  *fault = s_st.fault;
        xSemaphoreGive(s_mutex);

        if (hold_capped) {
            printf("@E,%ld,hold_limit,%s\r\n", ms_now(), phase_label);
            ESP_LOGI(TAG, "Hold limit reached in %s: continuing in real time", phase_label);
        }
        if (anomaly) {
            printf("@E,%ld,anomaly,heating_while_off,%.1f\r\n", ms_now(), (double)o.slope);
            ESP_LOGW(TAG, "Heater is off but the iron heats at %.1f C/s: switching off again",
                     (double)o.slope);
        }
        if (cmd != ALGO_CMD_NONE) {
            esp_err_t err = plug_ctrl_set(cmd == ALGO_CMD_ON);
            /* A plug that stays unreachable would otherwise log every tick. */
            if (err == ESP_OK || err != last_cmd_err)
                printf("@E,%ld,cmd,%s,%s\r\n", ms_now(), cmd == ALGO_CMD_ON ? "on" : "off",
                       err == ESP_OK ? "ok" : plug_ctrl_err_reason(err));
            last_cmd_err = err;
        }
        print_plug_events();

        if (logging) {
            int plug = plug_ok ? (ps.state == PLUG_ON) : -1;
            if (stepped)
                printf("@T,%ld,%s,%.1f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\r\n",
                       ms_now(), reflow_ctrl_state_str(state), (double)o.profile_t, (double)o.sp,
                       (double)o.sp_ff, ok ? (double)temp : -999.0, (double)o.tf, (double)o.slope,
                       (double)o.u_ff, (double)o.p, (double)o.i, (double)o.u, o.want_on,
                       o.forced_off, plug, ps.pending);
            else
                printf("@T,%ld,%s,0,0,0,%.2f,0,0,0,0,0,0,0,1,%d,%d\r\n", ms_now(),
                       reflow_ctrl_state_str(state), ok ? (double)temp : -999.0, plug, ps.pending);
        }
        if (ended) {
            printf("@E,%ld,end,%s,%s\r\n", ms_now(), reflow_ctrl_state_str(state), fault);
            ESP_LOGI(TAG, "Run ended: %s%s%s", reflow_ctrl_state_str(state),
                     *fault ? " — " : "", fault);
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────── */

esp_err_t reflow_ctrl_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_st.state = CTRL_STATE_IDLE;
    s_st.fault = "";
    /* After a reboot the plug is in whatever state it was left in, possibly
       ON (a crash or power blip mid-run). The plug service sends one OFF when
       Zigbee comes up; if that is lost the state stays Unknown and nothing
       else would switch it off (seen on hardware). So start in force-off:
       OFF is re-sent until the plug confirms it, then control is released. */
    s_force_off = true;
    params_load();
    ESP_RETURN_ON_ERROR(plug_ctrl_add_result_cb(plug_result_cb), TAG, "plug cb");
    xTaskCreate(control_task, "reflow_ctrl", 4096, NULL, 6, NULL);
    return ESP_OK;
}

typedef enum { START_PROFILE, START_STEP, START_HOLD } start_kind_t;

static esp_err_t start(start_kind_t kind, const reflow_profile_t *profile,
                       float a1, float a2, float a3, const char **reason)
{
    const char *why = NULL;
    esp_err_t   err = ESP_OK;
    plug_status_t ps;
    plug_ctrl_get_status(&ps);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_active) {
        why = "busy"; err = ESP_ERR_INVALID_STATE;
    } else if (ota_update_busy()) {
        why = "updating"; err = ESP_ERR_INVALID_STATE;
    } else if (!s_st.sensor_ok) {
        why = "sensor"; err = ESP_ERR_INVALID_STATE;
    } else if (!ps.available || ps.state == PLUG_UNKNOWN) {
        why = "plug"; err = ESP_ERR_NOT_FOUND;
    } else {
        switch (kind) {
        case START_PROFILE:
            s_profile = *profile;
            if (!algo_start_profile(&s_algo, &s_prm, &s_profile)) {
                why = "profile"; err = ESP_ERR_INVALID_ARG;
            }
            strlcpy(s_st.profile, s_profile.name, sizeof(s_st.profile));
            break;
        case START_STEP:
            algo_start_step(&s_algo, &s_prm, a1, a2, a3);
            strlcpy(s_st.profile, "step", sizeof(s_st.profile));
            break;
        case START_HOLD:
            algo_start_hold(&s_algo, &s_prm, a1);
            strlcpy(s_st.profile, "hold", sizeof(s_st.profile));
            break;
        }
    }
    if (err == ESP_OK) {
        s_t0_us        = esp_timer_get_time();
        s_active       = true;
        s_force_off    = false;
        s_st.state     = CTRL_STATE_RUNNING;
        s_st.elapsed   = 0;
        s_st.profile_t = s_algo.pt;
        s_st.setpoint  = 0;
        s_st.power     = 0;
        s_st.fault     = "";
        s_st.phase[0]  = '\0';
        s_st.run_id++;
        s_trace_n      = 0;
        s_trace_every  = 1;        /* every tick: 2 s heater pulses need it */
        s_trace_tick   = s_trace_every - 1;    /* first sample now */
    }
    float start_temp = s_st.temp;
    xSemaphoreGive(s_mutex);

    if (reason) *reason = why;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Start refused: %s", why);
        return err;
    }
    switch (kind) {
    case START_PROFILE:
        printf("@S,%ld,profile,%s,start_temp=%.1f\r\n", ms_now(),
               profile->name, (double)start_temp);
        break;
    case START_STEP:
        printf("@S,%ld,step,duty=%.2f,secs=%.0f,max=%.1f,start_temp=%.1f\r\n", ms_now(),
               (double)a1, (double)a2, (double)a3, (double)start_temp);
        break;
    case START_HOLD:
        printf("@S,%ld,hold,temp=%.1f,start_temp=%.1f\r\n", ms_now(), (double)a1,
               (double)start_temp);
        break;
    }
    reflow_ctrl_params_print();
    return ESP_OK;
}

esp_err_t reflow_ctrl_start(const reflow_profile_t *profile, const char **reason)
{
    return start(START_PROFILE, profile, 0, 0, 0, reason);
}

esp_err_t reflow_ctrl_start_step(float duty, float secs, float max_temp, const char **reason)
{
    return start(START_STEP, NULL, duty, secs, max_temp, reason);
}

esp_err_t reflow_ctrl_start_hold(float temp, const char **reason)
{
    return start(START_HOLD, NULL, temp, 0, 0, reason);
}

esp_err_t reflow_ctrl_stop(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool was_active = s_active;
    s_active    = false;
    s_force_off = true;
    s_st.state  = CTRL_STATE_IDLE;
    s_st.fault  = "";
    xSemaphoreGive(s_mutex);

    /* Right away rather than on the next tick; the task retries if this is
       rejected because a change is pending. */
    plug_ctrl_set(false);
    if (was_active) {
        printf("@E,%ld,end,stopped,\r\n", ms_now());
        ESP_LOGI(TAG, "Run stopped");
    }
    return ESP_OK;
}

void reflow_ctrl_get_status(ctrl_status_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_st;
    xSemaphoreGive(s_mutex);
}

bool reflow_ctrl_is_active(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool a = s_active;
    xSemaphoreGive(s_mutex);
    return a;
}

void reflow_ctrl_set_log(bool on)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_log = on;
    xSemaphoreGive(s_mutex);
}

const char *reflow_ctrl_state_str(ctrl_state_t s)
{
    switch (s) {
        case CTRL_STATE_RUNNING:  return "running";
        case CTRL_STATE_COOLING:  return "cooling";
        case CTRL_STATE_COMPLETE: return "complete";
        case CTRL_STATE_ERROR:    return "error";
        default:                  return "idle";
    }
}
