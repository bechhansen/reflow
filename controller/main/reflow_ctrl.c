#include "reflow_ctrl.h"
#include "temperature.h"
#include "zigbee_plug.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#define CTRL_TASK_MS        100
#define CYCLE_STEPS         4           /* 4 slots × 2 s = 8 s cycle */
#define CYCLE_SLOT_MS       2000
#define RAMP_HANDOFF_DELTA  10.0f

static const char *TAG = "reflow_ctrl";

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_temp;
    bool  initialized;
} pid_ctrl_t;

static void pid_reset(pid_ctrl_t *p, float kp, float ki, float kd)
{
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integral = 0.0f; p->initialized = false;
}

static float pid_update(pid_ctrl_t *p, float setpoint, float temp, float dt)
{
    if (!p->initialized) { p->prev_temp = temp; p->initialized = true; }
    float error      = setpoint - temp;
    float derivative = -(temp - p->prev_temp) / dt;
    p->prev_temp     = temp;
    float out = p->kp * error + p->ki * p->integral + p->kd * derivative;
    if (out > 0.0f && out < 1.0f) p->integral += error * dt;
    if (out < 0.0f) out = 0.0f;
    if (out > 1.0f) out = 1.0f;
    return out;
}

static SemaphoreHandle_t s_mutex;
static ctrl_state_t      s_state       = CTRL_STATE_IDLE;
static reflow_profile_t  s_profile;
static float             s_temp        = 0.0f;
static float             s_setpoint    = 0.0f;
static int               s_elapsed_s   = 0;
static char              s_phase[PROFILE_LABEL_LEN];
static bool              s_plug_on     = false;
static volatile int      s_duty_steps  = 0;
static bool              s_sensor_ok   = false;

/* ── Relay cycle task (always running) ────────────────────────────────────── */
static void relay_cycle_task(void *arg)
{
    bool was_running = false;

    while (1) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        ctrl_state_t state = s_state;
        int steps = s_duty_steps;
        xSemaphoreGive(s_mutex);

        if (state != CTRL_STATE_RUNNING) {
            if (was_running) {
                /* Turn plug off exactly once on the running→stopped transition */
                zigbee_plug_set(false);
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                s_plug_on = false;
                xSemaphoreGive(s_mutex);
                was_running = false;
            }
            /* Probe sensor while idle so telemetry shows live temp and sensor_ok */
            float probe_temp;
            bool ok = (temperature_read(&probe_temp) == ESP_OK);
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            if (ok != s_sensor_ok)
                ESP_LOGI(TAG, "Sensor %s", ok ? "connected" : "disconnected");
            s_sensor_ok = ok;
            if (ok) s_temp = probe_temp;
            xSemaphoreGive(s_mutex);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        was_running = true;

        /* steps ∈ [0,4]: on_ms = steps×2000, off_ms = (4-steps)×2000 */
        int on_ms  = steps * CYCLE_SLOT_MS;
        int off_ms = (CYCLE_STEPS - steps) * CYCLE_SLOT_MS;

        if (on_ms > 0) {
            zigbee_plug_set(true);
            xSemaphoreTake(s_mutex, portMAX_DELAY); s_plug_on = true; xSemaphoreGive(s_mutex);
            vTaskDelay(pdMS_TO_TICKS(on_ms));
        }
        if (off_ms > 0) {
            zigbee_plug_set(false);
            xSemaphoreTake(s_mutex, portMAX_DELAY); s_plug_on = false; xSemaphoreGive(s_mutex);
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

/* ── Run task (spawned per run, deletes itself when done) ─────────────────── */
static void run_task(void *arg)
{
    pid_ctrl_t pid;
    pid_reset(&pid, 0.05f, 0.001f, 1.5f);
    const float dt = (float)CTRL_TASK_MS / 1000.0f;
    int64_t start_us = esp_timer_get_time();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CTRL_TASK_MS));

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        ctrl_state_t state = s_state;
        xSemaphoreGive(s_mutex);
        if (state != CTRL_STATE_RUNNING) break;

        float temp = 0.0f;
        if (temperature_read(&temp) != ESP_OK) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_state = CTRL_STATE_ERROR; s_duty_steps = 0;
            xSemaphoreGive(s_mutex);
            ESP_LOGE(TAG, "Temperature read failed — stopping");
            break;
        }

        int elapsed_s = (int)((esp_timer_get_time() - start_us) / 1000000LL);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        int total_s       = profile_total_duration(&s_profile);
        float setpoint    = profile_interpolate_setpoint(&s_profile, elapsed_s);
        const char *phase = profile_phase_at(&s_profile, elapsed_s);
        xSemaphoreGive(s_mutex);

        if (elapsed_s >= total_s) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_state = CTRL_STATE_COMPLETE; s_duty_steps = 0; s_elapsed_s = elapsed_s;
            xSemaphoreGive(s_mutex);
            ESP_LOGI(TAG, "Reflow complete after %d s", elapsed_s);
            break;
        }

        float demand;
        bool cooling = profile_is_cooling_at(&s_profile, elapsed_s);
        if (cooling) {
            demand = 0.0f;
            pid_reset(&pid, 0.05f, 0.001f, 1.5f);
        } else if ((setpoint - temp) > RAMP_HANDOFF_DELTA) {
            demand = 1.0f;
            pid_reset(&pid, 0.05f, 0.001f, 1.5f);
            pid.prev_temp = temp; pid.initialized = true;
        } else {
            demand = pid_update(&pid, setpoint, temp, dt);
        }

        int steps = (int)(demand * (float)CYCLE_STEPS + 0.5f);
        if (steps < 0) steps = 0;
        if (steps > CYCLE_STEPS) steps = CYCLE_STEPS;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_temp = temp; s_setpoint = setpoint; s_elapsed_s = elapsed_s;
        strlcpy(s_phase, phase, sizeof(s_phase));
        s_duty_steps = steps;
        xSemaphoreGive(s_mutex);
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_duty_steps = 0;
    xSemaphoreGive(s_mutex);
    vTaskDelete(NULL);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t reflow_ctrl_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    xTaskCreate(relay_cycle_task, "relay", 2048, NULL, 6, NULL);
    return ESP_OK;
}

esp_err_t reflow_ctrl_start(const reflow_profile_t *profile)
{
    if (!zigbee_plug_is_paired()) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_sensor_ok) {
        xSemaphoreGive(s_mutex);
        ESP_LOGE(TAG, "Cannot start: temperature sensor not available");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_state == CTRL_STATE_RUNNING) { xSemaphoreGive(s_mutex); return ESP_ERR_INVALID_STATE; }
    memcpy(&s_profile, profile, sizeof(s_profile));
    s_state = CTRL_STATE_RUNNING; s_elapsed_s = 0; s_duty_steps = 0;
    xSemaphoreGive(s_mutex);

    xTaskCreate(run_task, "ctrl_run", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Started: %s", profile->name);
    return ESP_OK;
}

esp_err_t reflow_ctrl_stop(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_state = CTRL_STATE_IDLE; s_duty_steps = 0;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Stopped");
    return ESP_OK;
}

esp_err_t reflow_ctrl_set_plug(bool on)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    ctrl_state_t state = s_state;
    xSemaphoreGive(s_mutex);
    if (state == CTRL_STATE_RUNNING) return ESP_ERR_INVALID_STATE;

    zigbee_plug_set(on);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_plug_on = on;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void reflow_ctrl_get_status(ctrl_status_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    out->temp       = s_temp;
    out->setpoint   = s_setpoint;
    out->elapsed_s  = s_elapsed_s;
    out->state      = s_state;
    out->plug_on    = s_plug_on;
    out->duty_steps = s_duty_steps;
    out->sensor_ok  = s_sensor_ok;
    strlcpy(out->phase, s_phase, sizeof(out->phase));
    xSemaphoreGive(s_mutex);
}
