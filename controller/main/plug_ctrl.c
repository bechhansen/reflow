#include "plug_ctrl.h"
#include "plug_fsm.h"
#include "zigbee_plug.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"

/* ── Concurrency ─────────────────────────────────────────────────
   The FSM is only ever touched in Zigbee context: listener callbacks and the
   timer alarm run in the Zigbee task, and the public set/toggle calls take
   esp_zb_lock. So the stack lock is the FSM's lock too.

   Readers outside Zigbee context (telemetry, console) get a snapshot copied
   under a spinlock after every step, so they never wait on the stack. */

static const char *TAG = "plug_ctrl";

static plug_fsm_t        s_fsm;
static volatile bool     s_network_up = false;
static plug_result_cb_t  s_result_cbs[PLUG_CTRL_MAX_RESULT_CBS];
static int               s_num_result_cbs = 0;

static portMUX_TYPE      s_snap_mux = portMUX_INITIALIZER_UNLOCKED;
static plug_status_t     s_snap = { .state = PLUG_UNKNOWN };

static void timer_cb(uint8_t param);

static void publish_snapshot(void)
{
    plug_status_t st = {
        .available = s_network_up && zigbee_plug_is_paired(),
        .state     = s_fsm.state,
        .pending   = s_fsm.pending,
        .target    = s_fsm.target,
    };
    taskENTER_CRITICAL(&s_snap_mux);
    s_snap = st;
    taskEXIT_CRITICAL(&s_snap_mux);
}

/* Carry out what the FSM decided. Zigbee context only. */
static void run(const plug_actions_t *a)
{
    if (a->arm || a->disarm) esp_zb_scheduler_alarm_cancel(timer_cb, 0);
    if (a->send_cmd)  zigbee_plug_send_on_off(a->cmd_on);
    if (a->send_read) plug_fsm_set_read_tsn(&s_fsm, zigbee_plug_read_on_off());
    if (a->arm)       esp_zb_scheduler_alarm(timer_cb, 0, a->arm_ms);

    publish_snapshot();

    if (a->done) {
        if (a->done_ok)
            ESP_LOGI(TAG, "confirmed %s", plug_state_str(s_fsm.state));
        else
            ESP_LOGW(TAG, "change to %s NOT confirmed (%s); plug is %s",
                     s_fsm.target ? "ON" : "OFF", a->reason, plug_state_str(s_fsm.state));
        for (int i = 0; i < s_num_result_cbs; i++)
            s_result_cbs[i](a->done_ok, s_fsm.state, a->reason);
    }
}

static void timer_cb(uint8_t param)
{
    (void)param;
    plug_actions_t a = {0};
    plug_fsm_timer(&s_fsm, &a);
    run(&a);
}

/* ── Transport events (Zigbee task) ───────────────────────────────────── */

static void on_network_up(void)
{
    s_network_up = true;
    if (!zigbee_plug_is_paired()) {
        ESP_LOGI(TAG, "network up, no paired plug");
        publish_snapshot();
        return;
    }
    /* A power cut or crash leaves the plug in whatever state it was last
       commanded to, so it is never assumed off after a restart: switch it off
       and confirm, which also gives the first known state. */
    ESP_LOGI(TAG, "network up: switching plug OFF");
    plug_actions_t a = {0};
    if (plug_fsm_request(&s_fsm, false, &a) == PLUG_REQ_OK) run(&a);
}

static void on_paired(void)
{
    plug_actions_t a = {0};
    plug_fsm_start(&s_fsm, &a);
    run(&a);
}

static void on_unpaired(void)
{
    plug_actions_t a = {0};
    plug_fsm_reset(&s_fsm, &a);
    run(&a);
}

static void on_read_result(uint8_t tsn, bool ok, bool on)
{
    plug_actions_t a = {0};
    plug_fsm_read_result(&s_fsm, tsn, ok, on, &a);
    run(&a);
}

static void on_report(bool on)
{
    plug_fsm_report(&s_fsm, on);
    publish_snapshot();
}

static const zigbee_plug_listener_t s_listener = {
    .on_network_up  = on_network_up,
    .on_paired      = on_paired,
    .on_unpaired    = on_unpaired,
    .on_read_result = on_read_result,
    .on_report      = on_report,
};

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t plug_ctrl_init(void)
{
    plug_fsm_init(&s_fsm);
    publish_snapshot();
    zigbee_plug_set_listener(&s_listener);
    return ESP_OK;
}

esp_err_t plug_ctrl_add_result_cb(plug_result_cb_t cb)
{
    if (s_num_result_cbs >= PLUG_CTRL_MAX_RESULT_CBS) return ESP_ERR_NO_MEM;
    s_result_cbs[s_num_result_cbs++] = cb;
    return ESP_OK;
}

/* toggle=true: invert the confirmed state instead of using `on`. */
static esp_err_t request(bool on, bool toggle)
{
    /* Before the network is up the stack lock may not even exist. */
    if (!s_network_up)            return ESP_ERR_NOT_SUPPORTED;
    if (!zigbee_plug_is_paired()) return ESP_ERR_NOT_FOUND;

    esp_err_t err = ESP_OK;
    plug_actions_t a = {0};
    esp_zb_lock_acquire(portMAX_DELAY);
    if (s_fsm.pending) {
        err = ESP_ERR_INVALID_STATE;
    } else if (toggle && s_fsm.state == PLUG_UNKNOWN) {
        err = ESP_ERR_INVALID_RESPONSE;
    } else {
        if (toggle) on = (s_fsm.state != PLUG_ON);
        if (plug_fsm_request(&s_fsm, on, &a) == PLUG_REQ_BUSY) err = ESP_ERR_INVALID_STATE;
        else run(&a);
    }
    esp_zb_lock_release();

    if (err == ESP_OK) ESP_LOGI(TAG, "request %s accepted", on ? "ON" : "OFF");
    else               ESP_LOGW(TAG, "request rejected: %s", plug_ctrl_err_reason(err));
    return err;
}

esp_err_t plug_ctrl_set(bool on)  { return request(on, false); }
esp_err_t plug_ctrl_toggle(void)  { return request(false, true); }

void plug_ctrl_get_status(plug_status_t *out)
{
    taskENTER_CRITICAL(&s_snap_mux);
    *out = s_snap;
    taskEXIT_CRITICAL(&s_snap_mux);
}

const char *plug_ctrl_err_reason(esp_err_t err)
{
    switch (err) {
    case ESP_OK:                   return "ok";
    case ESP_ERR_INVALID_STATE:    return "busy";
    case ESP_ERR_INVALID_RESPONSE: return "unknown";
    case ESP_ERR_NOT_FOUND:        return "not_paired";
    case ESP_ERR_NOT_SUPPORTED:    return "unavailable";
    case ESP_ERR_NOT_ALLOWED:      return "running";   /* callers: a reflow run owns the plug */
    default:                       return esp_err_to_name(err);
    }
}
