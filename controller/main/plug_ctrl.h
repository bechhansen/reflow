#pragma once
/* Plug service: the one place that switches the Zigbee plug. Anything that
   wants the heater on or off (the UI's test toggle, the console, a future
   control algorithm) goes through here.

   Guarantees:
   - A change is DONE only when a read of the plug's OnOff attribute reports
     the requested value. Until then it is pending.
   - While a change is pending, further requests are rejected, never queued.
   - If the plug stops answering, its state is reported as Unknown rather than
     guessed. An idle poll every PLUG_POLL_MS recovers it.
   - A paired plug is switched off (and confirmed) whenever Zigbee starts. */
#include "esp_err.h"
#include "plug_fsm.h"
#include <stdbool.h>

typedef struct {
    bool         available;   /* Zigbee running and a plug paired */
    plug_state_t state;       /* last confirmed state */
    bool         pending;     /* a change awaits confirmation */
    bool         target;      /* requested value while pending */
} plug_status_t;

/* Called in the Zigbee task when a requested change finishes. On failure,
   reason is "timeout", "mismatch" or "unpaired", and state is the real state
   (possibly PLUG_UNKNOWN). Keep it short and non-blocking. */
typedef void (*plug_result_cb_t)(bool ok, plug_state_t state, const char *reason);

/* Before zigbee_plug_init(). */
esp_err_t   plug_ctrl_init(void);

/* Up to PLUG_CTRL_MAX_RESULT_CBS subscribers, e.g. the web UI and a control
   algorithm. Register during init, before Zigbee starts. */
#define PLUG_CTRL_MAX_RESULT_CBS 4
esp_err_t   plug_ctrl_add_result_cb(plug_result_cb_t cb);

/* Request the plug on or off. The result arrives via the result callbacks.
   Returns:
     ESP_OK                 accepted, now pending
     ESP_ERR_INVALID_STATE  a change is already pending; nothing was sent
     ESP_ERR_NOT_FOUND      no plug paired
     ESP_ERR_NOT_SUPPORTED  Zigbee is not running (AP / setup mode) */
esp_err_t   plug_ctrl_set(bool on);

/* Invert the confirmed state. Same returns as plug_ctrl_set(), plus
   ESP_ERR_INVALID_RESPONSE when the current state is Unknown. */
esp_err_t   plug_ctrl_toggle(void);

/* Non-blocking; safe from any task, including esp_timer callbacks. */
void        plug_ctrl_get_status(plug_status_t *out);

/* Short machine-readable reason for a plug_ctrl_set()/toggle() rejection. */
const char *plug_ctrl_err_reason(esp_err_t err);
