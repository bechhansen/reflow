#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "reflow_profile.h"
#include <stdbool.h>

typedef enum {
    CTRL_STATE_IDLE,
    CTRL_STATE_RUNNING,
    CTRL_STATE_COMPLETE,
    CTRL_STATE_ERROR,
} ctrl_state_t;

typedef struct {
    float        temp;
    ctrl_state_t state;
    bool         sensor_ok;
    float        ambient;      /* MLX90614 die temperature (°C) */
} ctrl_status_t;

esp_err_t reflow_ctrl_init(void);

/* The reflow control algorithm has been removed; these are deliberately empty
   so the UI's Start and Stop buttons remain wired to something that exists. */
esp_err_t reflow_ctrl_start(const reflow_profile_t *profile);
esp_err_t reflow_ctrl_stop(void);

void      reflow_ctrl_get_status(ctrl_status_t *out);
