#pragma once
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
    float        setpoint;
    int          elapsed_s;
    char         phase[PROFILE_LABEL_LEN];
    ctrl_state_t state;
    bool         plug_on;
    int          duty_steps;
    bool         sensor_ok;
} ctrl_status_t;

esp_err_t reflow_ctrl_init(void);
esp_err_t reflow_ctrl_start(const reflow_profile_t *profile);
esp_err_t reflow_ctrl_stop(void);
esp_err_t reflow_ctrl_set_plug(bool on);
void      reflow_ctrl_get_status(ctrl_status_t *out);
