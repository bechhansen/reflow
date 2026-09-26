#pragma once
/* Reflow controller: samples the MLX90614 every ALGO_DT_S, runs the control
   law in reflow_algo.c while a run is active, and switches the heater through
   plug_ctrl (the only thing that switches the plug).

   A run is a profile, or one of two open-loop / fixed-point tests used to
   identify the iron's thermal model (see controller/tools/hil.py). Whenever a
   run ends, however it ends, the plug is switched off and that is confirmed.

   While a run is active, or logging is switched on, the controller prints one
   machine-readable line per tick on the console:
     @T,ms,state,pt,sp,sp_ff,temp,tf,slope,u_ff,p,i,u,want,forced,plug,pending
   plug is 1/0 (confirmed on/off) or -1 (unknown/unavailable). Events:
     @S,ms,<mode>,<args>              run started
     @P,<key>=<val>,...               parameters in force
     @E,ms,cmd,<on|off>,<result>      plug command issued (result: ok or reason)
     @E,ms,plug,<ok|fail>,<state>,<reason>   plug change finished
     @E,ms,end,<state>,<fault>        run ended */
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "reflow_profile.h"
#include "reflow_algo.h"

typedef enum {
    CTRL_STATE_IDLE,
    CTRL_STATE_RUNNING,
    CTRL_STATE_COOLING,     /* heater off for good, profile clock still running */
    CTRL_STATE_COMPLETE,
    CTRL_STATE_ERROR,
} ctrl_state_t;

typedef struct {
    float        temp;
    ctrl_state_t state;
    bool         sensor_ok;
    float        ambient;      /* MLX90614 die temperature (°C) */
    /* Valid while not IDLE: */
    char         profile[PROFILE_NAME_LEN];  /* profile name, or "step"/"hold" */
    char         phase[PROFILE_LABEL_LEN];   /* label of the waypoint being approached */
    float        elapsed;      /* s since start */
    float        profile_t;    /* profile clock, s (lags elapsed when held) */
    float        setpoint;     /* °C, 0 in STEP mode */
    float        power;        /* commanded duty 0..1 */
    const char  *fault;        /* "" or algo_fault_str() */
    uint32_t     run_id;       /* increments at every start */
} ctrl_status_t;

/* After plug_ctrl_init(): registers a plug result callback. */
esp_err_t reflow_ctrl_init(void);

/* A profile run always covers the whole profile from its start, whatever the
   iron's temperature. On failure *reason (if given) is a short
   machine-readable cause: "busy" (a run is active), "sensor", "plug" or
   "profile" (invalid). */
esp_err_t reflow_ctrl_start(const reflow_profile_t *profile, const char **reason);
esp_err_t reflow_ctrl_start_step(float duty, float secs, float max_temp, const char **reason);
esp_err_t reflow_ctrl_start_hold(float temp, const char **reason);

/* Ends any run (heater off, confirmed) and returns to IDLE. */
esp_err_t reflow_ctrl_stop(void);

void        reflow_ctrl_get_status(ctrl_status_t *out);
bool        reflow_ctrl_is_active(void);      /* a run owns the plug */
const char *reflow_ctrl_state_str(ctrl_state_t s);

/* One point of the current (or last) run's trace, for a page opened mid-run. */
typedef struct {
    float  t;       /* s since start */
    float  temp;    /* measured, °C */
    float  sp;      /* target in use, °C (0 in STEP mode) */
    int8_t plug;    /* confirmed heater state: 1 on, 0 off, -1 unknown */
    float  power;   /* requested duty 0..1 */
} ctrl_trace_pt_t;

/* Sampled every tick (0.5 s, 10 minutes before thinning); when the buffer
   fills, it is thinned to every other point and the interval doubles, so a
   long run is kept whole. Copies up to
   max points from index first; returns how many were copied. */
int         reflow_ctrl_trace_get(int first, ctrl_trace_pt_t *out, int max);

/* Console log lines while idle too (always on during a run). */
void        reflow_ctrl_set_log(bool on);

/* Tuning parameters (algo_params_t), by name. Changes apply to the next run;
   save persists them in NVS (namespace "ctrl"), reset restores the defaults. */
esp_err_t   reflow_ctrl_param_set(const char *name, float value);
void        reflow_ctrl_params_print(void);
esp_err_t   reflow_ctrl_params_save(void);
void        reflow_ctrl_params_reset(void);
