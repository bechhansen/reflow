#pragma once
/* Reflow control law: model feedforward + PI(D) trim, driving the Zigbee plug
   through a dwell-limited sigma-delta output stage. Deliberately free of
   ESP-IDF headers so it can be tested on the host (controller/test/reflow_algo).
   reflow_ctrl.c is the shell that feeds it samples every ALGO_DT_S and carries
   out the plug command it returns.

   Plant model (first order plus dead time, from a step test on this iron):
       tau * dT/dt = K * u - L(T - T_room),   L(d) = d + loss_quad * d^2
   where u in [0,1] is the heater's duty, K the steady-state rise at full power
   and theta the dead time between switching and the sensor seeing it.

   Per tick:
   1. Filter the reading; estimate its slope by regression over ALGO_SLOPE_N.
   2. Advance the profile clock, in real time by default (max_hold = 0): the
      goal is the profile's temperature at the profile's time. Optionally
      (max_hold > 0), while the iron lags a non-falling segment by more than
      lag_band, the clock slows (stops at lag_band + lag_span) to keep a soak
      or peak from being cut short, for at most max_hold seconds per phase.
   3. Feedforward from the model for the curve lookahead seconds ahead, which
      covers the dead time: u_ff = (tau * dsp/dt + L(sp - T_room)) / K.
   4. PI(D) on the error against the curve now. Over-temperature error is
      weighted (heat can't be taken back out). The integral only moves while
      the output is not saturated in the error's direction, |e| < ALGO_I_BAND,
      the profile clock is running at full rate, and the heater is not forced
      off.
   5. Coast guard: where the curve peaks within the next lookahead + coast
      seconds (and in HOLD mode), heat is forced off while the filtered
      temperature plus slope * coast would pass that peak minus coast_margin.
      Stops the heat stored in the iron from overshooting the peak.
   6. Cool-down: past the profile's last rise the heater is off for good and
      the run is COOLING until the curve ends, then DONE. Cool-down is
      passive, so the run never waits for the iron to reach any temperature.
   7. Output: sigma-delta on (u - plug state), hysteresis sd_hyst seconds of
      full-power error. Below pulse_duty, where heat comes as isolated
      minimum pulses, each pulse is instead timed on the predicted
      temperature so its bump is centred on the target. On-pulses last at
      least min_on. The plug state is
      the last one it confirmed, or the
      last one commanded since: its read replies are often lost, so Unknown
      is normal and not a reason to stop. A change is only commanded after the
      state has held for min_dwell and when no change is pending. The wanted
      state is re-sent reassert seconds after each change and after refresh
      seconds without a command, and a heater
      that is meant to be off but still heats faster than anomaly_slope
      (anomaly_after seconds after switching off) is switched off at once:
      on this link a confirmed state is not proof of the relay's state.
      Unknown for longer than plug_timeout is a fault. Switching off at the
      end of a run or on a fault ignores the dwell. */
#include "reflow_curve.h"
#include <stdbool.h>
#include <stdint.h>

#define ALGO_DT_S        0.5f
#define ALGO_SLOPE_N     12      /* samples in the slope regression (6 s) */
#define ALGO_PULSE_SLOPE_N 5     /* samples in the pulse-timing slope (2 s) */
#define ALGO_I_BAND      15.0f   /* °C: no integration beyond this error */
#define ALGO_SENSOR_TO_S 2.0f
#define ALGO_JUMP_C      20.0f   /* a reading this far from the last is rejected */
#define ALGO_STALE_N     15      /* identical readings while heating = frozen sensor */

typedef struct {
    float K;            /* °C rise at full power, steady state */
    float tau;          /* s */
    float theta;        /* s, dead time (reported; used for coast defaults) */
    float loss_quad;    /* 1/°C, heat-loss curvature */
    float t_room;       /* °C */
    float kp;           /* duty per °C */
    float ti;           /* s; <= 0 disables the integral */
    float kd;           /* duty per (°C/s), on measurement */
    float i_max;        /* |integral| limit, duty */
    float lookahead;    /* s */
    float over_weight;  /* multiplier on over-temperature error */
    float coast;        /* s */
    float coast_margin; /* °C */
    float lag_band;     /* °C */
    float lag_span;     /* °C */
    float max_stretch;  /* s of profile-clock hold in total before a stall fault */
    float max_hold;     /* s the clock may be held per phase; then real time.
                           0 = always real time */
    float min_dwell;    /* s, least time between switches */
    float min_on;       /* s, least on-time (the dwell after switching on) */
    float sd_hyst;      /* s of full-power error before a switch */
    float pulse_duty;   /* below this duty, pulses are timed on the predicted
                           temperature (centred on the target); 0 = off */
    float pulse_rise;   /* °C bump one minimum pulse makes; 0 = estimate it
                           from the model and the current cooling rate */
    float plug_timeout; /* s of Unknown plug state before a plug fault */
    float refresh;      /* s without a command before the wanted state is re-sent */
    float reassert;     /* s after a change before it is sent once more */
    float anomaly_after;/* s after switching off before the heating check applies */
    float anomaly_slope;/* °C/s: faster than this while off means the relay is on */
    float filt_tau;     /* s */
    float over_margin;  /* °C above the run's highest target: over-temperature
                           fault. Relative, so no heat source is capped. */
} algo_params_t;

void algo_params_default(algo_params_t *p);

typedef enum {
    ALGO_MODE_PROFILE,  /* follow a reflow profile */
    ALGO_MODE_STEP,     /* open loop: fixed duty for a time or up to a temperature */
    ALGO_MODE_HOLD,     /* closed loop at a fixed temperature until stopped */
} algo_mode_t;

typedef enum {
    ALGO_RUNNING,
    ALGO_COOLING,       /* heater off for good, waiting for the curve */
    ALGO_DONE,
    ALGO_FAULT,
} algo_phase_t;

typedef enum {
    ALGO_FAULT_NONE,
    ALGO_FAULT_SENSOR,
    ALGO_FAULT_STALE,       /* reading frozen while heating: heating blind */
    ALGO_FAULT_OVERTEMP,
    ALGO_FAULT_STALL,
    ALGO_FAULT_PLUG,
} algo_fault_t;

typedef enum {
    ALGO_CMD_NONE,
    ALGO_CMD_OFF,
    ALGO_CMD_ON,
} algo_cmd_t;

typedef struct {
    float t;            /* s since start, monotonic */
    bool  sensor_ok;
    float temp;         /* ignored when !sensor_ok */
    bool  plug_ok;      /* plug available and its state known */
    bool  plug_on;      /* confirmed state (when plug_ok) */
    bool  plug_pending;
} algo_input_t;

typedef struct {
    algo_cmd_t   cmd;       /* carry out now; NONE otherwise */
    algo_phase_t phase;
    algo_fault_t fault;
    float profile_t;        /* profile clock, s */
    float sp;               /* target now */
    float sp_ff;            /* target the feedforward aims for */
    float tf;               /* filtered temperature */
    float slope;            /* °C/s */
    float u_ff, p, i, d, u; /* duty terms; u after clamping and forcing */
    bool  forced_off;
    bool  want_on;          /* output-stage decision (before dwell) */
    bool  anomaly;          /* heating while off: cmd is a corrective OFF */
    bool  hold_capped;      /* this phase's hold budget ran out on this tick */
    int   segment;          /* waypoint index ending the current segment */
} algo_output_t;

typedef struct {
    algo_params_t           prm;
    algo_mode_t             mode;
    const reflow_profile_t *prof;       /* PROFILE mode; must outlive the run */
    float step_duty, step_s, step_max;  /* STEP mode */
    float hold_temp;                    /* HOLD mode */
    float t_limit;                      /* over-temperature fault above this */

    bool  started;
    float t_start;
    float t_last;
    float pt;           /* profile clock */
    float stretch;      /* s the clock has been held back */
    int   hold_seg;         /* segment the per-phase hold budget belongs to */
    float seg_held;         /* s held in that segment */
    float cool_start;
    bool  cooling;
    bool  step_off;     /* STEP: heating finished */
    float tf;
    float hist_t[ALGO_SLOPE_N], hist_v[ALGO_SLOPE_N];
    int   hist_n, hist_i;
    float integ;
    float acc;          /* sigma-delta accumulator, s */
    bool  have_state;   /* a plug state is known or has been commanded */
    bool  last_on;      /* assumed plug state: last confirmed, or last commanded since */
    float t_change;     /* when that state last changed */
    float t_cmd;        /* when a command was last issued */
    float t_sensor_ok, t_plug_ok;
    int   same_n;           /* consecutive identical readings */
    algo_phase_t phase;
    algo_fault_t fault;
} algo_t;

/* Always runs the whole profile from its time 0. An iron that starts hotter
   than the curve gets no heat until the curve catches up with it. Returns
   false for an invalid profile. */
bool algo_start_profile(algo_t *a, const algo_params_t *prm, const reflow_profile_t *prof);
void algo_start_step(algo_t *a, const algo_params_t *prm, float duty, float secs, float max_temp);
void algo_start_hold(algo_t *a, const algo_params_t *prm, float temp);

void algo_step(algo_t *a, const algo_input_t *in, algo_output_t *out);

const char *algo_phase_str(algo_phase_t p);
const char *algo_fault_str(algo_fault_t f);
