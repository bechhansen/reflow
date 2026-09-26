#include "reflow_algo.h"
#include <math.h>
#include <string.h>

void algo_params_default(algo_params_t *p)
{
    /* Measured on this iron (hil.py fit over a full-power step to 199 C and
       its cool-down, 2026-09-25): 2.84 C/s gross at full power, heat loss
       almost purely quadratic (~0.38 C/s at 200 C), no dead time to speak of
       but a ~2 s lag from element to soleplate. The linear loss term is
       pinned small, hence the large tau and K. Gains are SIMC for the
       near-integrating plant (2.84 C/s per unit duty) with tau_c = 3 x the
       2 s lag, detuned for the 2 s dwell: kp = 1/(2.84 * 8), ti = 4 * 8. */
    *p = (algo_params_t){
        .K            = 28000.0f,
        .tau          = 10000.0f,
        .theta        = 2.0f,
        .loss_quad    = 0.12f,
        .t_room       = 22.0f,
        .kp           = 0.044f,
        .ti           = 32.0f,
        .kd           = 0.0f,
        .i_max        = 0.3f,
        .lookahead    = 4.0f,
        .over_weight  = 2.0f,
        .coast        = 3.0f,
        .coast_margin = 0.0f,
        .lag_band     = 3.0f,
        .lag_span     = 10.0f,
        .max_stretch  = 180.0f,
        .max_hold     = 0.0f,     /* follow the profile in real time */
        .min_dwell    = 2.0f,
        .min_on       = 2.0f,
        .sd_hyst      = 0.5f,
        .pulse_duty   = 0.3f,
        .pulse_rise   = 0.0f,
        .plug_timeout = 60.0f,
        .refresh      = 4.0f,
        .reassert     = 2.5f,
        .anomaly_after = 6.0f,   /* 5 s false-alarmed on stored heat after a fast ramp */
        .anomaly_slope = 1.0f,
        .filt_tau     = 1.0f,
        .over_margin  = 25.0f,
    };
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static void start_common(algo_t *a, const algo_params_t *prm, algo_mode_t mode)
{
    memset(a, 0, sizeof(*a));
    a->prm   = *prm;
    a->mode  = mode;
    a->phase = ALGO_RUNNING;
}

bool algo_start_profile(algo_t *a, const algo_params_t *prm, const reflow_profile_t *prof)
{
    if (!curve_valid(prof)) return false;
    start_common(a, prm, ALGO_MODE_PROFILE);
    a->prof       = prof;
    a->pt         = 0.0f;          /* always the whole profile, from its start */
    a->cool_start = curve_cool_start(prof);
    a->t_limit    = curve_max_between(prof, 0.0f, curve_duration(prof)) + prm->over_margin;
    return true;
}

void algo_start_step(algo_t *a, const algo_params_t *prm, float duty, float secs, float max_temp)
{
    start_common(a, prm, ALGO_MODE_STEP);
    a->step_duty = clampf(duty, 0.0f, 1.0f);
    a->step_s    = secs;
    a->step_max  = max_temp;
    a->t_limit   = max_temp + prm->over_margin;
}

void algo_start_hold(algo_t *a, const algo_params_t *prm, float temp)
{
    start_common(a, prm, ALGO_MODE_HOLD);
    a->hold_temp = temp;
    a->t_limit   = temp + prm->over_margin;
}

/* Least-squares slope over the newest n samples of the history, °C/s. */
static float history_slope_n(const algo_t *a, int n)
{
    if (n > a->hist_n) n = a->hist_n;
    if (n < 3) return 0.0f;
    float mt = 0, mv = 0;
    for (int k = 0; k < n; k++) {
        int i = (a->hist_i - 1 - k + ALGO_SLOPE_N) % ALGO_SLOPE_N;
        mt += a->hist_t[i]; mv += a->hist_v[i];
    }
    mt /= (float)n; mv /= (float)n;
    float num = 0, den = 0;
    for (int k = 0; k < n; k++) {
        int i = (a->hist_i - 1 - k + ALGO_SLOPE_N) % ALGO_SLOPE_N;
        float dt = a->hist_t[i] - mt;
        num += dt * (a->hist_v[i] - mv);
        den += dt * dt;
    }
    return den > 0.0f ? num / den : 0.0f;
}

static float history_slope(const algo_t *a) { return history_slope_n(a, ALGO_SLOPE_N); }

static float model_loss(const algo_params_t *p, float temp)
{
    float d = temp - p->t_room;
    return d > 0.0f ? d + p->loss_quad * d * d : d;
}

static void fault(algo_t *a, algo_fault_t f)
{
    if (a->phase == ALGO_FAULT) return;
    a->phase = ALGO_FAULT;
    a->fault = f;
}

/* Low power: heat comes as isolated minimum pulses, each making a bump of
   about one pulse-rise. Time them on the temperature instead of on the
   accumulated duty: fire when the temperature predicted pulse_h ahead falls
   half a rise below the target then, so each bump is centred on the target.
   Only where the target moves less than half a bump during one pulse cycle;
   on a steeper stretch single pulses cannot keep up and the duty-based
   output stage is better. Generic: it uses the model and the curve's shape,
   not phase names, so it serves any flat or slow stretch of any profile and
   any heat source once its model is fitted. While on, the same test turns
   the heater off again as soon as the dwell allows. Returns false where it
   does not apply. */
static bool pulse_timing(const algo_t *a, const algo_params_t *p, float pulse_h,
                         float sp_h, float sp_rate, bool *want)
{
    /* A short slope: the 6 s one lags by seconds at a turning point (where
       a ramp flattens into a soak), and a late pulse there sags. */
    float sl = history_slope_n(a, ALGO_PULSE_SLOPE_N);
    float rise = p->pulse_rise;
    if (rise <= 0.0f) {
        /* The heat of one pulse, less what the iron loses while its bump
           builds (it keeps cooling at about the current rate while off). */
        float heat = p->K / p->tau * p->min_on;
        rise = clampf(heat + (sl < 0.0f ? sl : 0.0f) * (p->min_on + pulse_h), 0.3f * heat, heat);
    }
    if (fabsf(sp_rate) * (p->min_on + pulse_h) > 0.5f * rise) return false;
    float t_last = a->hist_v[(a->hist_i - 1 + ALGO_SLOPE_N) % ALGO_SLOPE_N];
    *want = t_last + sl * pulse_h < sp_h - 0.5f * rise;
    return true;
}

void algo_step(algo_t *a, const algo_input_t *in, algo_output_t *out)
{
    const algo_params_t *p = &a->prm;
    memset(out, 0, sizeof(*out));

    float dt = 0.0f;
    if (!a->started) {
        a->started     = true;
        a->t_sensor_ok = in->t;
        a->t_plug_ok   = in->t;
        a->t_change    = -1e9f;
        a->t_cmd       = -1e9f;
        a->t_start     = in->t;
    } else {
        dt = in->t - a->t_last;
        if (dt < 0.0f) dt = 0.0f;
    }
    a->t_last = in->t;
    float t_run = in->t - a->t_start;

    /* ── Inputs ── */
    /* The MLX90614 can return a wildly wrong value that still passes its PEC
       check: a reading more than ALGO_JUMP_C from the last one is treated as
       no reading (the plate cannot move that fast in one tick); if it
       persists, the sensor timeout faults the run. */
    bool reading = in->sensor_ok;
    if (reading && a->hist_n > 0) {
        float last = a->hist_v[(a->hist_i - 1 + ALGO_SLOPE_N) % ALGO_SLOPE_N];
        if (fabsf(in->temp - last) > ALGO_JUMP_C) reading = false;
        /* A frozen reading while heating means heating blind: the
           over-temperature check could never trip. */
        if (reading && in->temp == last && a->have_state && a->last_on) {
            if (++a->same_n >= ALGO_STALE_N) fault(a, ALGO_FAULT_STALE);
        } else if (reading) {
            a->same_n = 0;
        }
    }
    if (reading) {
        a->t_sensor_ok = in->t;
        if (a->hist_n == 0) a->tf = in->temp;
        else                a->tf += (in->temp - a->tf) * dt / (p->filt_tau + dt);
        a->hist_t[a->hist_i] = in->t;
        a->hist_v[a->hist_i] = in->temp;
        a->hist_i = (a->hist_i + 1) % ALGO_SLOPE_N;
        if (a->hist_n < ALGO_SLOPE_N) a->hist_n++;
        if (in->temp > a->t_limit || a->tf > a->t_limit) fault(a, ALGO_FAULT_OVERTEMP);
    } else if (in->t - a->t_sensor_ok > ALGO_SENSOR_TO_S) {
        fault(a, ALGO_FAULT_SENSOR);
    }

    /* The plug's state as far as we know: the last one it confirmed, or the
       last one we commanded after that. A confirmed state only counts once
       no change is pending; one that differs from what we assumed (a lost
       command, or someone else switched it) replaces the assumption. */
    if (in->plug_ok) {
        a->t_plug_ok = in->t;
        if (!a->have_state) {
            a->have_state = true;
            a->last_on    = in->plug_on;
        } else if (!in->plug_pending && in->plug_on != a->last_on) {
            a->last_on  = in->plug_on;
            a->t_change = in->t;
        }
    } else if (in->t - a->t_plug_ok > p->plug_timeout) {
        fault(a, ALGO_FAULT_PLUG);
    }

    float slope = history_slope(a);
    out->tf    = a->tf;
    out->slope = slope;

    /* ── Targets ── */
    bool  forced = (a->hist_n == 0);   /* nothing to control on yet */
    float sp = 0, sp_ff = 0, dsp_ff = 0, limit = 1e9f, rate = 1.0f;
    /* Pulse timing looks this far ahead: the heat source's lag (it works
       on the raw reading, so no filter lag to add). */
    const float pulse_h = p->theta;
    float sp_h = 0;                    /* target pulse_h seconds from now */
    float sp_rate = 0;                 /* target slope now, °C/s */
    if (a->phase == ALGO_RUNNING || a->phase == ALGO_COOLING) {
        switch (a->mode) {
        case ALGO_MODE_PROFILE: {
            const reflow_profile_t *pr = a->prof;
            if (!a->cooling && a->pt >= a->cool_start) a->cooling = true;
            /* Hold budget per phase: max_hold seconds, then the phase runs
               in real time however far behind the iron is. */
            int seg = curve_segment_at(pr, a->pt);
            if (seg != a->hold_seg) { a->hold_seg = seg; a->seg_held = 0.0f; }
            bool may_hold = a->seg_held < p->max_hold;
            if (!a->cooling && a->hist_n > 0 && curve_slope_at(pr, a->pt) >= 0.0f && may_hold) {
                float lag = curve_temp_at(pr, a->pt) - a->tf;
                if (lag > p->lag_band)
                    rate = clampf(1.0f - (lag - p->lag_band) / p->lag_span, 0.0f, 1.0f);
            }
            a->pt       += rate * dt;
            a->stretch  += (1.0f - rate) * dt;
            a->seg_held += (1.0f - rate) * dt;
            if (may_hold && a->seg_held >= p->max_hold && p->max_hold > 0.0f)
                out->hold_capped = true;       /* budget used up just now */
            if (a->stretch > p->max_stretch) fault(a, ALGO_FAULT_STALL);

            sp     = curve_temp_at(pr, a->pt);
            sp_h   = curve_temp_at(pr, a->pt + pulse_h);
            sp_rate = curve_slope_at(pr, a->pt) * rate;
            sp_ff  = curve_temp_at(pr, a->pt + p->lookahead);
            dsp_ff = curve_slope_at(pr, a->pt + p->lookahead) * rate;
            /* The coast guard only matters where the curve tops out within
               the horizon (a peak before a fall). At a mere change of ramp
               rate the lookahead feedforward already eases off, and a guard
               there would open a lag the dead time can't recover. */
            float h_end = a->pt + p->lookahead + p->coast;
            float h_max = curve_max_between(pr, a->pt, h_end);
            if (curve_temp_at(pr, h_end) < h_max - 0.5f) limit = h_max;
            if (a->cooling) {
                forced   = true;
                a->phase = ALGO_COOLING;
                if (a->pt >= curve_duration(pr)) a->phase = ALGO_DONE;
            }
            out->segment = curve_segment_at(pr, a->pt);
            break;
        }
        case ALGO_MODE_STEP:
            if (!a->step_off && (t_run >= a->step_s || a->tf >= a->step_max)) a->step_off = true;
            if (a->step_off) { forced = true; a->phase = ALGO_COOLING; }
            break;
        case ALGO_MODE_HOLD:
            sp = sp_ff = sp_h = limit = a->hold_temp;
            break;
        }
    }
    out->profile_t = a->pt;
    out->sp        = sp;
    out->sp_ff     = sp_ff;

    bool ended = (a->phase == ALGO_DONE || a->phase == ALGO_FAULT);
    if (ended) forced = true;

    /* ── Control law ── */
    float u = 0.0f;
    if (!forced && a->mode == ALGO_MODE_STEP) {
        u = a->step_duty;
    } else if (!forced) {
        float u_ff = clampf((p->tau * dsp_ff + model_loss(p, sp_ff)) / p->K, 0.0f, 1.0f);
        float e    = sp - a->tf;
        float ew   = e < 0.0f ? e * p->over_weight : e;
        float P    = p->kp * ew;
        float D    = -p->kd * slope;
        /* Coast guard: what is already in the iron would carry it past the
           highest target coming up. */
        bool guard = a->tf + (slope > 0.0f ? slope : 0.0f) * p->coast > limit - p->coast_margin;
        /* While the clock is held the lag is being absorbed by the profile;
           integrating it too would overshoot once the iron catches up. The
           coast guard does NOT stop integration: it holds the heater off
           exactly while the iron is at or above a plateau, and an integrator
           blind to those periods sees only the below-target ones and winds
           up (seen: +0.21 duty extra in a hold, a +1.5 C mean offset). */
        if (!forced && rate >= 1.0f && p->ti > 0.0f && fabsf(e) < ALGO_I_BAND) {
            float raw = u_ff + P + a->integ + D;
            float di  = p->kp * ew * dt / p->ti;
            if (!((raw >= 1.0f && di > 0.0f) || (raw <= 0.0f && di < 0.0f)))
                a->integ = clampf(a->integ + di, -p->i_max, p->i_max);
        }
        u = clampf(u_ff + P + a->integ + D, 0.0f, 1.0f);
        if (guard) forced = true;
        out->u_ff = u_ff; out->p = P; out->i = a->integ; out->d = D;
    }
    if (forced) u = 0.0f;
    out->u          = u;
    out->forced_off = forced;

    /* ── Output stage ── */
    bool s = a->have_state && a->last_on;
    bool want;
    if (forced) {
        a->acc = 0.0f;
        want   = false;
    } else if (u < p->pulse_duty && a->mode != ALGO_MODE_STEP &&
               pulse_timing(a, p, pulse_h, sp_h, sp_rate, &want)) {
        /* Low power on a slow stretch: pulses timed on the temperature. */
        a->acc = 0.0f;
    } else {
        float lim = p->sd_hyst + p->min_dwell;
        a->acc = clampf(a->acc + (u - (s ? 1.0f : 0.0f)) * dt, -lim, lim);
        want = s;
        if (u >= 0.999f)                     want = true;
        else if (u <= 0.001f)                want = false;
        else if (!s && a->acc >  p->sd_hyst) want = true;
        else if ( s && a->acc < -p->sd_hyst) want = false;
    }
    out->want_on = want;

    if (!in->plug_pending) {
        if (ended) {
            /* Safety off: ignores the dwell. Unknown state is treated as on. */
            if (!in->plug_ok || in->plug_on || s) out->cmd = ALGO_CMD_OFF;
        } else if (!s && in->t - a->t_change >= p->anomaly_after &&
                   a->hist_n >= ALGO_SLOPE_N && slope > p->anomaly_slope) {
            /* Meant to be off for a while, yet still heating fast: the relay
               is on whatever the plug last confirmed (seen on hardware: a
               confirmed OFF followed by 14 s of full-power heating). Off at
               once, ignoring the dwell. */
            out->cmd     = ALGO_CMD_OFF;
            out->anomaly = true;
        } else if (want != s && in->t - a->t_change >= (s ? p->min_on : p->min_dwell)) {
            out->cmd = want ? ALGO_CMD_ON : ALGO_CMD_OFF;
        } else if (want == s && (in->t - a->t_cmd >= p->refresh ||
                                 (a->t_cmd == a->t_change && in->t - a->t_cmd >= p->reassert))) {
            /* Repeat the state we want: once reassert seconds after each
               change (where a late retransmission of the previous command
               would land), then every refresh seconds. It moves nothing if
               the plug is already there, undoes a change we did not make,
               and while the state is Unknown the transmission opens a window
               for the plug's reply. */
            out->cmd = want ? ALGO_CMD_ON : ALGO_CMD_OFF;
        }
    }
    if (out->cmd != ALGO_CMD_NONE) {
        bool on = (out->cmd == ALGO_CMD_ON);
        if (on != s) a->t_change = in->t;
        a->last_on    = on;
        a->have_state = true;
        a->t_cmd      = in->t;
    }

    out->phase = a->phase;
    out->fault = a->fault;
}

const char *algo_phase_str(algo_phase_t p)
{
    switch (p) {
        case ALGO_RUNNING: return "running";
        case ALGO_COOLING: return "cooling";
        case ALGO_DONE:    return "complete";
        default:           return "error";
    }
}

const char *algo_fault_str(algo_fault_t f)
{
    switch (f) {
        case ALGO_FAULT_SENSOR:   return "sensor";
        case ALGO_FAULT_STALE:    return "sensor_stale";
        case ALGO_FAULT_OVERTEMP: return "overtemp";
        case ALGO_FAULT_STALL:    return "stall";
        case ALGO_FAULT_PLUG:     return "plug";
        default:                  return "";
    }
}
