/* Host unit tests for reflow_algo.c and reflow_curve.c.
   Build and run: make -C controller/test/reflow_algo test

   The closed-loop tests run the control law against a simulated iron: the
   FOPDT model the feedforward assumes (optionally mismatched), with its dead
   time split into a pure delay and a first-order lag (the element heating
   the soleplate), and a plug that confirms a command after a fixed latency. */
#include "reflow_algo.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static reflow_profile_t make_profile(const int *t, const float *temp, int n)
{
    reflow_profile_t p;
    memset(&p, 0, sizeof(p));
    for (int i = 0; i < n; i++) {
        p.waypoints[i].time_s = t[i];
        p.waypoints[i].temp   = temp[i];
        snprintf(p.waypoints[i].label, PROFILE_LABEL_LEN, "wp%d", i);
    }
    p.num_waypoints = n;
    return p;
}

static reflow_profile_t sac305(void)
{
    static const int   t[]  = {0, 60, 180, 202, 217, 320};
    static const float tp[] = {25, 150, 180, 245, 245, 50};
    return make_profile(t, tp, 6);
}

/* A lower-peak profile this iron can follow at its default model. */
static reflow_profile_t gentle(void)
{
    static const int   t[]  = {0, 125, 185, 335, 365};
    static const float tp[] = {25, 150, 170, 170, 120};
    return make_profile(t, tp, 5);
}

/* Sn63Pb37-shaped: a steep preheat into a nearly flat soak (0.125 C/s). */
static reflow_profile_t flat_soak(void)
{
    static const int   t[]  = {0, 60, 180, 202, 222, 310};
    static const float tp[] = {25, 150, 165, 230, 230, 50};
    return make_profile(t, tp, 6);
}

#define near(a, b, tol) (fabsf((a) - (b)) <= (tol))

/* ── Curve ─────────────────────────────────────────────────────────── */

static void test_curve(void)
{
    reflow_profile_t p = sac305();
    assert(curve_valid(&p));
    assert(near(curve_temp_at(&p, -5), 25, 1e-4));
    assert(near(curve_temp_at(&p, 30), 87.5f, 1e-3));
    assert(near(curve_temp_at(&p, 400), 50, 1e-4));
    assert(near(curve_slope_at(&p, 30), 125.0f / 60, 1e-4));
    assert(near(curve_slope_at(&p, 210), 0, 1e-6));
    assert(curve_segment_at(&p, 0) == 1);
    assert(curve_segment_at(&p, 60) == 2);
    assert(curve_segment_at(&p, 500) == 5);
    assert(near(curve_duration(&p), 320, 0));
    assert(near(curve_cool_start(&p), 217, 0));       /* flat peak is not cooling */
    assert(near(curve_max_between(&p, 150, 190), 180 + 65.0f * 10 / 22, 1e-3));

    reflow_profile_t bad = p;
    bad.waypoints[2].time_s = 60;
    assert(!curve_valid(&bad));
    bad.num_waypoints = 1;
    assert(!curve_valid(&bad));
}

/* ── Simulated iron + plug ─────────────────────────────────────────── */

#define SIM_DT       ALGO_DT_S
#define SIM_MAX_STEPS 20000

typedef struct {
    float K, tau, theta, t_room;   /* plant; theta = delay + lag */
    float loss_quad;
    float lag_frac;                /* share of theta that is a first-order lag */
    float u_lag;
    float latency;                 /* plug: command -> confirmed */
    float T;
    bool  relay;                   /* real relay state */
    bool  confirmed;               /* what plug_ctrl reports */
    bool  pending;
    float pending_until;
    bool  pending_target;
    float relay_hist[256];         /* relay state per step, for the dead time */
    int   hist_i;
    /* observations */
    int   cmds, cmds_while_pending, switches;
    float last_switch_t, min_gap;
    float max_err_track, peak, max_T;
    bool  sensor_ok, plug_ok;
    bool  stuck_on;                /* relay welded on: ignores every command */
    /* Late retransmission: an OFF within ghost_window s of the previous ON is
       followed, 0.5 s later, by the relay switching back on. The plug still
       confirms OFF. */
    float ghost_window, ghost_at, last_on_cmd_t;
    int   ghosts, anomalies;
} sim_t;

static void sim_init(sim_t *s, const algo_params_t *m, float T0)
{
    memset(s, 0, sizeof(*s));
    s->K = m->K; s->tau = m->tau; s->theta = m->theta; s->t_room = m->t_room;
    s->loss_quad = m->loss_quad;
    s->latency = 1.0f;
    s->lag_frac = 0.7f;
    s->T = T0;
    s->last_switch_t = -1e9f;
    s->min_gap = 1e9f;
    s->sensor_ok = s->plug_ok = true;
    s->ghost_at = -1;
    s->last_on_cmd_t = -1e9f;
}

/* One tick: the controller sees the current state, then the world moves. */
static algo_output_t sim_tick(sim_t *s, algo_t *a, float t)
{
    algo_input_t in = {
        .t = t, .sensor_ok = s->sensor_ok, .temp = s->T,
        .plug_ok = s->plug_ok, .plug_on = s->confirmed, .plug_pending = s->pending,
    };
    algo_output_t o;
    algo_step(a, &in, &o);

    if (o.anomaly) s->anomalies++;
    if (s->ghost_at >= 0 && t >= s->ghost_at) { s->relay = true; s->ghost_at = -1; s->ghosts++; }
    if (o.cmd == ALGO_CMD_OFF && s->ghost_window > 0 && t - s->last_on_cmd_t < s->ghost_window &&
        s->ghosts < 3)
        s->ghost_at = t + 0.5f;
    if (o.cmd == ALGO_CMD_ON) s->last_on_cmd_t = t;
    if (s->stuck_on) s->relay = true;
    if (o.cmd != ALGO_CMD_NONE) {
        s->cmds++;
        if (s->pending) s->cmds_while_pending++;
        s->pending        = true;
        s->pending_target = (o.cmd == ALGO_CMD_ON);
        s->pending_until  = t + s->latency;
        /* The relay moves at once; a command to the state it is already in
           (a refresh) also cancels a pending ghost only if it is an OFF. */
        s->relay          = s->pending_target || s->stuck_on;
    }
    if (s->pending && t + SIM_DT >= s->pending_until) {
        s->pending = false;
        if (s->confirmed != s->pending_target) {
            float now = t + SIM_DT;
            float gap = now - s->last_switch_t;
            if (gap < s->min_gap) s->min_gap = gap;
            s->last_switch_t = now;
            s->switches++;
        }
        s->confirmed = s->pending_target;
    }

    float lag   = s->theta * s->lag_frac;
    int   delay = (int)((s->theta - lag) / SIM_DT);
    s->relay_hist[s->hist_i % 256] = s->relay ? 1.0f : 0.0f;
    float u_del = s->hist_i >= delay ? s->relay_hist[(s->hist_i - delay) % 256] : 0.0f;
    s->hist_i++;
    s->u_lag += (u_del - s->u_lag) * SIM_DT / (lag + SIM_DT);
    float u_eff = s->u_lag;
    float d = s->T - s->t_room;
    s->T += (s->K * u_eff - (d + s->loss_quad * d * fabsf(d))) / s->tau * SIM_DT;
    if (s->T > s->max_T) s->max_T = s->T;
    return o;
}

/* Run a profile to the end; returns the final output. */
static algo_output_t run_profile(sim_t *s, algo_t *a, bool check_tracking)
{
    algo_output_t o = {0};
    float peak_target = 0;
    for (int i = 0; i < a->prof->num_waypoints; i++)
        if (a->prof->waypoints[i].temp > peak_target) peak_target = a->prof->waypoints[i].temp;

    for (int k = 0; k < SIM_MAX_STEPS; k++) {
        float t = k * SIM_DT;
        o = sim_tick(s, a, t);
        if (o.phase == ALGO_DONE || o.phase == ALGO_FAULT) break;
        /* Tracking once the first 40 s of catch-up are over, until cooling. */
        if (check_tracking && t > 40 && o.phase == ALGO_RUNNING) {
            float e = fabsf(o.sp - s->T);
            if (e > s->max_err_track) s->max_err_track = e;
        }
        if (o.phase == ALGO_COOLING) assert(o.cmd != ALGO_CMD_ON);
    }
    s->peak = s->max_T - peak_target;
    return o;
}

/* ── Closed loop ──────────────────────────────────────────────────── */

static void test_tracks_matched_plant(void)
{
    algo_params_t prm; algo_params_default(&prm);
    reflow_profile_t p = gentle();
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    assert(algo_start_profile(&a, &prm, &p));
    algo_output_t o = run_profile(&s, &a, true);

    printf("  matched: max|e| %.1f C, peak %+.1f C, %d switches, min gap %.1f s, stretch %.0f s\n",
           (double)s.max_err_track, (double)s.peak, s.switches, (double)s.min_gap, (double)a.stretch);
    assert(o.phase == ALGO_DONE);
    assert(s.max_err_track < 5.0f);
    assert(s.peak < 3.0f);
    assert(s.cmds_while_pending == 0);
    assert(s.min_gap >= prm.min_dwell);
}

static void test_tracks_mismatched_plant(void)
{
    /* The iron is 20 % weaker and its lag twice what the model says: the PI
       trim and the clock hold have to absorb it. */
    algo_params_t prm; algo_params_default(&prm);
    reflow_profile_t p = gentle();
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    s.K *= 0.8f; s.theta *= 2;
    assert(algo_start_profile(&a, &prm, &p));
    algo_output_t o = run_profile(&s, &a, true);

    printf("  mismatched: max|e| %.1f C, peak %+.1f C, %d switches, stretch %.0f s\n",
           (double)s.max_err_track, (double)s.peak, s.switches, (double)a.stretch);
    assert(o.phase == ALGO_DONE);
    assert(s.max_err_track < 8.0f);
    assert(s.peak < 4.0f);
    assert(s.cmds_while_pending == 0);
    assert(s.min_gap >= prm.min_dwell);
}

static void test_dwell_with_slow_plug(void)
{
    /* Latency longer than the dwell: the pending rule has to hold on its own. */
    algo_params_t prm; algo_params_default(&prm);
    reflow_profile_t p = gentle();
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    s.latency = 3.5f;
    assert(algo_start_profile(&a, &prm, &p));
    run_profile(&s, &a, false);
    assert(s.cmds_while_pending == 0);
    assert(s.min_gap >= prm.min_dwell);
}

static void test_weak_iron_clock(void)
{
    /* A third of the power: the SAC305 ramps and peak are out of reach. */
    reflow_profile_t p = sac305();
    algo_t a; sim_t s;

    /* Default: real time. The run follows the profile's timeline, never holds,
       and completes (cold, but on time). */
    algo_params_t prm; algo_params_default(&prm);
    sim_init(&s, &prm, 25);
    s.K *= 0.33f;
    assert(algo_start_profile(&a, &prm, &p));
    algo_output_t o = run_profile(&s, &a, false);
    assert(o.phase == ALGO_DONE && a.stretch == 0.0f);

    /* max_hold 30: each phase is held at most 30 s, and the cap is reported. */
    prm.max_hold = 30;
    sim_init(&s, &prm, 25);
    s.K *= 0.33f;
    assert(algo_start_profile(&a, &prm, &p));
    int capped = 0;
    for (int k = 0; k < SIM_MAX_STEPS; k++) {
        o = sim_tick(&s, &a, k * SIM_DT);
        if (o.hold_capped) capped++;
        assert(a.seg_held <= prm.max_hold + SIM_DT);
        if (o.phase == ALGO_DONE || o.phase == ALGO_FAULT) break;
    }
    printf("  weak iron, max_hold 30: held %.0f s over the run, %d phase cap(s)\n",
           (double)a.stretch, capped);
    assert(o.phase == ALGO_DONE && capped > 0 && a.stretch > 30);

    /* Unlimited hold: the clock stretches until the stall fault ends the run,
       and the heater is switched off. */
    prm.max_hold = 1e9f;
    sim_init(&s, &prm, 25);
    s.K *= 0.33f;
    assert(algo_start_profile(&a, &prm, &p));
    o = run_profile(&s, &a, false);
    assert(o.phase == ALGO_FAULT && o.fault == ALGO_FAULT_STALL);
    for (int k = 0; k < 10; k++) sim_tick(&s, &a, a.t_last + (k + 1) * SIM_DT);
    assert(!s.relay && !s.confirmed);
}

static void test_hot_start_runs_whole_profile(void)
{
    /* The iron starts at 100 C on a profile that starts at 25 C. The run must
       cover the whole profile from its time 0, keep the heater off until the
       curve catches up with the (cooling) iron, and then track it. */
    algo_params_t prm; algo_params_default(&prm);
    reflow_profile_t p = gentle();
    algo_t a; sim_t s;
    sim_init(&s, &prm, 100);
    assert(algo_start_profile(&a, &prm, &p));
    assert(a.pt == 0.0f);
    float first_on_gap = -1, max_e_after = 0;
    bool  caught_up = false;
    algo_output_t o;
    for (int k = 0; k < SIM_MAX_STEPS; k++) {
        o = sim_tick(&s, &a, k * SIM_DT);
        if (o.cmd == ALGO_CMD_ON && first_on_gap < 0) first_on_gap = s.T - o.sp;
        if (!caught_up && o.sp >= s.T) caught_up = true;
        if (caught_up && o.phase == ALGO_RUNNING && o.profile_t > 80) {
            float e = fabsf(o.sp - s.T);
            if (e > max_e_after) max_e_after = e;
        }
        if (o.phase == ALGO_DONE) break;
    }
    printf("  hot start 100 C: first ON with iron %+.1f C above target, max|e| %.1f C after catch-up\n",
           (double)first_on_gap, (double)max_e_after);
    assert(o.phase == ALGO_DONE);
    assert(near(o.profile_t, curve_duration(&p), 1.0f));   /* whole profile */
    assert(first_on_gap < 3.0f);                           /* no heat while above the curve */
    assert(max_e_after < 5.0f);
}

static void test_cooling_ends_at_curve_end(void)
{
    /* No cooling to room temperature: the run is complete when the curve
       ends (or the iron falls below it), however hot the iron still is. */
    algo_params_t prm; algo_params_default(&prm);
    reflow_profile_t p = gentle();
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    assert(algo_start_profile(&a, &prm, &p));
    float t_done = -1, integ_at_cool = 0;
    bool  seen_cool = false;
    for (int k = 0; k < SIM_MAX_STEPS; k++) {
        algo_output_t o = sim_tick(&s, &a, k * SIM_DT);
        if (o.phase == ALGO_COOLING) {
            if (!seen_cool) { seen_cool = true; integ_at_cool = a.integ; }
            assert(o.u == 0 && o.cmd != ALGO_CMD_ON);
            assert(a.integ == integ_at_cool);            /* no windup */
        }
        if (o.phase == ALGO_DONE) { t_done = o.profile_t; break; }
    }
    assert(seen_cool);
    assert(t_done > 0 && t_done <= curve_duration(&p) + 1);
    assert(s.T > 100);                                    /* still hot: fine */
}

static void test_ghost_on_is_caught(void)
{
    /* Seen on hardware: OFF confirmed, relay back on (a late retransmission
       of the previous ON), 14 s of full-power heating. The refresh and the
       heating-while-off check must end it within a few seconds. */
    algo_params_t prm; algo_params_default(&prm);
    algo_t a; sim_t s;
    sim_init(&s, &prm, 140);
    s.ghost_window = 3.0f;
    algo_start_hold(&a, &prm, 150);
    float max_T = 0;
    for (int k = 0; k < 1200; k++) {
        sim_tick(&s, &a, k * SIM_DT);
        if (k > 200 && s.T > max_T) max_T = s.T;
    }
    printf("  ghost ON: %d ghost(s), %d anomaly OFF(s), max %.1f C after 100 s\n",
           s.ghosts, s.anomalies, (double)max_T);
    assert(s.ghosts > 0);
    assert(max_T < 150 + 10);
}

/* Peak-to-peak error on the Sn63Pb37-shaped soak, from 5 s after the
   preheat corner until 5 s before the reflow ramp (where the lookahead
   feedforward rightly starts heating early). */
static float flat_soak_pp(float pulse_duty, float *mean)
{
    algo_params_t prm; algo_params_default(&prm);
    prm.pulse_duty = pulse_duty;
    reflow_profile_t p = flat_soak();
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    assert(algo_start_profile(&a, &prm, &p));
    float lo = 1e9f, hi = -1e9f, sum = 0; int n = 0;
    for (int k = 0; k < SIM_MAX_STEPS; k++) {
        algo_output_t o = sim_tick(&s, &a, k * SIM_DT);
        if (o.profile_t >= 65 && o.profile_t < 175) {
            float e = s.T - o.sp;
            if (e < lo) lo = e;
            if (e > hi) hi = e;
            sum += e; n++;
        }
        if (o.phase == ALGO_DONE) break;
        assert(s.cmds_while_pending == 0 && s.min_gap >= prm.min_dwell);
    }
    *mean = sum / n;
    return hi - lo;
}

static void test_flat_soak_ripple(void)
{
    /* Low power on a nearly flat stretch: heat comes as isolated minimum
       pulses. Timing them on the temperature must beat timing them on the
       accumulated duty, and stay near one pulse's bump (~4 C here). */
    float m_duty, m_pulse;
    float pp_duty  = flat_soak_pp(0.0f, &m_duty);
    float pp_pulse = flat_soak_pp(0.3f, &m_pulse);
    printf("  flat soak p-p: duty-timed %.2f C (mean %+.2f), temperature-timed %.2f C (mean %+.2f)\n",
           (double)pp_duty, (double)m_duty, (double)pp_pulse, (double)m_pulse);
    assert(pp_pulse < pp_duty - 0.3f);
    assert(pp_pulse < 4.5f);
    assert(fabsf(m_pulse) < 1.0f);
}

/* ── Faults ──────────────────────────────────────────────────────── */

static void test_sensor_fault(void)
{
    algo_params_t prm; algo_params_default(&prm);
    reflow_profile_t p = gentle();
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    assert(algo_start_profile(&a, &prm, &p));
    algo_output_t o;
    int k = 0;
    for (; k < 400 && !s.confirmed; k++) o = sim_tick(&s, &a, k * SIM_DT);   /* heating */
    assert(s.confirmed);
    s.sensor_ok = false;
    for (int j = 0; j < 4; j++, k++) { o = sim_tick(&s, &a, k * SIM_DT); assert(o.phase == ALGO_RUNNING); }
    for (int j = 0; j < 4; j++, k++) o = sim_tick(&s, &a, k * SIM_DT);
    assert(o.phase == ALGO_FAULT && o.fault == ALGO_FAULT_SENSOR);
    for (int j = 0; j < 10; j++, k++) o = sim_tick(&s, &a, k * SIM_DT);
    assert(!s.confirmed);                                   /* switched off */
}

static void test_overtemp_fault(void)
{
    /* A relay stuck on during a 150 C hold: fault at 150 + over_margin, and
       the controller keeps commanding off. The limit follows the run, not a
       fixed ceiling: a hotter profile gets a higher one. */
    algo_params_t prm; algo_params_default(&prm);
    algo_t a; sim_t s;
    sim_init(&s, &prm, 140);
    algo_start_hold(&a, &prm, 150);
    assert(near(a.t_limit, 150 + prm.over_margin, 1e-3));
    s.stuck_on = true;
    algo_output_t o;
    for (int k = 0; k < 2000; k++) {
        o = sim_tick(&s, &a, k * SIM_DT);
        if (o.phase == ALGO_FAULT) break;
    }
    assert(o.fault == ALGO_FAULT_OVERTEMP);
    assert(s.T > 150 + prm.over_margin && s.T < 150 + prm.over_margin + 3);
    /* Commanded off (the heating-while-off check already did, and the fault
       keeps it so): the plug reports off even though the relay is stuck. */
    for (int k = 0; k < 6; k++) sim_tick(&s, &a, a.t_last + SIM_DT);
    assert(!s.confirmed && s.anomalies > 0);

    /* A 300 C profile on a stronger heat source is not capped. */
    static const int   t[]  = {0, 100, 150, 200};
    static const float tp[] = {25, 250, 300, 100};
    reflow_profile_t hot = make_profile(t, tp, 4);
    assert(algo_start_profile(&a, &prm, &hot));
    assert(near(a.t_limit, 300 + prm.over_margin, 1e-3));
}

static void test_plug_fault(void)
{
    /* Unknown for longer than plug_timeout: a fault, and switched off. */
    algo_params_t prm; algo_params_default(&prm);
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    algo_start_hold(&a, &prm, 150);
    algo_output_t o;
    int k = 0;
    for (; k < 20; k++) o = sim_tick(&s, &a, k * SIM_DT);
    s.plug_ok = false;
    int n = (int)(prm.plug_timeout / SIM_DT);
    for (int j = 0; j < n - 2; j++, k++) {
        o = sim_tick(&s, &a, k * SIM_DT);
        assert(o.phase == ALGO_RUNNING);
    }
    for (int j = 0; j < 6; j++, k++) o = sim_tick(&s, &a, k * SIM_DT);
    assert(o.phase == ALGO_FAULT && o.fault == ALGO_FAULT_PLUG);
    for (int j = 0; j < 10; j++, k++) o = sim_tick(&s, &a, k * SIM_DT);
    assert(!s.relay);
}

static void test_flapping_plug_state(void)
{
    /* The plug's replies are lost for long stretches (measured: known ~12 s,
       then Unknown 8-40 s). The relay still obeys commands. Control must go
       on, the dwell must hold, and no fault may be raised. */
    algo_params_t prm; algo_params_default(&prm);
    reflow_profile_t p = gentle();
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    assert(algo_start_profile(&a, &prm, &p));
    algo_output_t o;
    float max_e = 0, last_relay_t = -1e9f, min_relay_gap = 1e9f;
    bool  last_relay = false;
    for (int k = 0; k < SIM_MAX_STEPS; k++) {
        float t = k * SIM_DT;
        s.plug_ok = fmodf(t, 40.0f) < 12.0f;
        o = sim_tick(&s, &a, t);
        if (s.relay != last_relay) {
            if (t - last_relay_t < min_relay_gap) min_relay_gap = t - last_relay_t;
            last_relay_t = t; last_relay = s.relay;
        }
        if (o.phase != ALGO_RUNNING) break;
        if (t > 40) { float e = fabsf(o.sp - s.T); if (e > max_e) max_e = e; }
    }
    printf("  flapping plug: max|e| %.1f C, min relay gap %.1f s, %d cmds\n",
           (double)max_e, (double)min_relay_gap, s.cmds);
    assert(o.phase == ALGO_COOLING || o.phase == ALGO_DONE);
    assert(max_e < 5.0f);
    assert(min_relay_gap >= prm.min_dwell);
    assert(s.cmds_while_pending == 0);
}

/* ── Open-loop modes ─────────────────────────────────────────────── */

static void test_step_stops_at_max(void)
{
    algo_params_t prm; algo_params_default(&prm);
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    algo_start_step(&a, &prm, 1.0f, 1000, 80);
    algo_output_t o;
    int k = 0;
    for (; k < 4000; k++) {
        o = sim_tick(&s, &a, k * SIM_DT);
        if (o.phase == ALGO_COOLING) break;
    }
    assert(o.phase == ALGO_COOLING && a.tf >= 80);
    for (int j = 0; j < 20; j++, k++) o = sim_tick(&s, &a, k * SIM_DT);
    assert(!s.confirmed && o.phase == ALGO_COOLING);
}

static void test_step_duty_average(void)
{
    /* 30 % duty with no plant limits: sigma-delta delivers the average. */
    algo_params_t prm; algo_params_default(&prm);
    algo_t a; sim_t s;
    sim_init(&s, &prm, 25);
    algo_start_step(&a, &prm, 0.3f, 600, 1000);
    int on_ticks = 0, n = 0;
    for (int k = 0; k < 1200; k++) {
        sim_tick(&s, &a, k * SIM_DT);
        if (k >= 100) { n++; if (s.confirmed) on_ticks++; }
    }
    float duty = (float)on_ticks / (float)n;
    printf("  step 30%%: delivered %.3f, %d switches in 600 s, min gap %.1f s\n",
           (double)duty, s.switches, (double)s.min_gap);
    assert(near(duty, 0.3f, 0.03f));
    assert(s.min_gap >= prm.min_dwell);
}

static void test_hold(void)
{
    algo_params_t prm; algo_params_default(&prm);
    algo_t a; sim_t s;
    sim_init(&s, &prm, 150);
    algo_start_hold(&a, &prm, 160);
    float max_e = 0;
    for (int k = 0; k < 2400; k++) {
        algo_output_t o = sim_tick(&s, &a, k * SIM_DT);
        if (k > 400) { float e = fabsf(o.sp - s.T); if (e > max_e) max_e = e; }
    }
    printf("  hold 160: max|e| %.2f C after 200 s\n", (double)max_e);
    /* The floor is the pulse: at 2.84 C/s a 2 s minimum on-time adds ~5.7 C,
       smoothed only by the 2 s element lag. */
    assert(max_e < 3.5f);
}

int main(void)
{
    test_curve();
    test_tracks_matched_plant();
    test_tracks_mismatched_plant();
    test_dwell_with_slow_plug();
    test_weak_iron_clock();
    test_hot_start_runs_whole_profile();
    test_cooling_ends_at_curve_end();
    test_flat_soak_ripple();
    test_ghost_on_is_caught();
    test_sensor_fault();
    test_overtemp_fault();
    test_plug_fault();
    test_flapping_plug_state();
    test_step_stops_at_max();
    test_step_duty_average();
    test_hold();
    printf("reflow_algo: all tests passed\n");
    return 0;
}
