/* Host unit tests for plug_fsm.c. Build and run: make -C controller/test/plug_fsm test */
#include "plug_fsm.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static plug_actions_t a;
static uint8_t next_tsn = 1;

/* Mirror what the shell does after send_read: hand back a fresh TSN. */
static uint8_t step_tsn(plug_fsm_t *f)
{
    assert(a.send_read);
    uint8_t tsn = next_tsn++;
    plug_fsm_set_read_tsn(f, tsn);
    return tsn;
}

static void clear(void) { memset(&a, 0, sizeof(a)); }

static bool no_actions(void)
{
    plug_actions_t z = {0};
    return memcmp(&a, &z, sizeof(a)) == 0;
}

/* Drive a fresh FSM to a confirmed idle state. */
static void make_idle(plug_fsm_t *f, bool on)
{
    plug_fsm_init(f);
    clear(); plug_fsm_start(f, &a);
    uint8_t tsn = step_tsn(f);
    clear(); plug_fsm_read_result(f, tsn, true, on, &a);
    assert(f->state == (on ? PLUG_ON : PLUG_OFF));
    assert(a.arm && a.arm_ms == PLUG_POLL_MS);
}

static void test_init_unknown(void)
{
    plug_fsm_t f;
    plug_fsm_init(&f);
    assert(f.state == PLUG_UNKNOWN);
    assert(!f.pending && !f.active);
    clear(); plug_fsm_timer(&f, &a);
    assert(no_actions());
}

static void test_start_reads_state(void)
{
    plug_fsm_t f;
    plug_fsm_init(&f);
    clear(); plug_fsm_start(&f, &a);
    assert(a.send_read && a.arm && a.arm_ms == PLUG_READ_TIMEOUT_MS);
    assert(!a.send_cmd && !a.done);
    uint8_t tsn = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, tsn, true, true, &a);
    assert(f.state == PLUG_ON);
    assert(!a.done);
    assert(a.arm && a.arm_ms == PLUG_POLL_MS);
}

static void test_request_confirmed(void)
{
    plug_fsm_t f;
    make_idle(&f, false);

    clear();
    assert(plug_fsm_request(&f, true, &a) == PLUG_REQ_OK);
    assert(a.send_cmd && a.cmd_on);
    assert(a.arm && a.arm_ms == PLUG_SETTLE_MS);
    assert(!a.send_read && !a.done);
    assert(f.pending && f.target);
    assert(f.state == PLUG_OFF);    /* not confirmed yet */

    clear(); plug_fsm_timer(&f, &a);   /* settle elapsed */
    assert(a.send_read && a.arm && a.arm_ms == PLUG_READ_TIMEOUT_MS);
    uint8_t tsn = step_tsn(&f);

    clear(); plug_fsm_read_result(&f, tsn, true, true, &a);
    assert(a.done && a.done_ok);
    assert(f.state == PLUG_ON && !f.pending);
    assert(a.arm && a.arm_ms == PLUG_POLL_MS);
}

static void test_request_sent_even_if_state_matches(void)
{
    plug_fsm_t f;
    make_idle(&f, true);
    clear();
    assert(plug_fsm_request(&f, true, &a) == PLUG_REQ_OK);
    assert(a.send_cmd && a.cmd_on);
}

static void test_busy_rejected(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_request(&f, true, &a);

    clear();
    assert(plug_fsm_request(&f, false, &a) == PLUG_REQ_BUSY);
    assert(no_actions());
    assert(f.target == true);

    /* Still busy while the read is outstanding. */
    clear(); plug_fsm_timer(&f, &a);
    step_tsn(&f);
    clear();
    assert(plug_fsm_request(&f, false, &a) == PLUG_REQ_BUSY);
    assert(no_actions());
}

/* Send the remaining reads of the current sequence without any reply, so the
   next timer tick ends the sequence. */
static void run_out_reads(plug_fsm_t *f)
{
    for (int i = 1; i < PLUG_READ_TRIES; i++) {
        clear(); plug_fsm_timer(f, &a);
        assert(a.send_read && !a.done && !a.send_cmd);
        step_tsn(f);
    }
}

/* A reply with the old state is not yet a mismatch: with fast reads, the first
   reply can come before the relay has switched. The sequence keeps reading;
   only when it runs out with the plug still in the old state is the command
   resent, once. */
static void test_mismatch_resends_once_then_fails(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_request(&f, true, &a);
    clear(); plug_fsm_timer(&f, &a);
    uint8_t tsn = step_tsn(&f);

    clear(); plug_fsm_read_result(&f, tsn, true, false, &a);   /* still off */
    assert(no_actions());
    assert(f.pending && f.state == PLUG_OFF);

    run_out_reads(&f);
    clear(); plug_fsm_timer(&f, &a);           /* sequence ends: resend */
    assert(a.send_cmd && a.cmd_on);
    assert(a.arm && a.arm_ms == PLUG_SETTLE_MS);
    assert(!a.done && f.pending);

    clear(); plug_fsm_timer(&f, &a);           /* settle -> new sequence */
    tsn = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, tsn, true, false, &a);   /* off again */
    assert(no_actions());
    run_out_reads(&f);
    clear(); plug_fsm_timer(&f, &a);
    assert(!a.send_cmd);
    assert(a.done && !a.done_ok && strcmp(a.reason, "mismatch") == 0);
    assert(!f.pending);
    assert(f.state == PLUG_OFF);    /* the real state is kept */
    assert(a.arm && a.arm_ms == PLUG_POLL_MS);
}

static void test_early_old_reply_then_confirm(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_request(&f, true, &a);
    clear(); plug_fsm_timer(&f, &a);
    uint8_t tsn = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, tsn, true, false, &a);   /* relay not yet switched */
    clear(); plug_fsm_timer(&f, &a);           /* next read, no resend */
    assert(a.send_read && !a.send_cmd);
    tsn = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, tsn, true, true, &a);
    assert(a.done && a.done_ok && f.state == PLUG_ON);
}

static void test_read_timeouts_to_unknown_then_recover(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_request(&f, true, &a);
    clear(); plug_fsm_timer(&f, &a);           /* settle -> read 1 */
    step_tsn(&f);

    for (int i = 1; i < PLUG_READ_TRIES; i++) {
        clear(); plug_fsm_timer(&f, &a);       /* timeout -> retry */
        assert(a.send_read && a.arm && a.arm_ms == PLUG_READ_TIMEOUT_MS);
        assert(!a.done && f.pending);
        step_tsn(&f);
    }
    clear(); plug_fsm_timer(&f, &a);           /* last timeout */
    assert(!a.send_read);
    assert(a.done && !a.done_ok && strcmp(a.reason, "timeout") == 0);
    assert(f.state == PLUG_UNKNOWN && !f.pending);
    assert(a.arm && a.arm_ms == PLUG_UNKNOWN_POLL_MS);

    /* The next poll recovers the state on its own. */
    clear(); plug_fsm_timer(&f, &a);
    assert(a.send_read);
    uint8_t tsn = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, tsn, true, true, &a);
    assert(f.state == PLUG_ON && !a.done);
}

static void test_read_error_counts_as_failed_try(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_timer(&f, &a);           /* poll */
    uint8_t tsn = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, tsn, false, false, &a);
    assert(a.send_read && a.arm && a.arm_ms == PLUG_READ_TIMEOUT_MS);
    assert(f.state == PLUG_OFF);
}

static void test_idle_poll_failure_goes_unknown_without_done(void)
{
    plug_fsm_t f;
    make_idle(&f, true);
    clear(); plug_fsm_timer(&f, &a);           /* poll -> read 1 */
    step_tsn(&f);
    for (int i = 1; i < PLUG_READ_TRIES; i++) {
        clear(); plug_fsm_timer(&f, &a);
        step_tsn(&f);
    }
    clear(); plug_fsm_timer(&f, &a);
    assert(f.state == PLUG_UNKNOWN);
    assert(!a.done);
    assert(a.arm && a.arm_ms == PLUG_UNKNOWN_POLL_MS);
}

static void test_stale_tsn_ignored(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_request(&f, true, &a);
    clear(); plug_fsm_timer(&f, &a);
    uint8_t tsn = step_tsn(&f);

    clear(); plug_fsm_read_result(&f, (uint8_t)(tsn + 100), true, true, &a);
    assert(no_actions());
    assert(f.pending && f.state == PLUG_OFF);

}

/* Under Wi-Fi coexistence a reply can arrive after its read timed out and a
   retry went out. Every read in the current sequence was sent after the
   command, so a late reply to any of them is still a valid confirmation. */
static void test_late_reply_in_sequence_accepted(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_request(&f, true, &a);
    clear(); plug_fsm_timer(&f, &a);
    uint8_t first = step_tsn(&f);
    clear(); plug_fsm_timer(&f, &a);           /* timeout -> retry, new TSN */
    step_tsn(&f);

    clear(); plug_fsm_read_result(&f, first, true, true, &a);
    assert(a.done && a.done_ok);
    assert(f.state == PLUG_ON && !f.pending);
}

/* ...but a read from BEFORE a resend describes the state before the resend. */
static void test_reply_from_before_resend_ignored(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_request(&f, true, &a);
    clear(); plug_fsm_timer(&f, &a);
    uint8_t old = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, old, true, false, &a);   /* old state */
    run_out_reads(&f);
    clear(); plug_fsm_timer(&f, &a);           /* resend */
    assert(a.send_cmd);
    clear(); plug_fsm_timer(&f, &a);           /* settle -> new sequence */
    step_tsn(&f);

    clear(); plug_fsm_read_result(&f, old, true, true, &a);
    assert(no_actions());
    assert(f.pending);
}

/* A reply that arrives after its sequence ended is dropped. */
static void test_reply_after_sequence_ignored(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_timer(&f, &a);           /* poll */
    uint8_t tsn = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, tsn, true, false, &a);   /* idle again */
    clear(); plug_fsm_read_result(&f, tsn, true, true, &a);    /* duplicate */
    assert(no_actions());
    assert(f.state == PLUG_OFF);
}

static void test_result_during_settle_ignored(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_timer(&f, &a);           /* idle poll read outstanding */
    uint8_t tsn = step_tsn(&f);
    clear(); plug_fsm_request(&f, true, &a);   /* request takes over */
    assert(a.send_cmd && a.arm_ms == PLUG_SETTLE_MS);
    clear(); plug_fsm_read_result(&f, tsn, true, false, &a);
    assert(no_actions());
    assert(f.pending && f.phase == PLUG_PHASE_SETTLE);
}

static void test_report(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    plug_fsm_report(&f, true);
    assert(f.state == PLUG_ON);

    clear(); plug_fsm_request(&f, false, &a);
    plug_fsm_report(&f, true);
    assert(f.state == PLUG_ON && f.pending);
    plug_fsm_report(&f, false);
    assert(f.state == PLUG_ON);                /* ignored while pending */
}

static void test_report_ignored_when_inactive(void)
{
    plug_fsm_t f;
    plug_fsm_init(&f);
    plug_fsm_report(&f, true);
    assert(f.state == PLUG_UNKNOWN);
}

static void test_reset(void)
{
    plug_fsm_t f;
    make_idle(&f, true);
    clear(); plug_fsm_reset(&f, &a);
    assert(f.state == PLUG_UNKNOWN && !f.active && !f.pending);
    assert(a.disarm && !a.arm && !a.done);
    clear(); plug_fsm_timer(&f, &a);
    assert(no_actions());
}

static void test_reset_while_pending_fails_request(void)
{
    plug_fsm_t f;
    make_idle(&f, false);
    clear(); plug_fsm_request(&f, true, &a);
    clear(); plug_fsm_reset(&f, &a);
    assert(a.done && !a.done_ok && strcmp(a.reason, "unpaired") == 0);
    assert(!f.pending && f.state == PLUG_UNKNOWN);
}

static void test_request_from_unknown(void)
{
    /* Boot: nothing known, OFF requested before any read. */
    plug_fsm_t f;
    plug_fsm_init(&f);
    clear();
    assert(plug_fsm_request(&f, false, &a) == PLUG_REQ_OK);
    assert(a.send_cmd && !a.cmd_on && f.active);
    clear(); plug_fsm_timer(&f, &a);
    uint8_t tsn = step_tsn(&f);
    clear(); plug_fsm_read_result(&f, tsn, true, false, &a);
    assert(a.done && a.done_ok && f.state == PLUG_OFF);
}

int main(void)
{
    test_init_unknown();
    test_start_reads_state();
    test_request_confirmed();
    test_request_sent_even_if_state_matches();
    test_busy_rejected();
    test_mismatch_resends_once_then_fails();
    test_early_old_reply_then_confirm();
    test_read_timeouts_to_unknown_then_recover();
    test_read_error_counts_as_failed_try();
    test_idle_poll_failure_goes_unknown_without_done();
    test_stale_tsn_ignored();
    test_late_reply_in_sequence_accepted();
    test_reply_from_before_resend_ignored();
    test_reply_after_sequence_ignored();
    test_result_during_settle_ignored();
    test_report();
    test_report_ignored_when_inactive();
    test_reset();
    test_reset_while_pending_fails_request();
    test_request_from_unknown();
    printf("plug_fsm: all tests passed\n");
    return 0;
}
