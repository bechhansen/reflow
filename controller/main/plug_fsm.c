#include "plug_fsm.h"
#include <string.h>

void plug_fsm_init(plug_fsm_t *f)
{
    memset(f, 0, sizeof(*f));
    f->state = PLUG_UNKNOWN;
    f->phase = PLUG_PHASE_IDLE;
}

static void arm(plug_actions_t *a, uint32_t ms)
{
    a->arm    = true;
    a->arm_ms = ms;
}

static void done(plug_fsm_t *f, plug_actions_t *a, bool ok, const char *reason)
{
    f->pending = false;
    a->done    = true;
    a->done_ok = ok;
    a->reason  = reason;
}

static void end_sequence(plug_fsm_t *f)
{
    f->seq_span = 0;
}

static void go_idle(plug_fsm_t *f, plug_actions_t *a)
{
    f->phase = PLUG_PHASE_IDLE;
    end_sequence(f);
    arm(a, f->state == PLUG_UNKNOWN ? PLUG_UNKNOWN_POLL_MS : PLUG_POLL_MS);
}

static void send_cmd(plug_fsm_t *f, plug_actions_t *a)
{
    a->send_cmd       = true;
    a->cmd_on         = f->target;
    f->phase          = PLUG_PHASE_SETTLE;
    end_sequence(f);   /* reads sent before this command are now stale */
    arm(a, PLUG_SETTLE_MS);
}

/* One read attempt. reads_left is set at the start of a read sequence, and
   decremented per failed attempt. The shell reports the TSN afterwards. */
static void send_read(plug_fsm_t *f, plug_actions_t *a)
{
    a->send_read = true;
    f->phase     = PLUG_PHASE_READING;
    arm(a, PLUG_READ_TIMEOUT_MS);
}

static void begin_read(plug_fsm_t *f, plug_actions_t *a)
{
    f->reads_left  = PLUG_READ_TRIES;
    f->seq_replied = false;
    end_sequence(f);
    send_read(f, a);
}

static void read_failed(plug_fsm_t *f, plug_actions_t *a)
{
    if (--f->reads_left > 0) {
        send_read(f, a);
        return;
    }
    if (f->pending && f->seq_replied) {
        /* The plug answered, but never with the target: the command was lost
           or refused. Resend once, then give up with the real state kept. */
        if (f->resends_left > 0) {
            f->resends_left--;
            send_cmd(f, a);
        } else {
            done(f, a, false, "mismatch");
            go_idle(f, a);
        }
        return;
    }
    /* The plug is not answering: whatever state we held is no longer known. */
    f->state = PLUG_UNKNOWN;
    if (f->pending) done(f, a, false, "timeout");
    go_idle(f, a);
}

void plug_fsm_start(plug_fsm_t *f, plug_actions_t *a)
{
    f->active = true;
    if (f->pending) return;   /* the pending request's own read covers it */
    begin_read(f, a);
}

void plug_fsm_reset(plug_fsm_t *f, plug_actions_t *a)
{
    if (f->pending) done(f, a, false, "unpaired");
    f->state          = PLUG_UNKNOWN;
    f->active         = false;
    f->phase          = PLUG_PHASE_IDLE;
    end_sequence(f);
    a->disarm         = true;
}

plug_req_result_t plug_fsm_request(plug_fsm_t *f, bool on, plug_actions_t *a)
{
    if (f->pending) return PLUG_REQ_BUSY;
    f->pending      = true;
    f->target       = on;
    f->active       = true;
    f->resends_left = PLUG_RESENDS;
    send_cmd(f, a);
    return PLUG_REQ_OK;
}

void plug_fsm_timer(plug_fsm_t *f, plug_actions_t *a)
{
    if (!f->active) return;
    switch (f->phase) {
    case PLUG_PHASE_IDLE:
    case PLUG_PHASE_SETTLE:
        begin_read(f, a);
        break;
    case PLUG_PHASE_READING:
        read_failed(f, a);
        break;
    }
}

void plug_fsm_set_read_tsn(plug_fsm_t *f, uint8_t tsn)
{
    if (f->seq_span == 0) {
        f->seq_first_tsn = tsn;
        f->seq_span      = 1;
    } else {
        f->seq_span = (uint8_t)(tsn - f->seq_first_tsn + 1);
    }
}

static bool in_sequence(const plug_fsm_t *f, uint8_t tsn)
{
    return f->seq_span > 0 && (uint8_t)(tsn - f->seq_first_tsn) < f->seq_span;
}

void plug_fsm_read_result(plug_fsm_t *f, uint8_t tsn, bool ok, bool value,
                          plug_actions_t *a)
{
    if (f->phase != PLUG_PHASE_READING || !in_sequence(f, tsn))
        return;   /* stale: not a read of the current sequence */

    if (!ok) {
        read_failed(f, a);
        return;
    }

    f->state = value ? PLUG_ON : PLUG_OFF;
    if (!f->pending) {
        go_idle(f, a);
    } else if (value == f->target) {
        done(f, a, true, NULL);
        go_idle(f, a);
    } else {
        /* Old state. With reads this frequent the reply may predate the relay
           switching, so keep reading; read_failed() decides at the end. */
        f->seq_replied = true;
    }
}

void plug_fsm_report(plug_fsm_t *f, bool value)
{
    if (!f->active || f->pending) return;
    f->state = value ? PLUG_ON : PLUG_OFF;
}

const char *plug_state_str(plug_state_t s)
{
    switch (s) {
    case PLUG_OFF: return "off";
    case PLUG_ON:  return "on";
    default:       return "unknown";
    }
}
