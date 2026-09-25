#pragma once
/* Plug state machine: decides what to send to the Zigbee plug and when a state
   change counts as done. Deliberately free of ESP-IDF headers so it can be
   unit-tested on the host (controller/test/plug_fsm). plug_ctrl.c is the shell
   that feeds it events and carries out the actions it returns.

   A commanded change is only DONE once a Read Attributes response for OnOff
   reports the target value. esp_zb_zcl_on_off_cmd_req() returns a sequence
   number, not a delivery result, so the command itself proves nothing.

   One timer serves three purposes, depending on the phase:
     SETTLE   a command was sent; when it fires, read the state back
     READING  a read is outstanding; when it fires, send the next read
     IDLE     nothing outstanding; when it fires, poll the state

   Why reads are sent so often: the ESP32-C6 shares one radio between Wi-Fi
   and 802.15.4, and Wi-Fi owns it most of the time. Measured on this board,
   the plug's reply is reliably received only in the ~15 ms after WE transmit,
   when the radio listens for the MAC ACK; otherwise it waits for the plug's
   retries, 1-5 s. So a short read interval is what opens the receive windows,
   and a reply to any read of the sequence counts. */
#include <stdbool.h>
#include <stdint.h>

/* Tuned on hardware: a 100 ms read interval was worse than 200 ms (more
   traffic, more loss). The settle is short because an early old-state reply
   is harmless: a mismatch is only decided when the sequence runs out. */
#define PLUG_SETTLE_MS        200
#define PLUG_READ_TIMEOUT_MS  200     /* interval between reads of a sequence */
#define PLUG_READ_TRIES       10      /* ~2 s before a sequence gives up */
#define PLUG_RESENDS          1
#define PLUG_POLL_MS          10000
#define PLUG_UNKNOWN_POLL_MS  1000    /* while Unknown, recover quickly */

typedef enum {
    PLUG_UNKNOWN,
    PLUG_OFF,
    PLUG_ON,
} plug_state_t;

typedef enum {
    PLUG_PHASE_IDLE,
    PLUG_PHASE_SETTLE,
    PLUG_PHASE_READING,
} plug_phase_t;

typedef enum {
    PLUG_REQ_OK,
    PLUG_REQ_BUSY,      /* a change is already pending; nothing was sent */
} plug_req_result_t;

typedef struct {
    plug_state_t state;         /* last state a read or report confirmed */
    bool         pending;       /* a requested change awaits confirmation */
    bool         target;        /* requested value while pending */
    bool         active;        /* paired and polling */
    plug_phase_t phase;
    int          reads_left;
    int          resends_left;
    /* TSNs of the reads in the current read sequence: [seq_first_tsn,
       seq_first_tsn + seq_span), mod 256. seq_span 0 = no sequence. */
    uint8_t      seq_first_tsn;
    uint8_t      seq_span;
    bool         seq_replied;   /* the plug answered during this sequence */
} plug_fsm_t;

/* What the shell must do after an event. Timer actions replace any armed
   timer: arm = cancel then arm for arm_ms, disarm = cancel only. */
typedef struct {
    bool        send_cmd;
    bool        cmd_on;
    bool        send_read;      /* shell then reports the TSN via plug_fsm_set_read_tsn() */
    bool        arm;
    uint32_t    arm_ms;
    bool        disarm;
    bool        done;           /* a pending request finished */
    bool        done_ok;
    const char *reason;         /* why it failed: "timeout", "mismatch", "unpaired" */
} plug_actions_t;

void              plug_fsm_init(plug_fsm_t *f);

/* Begin polling (network up, or a plug was just paired). */
void              plug_fsm_start(plug_fsm_t *f, plug_actions_t *a);

/* Stop everything and forget the state (plug unpaired). */
void              plug_fsm_reset(plug_fsm_t *f, plug_actions_t *a);

/* Request a change. The command is always sent, even when the stored state
   already matches: a stored state may be stale. */
plug_req_result_t plug_fsm_request(plug_fsm_t *f, bool on, plug_actions_t *a);

void              plug_fsm_timer(plug_fsm_t *f, plug_actions_t *a);
void              plug_fsm_set_read_tsn(plug_fsm_t *f, uint8_t tsn);

/* Read Attributes response. ok=false for a non-success status; value is
   ignored then. A reply to ANY read of the current sequence is accepted: under
   Wi-Fi coexistence a reply can arrive after its read timed out and a retry
   went out, and every read in the sequence was sent after the last command.
   Replies from earlier sequences (before a resend, or after it ended) are
   dropped. */
void              plug_fsm_read_result(plug_fsm_t *f, uint8_t tsn, bool ok, bool value,
                                       plug_actions_t *a);

/* Unsolicited attribute report. Applied only while no change is pending:
   a pending change is confirmed by its own read, nothing else. */
void              plug_fsm_report(plug_fsm_t *f, bool value);

const char       *plug_state_str(plug_state_t s);
