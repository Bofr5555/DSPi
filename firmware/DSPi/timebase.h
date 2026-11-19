/*
 * timebase.h - the alink time-base contract's control plane (WP FWTB1,
 * PLAN-AT section 1.1; alink_protocol_spec.md section 11,
 * extension_channel_spec.md 2.11).
 *
 * One read for every time base a unit has (EXT_TIME_BASE 0x8030), one event
 * for every contract transition (NOTIFY_EVT_TIME_BASE 0xE4), on every
 * transport.  A transport (the line in alink_input.c, USB-direct in
 * alink_usb/alink_usb_core.c, S/PDIF later in alink_spdif.c, WP FWSP2) keeps a
 * tb_track_t: it reports its contract state and the reasons of its
 * disturbances into it, and registers a provider (timebase.c) that reads its
 * entry with the output counters taken at the same instant.
 *
 * This header is portable C11 (the tracker and the state maps are inline and
 * host-tested in tests/test_timebase.c); timebase.c is the aggregator.
 */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Wire constants (extension_channel_spec.md 2.11, alink_protocol_spec.md 11)
// ---------------------------------------------------------------------------

// Time base ids (11.3).  Never reused.
#define TB_ID_LINE          0u      // the line's frame counter F
#define TB_ID_USB           1u      // USB-direct sample index S
#define TB_ID_USB_BUS       2u      // USB bus-frame form: EXT_APPLY_AT_FRAME writes only, never op 9
#define TB_ID_SPDIF         3u      // S/PDIF F (WP FWSP2)
#define TB_ID_NONE          0xFFu

// EXT_TIME_BASE entry role.
#define TB_ROLE_FOLLOWER    0u
#define TB_ROLE_SOURCE      1u

// Contract states (11.2).
enum {
    TB_ST_NONE = 0,         // no usable time base
    TB_ST_ACQUIRING = 1,    // searching, pulling in, waiting for a superframe
    TB_ST_ANCHORING = 2,    // drain, prefill, start, verify (muted)
    TB_ST_LOCKED = 3,       // T valid, play-out fixed, apply-at-T works
    TB_ST_HOLDOVER = 4,     // T extrapolated, muted, the timeline kept
    TB_ST_LOCK_ERROR = 5,   // configured to lock but cannot
};

// Event 0xE4 reasons (2.11.3).
enum {
    TB_RS_NONE = 0,
    TB_RS_LOCKED = 1,
    TB_RS_LOST = 2,
    TB_RS_HOLDOVER = 3,
    TB_RS_RESUMED = 4,
    TB_RS_REANCHOR = 5,
    TB_RS_CORRECTION = 6,
    TB_RS_DISABLED = 7,
    TB_RS_CONFIG = 8,
    TB_RS_LOCK_ERROR = 9,
    TB_RS_LOCK_OK = 10,
    TB_RS_EPOCH = 11,
};

#define TB_VERSION              1u
#define TB_READ_LEN             64u     // EXT_TIME_BASE
#define TB_ENTRY_OFF            12u
#define TB_ENTRY_LEN            24u
#define TB_MAX_ENTRIES          2u
#define TB_TIMELINE_LEN         40u     // EXT_SIGGEN_TIMELINE
#define TB_EVENT_MIN_MS         250u    // 0xE4: at most one per 250 ms per time base

// EXT_SIGGEN_TIMELINE.state and .last_result (2.11.2).
enum { TB_TL_IDLE = 0, TB_TL_PENDING = 1, TB_TL_ARMED = 2, TB_TL_RUNNING = 3 };
enum {
    TB_RES_NONE = 0,
    TB_RES_STARTED = 1,
    TB_RES_REFUSED_LATE = 2,
    TB_RES_REFUSED_NOT_READY = 3,
    TB_RES_CANCELLED_LOSS = 4,
    TB_RES_CANCELLED_CONFIG = 5,
    TB_RES_CANCELLED_HOST = 6,
    TB_RES_STOPPED_LOSS = 7,
};

// Refusal codes of the extension handlers.  On the 1.1.6 envelope every
// nonzero code is a STALL / ERROR; the tunnel 0xE010 (WP FWPG2) answers them
// as these VC_ERR_* codes (extension_channel_spec.md 1.3.4).
#define TB_OK                   0u
#define TB_ERR_DIRECTION        0x11u   // VC_ERR_DIRECTION
#define TB_ERR_LENGTH           0x12u   // VC_ERR_LENGTH
#define TB_ERR_VALUE            0x14u   // VC_ERR_VALUE
#define TB_ERR_STATE            0x18u   // VC_ERR_STATE

// Op 9's lead (2.8.7.1): at least arm_lead = max(hold_ms, release_ms) + 50 ms
// ahead of the sample entering the pipeline now, and at most this much
// (review fwtb0: an upper bound; 120 s is the longest run a measurement
// schedules, PLAN-P5 S7, so a valid timetable never meets it).
#define TB_ARM_MARGIN_MS        50u
#define TB_OP9_LEAD_MAX_MS      120000u

// ---------------------------------------------------------------------------
// The tracker (C4): contract state, last reason, disturbance counter, event
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  state;             // contract state now
    uint8_t  last_reason;       // reason of the last transition (EXT_TIME_BASE +3)
    uint16_t dist;              // disturbances (wraps)
    bool     from_hold;         // HOLDOVER seen since the last LOCKED: the next LOCKED is a resume
    // Event 0xE4, held for the rate limit (latest state and reason win).
    bool     ev_pending, ev_once;
    uint8_t  ev_reason;
    uint32_t ev_t, ev_last_ms;
    // For the aggregator (timebase.c): loss and disturbances not yet handed
    // to the signal generator.
    bool     lost;
    uint16_t dist_seen;
} tb_track_t;

static inline void tb_track_init(tb_track_t *k) {
    k->state = TB_ST_NONE;
    k->last_reason = TB_RS_NONE;
    k->dist = 0;
    k->from_hold = false;
    k->ev_pending = k->ev_once = false;
    k->ev_reason = TB_RS_NONE;
    k->ev_t = k->ev_last_ms = 0;
    k->lost = false;
    k->dist_seen = 0;
}

// Reasons that disturb the timeline by themselves (C4).
static inline bool tb_reason_disturbs(uint8_t r) {
    return r == TB_RS_HOLDOVER || r == TB_RS_RESUMED || r == TB_RS_REANCHOR || r == TB_RS_CORRECTION ||
           r == TB_RS_EPOCH;
}

// One observation of the entry: its contract state now and the reason of what
// just happened (TB_RS_NONE for a plain poll), T low 32 bits at that moment.
// A transition without a reason gets the obvious one (LOCKED, LOST, HOLDOVER,
// RESUMED, LOCK_ERROR).  Every transition out of LOCKED and every disturbing
// reason counts once in `dist`, however many of them one occurrence is (a
// holdover that leaves LOCKED counts once).  Returns true when it counted.
static inline bool tb_track_note(tb_track_t *k, uint8_t state, uint8_t reason, uint32_t t_low) {
    uint8_t was = k->state;
    if (reason == TB_RS_NONE && state != was) {
        if (state == TB_ST_LOCKED) reason = k->from_hold ? TB_RS_RESUMED : TB_RS_LOCKED;
        else if (state == TB_ST_HOLDOVER) reason = TB_RS_HOLDOVER;
        else if (state == TB_ST_LOCK_ERROR) reason = TB_RS_LOCK_ERROR;
        else if (was == TB_ST_LOCKED || was == TB_ST_HOLDOVER) reason = TB_RS_LOST;
    }
    bool leaves = was == TB_ST_LOCKED && state != TB_ST_LOCKED;
    bool counted = leaves || tb_reason_disturbs(reason);
    if (counted) k->dist++;
    if (leaves) k->lost = true;
    if (state == TB_ST_HOLDOVER) k->from_hold = true;
    else if (state == TB_ST_LOCKED || state == TB_ST_NONE || state == TB_ST_ACQUIRING) k->from_hold = false;
    if (state != was || reason != TB_RS_NONE) {
        k->ev_pending = true;
        if (reason != TB_RS_NONE) {
            k->ev_reason = reason;
            k->last_reason = reason;
        }
        k->ev_t = t_low;
    }
    k->state = state;
    return counted;
}

// The 0xE4 to send now (state = k->state, reason = *reason, T = *t): false
// while none is pending or the 250 ms since the last one have not passed.
static inline bool tb_track_event_due(tb_track_t *k, uint32_t now_ms, uint8_t *reason, uint32_t *t) {
    if (!k->ev_pending) return false;
    if (k->ev_once && (uint32_t)(now_ms - k->ev_last_ms) < TB_EVENT_MIN_MS) return false;
    *reason = k->ev_reason;
    *t = k->ev_t;
    k->ev_pending = false;
    k->ev_reason = TB_RS_NONE;
    k->ev_once = true;
    k->ev_last_ms = now_ms;
    return true;
}

// ---------------------------------------------------------------------------
// The transports' states against the contract (11.2)
// ---------------------------------------------------------------------------

// The line: EXT_ALINK_STATUS.state (0 off, 1 searching, 2 locked, 3
// holdover), EXT_ALINK_PHASE.mode 1, the phase lock able to run (the delay
// within d_local_min..max), EXT_ALINK_PHASE.state (0 free, 1 anchoring, 2
// locked, 3 pending) and whether the outputs play the timeline.  Mode 0 (the
// fill servo) has no time base; mode 1 with the delay out of range is a lock
// error (configured to lock but cannot).
static inline uint8_t tb_line_state(uint8_t alink_state, bool mode1, bool active, uint8_t phase_state,
                                    bool started) {
    if (alink_state == 0 || !mode1) return TB_ST_NONE;
    if (alink_state == 3) return TB_ST_HOLDOVER;
    if (alink_state == 1) return TB_ST_ACQUIRING;
    if (!active) return TB_ST_LOCK_ERROR;
    if (phase_state == 3) return TB_ST_LOCK_ERROR;
    if (phase_state == 2 && started) return TB_ST_LOCKED;
    return TB_ST_ANCHORING;
}

// USB-direct: EXT_ALINK_USB_STATUS.state (0-6), phase_state, the outputs on
// the timeline, a holdover (no SOF, extrapolating) and lock error (2.9.2).
static inline uint8_t tb_usb_state(uint8_t usb_state, uint8_t phase_state, bool started, bool holdover,
                                   bool lock_error) {
    if (usb_state == 0) return TB_ST_NONE;
    if (lock_error || usb_state == 6) return TB_ST_LOCK_ERROR;
    if (usb_state == 1 || usb_state == 3) return TB_ST_NONE;
    if (usb_state == 2) return TB_ST_ACQUIRING;
    if (usb_state == 5 && holdover) return TB_ST_HOLDOVER;
    if (usb_state == 5 && phase_state == 2 && started) return TB_ST_LOCKED;
    return TB_ST_ANCHORING;
}

// T of an entry at the output instant (R7c): the frame playing now is
// next_w - (wr - played) (wr frames written to slot 0, played consumed), and
// a follower receives the sample D later.  All whole samples, so while the
// timeline is undisturbed t - played is the constant next_w - wr + d.
static inline uint64_t tb_t_at_output(uint64_t next_w, uint32_t wr, uint32_t played, int32_t d) {
    return next_w - (uint64_t)(uint32_t)(wr - played) + (uint64_t)(int64_t)d;
}

// ---------------------------------------------------------------------------
// Aggregator (timebase.c)
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  tb, role;
    uint32_t domain;            // C5: byte 3 = tb
    uint64_t t;
    uint16_t d;
    float    phase_error;
} tb_entry_t;

typedef struct {
    uint8_t     tb, role;
    tb_track_t *track;
    // The entry's contract state now (TB_ST_NONE when the transport is not
    // configured or not the active input: no entry).
    uint8_t (*state_now)(void);
    // Configured: is there an entry at all?
    bool (*present)(void);
    // With interrupts masked: the entry's domain, T, d and phase error, with
    // slot 0's ring counters (wr, played) taken at the same instant.
    void (*read)(tb_entry_t *e, bool have_ring, uint32_t wr, uint32_t played);
    // T of the next sample entering the pipeline: a plain START's sample 0
    // (used only while LOCKED) and the T of a transition the poll finds.
    bool (*t_next)(uint64_t *t);
    // The signal generator gave up the pending op 9 (cancelled or refused):
    // remove it from this transport's queue and report `apply_result` (0xE2).
    void (*drop_op9)(uint8_t apply_result);
} tb_provider_t;

#define TB_SLOT_LINE    0u
#define TB_SLOT_USB     1u
#define TB_SLOT_SPDIF   2u      // reserved for WP FWSP2
#define TB_SLOTS        3u

void timebase_register(unsigned slot, const tb_provider_t *p);
// Main loop, every pass: polls each transport's state into its tracker,
// hands losses and disturbances to the signal generator, sends 0xE4.
void timebase_service(uint32_t now_ms);
// EXT_TIME_BASE (64 bytes): the atomic read.
void timebase_read(uint8_t out[TB_READ_LEN]);
// The contract state of time base `tb` (TB_ST_NONE when it has no entry).
uint8_t timebase_state(uint8_t tb);

// Port (firmware: timebase_port.c; host tests: stubs).
uint32_t timebase_port_irq_save(void);
void     timebase_port_irq_restore(uint32_t s);
uint32_t timebase_port_now_us(void);
// Slot 0's ring counters: frames written and frames played (consumed).
bool     timebase_port_ring(uint32_t *wr, uint32_t *played);
void     timebase_port_event(uint8_t tb, uint8_t state, uint8_t reason, uint32_t t_low);

#endif // TIMEBASE_H
