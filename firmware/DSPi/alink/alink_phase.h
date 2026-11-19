/*
 * alink_phase.h - DSPi alink protocol v1: phase lock and apply-at-frame (WP LK6)
 *
 * The receiver's fixed latency (SYS2 3.1-3.4, extension_channel_spec.md
 * 2.8.5-2.8.7): every unit plays alink frame F exactly `d_local + hops_offset`
 * frames after F arrived, so units with the same delay play the same frame at
 * the same moment.  Portable C11 with no SDK, heap or writable statics, like
 * alink_fast: the firmware glue (alink_input.c) and the host simulation
 * (tools/alink_phase_sim) drive the same code.
 *
 * The timeline.  The receiver writes the decoded frames into the output rings
 * in F order.  `next_w` is the F of the next frame written; the rings hold the
 * frames before it, `fill` of them not yet played, so the frame playing now
 * is `next_w - fill`.  The phase error is then
 *
 *     e = F_arr - (next_w - fill) - D,      D = d_local + hops_offset
 *
 * with F_arr the frame arriving now, from the RX DMA position against the
 * decoder's reference (frame F_ref, comma ending at bit p_ref):
 * F_arr = F_ref + (bits - p_ref) / bits_per_frame.  Both positions are DMA
 * counters read together, so e is exact to a fraction of a frame and needs no
 * model of the decode, FIFO or batch delays: anything that moves audio (a
 * dropped block, a ring skip, silence inserted) moves next_w or fill, and e
 * shows it.
 *
 * Anchor (SYS2 3.1).  After the drain the rings fill to alink_phase_prefill()
 * frames while the outputs are stopped; the glue starts them when
 * F_arr - first reaches D (first = F of the first frame written), which makes
 * e zero.  The first measurement verifies it; an error of a frame or more
 * (a late main-loop pass) is corrected by dropping or inserting whole frames,
 * still muted, and the servo then trims the fraction: ppm = ff + PI(e).
 *
 * Loss (SYS2 3.3).  A loss while phase-locked enters holdover: the glue keeps
 * the timeline running on silence at the held VCXO rate (the rings stay at
 * their fill, F is extrapolated) and mutes.  On relock the new frames are
 * placed by F on the same timeline; the first measurement decides: within
 * half a frame the audio fades back in, otherwise the unit re-anchors.
 */

#ifndef ALINK_PHASE_H
#define ALINK_PHASE_H

#include <stdbool.h>
#include <stdint.h>

#include "alink_codec.h"

// EXT_ALINK_PHASE (2.8.5).
#define ALINK_PHASE_LEN         8
#define ALINK_PHASE_D_DEFAULT   128u
#define ALINK_PHASE_HOPS_MAX    64
#define ALINK_PHASE_FLAG_BELOW_MIN 0x01u

// EXT_ALINK_PHASE.state.
enum {
    ALINK_PHASE_FREE     = 0,   // mode 0, delay out of range, or no timeline
    ALINK_PHASE_ANCHORING = 1,  // prefill, start, verification (muted)
    ALINK_PHASE_LOCKED   = 2,   // |e| < 0.5 frame at the last check, servo running
    ALINK_PHASE_PENDING  = 3,   // re-anchor pending: off by more than allowed, automatic re-anchor off
};

// What the glue does after a measurement (alink_phase_check()).
enum {
    ALINK_PHASE_ACT_NONE    = 0,
    ALINK_PHASE_ACT_CORRECT = 1,    // move the timeline by *frames: > 0 drop rows, < 0 insert silence
    ALINK_PHASE_ACT_RESUME  = 2,    // verified: fade the audio in
    ALINK_PHASE_ACT_REANCHOR = 3,   // drain, prefill, start again (event 0xE1 REANCHOR)
};

// Decode and delivery margin of d_local_min (SYS2 3.2: batch + 16 + ring target).
#define ALINK_PHASE_DECODE_MARGIN   16u
// Verified when |e| is below this (SYS2 3.3's continuity check).
#define ALINK_PHASE_LOCK_FRAMES     0.5f
// Phase-locked units re-anchor when |e| exceeds this (an output underrun's
// skip or a dropped block moved the audio by whole frames).
#define ALINK_PHASE_REANCHOR_FRAMES 2.0f
// Whole-frame corrections allowed per anchor before it re-anchors instead.
#define ALINK_PHASE_MAX_CORRECTIONS 2u
// Largest correction (frames); beyond it the timeline is re-anchored.
#define ALINK_PHASE_CORRECT_MAX     64

// Servo, per INPUT_SERVO_TICK_US (75 ms).  Plant: 1 ppm moves e by
// Fs * 1e-6 frames/s (0.048 at 48 kHz), so KP gives a loop time constant of
// about 5 s at 48 kHz (2.6 s at 96); KI removes the feed-forward's residual
// (its zero sits a factor of 5 below the crossover); the IIR (0.3 s) smooths
// the quarter-frame quantisation of the output DMA position.
#define ALINK_PHASE_ALPHA       0.25f
#define ALINK_PHASE_KP_PPM      4.0f        // ppm per frame of smoothed error
#define ALINK_PHASE_KI_PPM      0.01f       // ppm per tick per frame
#define ALINK_PHASE_ICLAMP_PPM  5.0f
#define ALINK_PHASE_TRIM_MAX_PPM 100.0f

typedef struct {
    float e_s;                  // smoothed error, frames
    float integ;                // integrator, ppm
    float trim;                 // last output, ppm (EXT_ALINK_DIAG ppm_phase + integral)
    bool  seeded;
} alink_phase_pi_t;

typedef struct {
    // EXT_ALINK_PHASE (persisted).
    uint8_t  mode;
    uint16_t d_local;
    int16_t  hops_offset;
    // Limits under the current latency option and rate (EXT_ALINK_INFO 16-19).
    uint16_t d_min, d_max;
    // Run state.
    uint8_t  state;             // ALINK_PHASE_FREE.. (EXT_ALINK_PHASE.state)
    bool     timeline;          // next_w is valid
    bool     started;           // the outputs play this timeline (anchored)
    bool     hold;              // holdover: silence on the timeline
    bool     resync;            // relocked after holdover: the next rows are placed by F
    uint8_t  corrections;
    uint64_t first;             // F of the first frame written after the drain
    uint64_t next_w;            // F of the next frame written to the rings
    uint32_t hold_fill;         // ring fill the holdover keeps
    float    e;                 // last measured phase error, frames
    bool     e_valid;
    alink_phase_pi_t pi;
} alink_phase_t;

// Factory values: phase lock, d_local 128, hops_offset 0.
void alink_phase_init(alink_phase_t *ph);

static inline int32_t alink_phase_delay(const alink_phase_t *ph) {
    return (int32_t)ph->d_local + ph->hops_offset;
}

// The ring fill target for a latency option (frames), its block cap and the
// rings' writable span: d_local_min = target + cap + ALINK_PHASE_DECODE_MARGIN,
// d_local_max = span - cap (the steady fill, D minus the frames in flight,
// stays below the ring ceiling).
void alink_phase_limits(alink_phase_t *ph, uint32_t target, uint32_t block_cap, uint32_t span);

// Mode 1 with the delay inside the limits: the phase lock runs.  Otherwise
// the unit behaves as in mode 0 (fill servo; EXT_ALINK_PHASE flags
// D_BELOW_MIN when the delay is below the minimum).
bool alink_phase_active(const alink_phase_t *ph);

// The ring fill to prefill before the anchor: the delay less the block cap
// and the decode margin, within [target, span - cap].  `target` when the
// phase lock does not run.
uint32_t alink_phase_prefill(const alink_phase_t *ph, uint32_t target, uint32_t block_cap, uint32_t span);

// EXT_ALINK_PHASE OUT: -1 refused (nothing changed), 0 accepted with no
// change, 1 accepted and mode or delay changed (the glue re-anchors when the
// outputs play phase-locked).
int alink_phase_set(alink_phase_t *ph, const uint8_t p[ALINK_PHASE_LEN]);
void alink_phase_get(const alink_phase_t *ph, uint8_t out[ALINK_PHASE_LEN]);

// The rings were emptied (drain, reset): no timeline until the next write.
void alink_phase_drain(alink_phase_t *ph);

// n frames starting at F = f went to the rings (rows or silence).
static inline void alink_phase_written(alink_phase_t *ph, uint64_t f, uint32_t n) {
    if (!ph->timeline) {
        ph->first = f;
        ph->timeline = true;
    }
    ph->next_w = f + n;
}

// F_arr - F_x in frames: f_ref is the decoder's reference frame, bits_since
// the bits from its comma to the DMA write position.
static inline float alink_phase_since(uint64_t f_ref, uint64_t f_x, int32_t bits_since, float bits_per_frame) {
    return (float)(int64_t)(f_ref - f_x) + (float)bits_since / bits_per_frame;
}

// Frames still to wait before the anchor start (<= 0: start now).
static inline float alink_phase_anchor_wait(const alink_phase_t *ph, float arr_minus_first) {
    return (float)alink_phase_delay(ph) - arr_minus_first;
}

// The phase error: arr_minus_next_w = F_arr - next_w, fill = ring fill in
// frames (fractional) at the same moment.
static inline float alink_phase_error(const alink_phase_t *ph, float arr_minus_next_w, float fill) {
    return arr_minus_next_w + fill - (float)alink_phase_delay(ph);
}

// The outputs were started on the timeline (anchor).
void alink_phase_started(alink_phase_t *ph);

// One measurement e (frames).  auto_reanchor: EXT_ALINK_CONFIG flags bit 1.
// Returns ALINK_PHASE_ACT_*; for ACT_CORRECT *frames is the whole-frame
// correction (> 0: drop that many rows, < 0: insert that much silence).
int alink_phase_check(alink_phase_t *ph, float e, bool auto_reanchor, int32_t *frames);

// Loss of lock with the ring fill now: true = holdover (the glue keeps the
// timeline on silence); false = no timeline to hold (mute and reset, LK5).
bool alink_phase_loss(alink_phase_t *ph, uint32_t fill);

// Silent frames the holdover writes now to keep the ring at its fill.
static inline uint32_t alink_phase_hold_frames(const alink_phase_t *ph, uint32_t fill, uint32_t cap) {
    if (!ph->hold || fill >= ph->hold_fill) return 0;
    uint32_t n = ph->hold_fill - fill;
    return n < cap ? n : cap;
}

// Relocked after a holdover with the stream at frame f, where the held
// timeline expected f_expect (the holdover's extrapolation): within `span`
// frames (the rings' writable span) the next rows are placed by F, then
// verified, and it returns true.  Further off (a sender restarted its counter,
// another stream) the timeline is useless: the holdover ends, false, and the
// glue takes the full reset (review lk6 F1).
bool alink_phase_relock(alink_phase_t *ph, uint64_t f, uint64_t f_expect, uint32_t span);

// Where a row of frame f lands against the timeline: < 0 it is that many
// frames behind (drop), > 0 that much silence comes first.
static inline int64_t alink_phase_gap(const alink_phase_t *ph, uint64_t f) {
    return (int64_t)(f - ph->next_w);
}

// A placement gap the rings can take: within +-span frames.  Beyond it (either
// way) the glue re-anchors rather than dropping rows or writing silence for
// that long.
static inline bool alink_phase_gap_ok(int64_t gap, uint32_t span) {
    return gap <= (int64_t)span && gap >= -(int64_t)span;
}

// The servo's trim for one tick (ppm, added to the rate feed-forward).
float alink_phase_servo(alink_phase_t *ph, float e);
void alink_phase_pi_reset(alink_phase_pi_t *pi);

// Muted by the phase lock: anchoring (until verified) or holding.
static inline bool alink_phase_muted(const alink_phase_t *ph) {
    return ph->hold || (ph->started && ph->state == ALINK_PHASE_ANCHORING);
}

// ---------------------------------------------------------------------------
// Apply at frame N (extension_channel_spec.md 2.8.7; alink_protocol_spec.md 6.4)
// ---------------------------------------------------------------------------

#define ALINK_APPLY_LEN         16
#define ALINK_APPLY_DEPTH       4
#define ALINK_APPLY_CANCEL_ALL  0u

// EXT_APPLY_AT_FRAME.last_result and event 0xE2.
#define ALINK_APPLY_APPLIED     0u
#define ALINK_APPLY_LATE        1u
#define ALINK_APPLY_CANCELLED   2u
#define ALINK_APPLY_REFUSED     3u

typedef struct {
    uint64_t frame;
    uint8_t  op, arg;
    int16_t  value;
} alink_apply_op_t;

typedef struct {
    uint8_t  presets;           // PRESET_LOAD slots (10)
    uint8_t  outputs;           // OUT_DELAY / CLICK outputs
    uint16_t max_delay;         // OUT_DELAY values 0..max_delay - 1
    bool     siggen_ops;        // ops 9 SIGGEN_START / 10 SIGGEN_STOP (WP FWTB1; RP2350 builds
                                // with a time base, never RP2040: PLAN-AT OD-ATm).  false: unknown ops
} alink_apply_limits_t;

typedef struct {
    alink_apply_op_t q[ALINK_APPLY_DEPTH];  // pending, in frame order
    uint8_t  n;
    uint8_t  last_op, last_result;          // EXT_APPLY_AT_FRAME IN
    uint64_t last_frame;
    alink_apply_op_t sf;                    // the superframe apply taken last
    bool     sf_have;
} alink_apply_q_t;

void alink_apply_init(alink_apply_q_t *q);

// Arguments of ops 1-5, 7, 8 (2.8.7) and, with lim->siggen_ops, 9 (arg 0,
// value 0) and 10 (arg 0 fade or 1 now, value 0; 2.8.7.1).  RATE and unknown
// ops are refused.
bool alink_apply_args_ok(const alink_apply_limits_t *lim, uint8_t op, uint8_t arg, int16_t value);

// EXT_APPLY_AT_FRAME OUT payload into *op: false = refused (unknown op, RATE,
// bad arguments, nonzero reserved bytes).  CANCEL_ALL parses with op 0.  The
// caller refuses a full queue and a unit that is not locked.
bool alink_apply_parse(const alink_apply_limits_t *lim, const uint8_t p[ALINK_APPLY_LEN], alink_apply_op_t *op);

// Queue an op in frame order (after ops of the same frame).  False when full.
bool alink_apply_push(alink_apply_q_t *q, const alink_apply_op_t *op);

// F of the earliest pending op, UINT64_MAX when none.
static inline uint64_t alink_apply_next(const alink_apply_q_t *q) {
    return q->n ? q->q[0].frame : UINT64_MAX;
}

// Remove the earliest pending op into *op.
void alink_apply_pop(alink_apply_q_t *q, alink_apply_op_t *op);

// Pending ops of kind `op` (WP FWTB1: at most one op 9 per unit).
unsigned alink_apply_count(const alink_apply_q_t *q, uint8_t op);
// Remove the earliest pending op of kind `op` into *out: false when none.
bool alink_apply_remove(alink_apply_q_t *q, uint8_t op, alink_apply_op_t *out);

// Record a completed op (EXT_APPLY_AT_FRAME IN).
void alink_apply_done(alink_apply_q_t *q, uint8_t op, uint8_t result, uint64_t frame);

// The apply a valid superframe carries (alink_protocol_spec.md 6.4): 0 nothing
// new (none, RATE, or the one taken already), 1 queued, 2 new but refused
// (bad arguments or a full queue: the caller reports it as REFUSED).
int alink_apply_take_sf(alink_apply_q_t *q, const alink_apply_limits_t *lim, const alink_sf_t *sf);

// EXT_APPLY_AT_FRAME IN (16 bytes).
void alink_apply_in(const alink_apply_q_t *q, uint8_t out[ALINK_APPLY_LEN]);

#endif // ALINK_PHASE_H
