/*
 * link_phase.h - DSPi link protocol v1: phase lock and apply-at-frame (WP LK6)
 *
 * The receiver's fixed latency (SYS2 3.1-3.4, extension_channel_spec.md
 * 2.8.5-2.8.7): every unit plays link frame F exactly `d_local + hops_offset`
 * frames after F arrived, so units with the same delay play the same frame at
 * the same moment.  Portable C11 with no SDK, heap or writable statics, like
 * link_fast: the firmware glue (link_input.c) and the host simulation
 * (tools/link_phase_sim) drive the same code.
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
 * Anchor (SYS2 3.1).  After the drain the rings fill to link_phase_prefill()
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

#ifndef LINK_PHASE_H
#define LINK_PHASE_H

#include <stdbool.h>
#include <stdint.h>

#include "link_codec.h"

// EXT_LINK_PHASE (2.8.5).
#define LINK_PHASE_LEN          8
#define LINK_PHASE_D_DEFAULT    128u
#define LINK_PHASE_HOPS_MAX     64
#define LINK_PHASE_FLAG_BELOW_MIN 0x01u

// EXT_LINK_PHASE.state.
enum {
    LINK_PHASE_FREE      = 0,   // mode 0, delay out of range, or no timeline
    LINK_PHASE_ANCHORING = 1,   // prefill, start, verification (muted)
    LINK_PHASE_LOCKED    = 2,   // |e| < 0.5 frame at the last check, servo running
    LINK_PHASE_PENDING   = 3,   // re-anchor pending: off by more than allowed, automatic re-anchor off
};

// What the glue does after a measurement (link_phase_check()).
enum {
    LINK_PHASE_ACT_NONE     = 0,
    LINK_PHASE_ACT_CORRECT  = 1,    // move the timeline by *frames: > 0 drop rows, < 0 insert silence
    LINK_PHASE_ACT_RESUME   = 2,    // verified: fade the audio in
    LINK_PHASE_ACT_REANCHOR = 3,    // drain, prefill, start again (event 0xE1 REANCHOR)
};

// Decode and delivery margin of d_local_min (SYS2 3.2: batch + 16 + ring target).
#define LINK_PHASE_DECODE_MARGIN    16u
// Verified when |e| is below this (SYS2 3.3's continuity check).
#define LINK_PHASE_LOCK_FRAMES      0.5f
// Phase-locked units re-anchor when |e| exceeds this (an output underrun's
// skip or a dropped block moved the audio by whole frames).
#define LINK_PHASE_REANCHOR_FRAMES  2.0f
// Whole-frame corrections allowed per anchor before it re-anchors instead.
#define LINK_PHASE_MAX_CORRECTIONS  2u
// Largest correction (frames); beyond it the timeline is re-anchored.
#define LINK_PHASE_CORRECT_MAX      64

// Servo, per INPUT_SERVO_TICK_US (75 ms).  Plant: 1 ppm moves e by
// Fs * 1e-6 frames/s (0.048 at 48 kHz), so KP gives a loop time constant of
// about 5 s at 48 kHz (2.6 s at 96); KI removes the feed-forward's residual
// (its zero sits a factor of 5 below the crossover); the IIR (0.3 s) smooths
// the quarter-frame quantisation of the output DMA position.
#define LINK_PHASE_ALPHA        0.25f
#define LINK_PHASE_KP_PPM       4.0f        // ppm per frame of smoothed error
#define LINK_PHASE_KI_PPM       0.01f       // ppm per tick per frame
#define LINK_PHASE_ICLAMP_PPM   5.0f
#define LINK_PHASE_TRIM_MAX_PPM 100.0f

typedef struct {
    float e_s;                  // smoothed error, frames
    float integ;                // integrator, ppm
    float trim;                 // last output, ppm (EXT_LINK_DIAG ppm_phase + integral)
    bool  seeded;
} link_phase_pi_t;

typedef struct {
    // EXT_LINK_PHASE (persisted).
    uint8_t  mode;
    uint16_t d_local;
    int16_t  hops_offset;
    // Limits under the current latency option and rate (EXT_LINK_INFO 16-19).
    uint16_t d_min, d_max;
    // Run state.
    uint8_t  state;             // LINK_PHASE_FREE.. (EXT_LINK_PHASE.state)
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
    link_phase_pi_t pi;
} link_phase_t;

// Factory values: phase lock, d_local 128, hops_offset 0.
void link_phase_init(link_phase_t *ph);

static inline int32_t link_phase_delay(const link_phase_t *ph) {
    return (int32_t)ph->d_local + ph->hops_offset;
}

// The ring fill target for a latency option (frames), its block cap and the
// rings' writable span: d_local_min = target + cap + LINK_PHASE_DECODE_MARGIN,
// d_local_max = span - cap (the steady fill, D minus the frames in flight,
// stays below the ring ceiling).
void link_phase_limits(link_phase_t *ph, uint32_t target, uint32_t block_cap, uint32_t span);

// Mode 1 with the delay inside the limits: the phase lock runs.  Otherwise
// the unit behaves as in mode 0 (fill servo; EXT_LINK_PHASE flags
// D_BELOW_MIN when the delay is below the minimum).
bool link_phase_active(const link_phase_t *ph);

// The ring fill to prefill before the anchor: the delay less the block cap
// and the decode margin, within [target, span - cap].  `target` when the
// phase lock does not run.
uint32_t link_phase_prefill(const link_phase_t *ph, uint32_t target, uint32_t block_cap, uint32_t span);

// EXT_LINK_PHASE OUT: -1 refused (nothing changed), 0 accepted with no
// change, 1 accepted and mode or delay changed (the glue re-anchors when the
// outputs play phase-locked).
int link_phase_set(link_phase_t *ph, const uint8_t p[LINK_PHASE_LEN]);
void link_phase_get(const link_phase_t *ph, uint8_t out[LINK_PHASE_LEN]);

// The rings were emptied (drain, reset): no timeline until the next write.
void link_phase_drain(link_phase_t *ph);

// n frames starting at F = f went to the rings (rows or silence).
static inline void link_phase_written(link_phase_t *ph, uint64_t f, uint32_t n) {
    if (!ph->timeline) {
        ph->first = f;
        ph->timeline = true;
    }
    ph->next_w = f + n;
}

// F_arr - F_x in frames: f_ref is the decoder's reference frame, bits_since
// the bits from its comma to the DMA write position.
static inline float link_phase_since(uint64_t f_ref, uint64_t f_x, int32_t bits_since, float bits_per_frame) {
    return (float)(int64_t)(f_ref - f_x) + (float)bits_since / bits_per_frame;
}

// Frames still to wait before the anchor start (<= 0: start now).
static inline float link_phase_anchor_wait(const link_phase_t *ph, float arr_minus_first) {
    return (float)link_phase_delay(ph) - arr_minus_first;
}

// The phase error: arr_minus_next_w = F_arr - next_w, fill = ring fill in
// frames (fractional) at the same moment.
static inline float link_phase_error(const link_phase_t *ph, float arr_minus_next_w, float fill) {
    return arr_minus_next_w + fill - (float)link_phase_delay(ph);
}

// The outputs were started on the timeline (anchor).
void link_phase_started(link_phase_t *ph);

// One measurement e (frames).  auto_reanchor: EXT_LINK_CONFIG flags bit 1.
// Returns LINK_PHASE_ACT_*; for ACT_CORRECT *frames is the whole-frame
// correction (> 0: drop that many rows, < 0: insert that much silence).
int link_phase_check(link_phase_t *ph, float e, bool auto_reanchor, int32_t *frames);

// Loss of lock with the ring fill now: true = holdover (the glue keeps the
// timeline on silence); false = no timeline to hold (mute and reset, LK5).
bool link_phase_loss(link_phase_t *ph, uint32_t fill);

// Silent frames the holdover writes now to keep the ring at its fill.
static inline uint32_t link_phase_hold_frames(const link_phase_t *ph, uint32_t fill, uint32_t cap) {
    if (!ph->hold || fill >= ph->hold_fill) return 0;
    uint32_t n = ph->hold_fill - fill;
    return n < cap ? n : cap;
}

// Relocked after a holdover: the next rows are placed by F, then verified.
void link_phase_relock(link_phase_t *ph);

// Where a row of frame f lands against the timeline: < 0 it is that many
// frames behind (drop), > 0 that much silence comes first.
static inline int64_t link_phase_gap(const link_phase_t *ph, uint64_t f) {
    return (int64_t)(f - ph->next_w);
}

// The servo's trim for one tick (ppm, added to the rate feed-forward).
float link_phase_servo(link_phase_t *ph, float e);
void link_phase_pi_reset(link_phase_pi_t *pi);

// Muted by the phase lock: anchoring (until verified) or holding.
static inline bool link_phase_muted(const link_phase_t *ph) {
    return ph->hold || (ph->started && ph->state == LINK_PHASE_ANCHORING);
}

// ---------------------------------------------------------------------------
// Apply at frame N (extension_channel_spec.md 2.8.7; link_protocol_spec.md 6.4)
// ---------------------------------------------------------------------------

#define LINK_APPLY_LEN          16
#define LINK_APPLY_DEPTH        4
#define LINK_APPLY_CANCEL_ALL   0u

// EXT_APPLY_AT_FRAME.last_result and event 0xE2.
#define LINK_APPLY_APPLIED      0u
#define LINK_APPLY_LATE         1u
#define LINK_APPLY_CANCELLED    2u
#define LINK_APPLY_REFUSED      3u

typedef struct {
    uint64_t frame;
    uint8_t  op, arg;
    int16_t  value;
} link_apply_op_t;

typedef struct {
    uint8_t  presets;           // PRESET_LOAD slots (10)
    uint8_t  outputs;           // OUT_DELAY / CLICK outputs
    uint16_t max_delay;         // OUT_DELAY values 0..max_delay - 1
} link_apply_limits_t;

typedef struct {
    link_apply_op_t q[LINK_APPLY_DEPTH];    // pending, in frame order
    uint8_t  n;
    uint8_t  last_op, last_result;          // EXT_APPLY_AT_FRAME IN
    uint64_t last_frame;
    link_apply_op_t sf;                     // the superframe apply taken last
    bool     sf_have;
} link_apply_q_t;

void link_apply_init(link_apply_q_t *q);

// Arguments of ops 1-5, 7, 8 (2.8.7).  RATE and unknown ops are refused.
bool link_apply_args_ok(const link_apply_limits_t *lim, uint8_t op, uint8_t arg, int16_t value);

// EXT_APPLY_AT_FRAME OUT payload into *op: false = refused (unknown op, RATE,
// bad arguments, nonzero reserved bytes).  CANCEL_ALL parses with op 0.  The
// caller refuses a full queue and a unit that is not locked.
bool link_apply_parse(const link_apply_limits_t *lim, const uint8_t p[LINK_APPLY_LEN], link_apply_op_t *op);

// Queue an op in frame order (after ops of the same frame).  False when full.
bool link_apply_push(link_apply_q_t *q, const link_apply_op_t *op);

// F of the earliest pending op, UINT64_MAX when none.
static inline uint64_t link_apply_next(const link_apply_q_t *q) {
    return q->n ? q->q[0].frame : UINT64_MAX;
}

// Remove the earliest pending op into *op.
void link_apply_pop(link_apply_q_t *q, link_apply_op_t *op);

// Record a completed op (EXT_APPLY_AT_FRAME IN).
void link_apply_done(link_apply_q_t *q, uint8_t op, uint8_t result, uint64_t frame);

// The apply a valid superframe carries (link_protocol_spec.md 6.4): 0 nothing
// new (none, RATE, or the one taken already), 1 queued, 2 new but refused
// (bad arguments or a full queue: the caller reports it as REFUSED).
int link_apply_take_sf(link_apply_q_t *q, const link_apply_limits_t *lim, const link_sf_t *sf);

// EXT_APPLY_AT_FRAME IN (16 bytes).
void link_apply_in(const link_apply_q_t *q, uint8_t out[LINK_APPLY_LEN]);

#endif // LINK_PHASE_H
