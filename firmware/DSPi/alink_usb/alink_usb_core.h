/*
 * alink_usb_core.h - USB-direct sync (WP USB1): SOF-locked clock, USB-frame
 * time base, phase lock and apply-at-frame on the USB time base.
 *
 * Portable C11, no SDK, like alink/alink_phase.c: the firmware glue
 * (alink_usb.c) and the host simulation (tools/alink_usb_sim) run the same
 * code.  The firmware supplies the alink_usb_port_*() functions at the end of
 * this header; the simulation supplies models.  Specification:
 * extension_channel_spec.md 2.9; design: Documentation/Features/alink_usb.md.
 *
 * The time base.  Every device on one USB host sees the same 1 ms Start of
 * Frame with the same 11-bit frame number.  The unit extends it to a 32-bit
 * count N with N mod 2048 = the bus frame number, and defines the sample
 * index S = spf * N + k (spf = samples per frame, 48 or 96): the packet the
 * host sends in frame N carries S = spf*N .. spf*N + spf - 1.  The sample
 * "arriving now" is F_arr = spf * (N + time since that SOF / 1 ms).  Units
 * that booted at different times count N from different origins, but always
 * by multiples of 2048, so equal S means equal bus time on every unit.
 *
 * The phase lock is alink_phase.c's (WP LK6), with S in place of the alink's
 * frame F: e = F_arr - (next_w - fill) - D, anchor, verify, whole-sample
 * corrections, PI servo on the soft VCXO.  The rate feed-forward is the host's
 * SOF rate measured against the unit's crystal.  Unlike the alink, the time
 * base never goes away while the bus runs (SOF continues when the host stops
 * streaming), so once anchored the unit keeps its timeline on silence between
 * streams and places every new stream's packets by S: constant latency
 * across stream restarts, no re-anchor.
 *
 * Time is kept as uint32 ticks of 1/48 us ("t48"), the RP2350 USB PHY
 * counter's unit; all differences are taken modulo 2^32 (89 s), so the ISR
 * part needs no 64-bit arithmetic (no library calls on the M0+).
 */

#ifndef ALINK_USB_CORE_H
#define ALINK_USB_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "alink/alink_phase.h"

// ---------------------------------------------------------------------------
// Constants (extension_channel_spec.md 2.9)
// ---------------------------------------------------------------------------

#define ALINK_USB_CFG_LEN        8
#define ALINK_USB_STATUS_LEN     64
#define ALINK_USB_STORE_LEN      4       // persisted: mode, flags, d (u16)

// EXT_ALINK_USB.mode
#define ALINK_USB_MODE_OFF       0u      // stock: asynchronous feedback, crystal clock
#define ALINK_USB_MODE_RATE      1u      // SOF rate lock (U1)
#define ALINK_USB_MODE_PHASE     2u      // SOF rate lock + phase lock (U1, U3-U6)

// EXT_ALINK_USB.flags (OUT and IN)
#define ALINK_USB_FLAG_AUTO_REANCHOR 0x01u
#define ALINK_USB_FLAGS_DEFAULT      ALINK_USB_FLAG_AUTO_REANCHOR
#define ALINK_USB_D_DEFAULT          192u    // 4 ms at 48 kHz; 96 kHz needs 324..351

// EXT_ALINK_USB.status_flags (IN only)
#define ALINK_USB_SF_D_BELOW_MIN     0x01u   // mode 2: D below d_min (runs as mode 1)
#define ALINK_USB_SF_D_ABOVE_MAX     0x02u   // mode 2: D above d_max (runs as mode 1)
#define ALINK_USB_SF_RATE_UNSUPP     0x04u   // mode 2 at 44.1 kHz: phase lock needs 48 or 96 kHz (runs as mode 1)

// EXT_ALINK_USB_STATUS.state and event 0xE3 state
enum {
    ALINK_USB_ST_OFF       = 0,  // mode 0, or the input source is not USB
    ALINK_USB_ST_NO_SOF    = 1,  // no Start of Frame (not configured, suspended, unplugged)
    ALINK_USB_ST_PULL_IN   = 2,  // measuring the host rate, VCXO pulling in
    ALINK_USB_ST_RATE_LOCK = 3,  // rate-locked (mode 1, or mode 2 that cannot phase-lock)
    ALINK_USB_ST_ANCHORING = 4,  // mode 2: drain, prefill, start, verify (muted)
    ALINK_USB_ST_PHASE_LOCK= 5,  // mode 2: phase-locked (with or without a stream)
    ALINK_USB_ST_REJECTED  = 6,  // host SOF rate beyond ALINK_USB_HOST_MAX_PPM: stock behaviour
};

// Event 0xE3 reason
enum {
    ALINK_USB_RS_NONE      = 0,
    ALINK_USB_RS_LOCKED    = 1,  // rate lock reached, or phase-locked and playing
    ALINK_USB_RS_HOST_RANGE= 2,  // host rate beyond ALINK_USB_HOST_MAX_PPM (state 6), or back in range
    ALINK_USB_RS_SOF_LOST  = 3,  // no SOF for ALINK_USB_SOF_LOST_MS
    ALINK_USB_RS_REANCHOR  = 4,  // phase error beyond 2 samples, a new D or mode, a reset of the outputs
    ALINK_USB_RS_GAP       = 5,  // packets missing while streaming: silence on the timeline, muted, fade back in
    ALINK_USB_RS_DISABLED  = 6,  // mode 0 written or the input source left USB
    ALINK_USB_RS_CONFIG    = 7,  // mode 2 cannot phase-lock (rate, D range): runs rate-locked
    ALINK_USB_RS_HOLDOVER  = 8,  // no SOF for 50 ms: the time base is extrapolated, the timeline kept (WP USB2)
    ALINK_USB_RS_RESUMED   = 9,  // SOF back within the ride-through limit, frame and time consistent
};

// EXT_ALINK_USB_DIAG: why a timeline (or the time base) ended (WP USB2).
enum {
    ALINK_USB_END_SOF_LOST = 0,  // no SOF for longer than the ride-through limit, or no estimate to ride on
    ALINK_USB_END_LONG_GAP,      // a gap of over 1 s between two passes' SOFs outside a ride-through
    ALINK_USB_END_DISCONT,       // frame count and clock disagree (F2), or on the return from a ride-through
    ALINK_USB_END_REJECTED,      // host beyond 295 ppm
    ALINK_USB_END_CONFIG,        // rate change or D / mode no longer allows the phase lock
    ALINK_USB_END_RE_PHASE,      // re-anchor: phase error, failed verification
    ALINK_USB_END_RE_OUTPUTS,    // re-anchor: the output rings were reset (preset load, flash write, ...)
    ALINK_USB_END_RE_OTHER,      // re-anchor: placement far off, hold stall, EXT_ALINK_USB write
    ALINK_USB_END_STOP,          // mode 0 or the source left USB
    ALINK_USB_END_N
};

typedef struct {
    uint16_t ends[ALINK_USB_END_N];
    uint16_t rides;              // SOF absences ridden through
    uint16_t holdovers;          // SOF absences entered (ridden or not)
    uint16_t suspends, resumes, mounts, umounts;   // firmware: suspend, resume, bus reset, unplug events
    uint8_t  last_cause;         // ALINK_USB_END_*, 0xFF none
    uint32_t last_frames;        // the last absence: frames counted on the bus
    uint32_t last_us;            // its duration by the unit's clock
    int32_t  last_dev_us;        // clock minus frames (host rate) at the return, us
    uint32_t longest_ride_us;
    uint32_t max_pass_us;        // longest main-loop pass (USB source), since the last clear
    uint32_t max_isr_quiet_us;   // longest time without a SOF interrupt entry seen by the main loop
    uint32_t max_isr_late_us;    // longest SOF interrupt entry after its SOF (RP2350 exact; RP2040 predicted)
    uint32_t last_pass_us;       // internal: the previous service call
} alink_usb_diag_t;

#define ALINK_USB_HOST_MAX_PPM     295.0f    // VCXO authority 300 ppm less trim headroom
#define ALINK_USB_HOST_HYST_PPM    10.0f     // accepted again below MAX - HYST
#define ALINK_USB_SOF_LOST_MS      50u
#define ALINK_USB_READY_PPM        1.0f      // VCXO within this of the feed-forward: anchor
#define ALINK_USB_EST_HALF         8000u     // estimator anchors rotate every 8 s: span 8-16 s
#define ALINK_USB_EST_MIN          2000u     // estimate used from a 2 s span
#define ALINK_USB_TICK_US          75000u    // = INPUT_SERVO_TICK_US
#define ALINK_USB_EVENT_MIN_MS     250u
#define ALINK_USB_GAP_MUTE_MS      20u       // a stream gap mutes this long, then fades back in
#define ALINK_USB_ANCHOR_MUTE_MS   20u       // soft mute refresh while anchoring
#define ALINK_USB_RIDE_MAX_MS      10000u    // a SOF absence is ridden through this long (WP USB2)
#define ALINK_USB_EST_GAP_MAX      12000u    // the estimator keeps its anchors across a verified absence
#define ALINK_USB_DIAG_LEN         64
#define ALINK_USB_STOP_FRAMES      16u       // a GAP this close before alt 0 was the stream's end
#define ALINK_USB_DECODE_MARGIN    16u       // D - (cap + margin) is the hold threshold

// t48: 48 ticks per microsecond, 48000 per frame.
#define ALINK_USB_T48_FRAME        48000u
// A packet cannot complete sooner than this after its own SOF (the shortest
// phase-mode packet, 44 x 2 ch x 2 B, takes 117 us on the Full Speed wire): a
// completion seen earlier after the latest SOF belongs to the frame before.
#define ALINK_USB_STAMP_MIN48      (100u * 48u)
#define ALINK_USB_FRAME_NONE       0xFFFFFFFFu

// ---------------------------------------------------------------------------
// Time base: SOF ISR and packet stamp (RAM in the firmware, inline here)
// ---------------------------------------------------------------------------

typedef struct {
    volatile uint32_t n;        // extended frame number of the latest SOF (N mod 2048 = bus frame)
    volatile uint32_t t;        // its time, t48 (best estimate)
    volatile uint32_t n_c;      // latest clean pair: frame and exact time
    volatile uint32_t t_c;
    volatile uint32_t isr;      // t48 of the latest SOF interrupt entry (WP USB2 diagnostics)
    volatile bool     valid;    // n, t hold a SOF
    volatile bool     have_c;   // n_c, t_c hold a clean pair
    volatile uint32_t stamp;    // frame of the last OUT packet, ALINK_USB_FRAME_NONE none
    // Firmware only, kept here to share one address with the stamp in the USB
    // IRQ (WP USB2): the OUT arm flag and TinyUSB events by DCD_EVENT_* id
    // ([1] bus reset, [2] unplugged, [4] suspend, [5] resume).
    volatile bool     out_wait;
    volatile uint32_t ev[16];   // indexed by event id without a mask: ids run to USBD_EVENT_FUNC_CALL (8)
} alink_usb_tb_t;

// One SOF.  f11: SOF frame number; now: t48 at the call; since: t48 ticks
// since the SOF from the RP2350's SOF_TIMESTAMP_RAW - _LAST (exact), or
// UINT32_MAX on the RP2040 (the call time stands for the SOF time).
static inline void alink_usb_tb_sof(alink_usb_tb_t *tb, uint32_t f11, uint32_t now, uint32_t since) {
    bool exact = since != UINT32_MAX;
    uint32_t t = exact ? now - since : now;
    f11 &= 0x7FFu;
    if (!tb->valid) {
        tb->n = f11;
        tb->t = t;
        tb->valid = true;
        tb->have_c = false;
        tb->isr = now;
        return;
    }
    uint32_t delta = (f11 - tb->n) & 0x7FFu;
    uint32_t el = t - tb->t;
    uint32_t n = tb->n + delta;
    bool clean;
    uint32_t t_est = t;
    if (exact) {
        // The PHY counter is exact while it has not wrapped (21 bits, 43.7 ms).
        clean = delta >= 1u && delta <= 32u;
    } else {
        int32_t dev = (int32_t)(el - ALINK_USB_T48_FRAME);
        clean = delta == 1u && dev <= 144 && dev >= -144;     // both ISR entries alike: 3 us
        if (tb->have_c && n - tb->n_c <= 16u) {
            // A late entry: the SOF happened when the clean pair predicts.
            uint32_t pred = tb->t_c + (n - tb->n_c) * ALINK_USB_T48_FRAME;
            int32_t late = (int32_t)(t - pred);
            if (late > 0 && late < (int32_t)ALINK_USB_T48_FRAME) t_est = pred;
        }
    }
    tb->n = n;
    tb->t = t_est;
    if (clean) {
        tb->n_c = n;
        tb->t_c = t;
        tb->have_c = true;
    }
    tb->isr = now;
}

// The OUT packet completed now (t48): its frame is the latest SOF's, or the
// one before when the completion is too soon after that SOF to be its own
// (the interrupt was held off across the SOF).
static inline void alink_usb_tb_stamp(alink_usb_tb_t *tb, uint32_t now) {
    if (!tb->valid) {
        tb->stamp = ALINK_USB_FRAME_NONE;
        return;
    }
    uint32_t dt = now - tb->t;
    tb->stamp = tb->n - (dt < ALINK_USB_STAMP_MIN48 ? 1u : 0u);
}

// ---------------------------------------------------------------------------
// Host rate estimator (main loop): dual anchors on clean SOF pairs
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t n[2], t[2];        // [0] older, [1] newer anchor
    uint32_t last_n;
    bool     seeded, valid;
    float    ppm;               // host SOF rate against the unit's crystal, + = host fast
} alink_usb_est_t;

void alink_usb_est_reset(alink_usb_est_t *e);
// One clean pair (n, t48).  Repeated pairs are ignored.
void alink_usb_est_sample(alink_usb_est_t *e, uint32_t n, uint32_t t);

// ---------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t frame;             // stamp (extended N), ALINK_USB_FRAME_NONE unknown
    uint16_t samples;           // samples (frames) in the packet
    const void *data;           // passed back to alink_usb_port_write_packet()
    uint16_t len;
} alink_usb_pkt_t;

typedef struct {
    // EXT_ALINK_USB (persisted)
    uint8_t  mode, flags;
    uint16_t d;
    // Limits under the latency option at the current rate
    uint16_t d_min, d_max;
    uint32_t block_cap;
    // Run state
    uint8_t  state, reason;
    uint8_t  ev_state, ev_reason;           // last event sent
    bool     ev_pending;
    // The latest event change was a GAP (count_gap): what was pending before
    // it, restored when a stream stop takes that GAP back (review usb1 N1).
    bool     ev_gap_last, ev_gap_pending0;
    uint8_t  ev_gap_reason0;
    uint32_t ev_last_ms;
    bool     ev_once;
    uint32_t fs, spf;                        // rate the timeline runs at
    bool     rejected;
    bool     tb_ok;                          // SOF time base usable
    alink_usb_est_t est;
    uint32_t tick_last_us;
    bool     ticked;                         // the servo has actuated at least once
    bool     rate_locked;                    // the VCXO reached the feed-forward (latched until a reset)
    float    ff_ppm;                         // last feed-forward commanded
    alink_phase_t ph;
    alink_apply_q_t aq;
    bool     prefilling;                     // outputs drained, silence prefill (main.c)
    bool     started;                        // the outputs play our timeline
    bool     muted_gap;                      // a gap muted the stream; fade in on the next packet
    uint32_t pkt_off;                        // first sample of the held packet still to write
    bool     held;                           // hp: the newest placed packet, written when the next arrives
    bool     fade_in;                        // the next packet written fades in (after silence)
    alink_usb_pkt_t hp;
    // Ride-through of SOF absences (WP USB2) and diagnostics.
    bool     dark;                           // no SOF for 50 ms: extrapolating
    uint32_t dark_n, dark_t, dark_us;        // the last SOF before it, and when it began (us)
    alink_usb_diag_t diag;
    // SOF discontinuity check (review usb1 F2), main loop: the pair seen last.
    bool     exact;                          // SOF times from the PHY (RP2350): 100 us limit, else 0.5 ms
    bool     seen_valid;
    // The last GAP, taken back when the stream stops right after it (R3).
    bool     gap_recent;
    uint32_t gap_n, gap_missed0;
    uint32_t seen_n, seen_t;
    uint32_t last_frame;                     // last packet frame (continuity for unstamped ones)
    // Stream (alt > 0) accounting, EXT_ALINK_USB_STATUS (U6)
    bool     stream;                         // alt > 0
    bool     stream_seen;                    // a packet of this stream was placed
    uint32_t stream_first;                   // N of its first packet
    int32_t  stream_shift;                   // silence inserted - samples dropped since
    uint32_t stream_samples;
    // Counters (from boot)
    uint32_t anchors, reanchors, gaps, missed, late, unstamped, sync_pulses;
    // SYNC_PULSE (bench builds): frame to pulse at, armed in the ISR by the port
} alink_usb_t;

void alink_usb_init(alink_usb_t *us);

// EXT_ALINK_USB OUT: false = refused (nothing changed).
bool alink_usb_set_config(alink_usb_t *us, const uint8_t p[ALINK_USB_CFG_LEN]);
void alink_usb_get_config(const alink_usb_t *us, uint8_t o[ALINK_USB_CFG_LEN]);
void alink_usb_store(const alink_usb_t *us, uint8_t s[ALINK_USB_STORE_LEN]);
void alink_usb_load(alink_usb_t *us, const uint8_t s[ALINK_USB_STORE_LEN]);   // implausible values: defaults

// Limits for the latency option's block cap at rate fs; span = the rings'
// writable span.  d_min = 2.2 spf + cap + 16, d_max = span - cap: 170..399
// at 48 kHz and 324..351 at 96 kHz for every option USB runs (block cap 49 / 97).
void alink_usb_limits(alink_usb_t *us, uint32_t fs, uint32_t target, uint32_t cap, uint32_t span);
// Mode 2 at 48/96 kHz with D in range: the phase lock runs.
bool alink_usb_phase_possible(const alink_usb_t *us, uint32_t fs);
// The ring target while USB is the source: the phase prefill when the phase
// lock runs, else `target` unchanged (output_latency_apply()).
uint32_t alink_usb_ring_target(alink_usb_t *us, uint32_t fs, uint32_t target, uint32_t cap, uint32_t span);

// Packets pass through the engine (alink_usb_deliver) instead of the stock path.
static inline bool alink_usb_engaged(const alink_usb_t *us) { return us->started || us->prefilling; }
// Feedback: report nominal (the host follows our SOF-locked clock).
bool alink_usb_nominal_feedback(const alink_usb_t *us);

// One packet off the USB ring.  False: not consumed (the rings have no room
// yet), try again on the next pass.
bool alink_usb_deliver(alink_usb_t *us, const alink_usb_pkt_t *p);
// Once per main-loop pass while USB is the source.
void alink_usb_service(alink_usb_t *us, uint32_t now_ms);
// The input source left USB, or mode 0: back to stock (VCXO parked, ops cancelled).
void alink_usb_stop(alink_usb_t *us, uint8_t reason);

// The main loop's anchor handshake (main.c, like the alink's): wanted -> mute
// and drain -> on_drain -> prefill (service) -> anchor_wait() <= 0 ->
// start the outputs in sync -> on_enabled.
bool alink_usb_anchor_wanted(const alink_usb_t *us);
void alink_usb_on_drain(alink_usb_t *us);
// Samples still to wait before the start (<= 0: start now); +inf-like 1e9
// while the prefill is not complete or no time base.
float alink_usb_anchor_wait(alink_usb_t *us);
void alink_usb_on_enabled(alink_usb_t *us);
// Somebody else reset the output rings (preset load, flash write, type
// switch, rate change): the timeline is gone.
void alink_usb_outputs_reset(alink_usb_t *us);
// Alt setting > 0 / 0 (stream start / stop).
void alink_usb_stream(alink_usb_t *us, bool on);

// Time base reads (main loop; take the time base with interrupts off).
// S now (F_arr), floor; false without a time base.
bool alink_usb_sample_now(alink_usb_t *us, uint64_t *s, uint16_t *bus_frame);

// EXT_APPLY_AT_FRAME OUT with time_base 1 (S) or 2 (bus frame): false = refused.
bool alink_usb_apply(alink_usb_t *us, const uint8_t p[ALINK_APPLY_LEN]);
void alink_usb_cancel_all(alink_usb_t *us);
// The 0xE2 frame field of a USB op: bus frame (bits 0-10) and the sample in
// it (bits 16-23).
uint32_t alink_usb_bus_form(const alink_usb_t *us, uint64_t s);

// EXT_ALINK_USB_DIAG (64 bytes) and its OUT (1 byte, 1 = clear).
void alink_usb_get_diag(const alink_usb_t *us, uint8_t o[ALINK_USB_DIAG_LEN]);
bool alink_usb_clear_diag(alink_usb_t *us, uint8_t v);

// EXT_ALINK_USB_STATUS (64 bytes).
void alink_usb_get_status(alink_usb_t *us, uint8_t o[ALINK_USB_STATUS_LEN]);

// ---------------------------------------------------------------------------
// Port: supplied by the firmware (alink_usb.c) or the simulation
// ---------------------------------------------------------------------------

extern alink_usb_tb_t alink_usb_tb;
extern const alink_apply_limits_t alink_usb_apply_limits;    // presets, outputs, delay
uint32_t alink_usb_port_now48(void);
uint32_t alink_usb_port_now_us(void);
uint32_t alink_usb_port_irq_save(void);
void     alink_usb_port_irq_restore(uint32_t s);
uint32_t alink_usb_port_fs(void);                       // audio_state.freq
bool     alink_usb_port_play(float *fill);              // output ring fill (slot 0), fractional
uint32_t alink_usb_port_room(void);                     // producer_block_bounds(): most samples per call now
// Samples off..off+k-1 of the packet, the input gain ramping linearly from g0
// (first) to g1 (last); 1, 1 = unchanged.
void     alink_usb_port_write_packet(const alink_usb_pkt_t *p, uint32_t off, uint32_t k, float g0, float g1);
// Copy the packet's bytes (the USB ring slot is reused) and return the copy.
const void *alink_usb_port_keep(const alink_usb_pkt_t *p);
void     alink_usb_port_write_silence(uint32_t k);
// ff_trim: the feed-forward plus `trim` (input_servo_apply_trim); false: the
// feed-forward plus the fill servo on slot 0 (input_servo_apply).
void     alink_usb_port_actuate(float hz, float trim, bool ff_trim);
void     alink_usb_port_park(void);                     // input_servo_reset()
float    alink_usb_port_vcxo_ppm(void);
uint8_t  alink_usb_port_apply(const alink_apply_op_t *op);
void     alink_usb_port_apply_result(uint8_t op, uint8_t result, uint32_t frame);
void     alink_usb_port_event(uint8_t state, uint8_t reason, uint32_t frame);
void     alink_usb_port_soft_mute(uint32_t samples);
bool     alink_usb_port_pulse(uint32_t frame);          // SYNC_PULSE: arm the GPIO at that SOF; false = no GPIO
void     alink_usb_port_ring_target(void);              // re-apply the latency option (prefill target)
bool     alink_usb_port_stream_active(void);            // usb_audio_stream_active(): packets within the gap threshold

#endif // ALINK_USB_CORE_H
