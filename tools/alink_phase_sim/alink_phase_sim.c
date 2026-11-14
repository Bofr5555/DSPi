// alink_phase_sim.c - two phase-locked alink receivers fed by one sender (WP LK6)
//
// What runs as in the firmware:
//  - the sender: LK4's encoder core (firmware/DSPi-Sender/sender_alink.c,
//    snd_frame, alink_tx_schedule for the superframe applies);
//  - the decoder: LK5's alink_fast over a 32 KB raw ring per receiver, at the
//    ring positions the receiver's DMA would have written;
//  - the phase lock and the apply queue: alink/alink_phase.c (WP LK6);
//  - the rate measurement: alink/alink_ring_clock.h (LK45F), on the receiver's
//    own microsecond timer;
//  - the glue: deliver(), deliver_span(), deliver_place(), deliver_hold(),
//    the anchor, the verification, the servo tick and the holdover/relock
//    transitions are line-for-line copies of firmware/DSPi/alink_input.c
//    (marked "mirrors"), and the main loop's ALINK prefill handshake of main.c.
// What is modelled:
//  - the line: sender bit k arrives at k / baud on the sender's clock (an
//    ideal line; LK2's PIO model shows the RX programs deliver exactly one bit
//    per line bit at any offset inside +-1000 ppm, and the "pio" check below
//    measures the timing on LK2's model);
//  - each receiver's crystal (ppm), boot time, main-loop pass jitter, the
//    output rings (512 frames, word-granular DMA read position), the soft VCXO
//    (0.11 ppm steps, 0.5 ppm per tick slew) and the fill servo of mode 0.
//
// Output: per scenario, the frame each receiver plays at the same instants,
// the skew between them, each one's latency against D, and the time between
// the two receivers' CLICK samples (the bench's test (e) on the model).
//
//   tools/alink_phase_sim/run.sh           (all scenarios; exit 1 on a failure)

#include "alink_codec.h"
#include "alink_fast.h"
#include "alink_phase.h"
#include "alink_ring_clock.h"
#include "sender_alink.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures, checks;
#define CHECK(c, ...)                                                                                                \
    do {                                                                                                             \
        checks++;                                                                                                    \
        if (!(c)) {                                                                                                  \
            failures++;                                                                                              \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                                                            \
            printf(__VA_ARGS__);                                                                                     \
            printf("\n");                                                                                            \
        }                                                                                                            \
    } while (0)

static FILE *dump_f;
static const char *wav_path;
static double dump_from, dump_to;
static uint64_t rs = 0x9E3779B97F4A7C15ull;
static double urand(void) {
    rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17;
    return (double)(rs >> 11) * (1.0 / 9007199254740992.0);
}

static int32_t sample_of(uint64_t f, unsigned ch) {
    uint32_t h = (uint32_t)f * 0x9E3779B1u ^ (ch + 1u) * 0x85EBCA77u;
    h ^= h >> 15; h *= 0x2C1B3C6Du; h ^= h >> 12;
    return (int32_t)(h << 8) >> 8;
}

// ============================================================================
// Scenario
// ============================================================================

#define F0          1000000ull      // the sender's first frame
#define NRX         2

typedef struct {
    const char *name;
    unsigned cls, rate, c;
    double sender_ppm;
    double rx_ppm[NRX];
    double boot_s[NRX];             // receiver power-up (absolute seconds)
    double loss_from[NRX], loss_to[NRX];   // dark line on that receiver's input (absolute s)
    uint8_t mode;                   // EXT_ALINK_PHASE.mode on both
    uint16_t d_local;
    double duration_s;
    double click_at_s;              // sender schedules CLICK (superframe) at about this time
    double unit_ops_at_s;           // per-unit ops on receiver 0 (EXT_APPLY_AT_FRAME path), 0 = none
    double expect_skew;             // max |skew| (frames) once both play phase-locked; < 0: expect a mismatch
    double restart_at_s;            // review lk6 probe: the sender power-cycles at this time (0 none)...
    uint64_t restart_f0;            // ... and its frame counter starts again here
    double hold_drift_ppm;          // the held VCXO drifts by this much in a holdover (temperature model)
} scenario_t;

// The 1 ms latency option at the stream's rate: ring target 48 frames at
// 48 kHz, batch 16, block cap 17; the rings' writable span 448 frames.
typedef struct { uint32_t target, batch, cap, span; } latency_t;
static latency_t latency_for(unsigned fs) {
    latency_t l;
    l.target = (uint32_t)((1000ull * fs + 500000u) / 1000000u);
    l.batch = (uint32_t)((333ull * fs + 999999u) / 1000000u);
    l.cap = l.batch + 1u;
    l.span = 512u - 64u;
    return l;
}

static unsigned rate_hz(unsigned code) { return code == 0 ? 44100u : code == 1 ? 48000u : 96000u; }

// ============================================================================
// The sender's stream, regenerated per receiver (the same bits each time)
// ============================================================================

#define SND_WORDS   (1u << 20)

typedef struct {
    alink_tx_t tx;
    snd_sink_t sink;
    uint32_t *ring;
    bool click_scheduled;
    uint64_t click_frame;
    bool restarted;
    uint64_t restart_bits;          // line bit where the restarted stream begins
} sender_t;

static snd_tables_t T;

static void sender_init(sender_t *s, const scenario_t *sc) {
    memset(s, 0, sizeof(*s));
    s->ring = calloc(SND_WORDS, 4);
    alink_tx_init(&s->tx, sc->cls, sc->rate, sc->c, F0, 0x5A);
    s->tx.flags = ALINK_SF_FLAG_USB_LIVE;
    snd_sink_init(&s->sink, s->ring, SND_WORDS, NULL);
}

// Encode until `words` whole words exist; the CLICK is scheduled once, 2400
// frames ahead of the frame being encoded at click_at_s (sender time).
static void sender_fill(sender_t *s, const scenario_t *sc, uint64_t words) {
    while (s->sink.words < words) {
        if (sc->restart_at_s > 0 && !s->restarted &&
            (double)snd_sink_bits(&s->sink) / ((double)alink_class_baud(sc->cls) * (1.0 + sc->sender_ppm * 1e-6)) >=
                sc->restart_at_s) {
            // A power cycle: the encoder starts over (its own disparity, F, superframe).
            alink_tx_init(&s->tx, sc->cls, sc->rate, sc->c, sc->restart_f0, 0x5A);
            s->tx.flags = ALINK_SF_FLAG_USB_LIVE;
            s->restarted = true;
            s->restart_bits = snd_sink_bits(&s->sink);
            s->click_scheduled = sc->click_at_s <= sc->restart_at_s;   // a CLICK only on one stream
        }
        if (sc->click_at_s > 0 && !s->click_scheduled &&
            (double)snd_sink_bits(&s->sink) / ((double)alink_class_baud(sc->cls) * (1.0 + sc->sender_ppm * 1e-6)) >=
                sc->click_at_s) {
            uint64_t at = s->tx.frame + 2400u;
            if (alink_tx_schedule(&s->tx, at, ALINK_OP_CLICK, 0, 0)) {
                s->click_scheduled = true;
                s->click_frame = at;
            }
        }
        int32_t smp[64];
        for (unsigned ch = 0; ch < s->tx.channels; ch++) smp[ch] = sample_of(s->tx.frame, ch);
        snd_frame(&s->tx, &T, smp, &s->sink, NULL);
    }
}

// ============================================================================
// One receiver: the firmware's state, the modelled hardware
// ============================================================================

#define ALINK_RING_WORDS    8192u
#define ALINK_RING_BITS      (ALINK_RING_WORDS * 32u)
#define ALINK_FIFO_ROWS     256u
#define SEARCH_BUDGET       (ALINK_RING_BITS / 4u)
#define RING_TAGS           4096u           // output ring history (frames), power of two
#define TAG_SILENCE         (-1ll)
#define ANCHOR_SPIN_US      1100u
#define MUTE_HOLD_MS        5u
#define SERVO_TICK_US       75000u
#define PROBE_US            1000u
// WP USB3 (alink_input.c, input_servo.h): the muted pull-in slew.
#define PULL_IN_FAST_SLEW_PPM          2.0f
#define PULL_IN_FAST_MIN_PPM           4.0f
#define PULL_IN_SETTLE_US              300000u
#define INPUT_SERVO_SLEW_PPM_PER_TICK  0.5f

enum { PH_SEARCH, PH_LOCKED, PH_RELOCK };

typedef struct {
    const scenario_t *sc;
    int id;
    sender_t snd;
    double baud_s;                  // the sender's line rate (absolute)
    unsigned fs;
    latency_t lat;
    // DMA ring
    uint32_t ring[ALINK_RING_WORDS];
    uint64_t w0;                    // first sender word this receiver's ring captured
    uint64_t copied;                // sender words copied so far
    // Firmware state (alink_input.c)
    alink_fast_t lf;
    int phase;
    uint32_t detected_rate;
    bool loss_latch;
    int32_t fifo[ALINK_FIFO_ROWS][8];
    uint32_t fifo_head, fifo_tail;
    uint64_t pub_bits, pub_f, pub_head_f;
    uint32_t pub_p, pub_head;
    bool pub_locked;
    alink_phase_t ph;
    alink_apply_q_t aq;
    uint32_t drop_rows;
    uint64_t hold_f, hold_us;
    alink_meas_t meas;
    uint64_t servo_last_us;
    float last_cmd, fill_smooth, fill_int, last_ff;
    bool pull_in;
    uint64_t fast_settle_us;        // WP USB3: no anchor before this (0 none)
    // Main loop (main.c ALINK block)
    bool preset_loading, prefilling;
    // Output side
    bool running;                   // outputs enabled
    double out_phase;               // frames played since the ring reset (fractional)
    double vcxo_ppm;
    uint32_t wr;                    // frames written since the ring reset
    uint32_t last_prewrite;
    int64_t tag[RING_TAGS];         // alink frame per ring frame, TAG_SILENCE for silence
    uint8_t click[RING_TAGS];
    int8_t click_out;               // pipeline_click_output
    double soft_mute_until;         // local us
    bool user_mute, stream_mute;
    bool relocked;
    // Events and checks
    uint64_t now_us;                // local timer
    double t;                       // absolute seconds
    unsigned anchors, reanchors, holdovers, resumes, corrections, drops, underruns;
    double t_resume, e_resume;
    uint64_t vol_target;
    unsigned ev_applied, ev_late, ev_cancel, ev_refused;
    double click_play_t;            // absolute time the CLICK sample played
    uint64_t click_frame_seen;
    int64_t vol_frame;              // frame of the first sample processed after MASTER_VOLUME
    float master_db;
    unsigned unit_ops;
} rx_t;

static double local_to_abs(const rx_t *r, uint64_t us) {
    return r->sc->boot_s[r->id] + (double)us * 1e-6 / (1.0 + r->sc->rx_ppm[r->id] * 1e-6);
}

// ---- modelled pipeline ------------------------------------------------------

static uint32_t ring_fill(const rx_t *r) {
    uint32_t played = (uint32_t)r->out_phase;
    int32_t f = (int32_t)(r->wr - played);
    return f < 0 ? 0u : (uint32_t)f;
}

// slot_ring_play_position(): the reader's DMA position to the word (4 per frame).
static float ring_fill_exact(const rx_t *r) {
    double q = floor(r->out_phase * 4.0) / 4.0;
    return (float)((double)r->wr - q);
}

static uint32_t producer_block_bounds(const rx_t *r, uint32_t *min_frames) {
    uint32_t max = r->lat.cap;
    if (!r->running) {
        uint32_t fill = ring_fill(r);
        uint32_t target = alink_phase_prefill(&r->ph, r->lat.target, r->lat.cap, r->lat.span);
        uint32_t room = fill < target ? target - fill : 0;
        if (room < max) max = room;
    }
    *min_frames = r->lat.batch < max ? r->lat.batch : max;
    return max;
}

// process_input_block(): the block goes to the rings (group admission, the
// recovery skip after an underrun), CLICK on its first sample.
static void process_block(rx_t *r, uint32_t n, const int64_t *frames) {
    uint32_t played = (uint32_t)r->out_phase;
    if (r->running && (int32_t)(played - r->wr) > 0) {
        r->underruns++;
        // Ring ran dry: the pipeline's recovery skip lands the block at the
        // pre-write fill it had (audio_pipeline.c output_rings_pre_give).
        uint32_t skip = (played - r->wr) + r->last_prewrite;
        for (uint32_t i = 0; i < skip; i++) {
            r->tag[(r->wr + i) & (RING_TAGS - 1u)] = TAG_SILENCE;
            r->click[(r->wr + i) & (RING_TAGS - 1u)] = 0;
        }
        r->wr += skip;
    }
    if (ring_fill(r) + n > r->lat.span) {
        r->drops++;
        return;     // refused by group admission
    }
    r->last_prewrite = ring_fill(r);
    for (uint32_t i = 0; i < n; i++) {
        uint32_t k = (r->wr + i) & (RING_TAGS - 1u);
        r->tag[k] = frames ? frames[i] : TAG_SILENCE;
        r->click[k] = 0;
    }
    if (r->click_out >= 0 && n) r->click[r->wr & (RING_TAGS - 1u)] = 1;
    if (r->vol_frame == -2 && n && frames) r->vol_frame = frames[0];
    r->wr += n;
}

static void drain_and_disable(rx_t *r) {
    r->running = false;
    r->out_phase = 0;
    r->wr = 0;
    r->last_prewrite = 0;
}

static void enable_outputs(rx_t *r) {
    r->running = true;
    r->out_phase = 0;
}

static void soft_mute(rx_t *r) {
    r->soft_mute_until = (double)r->now_us + MUTE_HOLD_MS * 1000.0;
}

static bool audible(const rx_t *r) {
    return r->running && !r->preset_loading && (double)r->now_us >= r->soft_mute_until;
}

// ---- mirrors alink_input.c ----------------------------------------------------

static void publish(rx_t *r, uint64_t bits) {
    r->pub_bits = bits;
    r->pub_locked = r->lf.state == ALINK_RX_LOCKED;
    if (r->pub_locked) {
        r->pub_f = r->lf.frame;
        r->pub_p = r->lf.p;
        r->pub_head = r->fifo_head;
        r->pub_head_f = r->lf.frame + (r->lf.body_done ? 1u : 0u);
    }
}

// Sender words that have arrived by absolute time t, into the RX ring.
static uint64_t ring_writer_bits(rx_t *r) {
    uint64_t words = (uint64_t)floor(r->t * r->baud_s / 32.0);
    if (words < r->w0) return 0;
    sender_fill(&r->snd, r->sc, words + 1);
    double t_lost0 = r->sc->loss_from[r->id], t_lost1 = r->sc->loss_to[r->id];
    for (; r->copied < words; r->copied++) {
        uint32_t w = r->snd.ring[r->copied & (SND_WORDS - 1u)];
        double ta = (double)(r->copied * 32u + 31u) / r->baud_s;
        if (t_lost1 > t_lost0 && ta >= t_lost0 && ta < t_lost1) w = 0;    // dark line
        r->ring[(r->copied - r->w0) & (ALINK_RING_WORDS - 1u)] = w;
        // ALINK_PHASE_SIM_DUMP=file:from_s:to_s writes receiver B's input words
        // in that window (raw u32, gen_stream's format) for LK5's M33 emulator.
        if (dump_f && r->id == 1 && ta >= dump_from && ta < dump_to) fwrite(&w, 4, 1, dump_f);
    }
    return (words - r->w0) * 32u;
}

// arrival_since(): frames arriving now relative to frame fx.
static bool arrival_since(rx_t *r, uint64_t fx, float *since) {
    if (!r->pub_locked || !r->detected_rate) return false;
    uint64_t words = (uint64_t)floor(r->t * r->baud_s / 32.0);
    uint64_t now_bits = words >= r->w0 ? (words - r->w0) * 32u : 0;
    *since = alink_phase_since(r->pub_f, fx, (int32_t)((uint32_t)now_bits - r->pub_p),
                              (float)alink_class_baud(r->lf.cls) / (float)r->detected_rate);
    return true;
}

static void decode_service(rx_t *r) {
    uint64_t bits = ring_writer_bits(r);
    if (r->phase != PH_LOCKED) {
        alink_fast_run(&r->lf, bits, SEARCH_BUDGET);
    } else {
        uint32_t head = r->fifo_head;
        for (int piece = 0; piece < 2; piece++) {
            uint32_t space = ALINK_FIFO_ROWS - (head - r->fifo_tail);
            if (!space) break;
            uint32_t idx = head & (ALINK_FIFO_ROWS - 1u), n = ALINK_FIFO_ROWS - idx;
            if (n > space) n = space;
            alink_fast_set_output(&r->lf, &r->fifo[idx], (uint16_t)n);
            alink_fast_run(&r->lf, bits, SEARCH_BUDGET);
            head += r->lf.out_n;
            if (r->lf.out_n < n) break;
        }
        alink_fast_set_output(&r->lf, NULL, 0);
        r->fifo_head = head;
    }
    publish(r, bits);
}

static void apply_report(rx_t *r, uint8_t op, uint8_t result, uint64_t frame) {
    alink_apply_done(&r->aq, op, result, frame);
    if (result == ALINK_APPLY_APPLIED) r->ev_applied++;
    else if (result == ALINK_APPLY_LATE) r->ev_late++;
    else if (result == ALINK_APPLY_CANCELLED) r->ev_cancel++;
    else r->ev_refused++;
}

static uint8_t apply_exec(rx_t *r, const alink_apply_op_t *op) {
    switch (op->op) {
    case ALINK_OP_MUTE: r->user_mute = true; return ALINK_APPLY_APPLIED;
    case ALINK_OP_UNMUTE: r->user_mute = false; return ALINK_APPLY_APPLIED;
    case ALINK_OP_MASTER_VOLUME: r->master_db = op->value * 0.01f; r->vol_frame = -2; return ALINK_APPLY_APPLIED;
    case ALINK_OP_CLICK:
        if (op->arg >= 8 || r->user_mute || r->stream_mute) return ALINK_APPLY_REFUSED;
        r->click_out = (int8_t)op->arg;
        return ALINK_APPLY_APPLIED;
    case ALINK_OP_PRESET_LOAD:
    case ALINK_OP_OUT_DELAY: return ALINK_APPLY_APPLIED;
    default: return ALINK_APPLY_REFUSED;
    }
}

static void apply_run(rx_t *r, bool late) {
    alink_apply_op_t op;
    alink_apply_pop(&r->aq, &op);
    uint8_t res = apply_exec(r, &op);
    if (late && res == ALINK_APPLY_APPLIED) res = ALINK_APPLY_LATE;
    apply_report(r, op.op, res, op.frame);
}

static void deliver_rows(rx_t *r, uint32_t tail, uint32_t n, uint64_t f) {
    int64_t fr[192];
    for (uint32_t i = 0; i < n; i++) fr[i] = (int64_t)(f + i);
    r->fifo_tail = tail + n;
    process_block(r, n, fr);
    r->click_out = -1;
    alink_phase_written(&r->ph, f, n);
}

static void silence_block(rx_t *r, uint32_t n) {
    process_block(r, n, NULL);
    r->click_out = -1;
    alink_phase_written(&r->ph, r->ph.next_w, n);
}

static uint32_t deliver_span(rx_t *r, bool rows, uint32_t tail, uint64_t f, uint32_t n) {
    uint32_t done = 0;
    while (done < n) {
        uint64_t at = alink_apply_next(&r->aq), here = f + done;
        if (at <= here) {
            apply_run(r, at < here);
            continue;
        }
        uint32_t k = n - done;
        if (at - here < k) k = (uint32_t)(at - here);
        if (rows) deliver_rows(r, tail + done, k, here);
        else silence_block(r, k);
        done += k;
    }
    return done;
}

static void reanchor(rx_t *r) {
    r->drop_rows = 0;
    r->ph.resync = false;
    r->ph.hold = false;
    r->ph.started = false;
    r->loss_latch = true;
    r->reanchors++;
}

static uint32_t deliver_hold(rx_t *r, uint32_t head) {
    r->fifo_tail = head;
    uint32_t min_frames, max_frames = producer_block_bounds(r, &min_frames);
    uint32_t n = alink_phase_hold_frames(&r->ph, ring_fill(r), max_frames);
    return n ? deliver_span(r, false, 0, r->ph.next_w, n) : 0;
}

static bool deliver_place(rx_t *r, uint32_t *tail, uint32_t *avail, uint64_t *f) {
    uint32_t k;
    if (r->drop_rows) {
        k = r->drop_rows < *avail ? r->drop_rows : *avail;
        r->fifo_tail = *tail + k;
        alink_phase_written(&r->ph, *f, k);
        *tail += k;
        *avail -= k;
        *f += k;
        r->drop_rows -= k;
        if (r->drop_rows) return false;
    }
    if (!r->ph.resync) return true;
    int64_t gap = alink_phase_gap(&r->ph, *f);
    if (!alink_phase_gap_ok(gap, r->lat.span)) {
        reanchor(r);
        return false;
    }
    if (gap < 0) {
        k = (uint64_t)-gap < *avail ? (uint32_t)-gap : *avail;
        r->fifo_tail = *tail + k;
        *tail += k;
        *avail -= k;
        *f += k;
        if ((int64_t)k < -gap) return false;
    } else if (gap > 0) {
        uint32_t min_frames, max_frames = producer_block_bounds(r, &min_frames);
        k = (uint64_t)gap < max_frames ? (uint32_t)gap : max_frames;
        if (k) deliver_span(r, false, 0, r->ph.next_w, k);
        return false;
    }
    r->ph.resync = false;
    return true;
}

static uint32_t deliver(rx_t *r) {
    uint32_t head = r->pub_head;
    uint64_t head_f = r->pub_head_f;
    if (r->ph.hold) return deliver_hold(r, head);
    uint32_t tail = r->fifo_tail, avail = head - tail;
    if (r->phase != PH_LOCKED || r->detected_rate != r->fs || (r->pull_in && r->preset_loading)) {
        r->fifo_tail = head;
        return 0;
    }
    uint64_t f = head_f - avail;
    if ((r->ph.resync || r->drop_rows) && !deliver_place(r, &tail, &avail, &f)) return 0;
    uint32_t min_frames, max_frames = producer_block_bounds(r, &min_frames);
    if (max_frames == 0 || avail < min_frames || avail == 0) return 0;
    uint32_t n = avail < max_frames ? avail : max_frames;
    if (alink_apply_next(&r->aq) < f + n) return deliver_span(r, true, tail, f, n);
    deliver_rows(r, tail, n, f);
    return n;
}

static bool phase_error(rx_t *r, float *e) {
    if (!r->ph.timeline || r->ph.hold || r->ph.resync) return false;
    float since;
    if (!r->running || !arrival_since(r, r->ph.next_w, &since)) return false;
    *e = alink_phase_error(&r->ph, since, ring_fill_exact(r));
    return true;
}

static int trace;
static void phase_act(rx_t *r, int act, int32_t frames) {
    if (act == ALINK_PHASE_ACT_RESUME) {
        r->t_resume = r->t;
        r->e_resume = r->ph.e;
    }
    if (trace && act)
        printf("    t %.6f act %d frames %d e %+.3f state %u fill %.2f wr %u out %.2f next_w %" PRIu64 " fifo %u\n", r->t, act,
               (int)frames, r->ph.e, r->ph.state, ring_fill_exact(r), r->wr, r->out_phase, r->ph.next_w,
               r->pub_head - r->fifo_tail);
    if (act == ALINK_PHASE_ACT_REANCHOR) {
        reanchor(r);
    } else if (act == ALINK_PHASE_ACT_CORRECT) {
        r->corrections++;
        if (frames > 0) {
            r->drop_rows += (uint32_t)frames;
        } else {
            uint32_t k = (uint32_t)-frames;
            while (k) {
                uint32_t n = k < r->lat.cap ? k : r->lat.cap;
                silence_block(r, n);
                r->ph.next_w -= n;
                k -= n;
            }
        }
    } else if (act == ALINK_PHASE_ACT_RESUME) {
        r->resumes++;
    }
}

static void phase_verify(rx_t *r) {
    float e;
    if (!phase_error(r, &e)) return;
    int32_t k = 0;
    int act = alink_phase_check(&r->ph, e, true, &k);   // sets k: not in the same call's arguments
    phase_act(r, act, k);
}

// input_servo_apply_trim() / input_servo_apply(): feed-forward plus a trim,
// slew-limited; the soft VCXO moves in 0.11 ppm steps.
static void servo_actuate(rx_t *r, float hz, float trim, float slew) {
    float ff = (hz / (float)r->fs - 1.0f) * 1e6f;
    if (ff > 2000.0f || ff < -2000.0f) return;
    r->last_ff = ff;
    float cmd = ff + trim;
    if (cmd > r->last_cmd + slew) cmd = r->last_cmd + slew;
    if (cmd < r->last_cmd - slew) cmd = r->last_cmd - slew;
    r->last_cmd = cmd;
    double q = cmd / 0.11;
    r->vcxo_ppm = 0.11 * (double)(int64_t)(q < 0 ? q - 0.5 : q + 0.5);
}

static bool phase_servo_tick(rx_t *r, float hz) {
    if (!alink_phase_active(&r->ph)) return false;
    float e, trim = 0.0f;
    if (r->ph.started && phase_error(r, &e)) {
        int32_t k = 0;
        int act = ALINK_PHASE_ACT_NONE;
        if (r->ph.state != ALINK_PHASE_ANCHORING) {
            act = alink_phase_check(&r->ph, e, true, &k);
            phase_act(r, act, k);
        }
        if (act == ALINK_PHASE_ACT_REANCHOR) alink_phase_pi_reset(&r->ph.pi);
        else if (r->ph.state == ALINK_PHASE_LOCKED || r->ph.state == ALINK_PHASE_PENDING) trim = alink_phase_servo(&r->ph, e);
    }
    // Pull-in before the anchor with the output silent: nothing is audible,
    // so the VCXO may move faster (WP USB3); the normal slew everywhere else.
    float v0 = (float)r->vcxo_ppm, off = r->last_ff - v0;
    bool fast = r->pull_in && !r->ph.started && r->preset_loading &&
                (off > PULL_IN_FAST_MIN_PPM || off < -PULL_IN_FAST_MIN_PPM);
    servo_actuate(r, hz, trim, fast ? PULL_IN_FAST_SLEW_PPM : INPUT_SERVO_SLEW_PPM_PER_TICK);
    float dv = (float)r->vcxo_ppm - v0;
    if (dv > 1.0f || dv < -1.0f) r->fast_settle_us = (r->now_us + PULL_IN_SETTLE_US) | 1u;
    return true;
}

// Mode 0: the fill servo (input_servo.c Loop B, in frames).
static void fill_servo(rx_t *r, float hz) {
    float target = (float)r->lat.target / 48.0f;
    float fill = (float)ring_fill(r) / 48.0f;
    r->fill_smooth += 0.125f * (fill - r->fill_smooth);
    float err = r->fill_smooth - target;
    float p = err * 2.0f;
    if (err > 2.0f) p += (err - 2.0f) * 4.0f;
    if (err < -2.0f) p += (err + 2.0f) * 4.0f;
    r->fill_int += err * 5.0e-5f;
    if (r->fill_int > 3.0f) r->fill_int = 3.0f;
    if (r->fill_int < -3.0f) r->fill_int = -3.0f;
    servo_actuate(r, hz, p + r->fill_int, INPUT_SERVO_SLEW_PPM_PER_TICK);
}

static void update_clock_servo(rx_t *r) {
    if (r->phase != PH_LOCKED || r->detected_rate != r->fs) return;
    float hz = alink_meas_hz(&r->meas);
    if (hz == 0.0f) return;
    if (r->now_us - r->servo_last_us < SERVO_TICK_US) return;
    r->servo_last_us = r->now_us;
    if (r->relocked && !r->meas.long_valid) return;
    r->relocked = false;
    if (phase_servo_tick(r, hz)) return;
    fill_servo(r, hz);
}

static void enter_locked(rx_t *r) {
    r->phase = PH_LOCKED;
    uint32_t was = r->detected_rate;
    r->detected_rate = rate_hz(r->lf.rate);
    r->fifo_tail = r->fifo_head;
    alink_meas_reset(&r->meas);
    if (r->ph.hold) {
        uint64_t f_expect = r->hold_f + (r->now_us - r->hold_us) * (uint64_t)r->detected_rate / 1000000u;
        bool same_rate = r->detected_rate == was && r->detected_rate == r->fs;
        r->relocked = same_rate;
        if (same_rate && alink_phase_relock(&r->ph, r->lf.frame, f_expect, r->lat.span)) {
        } else {
            r->ph.hold = false;
            r->ph.started = false;
            r->loss_latch = true;
            r->reanchors++;
        }
    }
}

static bool enter_holdover(rx_t *r) {
    uint32_t fill = ring_fill(r);
    if (!alink_phase_active(&r->ph) || !alink_phase_loss(&r->ph, fill)) return false;
    r->hold_f = r->ph.next_w - fill + (uint64_t)(int64_t)alink_phase_delay(&r->ph);
    r->hold_us = r->now_us;
    r->phase = PH_RELOCK;
    r->holdovers++;
    return true;
}

static void alink_up_step(rx_t *r) {
    if (r->phase == PH_LOCKED) {
        if (r->lf.state != ALINK_RX_LOCKED) {
            if (!enter_holdover(r)) {
                r->pull_in = false;
                r->relocked = false;
                r->phase = PH_RELOCK;
                r->loss_latch = true;
                r->detected_rate = 0;
            }
        }
    } else if (r->lf.state == ALINK_RX_LOCKED) {
        enter_locked(r);
    }
}

// alink_input_anchor_ready(): spin for the last few frames.
static bool anchor_ready(rx_t *r) {
    if (!alink_phase_active(&r->ph) || !r->ph.timeline) return true;
    float since;
    if (!arrival_since(r, r->ph.first, &since)) return false;
    float wait = alink_phase_anchor_wait(&r->ph, since);
    if (wait > (float)(r->detected_rate / 1000u)) return false;
    uint64_t t0 = r->now_us;
    while (wait > 0.0f && r->now_us - t0 < ANCHOR_SPIN_US) {
        r->now_us += 1;             // one spin iteration: about a microsecond
        r->t = local_to_abs(r, r->now_us);
        if (!arrival_since(r, r->ph.first, &since)) return false;
        wait = alink_phase_anchor_wait(&r->ph, since);
    }
    return true;
}

// alink_input_poll() and the main loop's ALINK block, once per pass.
static void poll(rx_t *r) {
    decode_service(r);
    alink_up_step(r);
    if (r->lf.state == ALINK_RX_LOCKED && r->phase == PH_LOCKED) {
        if (alink_apply_take_sf(&r->aq, &(alink_apply_limits_t){ 10, 9, 2048 }, &r->lf.sf) == 2)
            apply_report(r, r->lf.sf.apply_op, ALINK_APPLY_REFUSED, r->lf.sf.apply_frame);
    }
    if (r->phase == PH_LOCKED && r->detected_rate)
        alink_meas_sample(&r->meas, r->now_us, r->pub_bits,
                         1e6f * (float)r->detected_rate / (float)alink_class_baud(r->lf.cls));
    deliver(r);
    if (alink_phase_active(&r->ph)) {
        if (r->ph.started && r->ph.state == ALINK_PHASE_ANCHORING && !r->ph.hold && !r->ph.resync &&
            !r->drop_rows && r->phase == PH_LOCKED)
            phase_verify(r);
        if (r->ph.hold || r->ph.state == ALINK_PHASE_ANCHORING) soft_mute(r);
    }
    // main.c: the ALINK prefill handshake.
    bool locked = r->phase == PH_LOCKED && !r->ph.hold;
    if (r->loss_latch) {
        r->loss_latch = false;
        r->prefilling = false;
        r->preset_loading = true;       // prepare_pipeline_reset(): mute
    }
    bool rate_ready = true;         // alink_input_rate_ready()
    if (locked && r->preset_loading && !r->prefilling) {
        r->pull_in = false;
        if (alink_phase_active(&r->ph) && !(r->relocked && !r->meas.long_valid)) {
            r->pull_in = !(r->meas.long_valid && r->last_ff != 0.0f && fabs(r->vcxo_ppm - r->last_ff) < 1.0);
            rate_ready = !r->pull_in;
            // WP USB3: after a fast pull-in step the receivers settle first
            // (still pulling in: rows dropped).
            if (rate_ready && r->fast_settle_us) {
                if ((int64_t)(r->now_us - r->fast_settle_us) < 0) r->pull_in = true, rate_ready = false;
                else r->fast_settle_us = 0;
            }
        }
    }
    if (locked && r->preset_loading && !r->prefilling && rate_ready) {
        drain_and_disable(r);
        r->preset_loading = false;
        r->prefilling = true;
        r->fifo_tail = r->fifo_head;    // alink_input_discard_queued()
        alink_phase_drain(&r->ph);       // alink_input_on_drain()
        r->drop_rows = 0;
        r->pull_in = false;
        if (alink_phase_active(&r->ph)) r->ph.state = ALINK_PHASE_ANCHORING;
    }
    uint32_t target = alink_phase_prefill(&r->ph, r->lat.target, r->lat.cap, r->lat.span);
    if (r->prefilling && locked && ring_fill(r) >= target && anchor_ready(r)) {
        enable_outputs(r);
        alink_phase_started(&r->ph);     // alink_input_on_enabled()
        if (trace && r->id == 0) {
            float since = 0;
            arrival_since(r, r->ph.first, &since);
            printf("    t %.6f anchor: first %" PRIu64 " next_w %" PRIu64 " wr %u since(first) %.2f fifo %u\n", r->t,
                   r->ph.first, r->ph.next_w, r->wr, since, r->pub_head - r->fifo_tail);
        }
        r->prefilling = false;
        r->anchors++;
    }
    update_clock_servo(r);
}

// ============================================================================
// Running a receiver
// ============================================================================

typedef struct {
    double *fplay;                  // frame playing at each probe (fractional), NAN when none
    uint8_t *aud;                   // audible and phase-locked at that probe
    double *lat;                    // latency in frames against the arrival (comma end)
    unsigned n;
} probe_log_t;

static void rx_init(rx_t *r, const scenario_t *sc, int id) {
    memset(r, 0, sizeof(*r));
    r->sc = sc;
    r->id = id;
    sender_init(&r->snd, sc);
    r->baud_s = (double)alink_class_baud(sc->cls) * (1.0 + sc->sender_ppm * 1e-6);
    r->fs = rate_hz(sc->rate);
    r->lat = latency_for(r->fs);
    r->w0 = (uint64_t)ceil(sc->boot_s[id] * r->baud_s / 32.0);
    r->copied = r->w0;
    alink_fast_init(&r->lf, r->ring, ALINK_RING_WORDS, sc->cls, 1, NULL, NULL, 0);
    static const uint8_t ident[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    alink_fast_set_slots(&r->lf, ident);
    alink_phase_init(&r->ph);
    alink_phase_limits(&r->ph, r->lat.target, r->lat.cap, r->lat.span);
    uint8_t p[8] = { sc->mode, 0, (uint8_t)sc->d_local, (uint8_t)(sc->d_local >> 8), 0, 0, 0, 0 };
    if (alink_phase_set(&r->ph, p) < 0) printf(" (phase write refused)\n");
    alink_apply_init(&r->aq);
    r->phase = PH_SEARCH;
    r->preset_loading = true;
    r->click_out = -1;
    r->vol_frame = -1;
    r->click_play_t = -1;
    for (unsigned i = 0; i < RING_TAGS; i++) r->tag[i] = TAG_SILENCE;
}

// The frame playing at the receiver's output now (fractional): the ring's
// tag at the read position plus the fraction of that frame played.
static double frame_playing(const rx_t *r) {
    if (!r->running) return NAN;
    uint32_t o = (uint32_t)r->out_phase;
    if ((int32_t)(o - r->wr) >= 0) return NAN;
    int64_t f = r->tag[o & (RING_TAGS - 1u)];
    if (f < 0) return NAN;
    return (double)f + (r->out_phase - floor(r->out_phase));
}

// The frame arriving now, by the comma end (absolute; the reference the
// receiver's e uses, before its DMA word quantisation).
static double frame_arriving(const rx_t *r) {
    double bits = r->t * r->baud_s;
    double bpf = (double)alink_class_baud(r->sc->cls) / (double)r->fs;
    if (r->snd.restarted && bits >= (double)r->snd.restart_bits)
        return (double)r->sc->restart_f0 + (bits - (double)r->snd.restart_bits - 10.0) / bpf;
    return (double)F0 + (bits - 10.0) / bpf;
}

static void run_rx(rx_t *r, probe_log_t *log) {
    const scenario_t *sc = r->sc;
    unsigned nprobe = (unsigned)(sc->duration_s * 1e6 / PROBE_US) + 1;
    log->fplay = calloc(nprobe, sizeof(double));
    log->aud = calloc(nprobe, 1);
    log->lat = calloc(nprobe, sizeof(double));
    log->n = nprobe;
    for (unsigned i = 0; i < nprobe; i++) log->fplay[i] = log->lat[i] = NAN;
    double t_prev = sc->boot_s[r->id];
    unsigned next_probe = (unsigned)ceil(sc->boot_s[r->id] * 1e6 / PROBE_US);
    bool unit_ops_done = false;
    r->now_us = 1;
    while (1) {
        // Main-loop pass: 40-250 us (a quiet pass is about 75 us, a block of
        // 17 frames through the DSP a few hundred), now and then 800 us.
        double pass = 40.0 + urand() * 210.0;
        if (urand() < 0.005) pass = 800.0;
        uint64_t next_us = r->now_us + (uint64_t)pass;
        double t_next = local_to_abs(r, next_us);
        if (t_next > sc->duration_s) break;
        // Advance the outputs to t_next, probing on the way.
        while (next_probe < nprobe && (double)next_probe * PROBE_US * 1e-6 <= t_next) {
            double tp = (double)next_probe * PROBE_US * 1e-6;
            double rate = r->fs * (1.0 + sc->rx_ppm[r->id] * 1e-6) * (1.0 + r->vcxo_ppm * 1e-6);
            if (r->running) r->out_phase += rate * (tp - t_prev);
            t_prev = tp;
            r->t = tp;
            double fp = frame_playing(r);
            log->fplay[next_probe] = fp;
            log->aud[next_probe] = audible(r) && alink_phase_active(&r->ph) ? (r->ph.state == ALINK_PHASE_LOCKED) : 0;
            if (sc->mode == 0) log->aud[next_probe] = audible(r) && !isnan(fp);
            if (!isnan(fp)) log->lat[next_probe] = frame_arriving(r) - fp;
            next_probe++;
        }
        double rate = r->fs * (1.0 + sc->rx_ppm[r->id] * 1e-6) *
                      (1.0 + (r->vcxo_ppm + (r->ph.hold ? sc->hold_drift_ppm : 0.0)) * 1e-6);
        double before = r->out_phase;
        if (r->running) r->out_phase += rate * (t_next - t_prev);
        // The CLICK sample's play time, by interpolation inside this pass.
        if (r->running && r->click_play_t < 0) {
            for (uint32_t o = (uint32_t)ceil(before); (double)o <= r->out_phase && (int32_t)(o - r->wr) < 0; o++) {
                if (r->click[o & (RING_TAGS - 1u)] && r->snd.click_scheduled &&
                    r->tag[o & (RING_TAGS - 1u)] == (int64_t)r->snd.click_frame) {
                    r->click_play_t = t_prev + ((double)o - before) / rate;
                    r->click_frame_seen = (uint64_t)r->tag[o & (RING_TAGS - 1u)];
                    break;
                }
            }
        }
        t_prev = t_next;
        r->now_us = next_us;
        r->t = t_next;
        // Per-unit ops on receiver 0 (EXT_APPLY_AT_FRAME): MASTER_VOLUME 100
        // frames ahead (applied), a CLICK 200 frames in the past (late), two
        // MUTEs then CANCEL_ALL (cancelled).
        if (sc->unit_ops_at_s > 0 && !unit_ops_done && r->id == 0 && r->t >= sc->unit_ops_at_s &&
            r->phase == PH_LOCKED && !r->ph.hold) {
            uint64_t f = r->pub_f;
            alink_apply_op_t vol = { f + 100u, ALINK_OP_MASTER_VOLUME, 0, -600 };
            r->vol_target = f + 100u;
            alink_apply_op_t late = { f - 200u, ALINK_OP_CLICK, 1, 0 };
            alink_apply_push(&r->aq, &vol);
            alink_apply_push(&r->aq, &late);
            r->unit_ops = 2;
            r->ev_applied = r->ev_late = r->ev_cancel = 0;
            unit_ops_done = true;
        }
        poll(r);
        t_prev = r->t;      // the anchor's spin may have moved the clock (outputs stopped meanwhile)
        if (unit_ops_done && r->unit_ops == 2 && r->aq.n == 0) {
            alink_apply_op_t m1 = { r->pub_f + 100000u, ALINK_OP_MUTE, 0, 0 }, m2 = { r->pub_f + 100001u, ALINK_OP_UNMUTE, 0, 0 };
            alink_apply_push(&r->aq, &m1);
            alink_apply_push(&r->aq, &m2);
            alink_apply_op_t op;
            while (r->aq.n) {   // CANCEL_ALL (alink_input_apply())
                alink_apply_pop(&r->aq, &op);
                apply_report(r, op.op, ALINK_APPLY_CANCELLED, op.frame);
            }
            r->unit_ops = 3;
        }
    }
}

// ============================================================================
// Scenarios and checks
// ============================================================================

static void stats(const double *x, unsigned n, double *mean, double *sd, double *mx) {
    double s = 0, s2 = 0, m = 0;
    for (unsigned i = 0; i < n; i++) {
        s += x[i];
        s2 += x[i] * x[i];
        if (fabs(x[i]) > m) m = fabs(x[i]);
    }
    *mean = n ? s / n : 0;
    *sd = n > 1 ? sqrt(fmax(0.0, s2 / n - *mean * *mean)) : 0;
    *mx = m;
}

// ALINK_PHASE_SIM_WAV=file: the last 10 s of both outputs as one stereo
// recording (A left, B right, 24-bit, 48 kHz on the interface's own clock):
// each receiver's DAC plays a band-limited programme at the alink frame it is
// playing, so tools/alink/skew_xcorr.py measures the model's skew the way the
// bench measures the hardware's.
static void write_wav(const scenario_t *sc, const probe_log_t *log) {
    FILE *f = fopen(wav_path, "wb");
    if (!f) { printf("   cannot write %s\n", wav_path); return; }
    const unsigned rate = 48000u, secs = 10u, n = rate * secs;
    double fs = rate_hz(sc->rate), t0 = sc->duration_s - secs - 0.01;
    double fr[40], ph[40], am[40];
    uint64_t save = rs;
    for (int i = 0; i < 40; i++) { fr[i] = 40.0 + urand() * 11000.0; ph[i] = urand() * 6.283; am[i] = 0.2 + 0.8 * urand(); }
    rs = save;
    uint32_t data = n * 6u, riff = 36u + data, sr = rate, br = rate * 6u;
    fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f);
    uint32_t fmt = 16; uint16_t pcm = 1, ch = 2, align = 6, bits = 24;
    fwrite(&fmt, 4, 1, f); fwrite(&pcm, 2, 1, f); fwrite(&ch, 2, 1, f); fwrite(&sr, 4, 1, f); fwrite(&br, 4, 1, f);
    fwrite(&align, 2, 1, f); fwrite(&bits, 2, 1, f); fwrite("data", 1, 4, f); fwrite(&data, 4, 1, f);
    for (unsigned k = 0; k < n; k++) {
        double t = t0 + (double)k / rate;
        double pf = t * 1e6 / PROBE_US;
        unsigned p = (unsigned)pf;
        for (int c = 0; c < NRX; c++) {
            double v = 0.0;
            if (p + 1 < log[c].n && !isnan(log[c].fplay[p]) && !isnan(log[c].fplay[p + 1])) {
                double fp = log[c].fplay[p] + (log[c].fplay[p + 1] - log[c].fplay[p]) * (pf - p);
                double tt = fp / fs;
                for (int i = 0; i < 40; i++) v += 0.015 * am[i] * sin(6.283185307179586 * fr[i] * tt + ph[i]);
            }
            int32_t q = (int32_t)lrint(v * 8388607.0);
            uint8_t b3[3] = { (uint8_t)q, (uint8_t)(q >> 8), (uint8_t)(q >> 16) };
            fwrite(b3, 1, 3, f);
        }
    }
    fclose(f);
    printf("   wrote %s: the last 10 s of A (left) and B (right)\n", wav_path);
}

static void run_scenario(const scenario_t *sc) {
    printf("\n== %s\n", sc->name);
    // Every scenario from its own random sequence (WP USB3): a change in one
    // scenario, or a filtered run, no longer moves the others' figures.
    // ALINK_PHASE_SIM_SALT picks other sequences (default 7).  Known: the two
    // 60 s holdover scenarios' "resume without a re-anchor" and "latency
    // 9-10 s after" checks are marginal (WP USB3 report: they fail for 7 of 16
    // salts at bofr-main ebdf538 and 7 of 16 with WP USB3): after 60 s on a
    // held VCXO the resume can need a third whole-frame correction.
    rs = 0x9E3779B97F4A7C15ull;
    for (const char *c = sc->name; *c; c++) rs = (rs ^ (uint8_t)*c) * 0x100000001B3ull;
    const char *salt = getenv("ALINK_PHASE_SIM_SALT");
    rs ^= (salt ? strtoull(salt, NULL, 0) : 7u) * 0x9E3779B97F4A7C15ull;
    if (!rs) rs = 1;
    printf("   L%u %u Hz C %u, sender %+.0f ppm; rx A %+.0f ppm boot %.2f s, rx B %+.0f ppm boot %.2f s; mode %u, D %u\n",
           sc->cls, rate_hz(sc->rate), sc->c, sc->sender_ppm, sc->rx_ppm[0], sc->boot_s[0], sc->rx_ppm[1],
           sc->boot_s[1], sc->mode, sc->d_local);
    static rx_t r[NRX];
    probe_log_t log[NRX];
    for (int i = 0; i < NRX; i++) {
        rx_init(&r[i], sc, i);
        if (sc->loss_to[i] > sc->loss_from[i])
            printf("   rx %c: dark line %.3f..%.3f s (%.1f ms)\n", 'A' + i, sc->loss_from[i], sc->loss_to[i],
                   (sc->loss_to[i] - sc->loss_from[i]) * 1e3);
        run_rx(&r[i], &log[i]);
        printf("   rx %c: anchors %u, re-anchors %u, holdovers %u, verified %u, corrections %u, admission drops %u, "
               "underruns %u, VCXO %+.2f ppm, phase %s e %+.3f\n",
               'A' + i, r[i].anchors, r[i].reanchors, r[i].holdovers, r[i].resumes, r[i].corrections, r[i].drops,
               r[i].underruns,
               r[i].vcxo_ppm, r[i].ph.state == ALINK_PHASE_LOCKED ? "locked" : "not locked", r[i].ph.e);
        free(r[i].snd.ring);
        if (trace) printf("    rx %c: e_s %+.4f integ %+.3f trim %+.3f last_cmd %+.3f hz %.4f\n", 'A' + i, r[i].ph.pi.e_s,
                          r[i].ph.pi.integ, r[i].ph.pi.trim, r[i].last_cmd, alink_meas_hz(&r[i].meas));
    }
    // WP USB3: start-up time, boot to audible and phase-locked.
    for (int i = 0; i < NRX; i++) {
        unsigned j = 0;
        while (j < log[i].n && !log[i].aud[j]) j++;
        if (j < log[i].n)
            printf("   rx %c: audible and phase-locked %.2f s after boot\n", 'A' + i, j * PROBE_US * 1e-6 - sc->boot_s[i]);
    }
    // Skew: both audible and phase-locked at the same instant, the last 10 s.
    unsigned n = log[0].n, from = n > 10000 ? n - 10000 : 0;
    double *sk = malloc(n * sizeof(double)), *la = malloc(n * sizeof(double)), *lb = malloc(n * sizeof(double));
    unsigned k = 0, both = 0;
    for (unsigned i = from; i < n; i++) {
        if (!log[0].aud[i] || !log[1].aud[i] || isnan(log[0].fplay[i]) || isnan(log[1].fplay[i])) continue;
        sk[k] = log[0].fplay[i] - log[1].fplay[i];
        la[k] = log[0].lat[i];
        lb[k] = log[1].lat[i];
        k++;
    }
    for (unsigned i = 0; i < n; i++) both += log[0].aud[i] && log[1].aud[i];
    double m, sd, mx, lam, lasd, lamx, lbm, lbsd, lbmx;
    stats(sk, k, &m, &sd, &mx);
    stats(la, k, &lam, &lasd, &lamx);
    stats(lb, k, &lbm, &lbsd, &lbmx);
    double fs = rate_hz(sc->rate);
    printf("   last 10 s, %u probes with both playing: skew mean %+.4f frame (%+.2f us), sd %.4f, max |skew| %.4f frame\n",
           k, m, m / fs * 1e6, sd, mx);
    printf("   latency (arrival of the comma to the output, frames): A %.3f sd %.4f, B %.3f sd %.4f (D %u)\n",
           lam, lasd, lbm, lbsd, sc->d_local);
    if (sc->expect_skew >= 0) {
        CHECK(k > 5000, "%s: both receivers playing phase-locked in the last 10 s (%u probes)", sc->name, k);
        CHECK(mx <= sc->expect_skew, "%s: max |skew| %.4f frame > %.3f", sc->name, mx, sc->expect_skew);
        CHECK(fabs(lam - lbm) < sc->expect_skew, "%s: latencies differ %.4f", sc->name, lam - lbm);
        // The latency is D up to the measurement constants (comma end and the
        // DMA word positions): within a frame of D, the same on both.
        CHECK(fabs(lam - sc->d_local) < 1.0, "%s: latency %.3f not D %u", sc->name, lam, sc->d_local);
    } else {
        CHECK(mx > 0.5, "%s: mode 0 was expected to leave the units apart (max %.3f)", sc->name, mx);
    }
    for (int i = 0; i < NRX; i++) {
        if (sc->loss_to[i] <= sc->loss_from[i]) continue;
        // During the loss: real frames only up to the loss, then silence
        // within D + 1 ms; after the line returns, audible again within
        // 200 ms (SYS2 3.3), back at the same latency 10 s later (the resume
        // itself is within half a frame; the VCXO holds 2 s, then the servo).
        unsigned p0 = (unsigned)(sc->loss_from[i] * 1e6 / PROBE_US), p1 = (unsigned)(sc->loss_to[i] * 1e6 / PROBE_US);
        double last_f = NAN, lat_before = NAN, lat_after = NAN;
        unsigned silent_from = 0, back = 0;
        double sb = 0; unsigned nb = 0;
        for (unsigned j = p0 > 1000 ? p0 - 1000 : 0; j < p0; j++)
            if (log[i].aud[j]) { last_f = log[i].fplay[j]; sb += log[i].lat[j]; nb++; }
        lat_before = nb ? sb / nb : NAN;
        bool garbage = false;
        for (unsigned j = p0; j < p1 && j < log[i].n; j++) {
            if (isnan(log[i].fplay[j])) { if (!silent_from) silent_from = j; }
            else if (log[i].fplay[j] > last_f + (double)(sc->d_local + 200)) garbage = true;
        }
        for (unsigned j = p1; j < log[i].n; j++)
            if (log[i].aud[j]) { back = j; break; }
        double lat_other = NAN;
        if (back) {     // 1 s means, 9-10 s after the return, of this unit and the other one
            double sa = 0, so = 0; unsigned na = 0, no = 0;
            for (unsigned j = back + 9000; j < back + 10000 && j < log[i].n; j++) {
                if (log[i].aud[j]) { sa += log[i].lat[j]; na++; }
                if (log[1 - i].aud[j]) { so += log[1 - i].lat[j]; no++; }
            }
            lat_after = na ? sa / na : NAN;
            lat_other = no ? so / no : NAN;
        }
        double silent_ms = silent_from ? (silent_from - p0) * PROBE_US * 1e-3 : -1;
        double back_ms = back ? (back - p1) * PROBE_US * 1e-3 : -1;
        printf("   rx %c loss: silent %.1f ms after the line went dark, audible %.1f ms after it returned "
               "(resume e %+.3f), latency before %.3f, 10 s after %.3f (other unit %.3f)\n",
               'A' + i, silent_ms, back_ms, r[i].e_resume, lat_before, lat_after, lat_other);
        CHECK(!garbage, "%s: only frames from before the loss play during it", sc->name);
        CHECK(sc->loss_to[i] - sc->loss_from[i] < 0.003 || (silent_from && silent_ms <= sc->d_local * 1e3 / fs + 1.0),
              "%s: silent within D + 1 ms (%.1f ms)", sc->name, silent_ms);
        CHECK(back && back_ms < 200.0, "%s: audible again within 200 ms (%.1f ms)", sc->name, back_ms);
        if (sc->restart_at_s > 0)
            CHECK(r[i].reanchors <= 1, "%s: one re-anchor at most (%u)", sc->name, r[i].reanchors);
        else
            CHECK(r[i].reanchors == 0 && r[i].holdovers == 1, "%s: holdover and resume without a re-anchor", sc->name);
        if (sc->hold_drift_ppm != 0)
            CHECK(r[i].corrections >= 1 && r[i].corrections <= ALINK_PHASE_MAX_CORRECTIONS,
                  "%s: whole-frame corrections within the budget, each applied once (%u)", sc->name, r[i].corrections);
        CHECK(fabs(lat_after - lat_other) < 0.1, "%s: latency 9-10 s after the loss %.3f, the other unit %.3f",
              sc->name, lat_after, lat_other);
    }
    if (sc->restart_at_s > 0) {
        // Review lk6 F1: a sender that restarts its counter.  Every unit plays
        // again within 200 ms of its line carrying the new stream, re-anchored
        // once at most.
        for (int i = 0; i < NRX; i++) {
            double from = sc->restart_at_s;
            if (sc->loss_to[i] > sc->loss_from[i] && sc->loss_to[i] > from) from = sc->loss_to[i];
            unsigned p0 = (unsigned)(from * 1e6 / PROBE_US), back = 0;
            for (unsigned j = p0; j < log[i].n; j++)
                if (log[i].aud[j]) { back = j; break; }
            double ms = back ? (back - p0) * PROBE_US * 1e-3 : -1;
            printf("   rx %c after the sender restart (F from %" PRIu64 "): audible %.1f ms after its line carried it, "
                   "re-anchors %u\n", 'A' + i, sc->restart_f0, ms, r[i].reanchors);
            CHECK(back && ms < 200.0, "%s: rx %c audible within 200 ms of the restart (%.1f ms)", sc->name, 'A' + i, ms);
            CHECK(r[i].reanchors <= 1, "%s: rx %c one re-anchor at most (%u)", sc->name, 'A' + i, r[i].reanchors);
        }
    }
    if (sc->click_at_s > 0) {
        double dt = r[0].click_play_t - r[1].click_play_t;
        printf("   CLICK at frame %" PRIu64 " (superframe): A plays it at %.6f s (frame %" PRIu64 "), B at %.6f s "
               "(frame %" PRIu64 "): A - B = %+.2f us (%.4f sample)\n",
               r[0].snd.click_frame, r[0].click_play_t, r[0].click_frame_seen, r[1].click_play_t, r[1].click_frame_seen,
               dt * 1e6, dt * fs);
        CHECK(r[0].click_play_t > 0 && r[1].click_play_t > 0, "%s: both CLICKs played", sc->name);
        CHECK(r[0].click_frame_seen == r[0].snd.click_frame && r[1].click_frame_seen == r[1].snd.click_frame,
              "%s: CLICK on its frame", sc->name);
        if (sc->expect_skew >= 0)
            CHECK(fabs(dt * fs) < 1.0, "%s: CLICK skew %.4f sample >= 1", sc->name, dt * fs);
    }
    if (sc->unit_ops_at_s > 0) {
        printf("   per-unit ops on A: applied %u, late %u, cancelled %u, refused %u; MASTER_VOLUME first on frame "
               "%" PRId64 " (target %" PRIu64 ")\n",
               r[0].ev_applied, r[0].ev_late, r[0].ev_cancel, r[0].ev_refused, r[0].vol_frame, r[0].vol_target);
        CHECK(r[0].ev_applied >= 1 && r[0].ev_late == 1 && r[0].ev_cancel == 2, "%s: per-unit results", sc->name);
        CHECK(r[0].vol_frame == (int64_t)r[0].vol_target, "%s: MASTER_VOLUME on its frame", sc->name);
    }
    if (wav_path) write_wav(sc, log);
    for (int i = 0; i < NRX; i++) {
        free(log[i].fplay);
        free(log[i].aud);
        free(log[i].lat);
    }
    free(sk);
    free(la);
    free(lb);
    (void)both;
}

int main(int argc, char **argv) {
    snd_tables_init(&T);
    const char *only = argc > 1 ? argv[1] : NULL;
    trace = getenv("ALINK_PHASE_SIM_TRACE") != NULL;
    wav_path = getenv("ALINK_PHASE_SIM_WAV");
    const char *dump = getenv("ALINK_PHASE_SIM_DUMP");
    if (dump) {
        char path[256];
        if (sscanf(dump, "%255[^:]:%lf:%lf", path, &dump_from, &dump_to) == 3) dump_f = fopen(path, "wb");
        if (!dump_f) printf("ALINK_PHASE_SIM_DUMP: cannot open %s\n", dump);
    }
    const scenario_t sc[] = {
        { "same stream, same boot, same clocks", 1, 1, 8, 0, { 0, 0 }, { 0.0, 0.0 }, { 0, 0 }, { 0, 0 },
          1, 128, 20.0, 15.0, 12.0, 0.1, 0, 0, 0 },
        { "different boot times (B 3.7 s later)", 1, 1, 8, 0, { 0, 0 }, { 0.0, 3.7 }, { 0, 0 }, { 0, 0 },
          1, 128, 30.0, 25.0, 0, 0.1, 0, 0, 0 },
        { "clock offsets: sender +20, A +45, B -38 ppm", 1, 1, 8, 20, { 45, -38 }, { 0.3, 1.1 }, { 0, 0 }, { 0, 0 },
          1, 128, 40.0, 35.0, 0, 0.1, 0, 0, 0 },
        { "short loss on B (1 ms dark line)", 1, 1, 8, 0, { 12, -9 }, { 0.0, 0.5 }, { 0, 20.0 }, { 0, 20.001 },
          1, 128, 40.0, 35.0, 0, 0.1, 0, 0, 0 },
        { "long loss on B (2 s dark line), holdover", 1, 1, 8, 0, { 30, -25 }, { 0.0, 0.8 }, { 0, 18.0 }, { 0, 20.0 },
          1, 128, 45.0, 40.0, 0, 0.1, 0, 0, 0 },
        { "L3 96 kHz C 16, offsets and boot times", 3, 2, 16, -15, { 25, -40 }, { 0.2, 2.9 }, { 0, 0 }, { 0, 0 },
          1, 200, 25.0, 20.0, 0, 0.1, 0, 0, 0 },
        { "mode 0 (fill servo) for contrast", 1, 1, 8, 0, { 45, -38 }, { 0.3, 1.1 }, { 0, 0 }, { 0, 0 },
          0, 128, 25.0, 0, 0, -1.0, 0, 0, 0 },
        // Review lk6's probes (F1, F2): a sender power cycle with F back at 0
        // or 10M ahead during B's dark line, and a 60 s holdover.
        { "sender power cycle (F restarts at 0) during B's 2 s dark line", 1, 1, 8, 0, { 30, -25 }, { 0.0, 0.8 },
          { 0, 18.0 }, { 0, 20.0 }, 1, 128, 45.0, 40.0, 0, 0.1, 19.0, 0, 0 },
        { "sender restart with F 10M ahead during B's 2 s dark line", 1, 1, 8, 0, { 30, -25 }, { 0.0, 0.8 },
          { 0, 18.0 }, { 0, 20.0 }, 1, 128, 45.0, 40.0, 0, 0.1, 19.0, F0 + 10000000ull, 0 },
        { "60 s dark line on B (long holdover), clocks +30/-25", 1, 1, 8, 0, { 30, -25 }, { 0.0, 0.8 },
          { 0, 10.0 }, { 0, 70.0 }, 1, 128, 100.0, 95.0, 0, 0.1, 0, 0, 0 },
        // Review lk6 F2: the held VCXO drifts 0.5 ppm in a 60 s holdover, so B
        // resumes 1.4-2.3 frames early and needs negative whole-frame
        // corrections (each applied once: before the fix the first one was
        // applied twice and the budget ran out into a re-anchor), no re-anchor.
        { "60 s holdover with 0.5 ppm VCXO drift on B (negative correction)", 1, 1, 8, 0, { 30, -25 }, { 0.0, 0.8 },
          { 0, 10.0 }, { 0, 70.0 }, 1, 128, 100.0, 95.0, 0, 0.1, 0, 0, 0.5 },
        // WP USB3: pull-in from far off (the muted fast slew).
        { "pull-in: receivers +58 and -100 ppm off the sender", 1, 1, 8, 0, { 58, -100 }, { 0.0, 0.4 }, { 0, 0 },
          { 0, 0 }, 1, 128, 40.0, 35.0, 0, 0.1, 0, 0, 0 },
    };
    for (unsigned i = 0; i < sizeof(sc) / sizeof(sc[0]); i++) {
        if (only && !strstr(sc[i].name, only)) continue;
        run_scenario(&sc[i]);
    }
    if (dump_f) fclose(dump_f);
    printf("\nalink_phase_sim: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
