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
// WP FWTB3: the generator on the time base.  Every receiver runs its own copy
// of the real firmware/DSPi/siggen.c and timebase.c (tools/alink_usb_sim/
// sg_unit.h), with apply ops 9 SIGGEN_START / 10 SIGGEN_STOP in apply_exec(),
// the line's time-base provider and tracker, the write path
// (alink_input_apply()) and the broadcast op 9, all as alink_input.c has
// them.  The rings carry each unit's rendered generator samples, and the
// scenarios "gen ..." check that two and four units' generators started at
// one T play their samples at the same instants (sample for sample, within
// the lock error), staggered starts on their samples, a unit behind a fan-out
// after hops_offset, a loss that cancels a pending start and stops a run,
// late starts refused.  Up to four receivers (NRX_MAX); each runs in its own
// process (fork), so every scenario starts from fresh generators.
//
//   tools/alink_phase_sim/run.sh           (all scenarios; exit 1 on a failure)

#define _DEFAULT_SOURCE         // fork()
#define PICO_RP2350 1
#define DSPI_SIGGEN_SYNC 1      // an RP2350 build with the ops (WP FWTB1)
#include "alink_codec.h"
#include "alink_fast.h"
#include "alink_phase.h"
#include "alink_ring_clock.h"
#include "sender_alink.h"
#include "sg_unit.h"
#include "siggen.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

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
#define NRX_MAX     4

// WP FWTB3: the generator plan of a scenario.  Unit i's op 9 is written for
// T_i = t0 + stagger[i] (t0: the sender's frame about t0_s after its start)
// through EXT_APPLY_AT_FRAME at write_s, or carried in the superframe
// (broadcast).  Each kind has its own checks (gen_checks()).
enum { GEN_COINCIDE = 1, GEN_LOSS, GEN_LATE };

typedef struct {
    int kind;
    uint8_t type;                   // SIGGEN_IMPULSE (exact) or SIGGEN_SWEEP_LOG
    uint16_t repeat;
    double t0_s, write_s;           // T about t0_s; the host writes at write_s (absolute)
    int32_t stagger[NRX_MAX];       // frames
    bool broadcast;                 // op 9 in the superframe (one schedule at the sender, at write_s)
} gen_plan_t;

typedef struct {
    const char *name;
    unsigned cls, rate, c;
    double sender_ppm;
    double rx_ppm[NRX_MAX];
    double boot_s[NRX_MAX];         // receiver power-up (absolute seconds)
    double loss_from[NRX_MAX], loss_to[NRX_MAX];   // dark line on that receiver's input (absolute s)
    uint8_t mode;                   // EXT_ALINK_PHASE.mode on both
    uint16_t d_local;
    double duration_s;
    double click_at_s;              // sender schedules CLICK (superframe) at about this time
    double unit_ops_at_s;           // per-unit ops on receiver 0 (EXT_APPLY_AT_FRAME path), 0 = none
    double expect_skew;             // max |skew| (frames) once both play phase-locked; < 0: expect a mismatch
    double restart_at_s;            // review lk6 probe: the sender power-cycles at this time (0 none)...
    uint64_t restart_f0;            // ... and its frame counter starts again here
    double hold_drift_ppm;          // the held VCXO drifts by this much in a holdover (temperature model)
    // WP FWTB3
    unsigned nrx;                   // receivers, 0 = 2
    const gen_plan_t *gen;          // the generator plan, NULL none
    uint8_t hop_frames[NRX_MAX];    // a fan-out in front of this receiver delays its line by this many frames (0 direct)
    bool hop_uncompensated[NRX_MAX];    // ... and the hub did not compensate it (hops_offset 0)
} scenario_t;

static unsigned nrx(const scenario_t *sc) { return sc->nrx ? sc->nrx : 2u; }

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
    bool gen_scheduled;             // WP FWTB3: the broadcast op 9 is in the superframes
    bool gen_sched_ok;
} sender_t;

static snd_tables_t T;

static uint64_t gen_t0(const scenario_t *sc);

// hops: the stream as a fan-out in front of the receiver re-emits it (its
// superframe's hop byte; the bits are delayed by the receiver's hop_s).
static void sender_init(sender_t *s, const scenario_t *sc, uint8_t hops) {
    memset(s, 0, sizeof(*s));
    s->ring = calloc(SND_WORDS, 4);
    alink_tx_init(&s->tx, sc->cls, sc->rate, sc->c, F0, 0x5A);
    s->tx.flags = ALINK_SF_FLAG_USB_LIVE;
    s->tx.hops = hops;
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
        // WP FWTB3: the broadcast op 9 (EXT_ALINK_CTRL_MSG at the sender),
        // for the frame t0 of the plan.
        if (sc->gen && sc->gen->broadcast && !s->gen_scheduled &&
            (double)snd_sink_bits(&s->sink) / ((double)alink_class_baud(sc->cls) * (1.0 + sc->sender_ppm * 1e-6)) >=
                sc->gen->write_s) {
            s->gen_scheduled = true;
            s->gen_sched_ok = alink_tx_schedule(&s->tx, gen_t0(sc), ALINK_OP_SIGGEN_START, 0, 0);
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
    // WP FWTB3: the unit's generator and the line's time base (alink_input.c)
    const sg_unit_t *g;
    tb_track_t tb_line;
    tb_provider_t prov;
    alink_sf_t sf_last;
    bool sf_valid;
    double hop_s;                   // the fan-out's delay in front of this receiver (s)
    float gen[RING_TAGS];           // the generator's sample (its output channel) per ring frame
    bool dac_want;                  // the DAC model: open DAC_OPEN_US after the generator wants it
    uint64_t dac_since_us;
    uint64_t t_write;               // T of the op 9 written (0 none)
    uint8_t wr_code, wr_code_late, wr_code_past;   // write results (TB_OK / TB_ERR_*)
    bool wrote, wrote_late;
    unsigned op9_res[4];            // 0xE2 results of op 9: applied, late, cancelled, refused
    unsigned tb_events;
    uint8_t tb_ev_state, tb_ev_reason;
    // The capture: frame F of [cap_lo, cap_lo + cap_len) played at cap_t with the generator's value cap_v.
    uint64_t cap_lo;
    uint32_t cap_len;
    double *cap_t;
    float *cap_v;
    uint8_t *cap_n;
    sg_timeline_t tl_run;           // EXT_SIGGEN_TIMELINE while the run played (GEN_LOSS: before the loss)
    bool tl_run_seen;
    double tb_inv0;                 // EXT_TIME_BASE: t - output_frames at the first read while LOCKED
    bool tb_inv_set;
    unsigned tb_inv_reads, tb_inv_bad;
    uint16_t tb_dist0;
    sg_timeline_t tl_end;           // EXT_SIGGEN_TIMELINE at the end
    bool want_end;                  // siggen_wants_output() at the end (the DAC handed back: false)
} rx_t;

// ---- WP FWTB3: the generator copies, the port, the macros for the mirrors ---

static const sg_unit_t *const SGU[SG_UNITS] = { &sgu0_sg_unit, &sgu1_sg_unit, &sgu2_sg_unit, &sgu3_sg_unit,
                                                &sgu4_sg_unit };    // [4]: the reference
static rx_t *cur_rx;                // the receiver running now (one at a time)
static const sg_unit_t *G;          // its generator
#define siggen_tb_fire(...)             G->tb_fire(__VA_ARGS__)
#define siggen_tb_stop_op(a)            G->tb_stop_op(a)
#define siggen_tb_dropped(a)            G->tb_dropped(a)
#define siggen_tb_accept(...)           G->tb_accept(__VA_ARGS__)
#define siggen_tb_accept_broadcast(...) G->tb_accept_broadcast(__VA_ARGS__)
#define siggen_tb_poll(...)             G->tb_poll(__VA_ARGS__)
#define siggen_tb_cancel_all()          G->tb_cancel_all()
#define ARM_LEAD_MS     150u        // max(hold_ms, release_ms) + 50: the K340's
#define DAC_OPEN_US     100000u     // the K340's DAC un-mute (FWSIGGEN: max(hold_ms, release_ms))
#define GEN_OUT         2u          // the generator's output channel
// alink_input.h
#define ALINK_INPUT_OFF         0
#define ALINK_INPUT_SEARCHING   1
#define ALINK_INPUT_LOCKED      2
#define ALINK_INPUT_HOLDOVER    3
#define ALINK_REASON_LOCKED     1
#define ALINK_REASON_REANCHOR   5
#define ALINK_REASON_DISABLED   7
typedef int AlinkInputStateSim;
static const alink_apply_limits_t apply_lim = { 10, 9, 2048, true };   // alink_input.c's, with ops 9/10

static double local_to_abs(const rx_t *r, uint64_t us) {
    return r->sc->boot_s[r->id] + (double)us * 1e-6 / (1.0 + r->sc->rx_ppm[r->id] * 1e-6);
}

// The time on this receiver's line: the sender's bits reach a unit behind a
// fan-out hop_s later (WP FWTB3; 0 for a unit fed directly).
static double line_t(const rx_t *r) { return r->t - r->hop_s; }

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
            r->gen[(r->wr + i) & (RING_TAGS - 1u)] = 0.0f;
        }
        r->wr += skip;
    }
    float gv[192];
    if (ring_fill(r) + n > r->lat.span) {
        r->drops++;
        // WP FWSIGGEN: the generator holds on a refused block (not advanced).
        for (uint32_t i = 0; i < n; i += 192) G->hold(gv, n - i < 192 ? n - i : 192);
        return;     // refused by group admission
    }
    r->last_prewrite = ring_fill(r);
    for (uint32_t i = 0; i < n; i++) {
        uint32_t k = (r->wr + i) & (RING_TAGS - 1u);
        r->tag[k] = frames ? frames[i] : TAG_SILENCE;
        r->click[k] = 0;
        // WP FWTB3: the generator renders into the block (on audio and on
        // silence), after the mix, as process_input_block() runs it.
        if (i % 192 == 0) G->render(gv, GEN_OUT, n - i < 192 ? n - i : 192, r->fs);
        r->gen[k] = gv[i % 192];
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
    double lt = line_t(r);
    uint64_t words = lt > 0 ? (uint64_t)floor(lt * r->baud_s / 32.0) : 0;
    if (words < r->w0) return 0;
    sender_fill(&r->snd, r->sc, words + 1);
    double t_lost0 = r->sc->loss_from[r->id], t_lost1 = r->sc->loss_to[r->id];
    for (; r->copied < words; r->copied++) {
        uint32_t w = r->snd.ring[r->copied & (SND_WORDS - 1u)];
        double ta = (double)(r->copied * 32u + 31u) / r->baud_s + r->hop_s;
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
    uint64_t words = (uint64_t)floor(line_t(r) * r->baud_s / 32.0);
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

// ---- WP FWTB3: the line's time base (alink_input.c, mirrors) ----------------

static AlinkInputStateSim alink_input_get_state(const rx_t *r) {
    if (r->ph.hold) return ALINK_INPUT_HOLDOVER;
    return r->phase == PH_LOCKED ? ALINK_INPUT_LOCKED : ALINK_INPUT_SEARCHING;
}

static bool tb_line_present(rx_t *r) { (void)r; return true; }

static uint8_t tb_line_now(rx_t *r) {
    if (!tb_line_present(r)) return TB_ST_NONE;
    return tb_line_state((uint8_t)alink_input_get_state(r), r->ph.mode == 1, alink_phase_active(&r->ph), r->ph.state,
                         r->ph.started);
}

static void tb_line_note(rx_t *r, uint8_t reason) {
    (void)tb_track_note(&r->tb_line, tb_line_now(r), reason, (uint32_t)r->ph.next_w);
}

static void tb_line_read(rx_t *r, tb_entry_t *e, bool ring, uint32_t wr, uint32_t played) {
    e->domain = r->sf_valid ? r->sf_last.sender : 0u;
    int32_t d = alink_phase_delay(&r->ph);
    e->d = d > 0 ? (uint16_t)d : 0u;
    e->phase_error = r->ph.e;
    uint8_t st = r->tb_line.state;
    if (st == TB_ST_NONE || st == TB_ST_ACQUIRING) {
        e->t = 0;
    } else if (r->ph.timeline && r->ph.started && ring) {
        e->t = tb_t_at_output(r->ph.next_w, wr, played, d);
    } else {
        e->t = r->pub_f;
    }
}

static bool tb_line_next(rx_t *r, uint64_t *t) {
    if (!r->ph.timeline) return false;
    *t = r->ph.next_w;
    return true;
}

static void apply_report(rx_t *r, uint8_t op, uint8_t result, uint64_t frame);
static void tb_line_drop(rx_t *r, uint8_t res) {
    alink_apply_op_t op;
    if (alink_apply_remove(&r->aq, ALINK_OP_SIGGEN_START, &op)) apply_report(r, op.op, res, op.frame);
}

// The provider's callbacks: the receiver running now.
static uint8_t prov_now(void) { return tb_line_now(cur_rx); }
static bool prov_present(void) { return tb_line_present(cur_rx); }
static void prov_read(tb_entry_t *e, bool ring, uint32_t wr, uint32_t played) { tb_line_read(cur_rx, e, ring, wr, played); }
static bool prov_next(uint64_t *t) { return tb_line_next(cur_rx, t); }
static void prov_drop(uint8_t res) { tb_line_drop(cur_rx, res); }

// The generator copy's port (sg_unit.h).
static bool port_ready_now(void) { return true; }
static uint16_t port_arm_lead_ms(void) { return ARM_LEAD_MS; }
static uint32_t port_slips(void) { return cur_rx->underruns; }
static uint32_t port_now_us(void) { return (uint32_t)cur_rx->now_us; }
static bool port_ring(uint32_t *wr, uint32_t *played) {
    *wr = cur_rx->wr;
    *played = (uint32_t)cur_rx->out_phase;
    return cur_rx->running;
}
static void port_event(uint8_t tb, uint8_t st, uint8_t rs, uint32_t t) {
    (void)tb; (void)t;
    cur_rx->tb_events++;
    cur_rx->tb_ev_state = st;
    cur_rx->tb_ev_reason = rs;
}
static sg_port_t sim_port = { port_ready_now, port_arm_lead_ms, port_slips, port_now_us, port_ring, port_event };

// event(): the receiver's 0xE1 (not modelled), the tracker's own reasons.
static void event(rx_t *r, uint8_t state, uint8_t reason) {
    (void)state;
#if DSPI_SIGGEN_SYNC
    if (reason == ALINK_REASON_REANCHOR) tb_line_note(r, TB_RS_REANCHOR);
    else if (reason == ALINK_REASON_DISABLED) tb_line_note(r, TB_RS_DISABLED);
#endif
    if (reason == ALINK_REASON_REANCHOR) r->reanchors++;
}

static void apply_report(rx_t *r, uint8_t op, uint8_t result, uint64_t frame) {
#if DSPI_SIGGEN_SYNC
    // A pending op 9 this receiver cancelled itself (it stopped: no frames
    // any more) is a loss; a cancel the generator asked for has already
    // cleared its pending op (siggen_tb_dropped() is then a no-op).
    if (op == ALINK_OP_SIGGEN_START && result == ALINK_APPLY_CANCELLED) siggen_tb_dropped(TB_RES_CANCELLED_LOSS);
#endif
    if (op == ALINK_OP_SIGGEN_START && result < 4) r->op9_res[result]++;
    alink_apply_done(&r->aq, op, result, frame);
    if (result == ALINK_APPLY_APPLIED) r->ev_applied++;
    else if (result == ALINK_APPLY_LATE) r->ev_late++;
    else if (result == ALINK_APPLY_CANCELLED) r->ev_cancel++;
    else r->ev_refused++;
}

static uint8_t apply_exec(rx_t *r, const alink_apply_op_t *op, bool late) {
    switch (op->op) {
#if DSPI_SIGGEN_SYNC
    case ALINK_OP_SIGGEN_START:
        // WP FWTB1: the generator's sample 0 is this frame's (the caller split
        // the block here).  Not ready muted (user mute, the superframe's
        // system mute) or without a LOCKED time base; never late.
        return siggen_tb_fire(TB_ID_LINE, op->frame, late,
                              r->tb_line.state == TB_ST_LOCKED && !r->user_mute &&
                              !(r->sf_last.flags & ALINK_SF_FLAG_MUTE));
    case ALINK_OP_SIGGEN_STOP:
        siggen_tb_stop_op(op->arg);
        return ALINK_APPLY_APPLIED;
#endif
    case ALINK_OP_MUTE: r->user_mute = true; return ALINK_APPLY_APPLIED;
    case ALINK_OP_UNMUTE: r->user_mute = false; return ALINK_APPLY_APPLIED;
    case ALINK_OP_MASTER_VOLUME: r->master_db = op->value * 0.01f; r->vol_frame = -2; return ALINK_APPLY_APPLIED;
    case ALINK_OP_CLICK:
        if (op->arg >= 8 || r->user_mute || r->stream_mute) return ALINK_APPLY_REFUSED;
        r->click_out = (int8_t)op->arg;
        return ALINK_APPLY_APPLIED;
    case ALINK_OP_PRESET_LOAD:
    case ALINK_OP_OUT_DELAY: return ALINK_APPLY_APPLIED;
    default:
        (void)late;
        return ALINK_APPLY_REFUSED;
    }
}

static void apply_run(rx_t *r, bool late) {
    alink_apply_op_t op;
    alink_apply_pop(&r->aq, &op);
    uint8_t res = apply_exec(r, &op, late);
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
    event(r, ALINK_INPUT_LOCKED, ALINK_REASON_REANCHOR);
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
#if DSPI_SIGGEN_SYNC
        tb_line_note(r, TB_RS_CORRECTION);         // WP FWTB1: a disturbance of the timeline
#endif
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
            event(r, ALINK_INPUT_LOCKED, ALINK_REASON_REANCHOR);
        }
    }
    event(r, ALINK_INPUT_LOCKED, ALINK_REASON_LOCKED);
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
    if (r->lf.state == ALINK_RX_LOCKED) {
        r->sf_last = r->lf.sf;
        r->sf_valid = true;
        if (r->phase == PH_LOCKED) {
            // The superframe's apply (the broadcast path): same queue.
            int took = alink_apply_take_sf(&r->aq, &apply_lim, &r->sf_last);
#if DSPI_SIGGEN_SYNC
            // A broadcast op 9 (2.8.7.1): refused at once when another is
            // pending or it was first seen less than arm_lead before T.
            if (took == 1 && r->sf_last.apply_op == ALINK_OP_SIGGEN_START &&
                !siggen_tb_accept_broadcast(TB_ID_LINE, r->sf_last.apply_frame, r->ph.next_w,
                                            r->detected_rate ? r->detected_rate : r->fs)) {
                // Take back the one just queued (a pending op 9 stays).
                for (unsigned i = r->aq.n; i-- > 0;)
                    if (r->aq.q[i].op == ALINK_OP_SIGGEN_START && r->aq.q[i].frame == r->sf_last.apply_frame) {
                        r->aq.n--;
                        memmove(&r->aq.q[i], &r->aq.q[i + 1], (r->aq.n - i) * sizeof(r->aq.q[0]));
                        break;
                    }
                took = 2;
            }
#endif
            if (took == 2)
                apply_report(r, r->sf_last.apply_op, ALINK_APPLY_REFUSED, r->sf_last.apply_frame);
        }
    }
    if (r->phase == PH_LOCKED && r->detected_rate)
        alink_meas_sample(&r->meas, r->now_us, r->pub_bits,
                         1e6f * (float)r->detected_rate / (float)alink_class_baud(r->lf.cls));
#if DSPI_SIGGEN_SYNC
    // An op 9 pending on F arms at T - arm_lead (WP FWTB1).
    if (r->phase == PH_LOCKED && r->ph.timeline) siggen_tb_poll(TB_ID_LINE, r->ph.next_w, r->detected_rate);
#endif
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
    // main.c, WP FWTB1/FWSIGGEN: the aggregator (losses and disturbances to
    // the generator, 0xE4), the generator's service, the DAC gate: open
    // DAC_OPEN_US after the generator first wants the outputs.
    G->tb_service((uint32_t)(r->now_us / 1000u));
    G->service();
    bool want = G->wants_output();
    if (want && !r->dac_want) r->dac_since_us = r->now_us;
    r->dac_want = want;
    G->output_gate(want && r->now_us - r->dac_since_us >= DAC_OPEN_US);
}

// EXT_APPLY_AT_FRAME OUT on this unit (alink_input_apply(), mirrored).
uint8_t alink_input_apply(rx_t *r, const uint8_t p[ALINK_APPLY_LEN]) {
    alink_apply_op_t op;
    if (!alink_apply_parse(&apply_lim, p, &op)) return TB_ERR_VALUE;
    if (op.op == ALINK_APPLY_CANCEL_ALL) {
#if DSPI_SIGGEN_SYNC
        // CANCEL_ALL of any time base: the generator first (CANCELLED_HOST,
        // and no later START on the staged config, R4), then the queue.
        siggen_tb_cancel_all();
#endif
        while (r->aq.n) {
            alink_apply_pop(&r->aq, &op);
            apply_report(r, op.op, ALINK_APPLY_CANCELLED, op.frame);
        }
        return TB_OK;
    }
    if (alink_input_get_state(r) != ALINK_INPUT_LOCKED) return TB_ERR_STATE; // no lock, no F
    if (r->aq.n >= ALINK_APPLY_DEPTH) return TB_ERR_STATE;
#if DSPI_SIGGEN_SYNC
    if (op.op == ALINK_OP_SIGGEN_START) {
        // The lead against the frame entering the pipeline now, one op 9 per
        // unit (2.8.7.1): the generator decides.
        if (!r->ph.timeline) return TB_ERR_STATE;
        uint8_t res = siggen_tb_accept(TB_ID_LINE, op.frame, r->ph.next_w, r->detected_rate);
        if (res != TB_OK) return res;
    }
#endif
    return alink_apply_push(&r->aq, &op) ? TB_OK : TB_ERR_STATE;
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
    uint8_t hop = sc->hop_frames[id];
    sender_init(&r->snd, sc, hop ? 1u : 0u);
    r->baud_s = (double)alink_class_baud(sc->cls) * (1.0 + sc->sender_ppm * 1e-6);
    r->fs = rate_hz(sc->rate);
    r->lat = latency_for(r->fs);
    // WP FWTB3: a fan-out re-emits the stream hop whole frames later (alink_protocol_spec.md 7.2).
    r->hop_s = hop / ((double)r->fs * (1.0 + sc->sender_ppm * 1e-6));
    double lt0 = sc->boot_s[id] - r->hop_s;
    r->w0 = lt0 > 0 ? (uint64_t)ceil(lt0 * r->baud_s / 32.0) : 0;
    r->copied = r->w0;
    alink_fast_init(&r->lf, r->ring, ALINK_RING_WORDS, sc->cls, 1, NULL, NULL, 0);
    static const uint8_t ident[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    alink_fast_set_slots(&r->lf, ident);
    alink_phase_init(&r->ph);
    alink_phase_limits(&r->ph, r->lat.target, r->lat.cap, r->lat.span);
    // The hub's values (alink_protocol_spec.md 7.2): behind one re-emitter
    // d_local = D - hops and hops_offset = -(hop_delay_frames - 1).
    uint16_t dl = (uint16_t)(sc->d_local - (hop ? 1u : 0u));
    int16_t ho = hop && !sc->hop_uncompensated[id] ? (int16_t)-(hop - 1) : 0;
    uint8_t p[8] = { sc->mode, 0, (uint8_t)dl, (uint8_t)(dl >> 8), (uint8_t)ho, (uint8_t)((uint16_t)ho >> 8), 0, 0 };
    if (alink_phase_set(&r->ph, p) < 0) printf(" (phase write refused)\n");
    alink_apply_init(&r->aq);
    // WP FWTB3: this unit's generator copy and the line's provider in its aggregator.
    r->g = SGU[id];
    tb_track_init(&r->tb_line);
    r->prov = (tb_provider_t){ TB_ID_LINE, TB_ROLE_FOLLOWER, &r->tb_line, prov_now, prov_present, prov_read, prov_next,
                               prov_drop };
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
    double bits = line_t(r) * r->baud_s;
    double bpf = (double)alink_class_baud(r->sc->cls) / (double)r->fs;
    if (r->snd.restarted && bits >= (double)r->snd.restart_bits)
        return (double)r->sc->restart_f0 + (bits - (double)r->snd.restart_bits - 10.0) / bpf;
    return (double)F0 + (bits - 10.0) / bpf;
}

// ---- WP FWTB3: the host's side of the generator plan --------------------------

static uint64_t gen_t0(const scenario_t *sc) { return F0 + (uint64_t)llround(sc->gen->t0_s * rate_hz(sc->rate)); }

// The config the hub stages on every unit (one per unit per run, OD-ATj):
// impulses every 20 ms (exact samples), or log sweeps 20 Hz-20 kHz of 300 ms
// with 20 ms gaps, on output GEN_OUT, -20 dBFS.
static SiggenConfig gen_cfg(const gen_plan_t *gp) {
    SiggenConfig c;
    memset(&c, 0, sizeof c);
    c.version = SIGGEN_CFG_VERSION;
    c.signal_type = gp->type;
    c.channel_mask = (uint16_t)(1u << GEN_OUT);
    c.level_db = -20.0f;
    c.repeat = gp->repeat;
    if (gp->type == SIGGEN_IMPULSE) {
        c.p1 = 20.0f;
    } else {
        c.duration_ms = 300;
        c.gap_ms = 20;
        c.p1 = 20.0f;
        c.p2 = 20000.0f;
    }
    return c;
}

// The run's length in frames (the capture covers it with margins).
static uint32_t gen_len(const gen_plan_t *gp, unsigned fs) {
    double ms = gp->type == SIGGEN_IMPULSE ? 20.0 * gp->repeat : 320.0 * gp->repeat;
    return (uint32_t)(ms * fs / 1000.0) + 1u;
}

// EXT_APPLY_AT_FRAME OUT: op 9 at T on the line's time base.
static uint8_t write_op9(rx_t *r, uint64_t t) {
    uint8_t p[ALINK_APPLY_LEN] = { 0 };
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(t >> (8 * i));
    p[8] = ALINK_OP_SIGGEN_START;
    p[12] = TB_ID_LINE;
    return alink_input_apply(r, p);
}

static void gen_host(rx_t *r) {
    const scenario_t *sc = r->sc;
    const gen_plan_t *gp = sc->gen;
    if (r->wrote || r->t < gp->write_s - (gp->broadcast ? 0.1 : 0.0) || r->tb_line.state != TB_ST_LOCKED) return;
    r->wrote = true;
    SiggenConfig c = gen_cfg(gp);
    if (!G->stage_config(&c, sizeof c)) printf("   rx %c: SET_CONFIG refused\n", 'A' + r->id);
    if (gp->kind == GEN_LATE) {
        // A START whose sample has passed, and one inside the arm lead:
        // refused at the write (VC_ERR_STATE), nothing queued.
        r->wr_code_past = write_op9(r, r->ph.next_w - 480u);
        r->wr_code_late = write_op9(r, r->ph.next_w + (uint64_t)ARM_LEAD_MS * r->fs / 1000u - 1u);
        r->wrote_late = true;
    }
    if (!gp->broadcast) {
        r->t_write = gen_t0(sc) + (uint64_t)(int64_t)gp->stagger[r->id];
        r->wr_code = write_op9(r, r->t_write);
    }
}

// The frames played between output positions `before` and out_phase (the
// pass from t_from at `rate`): their play instants and generator samples.
static void capture(rx_t *r, double before, double t_from, double rate) {
    if (!r->cap_len || !r->running) return;
    for (uint32_t o = (uint32_t)floor(before) + 1u; (double)o <= r->out_phase && (int32_t)(o - r->wr) < 0; o++) {
        int64_t f = r->tag[o & (RING_TAGS - 1u)];
        if (f < (int64_t)r->cap_lo || f >= (int64_t)(r->cap_lo + r->cap_len)) continue;
        uint32_t i = (uint32_t)(f - (int64_t)r->cap_lo);
        if (r->cap_n[i] < 255) r->cap_n[i]++;
        if (r->cap_n[i] == 1) {
            r->cap_t[i] = t_from + ((double)o - before) / rate;
            r->cap_v[i] = r->gen[o & (RING_TAGS - 1u)];
        }
    }
}

// EXT_TIME_BASE while LOCKED: t - output_frames constant while the
// disturbance counter stays (R7c; +-1 sample allowed, compared mod 2^32).
static void tb_invariant(rx_t *r) {
    if (r->tb_line.state != TB_ST_LOCKED || !r->running) return;
    uint8_t o[TB_READ_LEN];
    G->tb_read(o);
    if (o[1] < 1) return;
    const uint8_t *x = o + TB_ENTRY_OFF;
    uint32_t played = (uint32_t)o[8] | (uint32_t)o[9] << 8 | (uint32_t)o[10] << 16 | (uint32_t)o[11] << 24;
    uint32_t tl = (uint32_t)x[8] | (uint32_t)x[9] << 8 | (uint32_t)x[10] << 16 | (uint32_t)x[11] << 24;
    uint16_t dist = (uint16_t)(x[16] | x[17] << 8);
    double inv = (double)(int32_t)(tl - played);
    if (!r->tb_inv_set || dist != r->tb_dist0) {
        r->tb_inv_set = true;
        r->tb_inv0 = inv;
        r->tb_dist0 = dist;
        return;
    }
    r->tb_inv_reads++;
    if (fabs(inv - r->tb_inv0) > 1.0) r->tb_inv_bad++;
}

static void run_rx(rx_t *r, probe_log_t *log) {
    const scenario_t *sc = r->sc;
    // WP FWTB3: this receiver's generator copy, its port and provider.
    cur_rx = r;
    G = r->g;
    *G->port = &sim_port;
    G->tb_register(TB_SLOT_LINE, &r->prov);
    if (sc->gen) {
        int32_t lo = 0, hi = 0;
        for (unsigned i = 0; i < nrx(sc); i++) {
            if (sc->gen->stagger[i] < lo) lo = sc->gen->stagger[i];
            if (sc->gen->stagger[i] > hi) hi = sc->gen->stagger[i];
        }
        uint32_t margin = r->fs / 10u;
        r->cap_lo = gen_t0(sc) + (uint64_t)(int64_t)lo - margin;
        r->cap_len = (uint32_t)(hi - lo) + gen_len(sc->gen, r->fs) + 2u * margin;
        r->cap_t = malloc(r->cap_len * sizeof(double));
        r->cap_v = calloc(r->cap_len, sizeof(float));
        r->cap_n = calloc(r->cap_len, 1);
        for (uint32_t i = 0; i < r->cap_len; i++) r->cap_t[i] = NAN;
    }
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
            double b0 = r->out_phase;
            if (r->running) r->out_phase += rate * (tp - t_prev);
            capture(r, b0, t_prev, rate);
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
        capture(r, before, t_prev, rate);
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
        if (sc->gen) gen_host(r);
        poll(r);
        t_prev = r->t;      // the anchor's spin may have moved the clock (outputs stopped meanwhile)
        if (sc->gen && r->wrote) {
            tb_invariant(r);
            // The timeline while the run plays (before any loss stops it).
            sg_timeline_t tl = sg_timeline(G);
            if (tl.state == TB_TL_RUNNING) {
                r->tl_run = tl;
                r->tl_run_seen = true;
            }
        }
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
    r->tl_end = sg_timeline(G);
    r->want_end = G->wants_output();
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
        for (int c = 0; c < 2; c++) {
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

// ============================================================================
// WP FWTB3: the generator checks
// ============================================================================

static const char *const TL_RES[] = { "none", "started", "refused late", "refused not ready", "cancelled loss",
                                      "cancelled config", "cancelled host", "stopped loss" };

static double cap_t_of(const rx_t *r, uint64_t f) {
    return f >= r->cap_lo && f < r->cap_lo + r->cap_len ? r->cap_t[f - r->cap_lo] : NAN;
}
static float cap_v_of(const rx_t *r, uint64_t f) {
    return f >= r->cap_lo && f < r->cap_lo + r->cap_len ? r->cap_v[f - r->cap_lo] : 0.0f;
}

// Frames of the capture with a nonzero generator sample: the first, the last,
// how many.
static unsigned gen_nonzero(const rx_t *r, uint64_t *first, uint64_t *last) {
    unsigned n = 0;
    *first = *last = 0;
    for (uint32_t i = 0; i < r->cap_len; i++) {
        if (r->cap_v[i] == 0.0f) continue;
        if (!n) *first = r->cap_lo + i;
        *last = r->cap_lo + i;
        n++;
    }
    return n;
}

// Unit i's run against unit 0's, sample for sample over the run: the largest
// value difference and the play-instant error in frames of the domain (the
// sender's rate), after the expected offset T_i - T_0 (plus `extra` frames).
static void gen_compare(const scenario_t *sc, const rx_t *a, uint64_t ta, const rx_t *b, uint64_t tb, double extra,
                        double *vdiff, double *emean, double *emax, unsigned *n) {
    double fsn = rate_hz(sc->rate) * (1.0 + sc->sender_ppm * 1e-6);
    uint32_t len = gen_len(sc->gen, rate_hz(sc->rate));
    double s = 0, m = 0, v = 0;
    unsigned k = 0;
    for (uint32_t j = 0; j < len; j++) {
        double t1 = cap_t_of(a, ta + j), t2 = cap_t_of(b, tb + j);
        double d = fabs((double)cap_v_of(a, ta + j) - (double)cap_v_of(b, tb + j));
        if (d > v) v = d;
        if (isnan(t1) || isnan(t2)) continue;
        double e = (t2 - t1) * fsn - ((double)(int64_t)(tb - ta) + extra);
        s += e;
        if (fabs(e) > m) m = fabs(e);
        k++;
    }
    *vdiff = v;
    *emean = k ? s / k : NAN;
    *emax = m;
    *n = k;
}

// The reference: the same config on a generator of its own (copy 4), started
// at once by a plain START and rendered in back-to-back blocks of 16, no
// time base, no ring.
static bool ref_ready(void) { return true; }
static uint16_t ref_lead(void) { return ARM_LEAD_MS; }
static uint32_t ref_zero(void) { return 0; }
static bool ref_ring(uint32_t *wr, uint32_t *played) { *wr = *played = 0; return false; }
static void ref_event(uint8_t tb, uint8_t st, uint8_t rs, uint32_t t) { (void)tb; (void)st; (void)rs; (void)t; }
static sg_port_t ref_port = { ref_ready, ref_lead, ref_zero, ref_zero, ref_ring, ref_event };

static float *gen_reference(const gen_plan_t *gp, uint32_t len, unsigned fs) {
    const sg_unit_t *g = SGU[4];
    *g->port = &ref_port;
    SiggenConfig c = gen_cfg(gp);
    float *ref = calloc(len + 16u, sizeof(float));
    if (!g->stage_config(&c, sizeof c) || !g->control(SIGGEN_CTL_START)) return ref;
    for (uint32_t i = 0; i < len; i += 16) g->render(ref + i, GEN_OUT, 16, fs);
    return ref;
}

// The programme's skew of unit i against unit 0 (frames) over the probes
// of [t1, t2]: what the lock error is at the time of the run.
static double prog_skew(const probe_log_t *log, int i, double t1, double t2) {
    double s = 0;
    unsigned k = 0;
    for (unsigned j = (unsigned)(t1 * 1e6 / PROBE_US); j <= (unsigned)(t2 * 1e6 / PROBE_US) && j < log[0].n; j++) {
        if (!log[0].aud[j] || !log[i].aud[j] || isnan(log[0].fplay[j]) || isnan(log[i].fplay[j])) continue;
        s += log[i].fplay[j] - log[0].fplay[j];
        k++;
    }
    return k ? s / k : NAN;
}

static void gen_checks(const scenario_t *sc, rx_t *r, const probe_log_t *log) {
    const gen_plan_t *gp = sc->gen;
    const int n = (int)nrx(sc);
    const unsigned fs = rate_hz(sc->rate);
    const uint64_t t0 = gen_t0(sc);
    const uint32_t len = gen_len(gp, fs);
    const float amp = powf(10.0f, -20.0f / 20.0f);
    // The line's lock error budget (PLAN-AT FWTB3): every unit plays the
    // run within 0.05 frame of the domain's timeline, here the mean of the
    // units' instants (so two units' offset stays under 0.1, LK6's bound).
    const double tol = 0.05;
    float *ref = gen_reference(gp, len, fs);
    uint32_t k0 = 0;
    while (k0 < len && ref[k0] == 0.0f) k0++;
    printf("   reference: the first nonzero sample is sample %u (%s)\n", k0,
           gp->type == SIGGEN_IMPULSE ? "impulse" : "the sweep's window starts at 0");
    uint64_t T[NRX_MAX];
    for (int i = 0; i < n; i++) {
        T[i] = t0 + (gp->broadcast ? 0 : (uint64_t)(int64_t)gp->stagger[i]);
        const rx_t *u = &r[i];
        const sg_timeline_t *tl = &u->tl_end;
        uint64_t f1, f2;
        unsigned nz = gen_nonzero(u, &f1, &f2);
        printf("   rx %c generator: T %" PRIu64 " (t0 %+" PRId64 "), timeline state %u result %s start %" PRIu64
               " samples %u disc %u run_id %u; 0xE2 op 9 applied %u late %u cancelled %u refused %u; "
               "%u nonzero samples played (frames %" PRIu64 "..%" PRIu64 ")\n",
               'A' + i, T[i], (int64_t)(T[i] - t0), tl->state, tl->result < 8 ? TL_RES[tl->result] : "?", tl->start,
               tl->samples, tl->disc, tl->run_id, u->op9_res[0], u->op9_res[1], u->op9_res[2], u->op9_res[3], nz, f1,
               f2);
        printf("   rx %c EXT_TIME_BASE: t - output_frames over %u reads, %u off by more than 1 sample; 0xE4 events %u "
               "(last state %u reason %u)\n",
               'A' + i, u->tb_inv_reads, u->tb_inv_bad, u->tb_events, u->tb_ev_state, u->tb_ev_reason);
        CHECK(u->tb_inv_reads > 1000 && u->tb_inv_bad == 0, "%s: rx %c t - output_frames constant while LOCKED (%u of %u)",
              sc->name, 'A' + i, u->tb_inv_bad, u->tb_inv_reads);
        CHECK(!u->want_end, "%s: rx %c the DAC is handed back at the end", sc->name, 'A' + i);
        if (gp->kind == GEN_LATE)
            CHECK(u->wrote_late && u->wr_code_past == TB_ERR_STATE && u->wr_code_late == TB_ERR_STATE,
                  "%s: rx %c a START in the past and one inside the arm lead refused at the write (0x%02x, 0x%02x)",
                  sc->name, 'A' + i, u->wr_code_past, u->wr_code_late);
    }
    bool silent_run = gp->kind == GEN_LATE && gp->broadcast;     // the broadcast came too late
    if (gp->kind == GEN_LOSS) {
        // Unit 0: its start was pending when its line went dark: cancelled,
        // nothing played.  Unit 1: running when its line went dark: stopped
        // (STOP_NOW), the discontinuity counted, silent from then on.
        const rx_t *a = &r[0], *b = &r[1];
        uint64_t f1, f2;
        CHECK(a->wr_code == TB_OK && a->tl_end.result == TB_RES_CANCELLED_LOSS && a->op9_res[ALINK_APPLY_CANCELLED] == 1 &&
                  a->tl_end.start == 0 && !a->tl_run_seen && gen_nonzero(a, &f1, &f2) == 0,
              "%s: rx A the pending START cancelled by the loss (result %u), nothing played", sc->name, a->tl_end.result);
        CHECK(b->wr_code == TB_OK && b->tl_run_seen && b->tl_run.start == T[1] && b->tl_end.result == TB_RES_STOPPED_LOSS &&
                  b->tl_end.disc >= 1 && b->tl_end.start == T[1],
              "%s: rx B started on T, stopped by the loss, the timeline counts it (result %u, disc %u)", sc->name,
              b->tl_end.result, b->tl_end.disc);
        unsigned nz = gen_nonzero(b, &f1, &f2);
        double t_last = cap_t_of(b, f2);
        printf("   rx B: last nonzero generator sample frame %" PRIu64 " played at %.4f s (line dark from %.4f s, D %u)\n",
               f2, t_last, sc->loss_from[1], sc->d_local);
        CHECK(nz > 0 && f1 == T[1] + k0 && f2 < T[1] + len - 1 && b->tl_end.samples < len,
              "%s: rx B played from T and stopped before the end (%u samples of %u)", sc->name, b->tl_end.samples, len);
        // The ring holds D + a block of samples rendered before the stop.
        CHECK(t_last < sc->loss_from[1] + (sc->d_local + 64.0) / fs + 0.002,
              "%s: rx B silent within D + a block of the line going dark (%.4f s)", sc->name, t_last);
        double vd = 0;
        for (uint64_t f = T[1]; f <= f2; f++) vd = fmax(vd, fabs((double)cap_v_of(b, f) - ref[f - T[1]]));
        CHECK(vd <= 2e-4, "%s: rx B up to the stop the reference's samples (%.3g)", sc->name, vd);
        free(ref);
        return;
    }
    // Every unit started on its T, or (broadcast too late) none did.
    for (int i = 0; i < n; i++) {
        const rx_t *u = &r[i];
        uint64_t f1, f2;
        unsigned nz = gen_nonzero(u, &f1, &f2);
        if (silent_run) {
            CHECK(u->snd.gen_sched_ok && u->op9_res[ALINK_APPLY_REFUSED] == 1 &&
                      u->tl_end.result == TB_RES_REFUSED_NOT_READY && u->tl_end.run_id == 0 && nz == 0,
                  "%s: rx %c the broadcast START seen inside the arm lead refused (0xE2 refused, result %u), silent",
                  sc->name, 'A' + i, u->tl_end.result);
            continue;
        }
        if (!gp->broadcast) CHECK(u->wr_code == TB_OK, "%s: rx %c op 9 accepted (0x%02x)", sc->name, 'A' + i, u->wr_code);
        CHECK(u->tl_end.result == TB_RES_STARTED && u->tl_end.start == T[i] && u->tl_end.tb == TB_ID_LINE &&
                  u->tl_end.disc == 0 && u->op9_res[ALINK_APPLY_APPLIED] == 1 && u->tl_end.run_id == 1,
              "%s: rx %c started on T %" PRIu64 " (start %" PRIu64 ", result %u, disc %u)", sc->name, 'A' + i, T[i],
              u->tl_end.start, u->tl_end.result, u->tl_end.disc);
        unsigned miss = 0, twice = 0;
        for (uint32_t j = 0; j < len; j++) {
            uint64_t f = T[i] + j;
            uint8_t c = f >= u->cap_lo && f < u->cap_lo + u->cap_len ? u->cap_n[f - u->cap_lo] : 0;
            miss += c == 0;
            twice += c > 1;
        }
        CHECK(miss == 0 && twice == 0, "%s: rx %c every frame of the run played once (%u missing, %u twice)", sc->name,
              'A' + i, miss, twice);
        CHECK(nz > 0 && f1 == T[i] + k0, "%s: rx %c no generator sample before T, the first on T + %u (frame %" PRIu64 ")",
              sc->name, 'A' + i, k0, f1);
        // Sample for sample the reference's run (impulses exact; a sweep's
        // window gain is interpolated per block, so block splits round
        // differently by a few 1e-5).
        double vd = 0;
        for (uint32_t j = 0; j < len; j++) vd = fmax(vd, fabs((double)cap_v_of(u, T[i] + j) - ref[j]));
        CHECK(vd <= (gp->type == SIGGEN_IMPULSE ? 0.0 : 2e-4), "%s: rx %c the reference's samples on T.. (%.3g)", sc->name,
              'A' + i, vd);
        if (gp->type == SIGGEN_IMPULSE)
            CHECK(fabsf(cap_v_of(u, T[i]) - amp) < 1e-6f && nz == gp->repeat,
                  "%s: rx %c the impulse train: sample 0 on T at -20 dBFS, %u impulses (%u)", sc->name, 'A' + i, gp->repeat,
                  nz);
    }
    if (silent_run) {
        free(ref);
        return;
    }
    // Sample for sample against unit 0: the same samples on the same frames
    // of each run, played at the same instants within the lock error (units
    // behind an uncompensated fan-out: hop - 1 frames later).
    double em_of[NRX_MAX] = { 0 }, ex_of[NRX_MAX] = { 0 };
    for (int i = 1; i < n; i++) {
        double extra = sc->hop_uncompensated[i] ? (double)sc->hop_frames[i] - 1.0 : 0.0;
        double vd, em, ex;
        unsigned k;
        gen_compare(sc, &r[0], T[0], &r[i], T[i], extra, &vd, &em, &ex, &k);
        // The programme's skew over the run (fplay_i - fplay_A: B ahead is
        // positive, so its instants come earlier: e = -skew).
        double t1 = cap_t_of(&r[0], T[0]), t2 = cap_t_of(&r[0], T[0] + len - 1);
        double ps = prog_skew(log, i, t1, t2) + extra;
        printf("   rx %c against A: %u frames, the samples differ by %.3g at most; play instants %+.4f frame mean, "
               "%.4f max off T_%c - T_A = %+" PRId64 "%s; the programme's skew then %+.4f\n",
               'A' + i, k, vd, em, ex, 'A' + i, (int64_t)(T[i] - T[0]), extra ? " + the uncompensated hop" : "", ps);
        // The generator adds nothing to the lock error: its instants are
        // the programme's (within the probes' frame interpolation).
        CHECK(fabs(em + ps) < 0.01, "%s: rx %c the generator's offset is the programme's (%+.4f vs %+.4f)", sc->name,
              'A' + i, em, -ps);
        // Impulses are exact; a sweep's window gain is interpolated per
        // block, so two units' different block splits round differently.
        double vtol = gp->type == SIGGEN_IMPULSE ? 0.0 : 2e-4;
        CHECK(k == len && vd <= vtol, "%s: rx %c the same samples as A, sample for sample (%.3g)", sc->name, 'A' + i, vd);
        em_of[i] = em;
        ex_of[i] = ex;
        if (extra) {
            // The contrast (H5 on the model): the same run without the
            // compensation reads exactly hop - 1 whole frames late.
            double vd0, em0, ex0;
            unsigned k0;
            gen_compare(sc, &r[0], T[0], &r[i], T[i], 0.0, &vd0, &em0, &ex0, &k0);
            CHECK(fabs(em0 - extra) < tol, "%s: rx %c uncompensated: %.3f frames late (hop - 1 = %.0f)", sc->name,
                  'A' + i, em0, extra);
        }
    }
    // Each unit against the domain's timeline (the mean of the units'
    // instants; an uncompensated unit is the contrast, not a member).
    double mean = 0;
    int m = 0;
    for (int i = 0; i < n; i++)
        if (!sc->hop_uncompensated[i]) mean += em_of[i], m++;
    mean /= m;
    for (int i = 0; i < n; i++) {
        if (sc->hop_uncompensated[i]) continue;
        double dev = em_of[i] - mean;
        printf("   rx %c: the run's instants %+.4f frame off the domain's (the mean of %d units)\n", 'A' + i, dev, m);
        CHECK(fabs(dev) < tol && ex_of[i] < 2.0 * tol,
              "%s: rx %c plays the run within the lock error of the timeline (%+.4f frame, under %.2f; any sample "
              "against A %.4f, under %.2f)", sc->name, 'A' + i, dev, tol, ex_of[i], 2.0 * tol);
    }
    free(ref);
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
    const int NRX = (int)nrx(sc);
    printf("   L%u %u Hz C %u, sender %+.0f ppm; rx A %+.0f ppm boot %.2f s, rx B %+.0f ppm boot %.2f s; mode %u, D %u\n",
           sc->cls, rate_hz(sc->rate), sc->c, sc->sender_ppm, sc->rx_ppm[0], sc->boot_s[0], sc->rx_ppm[1],
           sc->boot_s[1], sc->mode, sc->d_local);
    for (int i = 2; i < NRX; i++)
        printf("   rx %c %+.0f ppm boot %.2f s\n", 'A' + i, sc->rx_ppm[i], sc->boot_s[i]);
    for (int i = 0; i < NRX; i++)
        if (sc->hop_frames[i])
            printf("   rx %c behind a fan-out: hop delay %u frames, %s\n", 'A' + i, sc->hop_frames[i],
                   sc->hop_uncompensated[i] ? "NOT compensated (hops_offset 0)" : "hops_offset -(hop - 1)");
    static rx_t r[NRX_MAX];
    probe_log_t log[NRX_MAX];
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
    if (sc->gen) {
        // The generator scenarios check their own (gen_checks): the run here
        // is short and starts the generators in its last seconds.
    } else if (sc->expect_skew >= 0) {
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
        if (sc->loss_to[i] <= sc->loss_from[i] || sc->gen) continue;     // GEN_LOSS: gen_checks()
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
    if (sc->gen) gen_checks(sc, r, log);
    if (wav_path) write_wav(sc, log);
    for (int i = 0; i < NRX; i++) {
        free(log[i].fplay);
        free(log[i].aud);
        free(log[i].lat);
        free(r[i].cap_t);
        free(r[i].cap_v);
        free(r[i].cap_n);
    }
    free(sk);
    free(la);
    free(lb);
    (void)both;
}

// Every scenario in its own process: the generator copies' file statics
// start fresh (WP FWTB3).  The child's checks and failures come back by a pipe.
static void run_forked(const scenario_t *sc) {
    fflush(NULL);
    int fd[2];
    if (pipe(fd) != 0) { perror("pipe"); exit(2); }
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); exit(2); }
    if (pid == 0) {
        close(fd[0]);
        checks = failures = 0;
        run_scenario(sc);
        fflush(NULL);
        int v[2] = { checks, failures };
        ssize_t w = write(fd[1], v, sizeof v);
        _exit(w == (ssize_t)sizeof v ? 0 : 3);
    }
    close(fd[1]);
    int v[2] = { 0, 0 }, st = 0;
    ssize_t got = read(fd[0], v, sizeof v);
    close(fd[0]);
    waitpid(pid, &st, 0);
    if (got != (ssize_t)sizeof v || !WIFEXITED(st) || WEXITSTATUS(st) != 0) {
        printf("  FAIL %s: the scenario's process died (status 0x%x)\n", sc->name, st);
        v[0]++;
        v[1]++;
    }
    checks += v[0];
    failures += v[1];
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
        // WP FWTB3: the generator on the time base.  Ops 9 written about 0.5 s
        // ahead (EXT_APPLY_AT_FRAME per unit) unless broadcast.
        { .name = "gen: two units, impulses at one T", .cls = 1, .rate = 1, .c = 8, .sender_ppm = 20,
          .rx_ppm = { 45, -38 }, .boot_s = { 0.3, 1.1 }, .mode = 1, .d_local = 128, .duration_s = 31.5,
          .gen = &(const gen_plan_t){ GEN_COINCIDE, SIGGEN_IMPULSE, 5, 30.0, 29.5, { 0 }, false } },
        { .name = "gen: two units, log sweeps at one T", .cls = 1, .rate = 1, .c = 8, .sender_ppm = -12,
          .rx_ppm = { -30, 52 }, .boot_s = { 1.4, 0.2 }, .mode = 1, .d_local = 128, .duration_s = 32.0,
          .gen = &(const gen_plan_t){ GEN_COINCIDE, SIGGEN_SWEEP_LOG, 3, 30.0, 29.5, { 0 }, false } },
        { .name = "gen: four units, impulses at one T", .nrx = 4, .cls = 1, .rate = 1, .c = 8, .sender_ppm = 20,
          .rx_ppm = { 45, -38, 12, -90 }, .boot_s = { 0.3, 1.1, 2.2, 0.7 }, .mode = 1, .d_local = 128,
          .duration_s = 31.5, .gen = &(const gen_plan_t){ GEN_COINCIDE, SIGGEN_IMPULSE, 5, 30.0, 29.5, { 0 }, false } },
        { .name = "gen: four units, L3 96 kHz C 16, log sweeps at one T", .nrx = 4, .cls = 3, .rate = 2, .c = 16,
          .sender_ppm = -15, .rx_ppm = { 25, -40, 60, -5 }, .boot_s = { 0.2, 2.9, 1.3, 0.6 }, .mode = 1,
          .d_local = 200, .duration_s = 32.0,
          .gen = &(const gen_plan_t){ GEN_COINCIDE, SIGGEN_SWEEP_LOG, 3, 30.0, 29.5, { 0 }, false } },
        { .name = "gen: staggered starts (B +4807, C +9613, D -3 frames)", .nrx = 4, .cls = 1, .rate = 1, .c = 8,
          .sender_ppm = 7, .rx_ppm = { -20, 33, 71, -64 }, .boot_s = { 0.5, 0.0, 1.9, 1.2 }, .mode = 1,
          .d_local = 128, .duration_s = 31.5,
          .gen = &(const gen_plan_t){ GEN_COINCIDE, SIGGEN_IMPULSE, 5, 30.0, 29.5, { 0, 4807, 9613, -3 }, false } },
        { .name = "gen: broadcast op 9 in the superframe, two units", .cls = 1, .rate = 1, .c = 8, .sender_ppm = 20,
          .rx_ppm = { 45, -38 }, .boot_s = { 0.3, 1.1 }, .mode = 1, .d_local = 128, .duration_s = 31.5,
          .gen = &(const gen_plan_t){ GEN_COINCIDE, SIGGEN_IMPULSE, 5, 30.0, 29.6, { 0 }, true } },
        { .name = "gen: fan-out: B behind a 3-frame hop (compensated), C behind one (not)", .nrx = 3, .cls = 1,
          .rate = 1, .c = 8, .sender_ppm = 20, .rx_ppm = { 45, -38, 17 }, .boot_s = { 0.3, 1.1, 0.8 }, .mode = 1,
          .d_local = 128, .duration_s = 31.5, .hop_frames = { 0, 3, 3 }, .hop_uncompensated = { false, false, true },
          .gen = &(const gen_plan_t){ GEN_COINCIDE, SIGGEN_IMPULSE, 5, 30.0, 29.5, { 0 }, false } },
        { .name = "gen: loss: A's pending START cancelled, B's run stopped", .cls = 1, .rate = 1, .c = 8,
          .sender_ppm = 20, .rx_ppm = { 45, -38 }, .boot_s = { 0.3, 1.1 }, .loss_from = { 30.2, 30.3 },
          .loss_to = { 30.4, 30.302 }, .mode = 1, .d_local = 128, .duration_s = 33.0,
          .gen = &(const gen_plan_t){ GEN_LOSS, SIGGEN_SWEEP_LOG, 3, 30.0, 29.5, { 72000, 0 }, false } },
        { .name = "gen: late STARTs refused at the write, then on T", .cls = 1, .rate = 1, .c = 8, .sender_ppm = 20,
          .rx_ppm = { 45, -38 }, .boot_s = { 0.3, 1.1 }, .mode = 1, .d_local = 128, .duration_s = 31.5,
          .gen = &(const gen_plan_t){ GEN_LATE, SIGGEN_IMPULSE, 5, 30.0, 29.5, { 0 }, false } },
        { .name = "gen: broadcast op 9 with a 100 ms lead refused, silent", .cls = 1, .rate = 1, .c = 8,
          .sender_ppm = 20, .rx_ppm = { 45, -38 }, .boot_s = { 0.3, 1.1 }, .mode = 1, .d_local = 128,
          .duration_s = 31.5, .gen = &(const gen_plan_t){ GEN_LATE, SIGGEN_IMPULSE, 5, 30.0, 29.9, { 0 }, true } },
    };
    for (unsigned i = 0; i < sizeof(sc) / sizeof(sc[0]); i++) {
        if (only && !strstr(sc[i].name, only)) continue;
        run_forked(&sc[i]);
    }
    if (dump_f) fclose(dump_f);
    printf("\nalink_phase_sim: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
