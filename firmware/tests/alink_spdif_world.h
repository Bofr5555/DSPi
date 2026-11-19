// alink_spdif_world.h - a model world for the S/PDIF time-base roles (WP FWSP2)
//
// Shared by the host test (firmware/tests/test_alink_spdif.c) and the
// simulation (tools/alink_spdif_sim): one S/PDIF source and N followers, each
// follower running the real alink_spdif.c behind the port below.
//
// What runs as in the firmware: the roles (alink_spdif.c), the phase lock and
// apply queue (alink/alink_phase.c), the carriage codec (alink_spdif_codec.c),
// the source's staging (alink_spdif_src_stage()) and the filler's lookup
// (alink_spdif_src_staged()), and the main loop's S/PDIF follower handshake
// (main.c: mute, drain, prefill, start in sync, the orphan restart),
// mirrored in sw_pass().
// What is modelled: the source's crystal; the wire (each 32-bit subframe
// word lands in the follower's receive FIFO when its last bit has arrived,
// plus a white timing jitter: the RX quantisation is the word, half a frame);
// the receive FIFO (8 blocks of 384 words, the live reader withholding the
// newest word as spdif_rx_read_fifo_live() does); each follower's crystal, the
// soft VCXO (0.5 ppm per 75 ms tick, 2 ppm while muted and pulling in), the
// rate feed-forward (the true ratio plus an error), main-loop passes with
// jitter, slot 0's output ring (448 frames writable) and its reader.
//
// The audio carries F: the source puts F's low 24 bits in channel A, the
// next 24 in channel B, so a follower's output shows which F it plays.
#ifndef ALINK_SPDIF_WORLD_H
#define ALINK_SPDIF_WORLD_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alink/alink_phase.c"
#include "alink/alink_spdif_codec.c"
#include "alink_spdif.c"

#define SW_FIFO_WORDS   (8u * ALINK_SPDIF_BLOCK_WORDS)
#define SW_RING         4096u           // model ring storage (the writable span is 448)
#define SW_SPAN         448u
#define SW_SILENCE      UINT64_MAX
#define SW_MAX_FOL      4
#define SW_OPLOG        32

typedef struct {
    alink_spdif_t u;
    double ppm;                 // crystal
    double vcxo;                // soft VCXO correction now, ppm
    double ff_err;              // the feed-forward's error, ppm
    double lock_t;              // when the receiver locked (s), < 0 not locked
    double pass_us, pass_jit_us;
    double next_pass;
    double next_tick;
    // The wire into this follower: on/off, corrupt U bits, words lost.
    bool   wire;
    int    kill_ubits;          // blocks whose U bits arrive zeroed (counted down)
    bool   foreign;             // U bits always zero (a foreign source)
    // Receive FIFO
    uint32_t fifo[SW_FIFO_WORDS];
    uint64_t wr_words, rd_words;
    uint64_t next_word;         // source word index the next arrival is
    bool     rx_locked;
    // Output ring (slot 0)
    bool     out_running;
    uint64_t ring_wr;
    double   play_pos;          // frames played since the ring reset (fractional)
    uint64_t ring_f[SW_RING];
    uint32_t target, cap, batch;
    // main.c's handshake
    bool     resetting;         // prepare_pipeline_reset() armed (mute, waiting for the DAC hold)
    double   reset_t;
    uint32_t underruns, overflows, idle_fill, n_reads;
    uint32_t mute_refresh;
    // Apply log: op, frame, and the F of the next frame written when it ran.
    int      n_op;
    struct { uint8_t op, late; uint64_t frame, next_f; uint32_t ring_pos; } oplog[SW_OPLOG];
    int      n_res;
    struct { uint8_t op, res; uint32_t frame; } reslog[SW_OPLOG];
} sw_fol_t;

typedef struct {
    alink_spdif_t u;
    double ppm;
    double t0;                  // when its outputs started (s)
    uint64_t x0;                // ring position at t0
    bool   running;
} sw_src_t;

static struct {
    double t;                   // seconds
    uint32_t fs;
    sw_src_t src;
    sw_fol_t fol[SW_MAX_FOL];
    int nfol;
    double word_jit_us;
    uint64_t rng;
} SW;

static sw_fol_t *SWF;           // the follower the port serves now

static double sw_rand(void) {
    SW.rng ^= SW.rng << 13; SW.rng ^= SW.rng >> 7; SW.rng ^= SW.rng << 17;
    return (double)(SW.rng >> 11) * (1.0 / 9007199254740992.0);
}

// ---------------------------------------------------------------------------
// The source: frames, words and their U bits
// ---------------------------------------------------------------------------

static double sw_src_rate(void) { return (double)SW.fs * (1.0 + SW.src.ppm * 1e-6); }

// Source ring position (frames since its synchronized start) at time t.
static double sw_src_pos(double t) { return (double)SW.src.x0 + (t - SW.src.t0) * sw_src_rate(); }

// The source's word w (2 per frame, counted from its start) as the TX
// library stamps it: audio = F, U from the block's staged superframe (zero
// when not staged, as an unfilled table entry), the B preamble at a block's
// first subframe.
static uint32_t sw_src_word(uint64_t w, bool kill_u) {
    uint64_t x = w / 2u;                    // ring position
    unsigned sub = (unsigned)(w % 2u);
    uint64_t f = alink_spdif_src_f(&SW.src.u, (uint32_t)x);
    uint32_t b = (uint32_t)(x / ALINK_SPDIF_BLOCK_FRAMES);
    unsigned k = (unsigned)(x % ALINK_SPDIF_BLOCK_FRAMES);
    const uint8_t *sf = SW.src.u.mode == ALINK_SPDIF_MODE_SOURCE ? alink_spdif_src_staged(&SW.src.u, b) : NULL;
    unsigned ub = (sf && !kill_u) ? (alink_spdif_frame_ubits(sf, k) >> sub) & 1u : 0u;
    uint32_t audio = sub ? (uint32_t)(f >> 24) & 0xFFFFFFu : (uint32_t)f & 0xFFFFFFu;
    uint32_t sync = (k == 0 && sub == 0) ? 0xFu : sub ? 0x7u : 0xBu;
    return sync | audio << 4 | (uint32_t)ub << 29;
}

// The source restarts its outputs at time t (an epoch change): its carrier
// stops for `gap` seconds (drain, prefill), every follower's receiver loses
// the stream and locks again on the new one.
static void sw_src_restart(double t, double gap) {
    uint64_t f_now = alink_spdif_src_f(&SW.src.u, (uint32_t)sw_src_pos(t));
    alink_spdif_src_stopping(&SW.src.u, f_now);
    SW.src.t0 = t + gap;
    SW.src.x0 = 0;
    alink_spdif_src_started(&SW.src.u);
    alink_spdif_src_stage(&SW.src.u, 0, false);
    for (int i = 0; i < SW.nfol; i++) {
        SW.fol[i].next_word = 0;
        SW.fol[i].rx_locked = false;
        SW.fol[i].lock_t = -1.0;
    }
}

static void sw_src_init(double ppm) {
    alink_spdif_init(&SW.src.u, 0x42, 0x37);
    SW.src.u.fs = SW.fs;
    uint8_t c[8] = { 2, 1, 0, 0, 0, 0, 0, 0 };
    (void)alink_spdif_set_config(&SW.src.u, c, 0x0F);
    SW.src.ppm = ppm;
    SW.src.t0 = 0.0;
    SW.src.x0 = 0;
    alink_spdif_src_stopping(&SW.src.u, 0);
    alink_spdif_src_started(&SW.src.u);
    alink_spdif_src_stage(&SW.src.u, 0, false);
    SW.src.running = true;
}

// ---------------------------------------------------------------------------
// The port (alink_spdif.h), serving SWF
// ---------------------------------------------------------------------------

const alink_apply_limits_t alink_spdif_apply_limits = { 10, 9, 2048, true };
uint32_t alink_spdif_port_irq_save(void) { return 0; }
void alink_spdif_port_irq_restore(uint32_t s) { (void)s; }
uint32_t alink_spdif_port_now_us(void) { return (uint32_t)(int64_t)(SW.t * 1e6); }

uint32_t alink_spdif_port_rx_avail(void) {
    if (!SWF || !SWF->rx_locked || SWF->wr_words <= SWF->rd_words) return 0;
    uint64_t live = SWF->wr_words - 1u - SWF->rd_words;     // the newest word withheld
    return (uint32_t)(live / 2u);
}

uint32_t alink_spdif_port_rx_read(uint32_t n, const uint32_t **w, uint32_t *idx) {
    uint32_t a = alink_spdif_port_rx_avail();
    SWF->n_reads++;
    if (n > a) n = a;
    uint32_t i = (uint32_t)(SWF->rd_words % SW_FIFO_WORDS);
    uint32_t room = (SW_FIFO_WORDS - i) / 2u;
    if (n > room) n = room;
    *w = &SWF->fifo[i];
    *idx = i;
    SWF->rd_words += 2u * n;
    return n;
}

bool alink_spdif_port_rx_ahead(uint32_t *words) {
    if (!SWF->rx_locked) return false;
    *words = (uint32_t)(SWF->wr_words - SWF->rd_words);
    return true;
}

static uint64_t sw_word_f(const uint32_t *w) {
    return (uint64_t)((w[0] >> 4) & 0xFFFFFFu) | (uint64_t)((w[1] >> 4) & 0xFFFFFFu) << 24;
}

void alink_spdif_port_deliver(const uint32_t *w, uint32_t k) {
    sw_fol_t *f = SWF;
    for (uint32_t i = 0; i < k; i++) {
        uint32_t fill = (uint32_t)(f->ring_wr - (uint64_t)f->play_pos);
        if (fill >= SW_SPAN) { f->overflows++; continue; }
        f->ring_f[f->ring_wr % SW_RING] = w ? sw_word_f(w + 2u * i) : SW_SILENCE;
        f->ring_wr++;
    }
}

uint32_t alink_spdif_port_bounds(uint32_t *min) {
    sw_fol_t *f = SWF;
    uint32_t max = f->cap;
    if (!f->out_running) {
        uint32_t fill = (uint32_t)f->ring_wr;
        uint32_t room = fill < f->target ? f->target - fill : 0;
        if (room < max) max = room;
    }
    *min = f->batch < max ? f->batch : max;
    return max;
}

bool alink_spdif_port_play(float *fill, uint32_t *played) {
    *fill = (float)((double)SWF->ring_wr - SWF->play_pos);
    *played = (uint32_t)SWF->play_pos;
    return true;
}
uint32_t alink_spdif_port_fill(void) { return (uint32_t)(SWF->ring_wr - (uint64_t)SWF->play_pos); }
uint32_t alink_spdif_port_ring_span(void) { return SW_SPAN; }

#ifndef SW_HAVE_GENERATOR
uint8_t alink_spdif_port_op9_accept(uint64_t t, uint64_t t_now, uint32_t fs) { (void)t; (void)t_now; (void)fs; return TB_OK; }
bool alink_spdif_port_op9_broadcast(uint64_t t, uint64_t t_now, uint32_t fs) { (void)t; (void)t_now; (void)fs; return true; }
void alink_spdif_port_op9_poll(uint64_t t_now, uint32_t fs) { (void)t_now; (void)fs; }
void alink_spdif_port_cancel_all(void) {}
void alink_spdif_port_op9_lost(void) {}
static uint8_t sw_fire(const alink_apply_op_t *op, bool late, bool locked) { (void)op; (void)late; return locked ? ALINK_APPLY_APPLIED : ALINK_APPLY_REFUSED; }
#endif

static int sw_src_n_op;
static struct { uint8_t op, late; uint64_t frame; } sw_src_oplog[SW_OPLOG];
uint8_t alink_spdif_port_apply(const alink_apply_op_t *op, bool late, bool locked) {
    sw_fol_t *f = SWF;
    if (!f && sw_src_n_op < SW_OPLOG) {
        sw_src_oplog[sw_src_n_op].op = op->op;
        sw_src_oplog[sw_src_n_op].late = late;
        sw_src_oplog[sw_src_n_op].frame = op->frame;
        sw_src_n_op++;
    }
    if (f && f->n_op < SW_OPLOG) {
        f->oplog[f->n_op].op = op->op;
        f->oplog[f->n_op].late = late;
        f->oplog[f->n_op].frame = op->frame;
        f->oplog[f->n_op].next_f = f->u.ph.next_w;
        f->oplog[f->n_op].ring_pos = (uint32_t)f->ring_wr;
        f->n_op++;
    }
    return sw_fire(op, late, locked);
}

void alink_spdif_port_apply_result(uint8_t op, uint8_t result, uint32_t frame) {
    sw_fol_t *f = SWF;
    if (f && f->n_res < SW_OPLOG) {
        f->reslog[f->n_res].op = op;
        f->reslog[f->n_res].res = result;
        f->reslog[f->n_res].frame = frame;
        f->n_res++;
    }
}

void alink_spdif_port_ring_target(void) {
    sw_fol_t *f = SWF;
    f->target = alink_spdif_ring_target(&f->u, f->target, f->cap, f->batch, SW_SPAN);
}
bool alink_spdif_port_usb_live(void) { return false; }
bool alink_spdif_port_rate_ready(void) {
    sw_fol_t *f = SWF;
    if (f->lock_t < 0 || SW.t - f->lock_t < 1.0) return false;     // the long rate window (1 s)
    double ff = (SW.src.ppm - f->ppm) + f->ff_err;
    return fabs(f->vcxo - ff) < 1.0;
}

bool timebase_port_ring(uint32_t *wr, uint32_t *played) {
    if (!SWF) {
        // The source's slot 0: its reader now, written 256 frames ahead.
        double x = sw_src_pos(SW.t);
        *played = (uint32_t)x;
        *wr = *played + 256u;
        return SW.src.running;
    }
    *wr = (uint32_t)SWF->ring_wr;
    *played = (uint32_t)SWF->play_pos;
    return SWF->out_running;
}
#ifndef SW_HAVE_TIMEBASE
void timebase_register(unsigned slot, const tb_provider_t *p) { (void)slot; (void)p; }
#endif

// ---------------------------------------------------------------------------
// The followers
// ---------------------------------------------------------------------------

static void sw_fol_init(sw_fol_t *f, double ppm, uint16_t d, double pass_us) {
    memset(f, 0, sizeof(*f));
    alink_spdif_init(&f->u, (uint8_t)(0x10 + (f - SW.fol)), 0x01);
    f->u.active = true;
    f->u.fs = SW.fs;
    f->cap = SW.fs / 1000u + 1u;
    f->batch = SW.fs / 1000u;
    f->target = SW.fs / 250u;           // 4 ms
    alink_spdif_limits(&f->u, f->cap, f->batch, SW_SPAN);
    uint8_t c[8] = { 1, 1, (uint8_t)d, (uint8_t)(d >> 8), 0, 0, 0, 0 };
    SWF = f;
    uint8_t r = alink_spdif_set_config(&f->u, c, 0);
    if (r) printf("  (follower config refused: %u, d %u..%u)\n", r, f->u.ph.d_min, f->u.ph.d_max);
    f->ppm = ppm;
    f->ff_err = 0.0;
    f->lock_t = -1.0;
    f->pass_us = pass_us;
    f->pass_jit_us = pass_us * 0.5;
    f->wire = true;
    f->out_running = true;               // the outputs run (stock) before the follower anchors
    f->next_tick = 0.075;
}

// Words that have landed by time t (with jitter), into the FIFO.
static void sw_fol_receive(sw_fol_t *f, double t) {
    if (!f->wire) {
        // No carrier: the receiver loses the stream; the words are gone.
        double wrate = 2.0 * sw_src_rate();
        f->rx_locked = false;
        f->lock_t = -1.0;
        double w = (t - SW.src.t0) * wrate + 2.0 * (double)SW.src.x0;
        if (w > (double)f->next_word) f->next_word = (uint64_t)w;
        return;
    }
    double wrate = 2.0 * sw_src_rate();
    for (;;) {
        uint64_t w = f->next_word;
        double tw = SW.src.t0 + ((double)w + 1.0 - 2.0 * (double)SW.src.x0) / wrate;   // its last bit
        tw += (sw_rand() - 0.5) * SW.word_jit_us * 1e-6;
        if (tw > t) break;
        bool kill = f->foreign || f->kill_ubits > 0;
        uint32_t x = sw_src_word(w, kill);
        if ((w % ALINK_SPDIF_BLOCK_WORDS) == ALINK_SPDIF_BLOCK_WORDS - 1u && f->kill_ubits > 0) f->kill_ubits--;
        if (f->rx_locked) {
            if (f->wr_words - f->rd_words >= SW_FIFO_WORDS) f->rd_words += ALINK_SPDIF_BLOCK_WORDS;   // overflow disposal
            f->fifo[f->wr_words % SW_FIFO_WORDS] = x;
            f->wr_words++;
        }
        f->next_word = w + 1u;
        // The receiver locks at a block start (the library aligns on the B preamble).
        if (!f->rx_locked && (w + 1u) % ALINK_SPDIF_BLOCK_WORDS == 0) {
            f->rx_locked = true;
            f->lock_t = t;
            f->wr_words = f->rd_words = 0;
        }
    }
}

static double sw_fol_rate(const sw_fol_t *f) {
    return (double)SW.fs * (1.0 + f->ppm * 1e-6) * (1.0 + f->vcxo * 1e-6);
}

// The output reader from t0 to t1.
static void sw_fol_play(sw_fol_t *f, double dt) {
    if (!f->out_running) return;
    f->play_pos += dt * sw_fol_rate(f);
    if (f->play_pos + 1.0 > (double)f->ring_wr) {
        // The rings ran dry: the idle pump keeps them silent (a fault while
        // the follower owns them).
        if (alink_spdif_owns_rings(&f->u)) f->underruns++;
        while ((double)f->ring_wr < f->play_pos + 8.0) f->ring_f[f->ring_wr++ % SW_RING] = SW_SILENCE;
        f->idle_fill++;
    }
}

// The F the follower plays at its output now (fractional), or -1 (silence).
static double sw_fol_playing(const sw_fol_t *f) {
    if (!f->out_running) return -1.0;
    uint64_t p = (uint64_t)f->play_pos;
    uint64_t v = f->ring_f[p % SW_RING];
    if (v == SW_SILENCE) return -1.0;
    return (double)v + (f->play_pos - (double)p);
}

// The F the source plays at its output now.
static double sw_src_playing(void) {
    double x = sw_src_pos(SW.t);
    return (double)alink_spdif_src_f(&SW.src.u, 0) + x;
}

static void (*sw_pass_hook)(sw_fol_t *f);

static void sw_drain(sw_fol_t *f) {
    f->out_running = false;
    f->ring_wr = 0;
    f->play_pos = 0.0;
}

// One main-loop pass of a follower (mirrors main.c's S/PDIF follower
// handshake and the S/PDIF servo).
static void sw_pass(sw_fol_t *f) {
    SWF = f;
    uint32_t now = alink_spdif_port_now_us();
    alink_spdif_rx_state(&f->u, f->rx_locked, f->rx_locked ? alink_spdif_rate_code(SW.fs) : 0xFF, SW.fs, now);
    if (f->rx_locked && SW.t - f->lock_t > 0.002) (void)alink_spdif_poll(&f->u, now);
    alink_spdif_service(&f->u, now);
    if (alink_spdif_muted(&f->u)) f->mute_refresh++;
    if (alink_spdif_outputs_orphaned(&f->u)) {
        f->out_running = true;
        alink_spdif_on_restarted(&f->u);
    }
    if (alink_spdif_anchor_wanted(&f->u)) {
        if (!f->resetting) {
            f->resetting = true;        // prepare_pipeline_reset(): mute, DAC hold
            f->reset_t = SW.t;
        } else if (SW.t - f->reset_t > 0.005) {
            f->resetting = false;
            sw_drain(f);                // drain_and_disable_outputs()
            alink_spdif_on_drain(&f->u);
        }
    } else {
        f->resetting = false;
    }
    if (f->u.prefilling && f->rx_locked && (uint32_t)f->ring_wr >= f->target) {
        float wait = alink_spdif_anchor_wait(&f->u);
        double fr = sw_fol_rate(f);
        if (wait <= (float)(SW.fs / 1000u)) {
            // The last millisecond in place (main.c spins on anchor_wait()).
            int spin = 0;
            while (wait > 0.0f && spin++ < 2000) {
                for (int j = 0; j < SW.nfol; j++) sw_fol_play(&SW.fol[j], 1e-6);
                SW.t += 1e-6;
                for (int j = 0; j < SW.nfol; j++) sw_fol_receive(&SW.fol[j], SW.t);
                SWF = f;
                wait = alink_spdif_anchor_wait(&f->u);
            }
            (void)fr;
            f->out_running = true;      // enable_outputs_in_sync()
            alink_spdif_on_enabled(&f->u);
        }
    }
    if (sw_pass_hook) sw_pass_hook(f);
}

// The 75 ms servo tick: the feed-forward plus the phase trim.
static void sw_tick(sw_fol_t *f) {
    SWF = f;
    double ff = (SW.src.ppm - f->ppm) + f->ff_err;
    float trim = 0.0f;
    bool hold = false;
    bool phase = alink_spdif_servo(&f->u, &trim, &hold);
    // No long rate window yet (1 s after the lock), or a holdover: the VCXO
    // stays where it is (the glue's rule, as the line's after a relock).
    if (hold || f->lock_t < 0 || SW.t - f->lock_t < 1.0) return;
    double want = ff + (phase ? (double)trim : 0.0);
    double slew = (alink_spdif_muted(&f->u) && !f->u.ph.started) ? 2.0 : 0.5;
    double d = want - f->vcxo;
    if (d > slew) d = slew;
    if (d < -slew) d = -slew;
    f->vcxo += d;
}

// Run the world until time t_end, observing every `obs` seconds.
typedef void (*sw_obs_fn)(double t);
static void sw_run(double t_end, double obs, sw_obs_fn fn) {
    double next_obs = SW.t;
    while (SW.t < t_end) {
        // The next event: a follower pass, a tick, an observation.
        double tn = t_end;
        for (int i = 0; i < SW.nfol; i++) {
            if (SW.fol[i].next_pass < tn) tn = SW.fol[i].next_pass;
            if (SW.fol[i].next_tick < tn) tn = SW.fol[i].next_tick;
        }
        if (fn && next_obs < tn) tn = next_obs;
        if (tn < SW.t) tn = SW.t;
        double dt = tn - SW.t;
        for (int i = 0; i < SW.nfol; i++) sw_fol_play(&SW.fol[i], dt);
        SW.t = tn;
        for (int i = 0; i < SW.nfol; i++) sw_fol_receive(&SW.fol[i], SW.t);
        // The source's main loop: stage ahead of its reader.
        SWF = NULL;
        alink_spdif_src_stage(&SW.src.u, (uint32_t)(sw_src_pos(SW.t) / ALINK_SPDIF_BLOCK_FRAMES), false);
        alink_spdif_service(&SW.src.u, alink_spdif_port_now_us());
        for (int i = 0; i < SW.nfol; i++) {
            sw_fol_t *f = &SW.fol[i];
            if (f->next_tick <= SW.t) {
                sw_tick(f);
                f->next_tick += 0.075;
            }
            if (f->next_pass <= SW.t) {
                sw_pass(f);
                f->next_pass = SW.t + (f->pass_us + (sw_rand() - 0.5) * f->pass_jit_us) * 1e-6;
            }
        }
        if (fn && next_obs <= SW.t) {
            fn(SW.t);
            next_obs += obs;
        }
    }
}

static void sw_world_init(uint32_t fs, double src_ppm, uint64_t seed) {
    memset(&SW, 0, sizeof(SW));
    SW.fs = fs;
    SW.rng = seed ? seed : 0x9E3779B97F4A7C15ull;
    SW.word_jit_us = 0.05;
    sw_src_init(src_ppm);
}

#endif // ALINK_SPDIF_WORLD_H
