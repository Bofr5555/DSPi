// alink_usb_sim.c - system simulation of alink USB-direct units (WP USB1).
//
// One USB host and two units on its bus.  Each unit runs the real engine
// (firmware/DSPi/alink_usb/alink_usb_core.c with alink/alink_phase.c) through the
// same port functions the firmware supplies, and the main loop's anchor
// handshake as main.c runs it.  Modelled: the host crystal (its SOF rate),
// each unit's crystal and boot time, Start of Frame through a hub (fixed delay
// plus jitter), the RP2350 SOF timestamp or the RP2040's interrupt latency,
// packet completion inside the frame, the IRQ stamp, TinyUSB's re-arm in
// tud_task (a packet that completes before the re-arm is lost), main-loop
// passes of random length, the 4-slot USB ring, the soft VCXO (0.5 ppm per
// tick slew, 0.11 ppm steps, 300 ppm authority), the input servo's fill loop,
// output rings with DMA-word quantised fill, and asynchronous feedback while
// the unit is not SOF-locked.
//
// The content of every packet is its bus sample index S (spf * absolute host
// frame + k), as a custom sender (USBL1 6.2 model B) would send it, plus a
// per-unit stream offset for the PipeWire-like start (model A).  So at any
// instant the sample each unit plays tells both the inter-unit skew and each
// unit's latency (arrival to output) in samples.
//
// WP FWTB3: the generator on the time base.  In the scenarios "G..." every
// unit runs its own copy of the real firmware/DSPi/siggen.c and timebase.c
// (sg_unit.h) behind the same port as alink_usb.c (ops 9/10 in
// alink_usb_port_apply(), the op 9 write check, the USB provider, the
// FWSIGGEN exemption), the rings carry the rendered generator samples tagged
// with each unit's S, and the checks show two and four units' generators
// started at one S (the hub's per-unit translation of the S origins) playing
// sample for sample at the same instants, staggered starts on their samples,
// a SOF loss that cancels a pending start and stops a run (the discontinuity
// counted), late and bus-frame-form starts refused.  Up to four units (NU);
// each generator scenario runs in its own process (fork), so its generators
// start fresh; the other scenarios run with two units and no generator, as
// before.
//
// Build: tools/alink_usb_sim/run.sh (the generator copies need nm and objcopy).
#define _DEFAULT_SOURCE         // fork()
#define PICO_RP2350 1
#define DSPI_SIGGEN_SYNC 1      // an RP2350 build with the ops: the core's time-base tracker
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "alink/alink_phase.c"
#include "alink_usb/alink_usb_core.c"
#include "sg_unit.h"
#include "siggen.h"

#define NU      4               // units at most (WP FWTB3); a scenario runs nu of them
#define RING    (1u << 16)
#define FS      48000u
#define GEN_OUT 2u              // the generator's output channel
#define ARM_LEAD_MS 150u        // max(hold_ms, release_ms) + 50: the K340's
#define GEN_DAC_OPEN 0.100      // the K340's DAC un-mute after the generator wants the outputs (s)

alink_usb_tb_t alink_usb_tb;
const alink_apply_limits_t alink_usb_apply_limits = { 10, 9, 2048, true };    // ops 9/10 (WP FWTB1)
static int nu = 2;              // units in this scenario

static int checks, failures;
#define CHECK(c, ...)                                                                                                \
    do {                                                                                                             \
        checks++;                                                                                                    \
        if (!(c)) {                                                                                                  \
            printf("  FAIL: ");                                                                                      \
            failures++;                                                                                              \
        } else {                                                                                                     \
            printf("  ok:   ");                                                                                      \
        }                                                                                                            \
        printf(__VA_ARGS__);                                                                                         \
        printf("\n");                                                                                                \
    } while (0)

static uint64_t rng_s = 88172645463325252ull;
static double urand(void) {
    rng_s ^= rng_s << 13;
    rng_s ^= rng_s >> 7;
    rng_s ^= rng_s << 17;
    return (double)(rng_s >> 11) / 9007199254740992.0;
}

typedef struct {
    double c0;                  // content of the first sample (S, or NAN)
    uint32_t frame;             // stamp
    uint16_t n;
    uint64_t hn;                // the host frame it was sent in (probe: stamp check)
} spkt_t;

typedef struct {
    // configuration
    double xtal_ppm, boot_s, hub_ns;
    bool rp2040;
    uint8_t mode;
    uint16_t d;
    double stream_offset;       // model A: content offset of this unit's stream (samples)
    // engine
    alink_usb_t us;
    alink_usb_tb_t tb;
    bool booted;
    // VCXO and the input servo
    double vcxo, last_cmd, fill_smooth, fill_int;
    // output ring
    bool running;
    double C;                   // consumed (fractional)
    uint64_t W;                 // written
    double content[RING];
    uint32_t target;
    // packet path
    bool armed, done;
    spkt_t pend;                // completed, waiting for tud_task
    double comp_t;              // completion time of the packet in flight (0 none)
    spkt_t inflight;
    spkt_t ring[4];
    int rh, rt;
    double next_pass;
    double start_at;            // pending synchronised start (the anchor's spin)
    double fb_acc;
    // counters
    uint32_t lost, overruns, underruns, events, reanchor_events, gap_events;
    double locked_at;           // first PHASE_LOCK (s after boot), 0 never
    double hold_until;          // the DAC mute hold before the anchor's drain, -1 none
    // stream
    bool streaming;
    double last_pkt_t;          // true time of the last packet delivered (-1 none)
    // clicks
    uint64_t click_w;
    bool click_armed;
    double click_t;
    // review usb1 probes: the pipeline's soft mute (request = samples from the
    // next written sample), holes (silence on a playing, unmuted stream),
    // a one-off main-loop stall, deferred ISRs while interrupts are masked.
    uint64_t mute_end_w;
    uint32_t holes, hole_samples, muted_silence;
    double last_gain;           // gain of the last sample written (0 silence)
    spkt_t keep;                // alink_usb_port_keep()
    bool stalled;
    bool pend_sof;
    uint32_t pend_f11;
    double pend_sof_true;
    bool stamp_off_known;
    uint32_t stamp_off, misstamps, stamped;
    // WP USB3: lock times and audible output while out of step (mode 2).
    bool was_locked;
    double lock_t, unlock_t;    // last entry into / exit from PHASE_LOCK (true time)
    uint32_t audible_unlocked;  // stream samples written unmuted while mode 2 was not phase-locked
    bool skipped, reset_done;
    double next_bad;            // WP USB3b: next bad RP2350 SOF reading (RAW behind LAST)
    uint32_t bad_given;
    uint32_t heard_lockerr;     // stream samples heard while in lock error (either action)
    bool dac_on;                // the DAC's hardware mute pin asserted
    double dac_t;               // since
    uint32_t presets;
    // WP FWTB3: the generator (NULL: a scenario without one)
    const sg_unit_t *g;
    tb_provider_t prov;
    int64_t stag[RING];         // S of each written sample on the timeline (-1 off it)
    float gen[RING];            // the generator's sample (GEN_OUT)
    bool gdac_want;             // the DAC model: open GEN_DAC_OPEN after the generator wants it
    double gdac_since;
    uint64_t t_write;           // T of the op 9 written (0 none)
    uint8_t wr_code, wr_past, wr_short, wr_bus;
    unsigned op9_res[4];        // 0xE2 results of op 9: applied, late, cancelled, refused
    unsigned tb_events;
    uint64_t cap_lo;            // the capture: S in [cap_lo, cap_lo + cap_len)
    uint32_t cap_len;
    double *cap_t, *cap_c;      // play instant, programme content (host S) of that sample
    float *cap_v;
    uint8_t *cap_n;
    sg_timeline_t tl_run, tl_end;
    bool tl_run_seen, want_end;
    double tb_inv0;
    bool tb_inv_set;
    unsigned tb_inv_reads, tb_inv_bad;
    uint16_t tb_dist0;
} unit_t;

typedef struct scen scen_t;
static const scen_t *S;
static const sg_unit_t *const SGU[SG_UNITS] = { &sgu0_sg_unit, &sgu1_sg_unit, &sgu2_sg_unit, &sgu3_sg_unit,
                                                &sgu4_sg_unit };    // [4]: the reference
static double g_stall_at, g_stall_len, g_dac_hold = 0.2;
static bool g_no_long_pass;
static double g_bad_period;     // unit 0 (RP2350): a SOF reading with RAW 22.7 us behind LAST every this many s

static unit_t U[NU];
static unit_t *cur;
static double T;                // true time, s
static double host_ppm;
static uint64_t host_n;         // absolute host frame count
static uint32_t host_f11;       // host's 11-bit frame number
static double next_sof;
static double sof_time;         // true time of the latest host SOF
static bool sof_on = true;
static uint32_t fs = FS;

// --------------------------------------------------------------------------
// Port
// --------------------------------------------------------------------------

static double local_us(double t) { return t * 1e6 * (1.0 + cur->xtal_ppm * 1e-6); }
uint32_t alink_usb_port_now48(void) { return (uint32_t)(uint64_t)local_us(T) * 48u; }
uint32_t alink_usb_port_now_us(void) { return (uint32_t)(uint64_t)local_us(T); }
uint32_t alink_usb_port_irq_save(void) { return 0; }
void alink_usb_port_irq_restore(uint32_t s) { (void)s; }
uint32_t alink_usb_port_fs(void) { return fs; }

static double ring_fill(void) {
    double q = floor(cur->C * 4.0) / 4.0;   // the DMA word: a quarter frame
    return (double)cur->W - q;
}
bool alink_usb_port_play(float *fill) {
    *fill = (float)ring_fill();
    return true;
}
static uint32_t cap(void) { return fs / 1000u + 1u; }
uint32_t alink_usb_port_room(void) {
    if (cur->running) return cap();
    double r = (double)cur->target - ring_fill();
    if (r <= 0) return 0;
    return r < cap() ? (uint32_t)r : cap();
}
// Should the unit be silent now?  Mode 2 and not playing phase-locked on its
// timeline, except a lock error with the PLAY action (WP USB3b).  Decided from
// the state the hub sees, independently of alink_usb_out_of_step().
static bool out_of_step_now(void) {
    const alink_usb_t *u = &cur->us;
    if (u->mode != ALINK_USB_MODE_PHASE) return false;
    bool in_step = u->state == ALINK_USB_ST_PHASE_LOCK && u->started && u->ph.state == ALINK_PHASE_LOCKED;
    if (in_step) return false;
    bool lock_error = u->state == ALINK_USB_ST_REJECTED || !alink_usb_phase_possible(u, fs) ||
                      (u->started && u->ph.state == ALINK_PHASE_PENDING);
    return !(lock_error && u->on_lock_error == ALINK_USB_LOCKERR_PLAY);
}
// WP FWTB3: one pipeline block of k samples (a packet write or silence on
// the timeline): the generator renders into it (process_input_block()), its
// samples tagged with the S they enter at.
static float g_blk[1024];
static uint32_t g_blk_n, g_blk_i;
static int64_t g_blk_s;
static void gen_block(uint32_t k) {
    g_blk_i = 0;
    g_blk_n = cur->g ? (k < 1024u ? k : 1024u) : 0;
    g_blk_s = cur->us.started ? (int64_t)cur->us.ph.next_w : -1;
    for (uint32_t i = 0; i < g_blk_n; i += 192) cur->g->render(g_blk + i, GEN_OUT, g_blk_n - i < 192 ? g_blk_n - i : 192, fs);
}
static void gen_block_end(void) { g_blk_n = 0; }
static void write_one(double c) {
    if (!isnan(c) && cur->W >= cur->mute_end_w && out_of_step_now()) cur->audible_unlocked++;
    if (!isnan(c) && cur->W >= cur->mute_end_w && alink_usb_lock_error(&cur->us)) cur->heard_lockerr++;
    cur->content[cur->W % RING] = c;
    float gv = 0.0f;
    int64_t st = -1;
    if (g_blk_i < g_blk_n) {
        gv = g_blk[g_blk_i];
        st = g_blk_s >= 0 ? g_blk_s + g_blk_i : -1;
        g_blk_i++;
    }
    cur->gen[cur->W % RING] = gv;
    cur->stag[cur->W % RING] = st;
    cur->W++;
    if (cur->running && ring_fill() > 448.0) cur->overruns++;
}
// Holes (review usb1 F1): a hard edge on the playing stream, not covered by
// the pipeline's soft mute: audio cut to silence without a fade-out, or
// silence followed by audio without a fade-in.
static bool playing(void) {
    return cur->us.started && cur->us.stream && cur->us.stream_seen && cur->us.ph.state == ALINK_PHASE_LOCKED &&
           cur->W >= cur->mute_end_w;
}
void alink_usb_port_write_packet(const alink_usb_pkt_t *p, uint32_t off, uint32_t k, float g0, float g1) {
    const spkt_t *sp = (const spkt_t *)p->data;
    if (k && cur->last_gain == 0.0 && g0 > 0.1f && playing()) {
        cur->holes++;
        cur->hole_samples++;
    }
    gen_block(k);
    for (uint32_t i = 0; i < k; i++) write_one(sp->c0 + off + i);
    gen_block_end();
    if (k) cur->last_gain = g1;
}
void alink_usb_port_write_silence(uint32_t k) {
    if (k && cur->last_gain > 0.1 && playing()) {
        cur->holes++;
        cur->hole_samples++;
    } else if (k && playing()) {
        cur->muted_silence += k;
    }
    gen_block(k);
    for (uint32_t i = 0; i < k; i++) write_one(NAN);
    gen_block_end();
    if (k) cur->last_gain = 0.0;
}
const void *alink_usb_port_keep(const alink_usb_pkt_t *p) {
    cur->keep = *(const spkt_t *)p->data;
    return &cur->keep;
}
static void vcxo_cmd(double ppm, double slew) {
    if (ppm > cur->last_cmd + slew) ppm = cur->last_cmd + slew;
    if (ppm < cur->last_cmd - slew) ppm = cur->last_cmd - slew;
    cur->last_cmd = ppm;
    if (ppm > 300) ppm = 300;
    if (ppm < -300) ppm = -300;
    cur->vcxo = round(ppm / 0.11) * 0.11;
}
void alink_usb_port_actuate(float hz, float trim, bool ff_trim, float slew) {
    double ff = ((double)hz / fs - 1.0) * 1e6;
    if (ff_trim) {
        cur->fill_int = 0;
        cur->fill_smooth = cur->target / 48.0;
        vcxo_cmd(ff + trim, slew);
        return;
    }
    // input_servo_apply(): P 2 ppm / buffer (48 frames) on the smoothed fill, I 5e-5.
    double fill = ring_fill() / 48.0, tgt = cur->target / 48.0;
    cur->fill_smooth += 0.125 * (fill - cur->fill_smooth);
    double e = cur->fill_smooth - tgt;
    double p = e * 2.0;
    if (e > 2) p += (e - 2) * 4.0;
    else if (e < -2) p += (e + 2) * 4.0;
    cur->fill_int += e * 5e-5;
    if (cur->fill_int > 3) cur->fill_int = 3;
    if (cur->fill_int < -3) cur->fill_int = -3;
    vcxo_cmd(ff + p + cur->fill_int, 0.5);
}
void alink_usb_port_preset(float ppm) {
    cur->last_cmd = ppm;
    cur->fill_int = 0;
    cur->vcxo = round(ppm / 0.11) * 0.11;
    cur->presets++;
}
void alink_usb_port_park(void) {
    cur->vcxo = cur->last_cmd = 0;
}
float alink_usb_port_vcxo_ppm(void) { return (float)cur->vcxo; }
// As alink_usb.c (WP FWTB1): op 9 by S goes to the generator; at its sample
// the generator starts (ready: LOCKED; the sim has no user or UAC mute).
uint8_t alink_usb_port_op9_accept(uint64_t t, uint64_t t_now) {
    return cur->g ? cur->g->tb_accept(TB_ID_USB, t, t_now, fs) : TB_ERR_VALUE;
}
uint8_t alink_usb_port_apply(const alink_apply_op_t *op, bool late) {
    if (op->op == ALINK_OP_SIGGEN_START)
        return cur->g ? cur->g->tb_fire(TB_ID_USB, op->frame, late, cur->us.tbk.state == TB_ST_LOCKED)
                      : ALINK_APPLY_REFUSED;
    if (op->op == ALINK_OP_SIGGEN_STOP) {
        if (cur->g) cur->g->tb_stop_op(op->arg);
        return ALINK_APPLY_APPLIED;
    }
    if (op->op == ALINK_OP_CLICK) {
        cur->click_w = cur->W;          // the next sample written
        cur->click_armed = true;
        return ALINK_APPLY_APPLIED;
    }
    return op->op == ALINK_OP_SYNC_PULSE ? ALINK_APPLY_REFUSED : ALINK_APPLY_APPLIED;
}
void alink_usb_port_apply_result(uint8_t op, uint8_t r, uint32_t f) {
    (void)f;
    if (op != ALINK_OP_SIGGEN_START || !cur->g) return;
    // As alink_usb.c: the core cancelled a pending op 9 itself (the timeline
    // ended): CANCELLED_LOSS.
    if (r == ALINK_APPLY_CANCELLED) cur->g->tb_dropped(TB_RES_CANCELLED_LOSS);
    if (r < 4) cur->op9_res[r]++;
}
void alink_usb_port_event(uint8_t st, uint8_t rs, uint32_t f) {
    (void)f;
    if (getenv("SIM_TRACE"))
        printf("    T %.3f unit %d: event state %u reason %u | est %.2f valid %d vcxo %.2f rate_locked %d started %d prefilling %d ph %u anchors %u reanchors %u\n",
               T, (int)(cur - U), st, rs, cur->us.est.ppm, cur->us.est.valid, cur->vcxo, cur->us.rate_locked,
               cur->us.started, cur->us.prefilling, cur->us.ph.state, cur->us.anchors, cur->us.reanchors);
    cur->events++;
    if (rs == ALINK_USB_RS_REANCHOR) cur->reanchor_events++;
    if (rs == ALINK_USB_RS_GAP) cur->gap_events++;
}
void alink_usb_port_soft_mute(uint32_t s) { cur->mute_end_w = cur->W + s; }   // refresh, never accumulate
bool alink_usb_port_pulse(uint32_t f) { (void)f; return false; }
// usb_audio_stream_active(): a packet within the 50 ms gap threshold.
bool alink_usb_port_stream_active(void) { return cur->last_pkt_t >= 0.0 && T - cur->last_pkt_t <= 0.050; }
void alink_usb_port_ring_target(void) { cur->target = alink_usb_ring_target(&cur->us, fs, fs / 500u, cap(), 448); }

// --------------------------------------------------------------------------
// Unit and host model
// --------------------------------------------------------------------------

// ---- WP FWTB3: the USB provider (alink_usb.c) and the generator's port ----

static uint8_t prov_now(void) { return cur->us.mode != ALINK_USB_MODE_OFF ? alink_usb_tb_state(&cur->us) : TB_ST_NONE; }
static bool prov_present(void) { return cur->us.mode != ALINK_USB_MODE_OFF; }
static void prov_read(tb_entry_t *e, bool ring, uint32_t wr, uint32_t played) {
    alink_usb_t *us = &cur->us;
    e->d = us->d;
    e->phase_error = us->ph.e;
    uint8_t st = us->tbk.state;
    if (st == TB_ST_NONE || st == TB_ST_ACQUIRING) {
        e->t = 0;
    } else if (us->started && us->ph.timeline && ring) {
        e->t = tb_t_at_output(us->ph.next_w, wr, played, (int32_t)us->d);
    } else {
        uint64_t s = 0;
        e->t = alink_usb_sample_now(us, &s, NULL) ? s : 0;
    }
}
static bool prov_next(uint64_t *t) {
    if (!cur->us.started || !cur->us.ph.timeline) return false;
    *t = cur->us.ph.next_w;
    return true;
}
static void prov_drop(uint8_t r) { alink_usb_drop_op9(&cur->us, r); }

static bool gport_ready(void) { return true; }
static uint16_t gport_lead(void) { return ARM_LEAD_MS; }
static uint32_t gport_slips(void) { return cur->underruns; }
static uint32_t gport_now_us(void) { return alink_usb_port_now_us(); }
static bool gport_ring(uint32_t *wr, uint32_t *played) {
    *wr = (uint32_t)cur->W;
    *played = (uint32_t)(uint64_t)cur->C;
    return cur->running;
}
static void gport_event(uint8_t tb, uint8_t st, uint8_t rs, uint32_t t) {
    (void)tb; (void)st; (void)rs; (void)t;
    cur->tb_events++;
}
static sg_port_t gport = { gport_ready, gport_lead, gport_slips, gport_now_us, gport_ring, gport_event };

static void gen_invariant(unit_t *u);
static void unit_sel(unit_t *u) {
    cur = u;
    alink_usb_tb = u->tb;
}
static void unit_unsel(void) {
    cur->tb = alink_usb_tb;
}

static void unit_init(unit_t *u) {
    alink_usb_init(&u->us);
    u->us.mode = u->mode;
    u->us.d = u->d;
    u->us.exact = !u->rp2040;
    memset(&u->tb, 0, sizeof(u->tb));
    u->tb.stamp = ALINK_USB_FRAME_NONE;
    u->armed = true;
    u->running = true;          // stock: outputs run from boot
    u->next_pass = u->boot_s;
    u->start_at = -1;
    u->hold_until = -1;
    u->last_pkt_t = -1.0;
    unit_sel(u);
    alink_usb_limits(&u->us, fs, fs / 500u, cap(), 448);
    alink_usb_port_ring_target();
    // the stock ring starts at its target
    for (uint32_t i = 0; i < u->target; i++) write_one(NAN);
    unit_unsel();
}

// The unit's SOF interrupt: bus frame f11 whose SOF was at sof_true, handled
// at tnow (= sof_true unless interrupts were masked).
static void unit_sof_at(unit_t *u, uint32_t f11, double sof_true, double tnow) {
    unit_sel(u);
    double tsof = sof_true + u->hub_ns * 1e-9 + (urand() - 0.5) * 20e-9;
    double lat = 1e-6 + urand() * 3e-6;
    double tproc = tnow + lat;
    if (!u->rp2040) {
        // RP2350: the PHY counter (48 MHz from the crystal) gives the ticks
        // since the SOF exactly, but "now" is time_us_32() * 48: whole microseconds.
        uint32_t since = (uint32_t)((tproc - tsof) * 48e6 * (1.0 + u->xtal_ppm * 1e-6));
        if (g_bad_period > 0 && u == &U[0] && T >= u->next_bad) {
            // The second bench hour's reading: the ISR's 21-bit difference wrapped.
            if (u->next_bad > 0) {
                since = (1u << 21) - 1088u;     // RAW 1088 ticks (22.7 us) behind LAST, masked to 21 bits
                u->bad_given++;
            }
            u->next_bad = T + g_bad_period;
        }
        uint32_t now48 = (uint32_t)(uint64_t)local_us(tproc) * 48u;
        alink_usb_tb_sof(&alink_usb_tb, f11, now48, since);
    } else {
        if (urand() < 0.02) tproc += 30e-6 + urand() * 200e-6;
        uint32_t now48 = (uint32_t)(uint64_t)local_us(tproc) * 48u;
        alink_usb_tb_sof(&alink_usb_tb, f11, now48, UINT32_MAX);
    }
    unit_unsel();
}
static void unit_sof(unit_t *u) { unit_sof_at(u, host_f11, sof_time, T); }

// The host's packet for this frame (completes later in the frame).
static void host_packet(unit_t *u, bool skip) {
    if (!u->streaming || skip) return;
    if (u->comp_t > 0) {
        // The previous packet sits in the OUT buffer (FULL) with its completion
        // interrupt still held off: this one has nowhere to go (not re-armed).
        u->lost++;
        return;
    }
    uint32_t n = fs / 1000u;
    if (!alink_usb_nominal_feedback(&u->us)) {
        // Asynchronous feedback: the host follows the unit's clock.
        double r = (1 + u->xtal_ppm * 1e-6) * (1 + u->vcxo * 1e-6) / (1 + host_ppm * 1e-6);
        u->fb_acc += n * r;
        n = (uint32_t)u->fb_acc;
        u->fb_acc -= n;
    }
    u->inflight.c0 = (double)host_n * (fs / 1000u) + u->stream_offset;
    u->inflight.n = (uint16_t)n;
    u->inflight.hn = host_n;
    u->comp_t = T + 120e-6 + urand() * 700e-6;
}

static void unit_complete(unit_t *u) {
    u->comp_t = 0;
    if (!u->armed) {
        u->lost++;
        return;
    }
    unit_sel(u);
    double lat = 1e-6 + urand() * 2e-6;
    if (u->rp2040 && urand() < 0.02) lat += 30e-6 + urand() * 200e-6;
    double tt = T + lat;
    uint32_t now48 = (uint32_t)(uint64_t)local_us(tt) * 48u;
    alink_usb_tb_stamp(&alink_usb_tb, now48);
    u->pend = u->inflight;
    u->pend.frame = alink_usb_tb.stamp;
    // Probe: the stamp must stay a constant offset from the host's frame.
    if (alink_usb_tb.stamp != ALINK_USB_FRAME_NONE) {
        uint32_t off = (alink_usb_tb.stamp - (uint32_t)u->inflight.hn) & 0x7FFu;
        if (!u->stamp_off_known) {
            u->stamp_off = off;
            u->stamp_off_known = true;
        } else if (off != u->stamp_off) {
            u->misstamps++;
        }
        u->stamped++;
    }
    u->armed = false;
    u->done = true;
    unit_unsel();
}

static void drain(unit_t *u) {
    while (u->rt != u->rh) {
        spkt_t *p = &u->ring[u->rt];
        u->last_pkt_t = T;
        if (alink_usb_engaged(&u->us)) {
            alink_usb_pkt_t ap = { p->frame, p->n, p, 0 };
            alink_usb_deliver(&u->us, &ap);
        } else {
            for (uint32_t i = 0; i < p->n; i++) write_one(p->c0 + i);
        }
        u->rt = (u->rt + 1) & 3;
    }
}

// The DAC's hardware mute (main.c, alink_usb.c dac_hold_update(), WP USB3):
// asserted once the soft mute has silenced the output while out of step;
// complete_pipeline_reset() keeps it asserted when the USB anchor follows (the
// reset's own bracket waited the hold before it); the anchor drains when the
// hold has elapsed since the assert and releases it after the start.
static void sim_reset_dac(unit_t *u) {
    if (alink_usb_out_of_step(&u->us)) {
        u->dac_on = true;
        u->dac_t = T - g_dac_hold;
    }
}

static void unit_pass(unit_t *u) {
    unit_sel(u);
    // tud_task: xfer_cb pushes the packet and re-arms.
    if (u->done) {
        int nh = (u->rh + 1) & 3;
        if (nh != u->rt) {
            u->ring[u->rh] = u->pend;
            u->rh = nh;
        }
        u->done = false;
        u->armed = true;
        alink_usb_tb.stamp = ALINK_USB_FRAME_NONE;
    }
    // main.c handshake (phase mode)
    if (u->us.mode == ALINK_USB_MODE_PHASE) {
        // main.c: prepare_pipeline_reset() asserts the DAC mute, and the drain
        // waits for its hold (the K340's GP11 mute: modelled as 200 ms) while
        // the anchor is still wanted (round 3, hardware H2).
        bool oos = alink_usb_out_of_step(&u->us);
        if (oos && !u->dac_on && !u->us.started && u->W < u->mute_end_w) {
            u->dac_on = true;
            u->dac_t = T;
        } else if (!oos && u->dac_on && !u->us.prefilling) {
            u->dac_on = false;
        }
        bool want = alink_usb_anchor_wanted(&u->us);
        // prepare_pipeline_reset(): muted while the hold runs (preset_loading).
        if (want) alink_usb_port_soft_mute(fs / 10u);
        if (want && !u->dac_on) {
            u->dac_on = true;
            u->dac_t = T;
        }
        if (want && T >= u->dac_t + g_dac_hold) {
            u->running = false;     // drain_and_disable_outputs()
            u->W = 0;
            u->C = 0;
            alink_usb_on_drain(&u->us);
        }
        if (u->us.prefilling && u->start_at < 0) {
            float wait = alink_usb_anchor_wait(&u->us);
            if (wait <= (float)u->us.spf) {
                // alink_usb_fw_anchor_ready()'s spin: the start lands on the delay.
                double w = wait > 0 ? wait : 0;
                u->start_at = T + w / (fs * (1 + host_ppm * 1e-6)) + urand() * 0.3e-6;
            }
        }
    }
    drain(u);
    if (u->g) u->us.siggen_exempt = u->g->wants_output();     // alink_usb_fw_service(): WP FWSIGGEN
    alink_usb_service(&u->us, (uint32_t)(local_us(T) / 1000.0));
    if (u->g) {
        // alink_usb_fw_service(): an op 9 pending on S arms at T - arm_lead;
        // main.c: the aggregator (losses and disturbances to the generator,
        // 0xE4), the generator's service, the DAC gate.
        if (u->us.started) u->g->tb_poll(TB_ID_USB, u->us.ph.next_w, fs);
        u->g->tb_service((uint32_t)(local_us(T) / 1000.0));
        u->g->service();
        bool want = u->g->wants_output();
        if (want && !u->gdac_want) u->gdac_since = T;
        u->gdac_want = want;
        u->g->output_gate(want && T - u->gdac_since >= GEN_DAC_OPEN);
        if (u->cap_len) {
            gen_invariant(u);
            sg_timeline_t tl = sg_timeline(u->g);
            if (tl.state == TB_TL_RUNNING) {
                u->tl_run = tl;
                u->tl_run_seen = true;
            }
        }
    }
    if (alink_usb_outputs_orphaned(&u->us)) {      // main.c, review usb3 N2
        u->running = true;
        u->start_at = -1;
        alink_usb_on_restarted(&u->us);
    }
    if (!u->locked_at && u->us.state == ALINK_USB_ST_PHASE_LOCK) u->locked_at = T - u->boot_s;
    bool lk = u->us.state == ALINK_USB_ST_PHASE_LOCK;
    if (lk && !u->was_locked) u->lock_t = T;
    if (!lk && u->was_locked) u->unlock_t = T;
    u->was_locked = lk;
    double pass = 60e-6 + urand() * 240e-6;
    if (urand() < 0.01 && !g_no_long_pass) pass = 900e-6;
    if (g_stall_len > 0 && u == &U[0] && !u->stalled && T >= g_stall_at) {
        pass = g_stall_len;             // one long pass (a flash write, a long command)
        u->stalled = true;
    }
    u->next_pass = T + pass;
    unit_unsel();
}

static void unit_start(unit_t *u) {
    unit_sel(u);
    u->dac_on = false;              // dac_hw_mute_release() after enable_outputs_in_sync()
    u->running = true;
    u->start_at = -1;
    alink_usb_on_enabled(&u->us);
    unit_unsel();
}

// --------------------------------------------------------------------------
// Measurement
// --------------------------------------------------------------------------

typedef struct {
    double lat_min[NU], lat_max[NU], lat_sum[NU];
    int lat_n[NU];
    double skew_max, skew_sum;
    int skew_n;
} meas_t;

static void meas_reset(meas_t *m) {
    for (int i = 0; i < nu; i++) {
        m->lat_min[i] = 1e9;
        m->lat_max[i] = -1e9;
        m->lat_sum[i] = 0;
        m->lat_n[i] = 0;
    }
    m->skew_max = m->skew_sum = 0;
    m->skew_n = 0;
}

// The content sample each unit outputs now, and its latency: F_arr (true
// host time in samples) minus the content.
static void measure(meas_t *m) {
    double farr = (double)host_n * (fs / 1000.0) + (T - sof_time) * fs * (1 + host_ppm * 1e-6);
    double pos[NU];
    bool ok[NU];
    for (int i = 0; i < nu; i++) {
        unit_t *u = &U[i];
        ok[i] = false;
        if (!u->booted || !u->running || u->C < 1) continue;
        uint64_t idx = (uint64_t)u->C;
        double c = u->content[idx % RING];
        if (isnan(c) || idx >= u->W) continue;
        pos[i] = c + (u->C - (double)idx) - u->stream_offset;
        ok[i] = true;
        double lat = farr - pos[i];
        if (lat < m->lat_min[i]) m->lat_min[i] = lat;
        if (lat > m->lat_max[i]) m->lat_max[i] = lat;
        m->lat_sum[i] += lat;
        m->lat_n[i]++;
    }
    if (ok[0] && ok[1]) {
        double s = fabs((pos[0] + U[0].stream_offset) - (pos[1] + U[1].stream_offset));
        if (s > m->skew_max) m->skew_max = s;
        m->skew_sum += s;
        m->skew_n++;
    }
}

// --------------------------------------------------------------------------
// Scenario driver
// --------------------------------------------------------------------------

// WP FWTB3: a generator plan.  At write_s the hub stages the config on every
// unit and writes op 9 for T_i = T_0 + stagger[i] in each unit's own S: T_0
// is unit 0's S about t0_s - write_s ahead, and a unit whose S counts from
// another origin (a multiple of 2048 * spf apart, C1) gets T_0 moved by that
// difference, read from both units' S now (EXT_FRAME_COUNTER).
enum { G_COINCIDE = 1, G_LOSS, G_LATE };
typedef struct {
    int kind;
    uint8_t type;               // SIGGEN_IMPULSE or SIGGEN_SWEEP_LOG
    uint16_t repeat;
    double t0_s, write_s;
    int32_t stagger[NU];
} ugen_t;

struct scen {
    const char *name;
    int nu;                     // WP FWTB3: units (0: 2)
    const ugen_t *gen;          // the generator plan (NULL none)
    double host_ppm;
    // review usb1 probes
    double burst_at; int burst_n;               // unit 1: burst_n consecutive packets skipped from burst_at
    double host_step_at, host_ppm2;             // the host's rate changes at host_step_at
    double host_step2_at, host_ppm3;            // ... and again
    double stall_at, stall_len;                 // unit 0: one main-loop pass of stall_len s
    double mask_period, mask_len;               // unit 0: interrupts masked mask_len s every mask_period s
    bool no_jump;                               // SOF gap without a frame-number jump (suspend / resume, bus reset)
    bool gap_counts;                            // the host's frame counter keeps running during the SOF gap (a port reset)
    double xtal[NU], boot[NU], hub_ns[NU];
    bool rp2040[NU];
    uint8_t mode;
    uint16_t d;
    double stream_start[NU];    // s
    double stream_offset[NU];
    double duration;
    double miss_rate;           // probability a packet is skipped (both units independently)
    double stop_at, stop_len;   // stream stopped on unit 1 (alt 0) for stop_len s
    double stop_lead;           // the host's last packet comes this long before the alt 0
    double dac_hold;            // DAC mute hold before the anchor's drain (0: 200 ms)
    double sof_gap_at, sof_gap_len;     // SOF stops (bus suspend / host controller restart), frame number jumps
    double click_at;            // CLICK on both units at the same bus frame
    uint32_t fs;                // 0: 48 kHz
    // WP USB3 lock-time probes
    double skip_at; uint32_t skip_n;            // unit 0's output reader skips skip_n samples (a ring skip)
    double reset_at;                            // unit 0's output rings reset (preset load, flash write)
    double ev_at;                               // the event lock times are measured from
    bool seeded; double seed;                   // a stored host rate (ppm against the unit's crystal)
    bool no_long_pass;                          // no 900 us main-loop passes (they lose packets at random)
    double bad_period;                          // WP USB3b: bad RP2350 SOF readings on unit 0, every this many s
    bool lock_play;                             // EXT_ALINK_USB byte 4 = 1: play unaligned in lock error
    bool no_auto;                               // AUTO_REANCHOR clear (PENDING instead of a re-anchor)
};

typedef struct {
    meas_t m;                   // last 10 s
    uint32_t anchors[NU], reanchors[NU], missed[NU], late[NU];
    uint8_t state[NU];
    double vcxo[NU], ff[NU];
    double click_skew;
    bool click_ok;
    uint32_t underruns[NU], overruns[NU], lost[NU];
    uint32_t holes[NU], hole_samples[NU], muted_silence[NU], gap_events[NU], events[NU], misstamps[NU], stamped[NU];
    double locked_at[NU];
    uint32_t rides[NU];
    double relock[NU];              // from ev_at to the last entry into PHASE_LOCK (-1: never left / not after)
    uint32_t audible_unlocked[NU];
    uint32_t bad_sof[NU], bad_given[NU], disc[NU], lock_errors[NU];
    bool lock_err[NU];
    uint32_t heard_lockerr[NU];
    double lat_after_restart[NU];
    int gaps_at_stop;               // unit 1: GAPs counted from 30 ms before the alt 0 to 100 ms after it
    uint32_t gaps_pre;
    bool stop_sampled;
    uint8_t first_bus[NU];
    uint8_t status[NU][ALINK_USB_STATUS_LEN];
} res_t;

// ---- WP FWTB3: the hub's side of the generator plan, the capture ----------

static SiggenConfig gen_cfg(const ugen_t *gp) {
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
static uint32_t gen_len(const ugen_t *gp) {
    double ms = gp->type == SIGGEN_IMPULSE ? 20.0 * gp->repeat : 320.0 * gp->repeat;
    return (uint32_t)(ms * fs / 1000.0) + 1u;
}
static uint8_t write_op9(unit_t *u, uint64_t t, uint8_t tb) {
    uint8_t p[ALINK_APPLY_LEN] = { 0 };
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(t >> (8 * i));
    p[8] = ALINK_OP_SIGGEN_START;
    p[12] = tb;
    return alink_usb_apply_code(&u->us, p);
}

// At write_s, at one instant for all units (the hub's requests): SET_CONFIG,
// EXT_FRAME_COUNTER, then op 9 by S in each unit's own numbering.
static void gen_host(const scen_t *s) {
    const ugen_t *gp = s->gen;
    uint64_t now[NU] = { 0 };
    for (int i = 0; i < nu; i++) {
        unit_sel(&U[i]);
        if (!alink_usb_sample_now(&U[i].us, &now[i], NULL) || U[i].us.tbk.state != TB_ST_LOCKED)
            printf("    unit %d: not LOCKED at the write (tb state %u)\n", i, U[i].us.tbk.state);
        unit_unsel();
    }
    const uint64_t wrap = 2048ull * (fs / 1000u);
    uint64_t t0 = now[0] + (uint64_t)llround((gp->t0_s - gp->write_s) * fs);
    for (int i = 0; i < nu; i++) {
        unit_t *u = &U[i];
        unit_sel(u);
        SiggenConfig c = gen_cfg(gp);
        if (!u->g->stage_config(&c, sizeof c)) printf("    unit %d: SET_CONFIG refused\n", i);
        int64_t k = llround((double)(int64_t)(now[i] - now[0]) / (double)wrap);
        u->t_write = t0 + (uint64_t)(k * (int64_t)wrap) + (uint64_t)(int64_t)gp->stagger[i];
        if (gp->kind == G_LATE) {
            // A START whose sample has passed, one inside the arm lead (both
            // VC_ERR_STATE) and one in the bus-frame form (R5, VC_ERR_VALUE).
            u->wr_past = write_op9(u, u->us.ph.next_w - 480u, 1);
            u->wr_short = write_op9(u, u->us.ph.next_w + (uint64_t)ARM_LEAD_MS * fs / 1000u - 1u, 1);
            u->wr_bus = write_op9(u, alink_usb_bus_form(&u->us, u->t_write), 2);
        }
        u->wr_code = write_op9(u, u->t_write, 1);
        uint32_t margin = fs / 10u;
        int32_t lo = 0, hi = 0;
        for (int j = 0; j < nu; j++) {
            if (gp->stagger[j] < lo) lo = gp->stagger[j];
            if (gp->stagger[j] > hi) hi = gp->stagger[j];
        }
        u->cap_lo = u->t_write - (uint64_t)(int64_t)gp->stagger[i] + (uint64_t)(int64_t)lo - margin;
        u->cap_len = (uint32_t)(hi - lo) + gen_len(gp) + 2u * margin;
        u->cap_t = malloc(u->cap_len * sizeof(double));
        u->cap_c = malloc(u->cap_len * sizeof(double));
        u->cap_v = calloc(u->cap_len, sizeof(float));
        u->cap_n = calloc(u->cap_len, 1);
        for (uint32_t j = 0; j < u->cap_len; j++) u->cap_t[j] = u->cap_c[j] = NAN;
        unit_unsel();
    }
}

// The samples the output played in this step (sample w plays when C reaches w).
static void capture(unit_t *u, double c0, double rate, double dt) {
    for (uint64_t w = (uint64_t)floor(c0) + 1u; (double)w <= u->C && w < u->W; w++) {
        int64_t st = u->stag[w % RING];
        if (st < (int64_t)u->cap_lo || st >= (int64_t)(u->cap_lo + u->cap_len)) continue;
        uint32_t i = (uint32_t)(st - (int64_t)u->cap_lo);
        if (u->cap_n[i] < 255) u->cap_n[i]++;
        if (u->cap_n[i] == 1) {
            u->cap_t[i] = T - dt + ((double)w - c0) / rate;
            u->cap_v[i] = u->gen[w % RING];
            u->cap_c[i] = u->content[w % RING] - u->stream_offset;
        }
    }
}

// EXT_TIME_BASE while LOCKED: t - output_frames constant while the
// disturbance counter stays (R7c).
static void gen_invariant(unit_t *u) {
    if (u->us.tbk.state != TB_ST_LOCKED || !u->running) return;
    uint8_t o[TB_READ_LEN];
    u->g->tb_read(o);
    if (o[1] < 1) return;
    const uint8_t *x = o + TB_ENTRY_OFF;
    uint32_t played = (uint32_t)o[8] | (uint32_t)o[9] << 8 | (uint32_t)o[10] << 16 | (uint32_t)o[11] << 24;
    uint32_t tl = (uint32_t)x[8] | (uint32_t)x[9] << 8 | (uint32_t)x[10] << 16 | (uint32_t)x[11] << 24;
    uint16_t dist = (uint16_t)(x[16] | x[17] << 8);
    double inv = (double)(int32_t)(tl - played);
    if (!u->tb_inv_set || dist != u->tb_dist0) {
        u->tb_inv_set = true;
        u->tb_inv0 = inv;
        u->tb_dist0 = dist;
        return;
    }
    u->tb_inv_reads++;
    if (fabs(inv - u->tb_inv0) > 1.0) u->tb_inv_bad++;
}

static void run(const scen_t *s, res_t *r) {
    memset(U, 0, sizeof(U));
    memset(r, 0, sizeof(*r));
    S = s;
    g_stall_at = s->stall_at;
    g_stall_len = s->stall_len;
    g_no_long_pass = s->no_long_pass;
    g_bad_period = s->bad_period;
    g_dac_hold = s->dac_hold > 0 ? s->dac_hold : 0.2;
    int burst_done = 0;
    host_ppm = s->host_ppm;
    host_n = 1000000u + (uint64_t)(urand() * 2048);
    host_f11 = (uint32_t)host_n & 0x7FF;
    T = 0;
    next_sof = 0;
    sof_on = true;
    fs = s->fs ? s->fs : FS;
    nu = s->nu ? s->nu : 2;
    bool gen_written = false;
    for (int i = 0; i < nu; i++) {
        U[i].g = s->gen ? SGU[i] : NULL;      // WP FWTB3: this unit's generator copy
        U[i].xtal_ppm = s->xtal[i];
        U[i].boot_s = s->boot[i];
        U[i].hub_ns = s->hub_ns[i];
        U[i].rp2040 = s->rp2040[i];
        U[i].mode = s->mode;
        U[i].d = s->d;
        U[i].stream_offset = s->stream_offset[i];
        unit_init(&U[i]);
        if (U[i].g) {
            *U[i].g->port = &gport;
            U[i].prov = (tb_provider_t){ TB_ID_USB, TB_ROLE_FOLLOWER, &U[i].us.tbk, prov_now, prov_present, prov_read,
                                         prov_next, prov_drop };
            unit_sel(&U[i]);
            U[i].g->tb_register(TB_SLOT_USB, &U[i].prov);
            unit_unsel();
        }
        U[i].us.on_lock_error = s->lock_play ? ALINK_USB_LOCKERR_PLAY : ALINK_USB_LOCKERR_MUTE;
        if (s->no_auto) U[i].us.flags = 0;
        if (s->seeded) {
            int16_t v = (int16_t)lround(s->seed * 100.0);
            uint8_t b[ALINK_USB_SEED_LEN] = { (uint8_t)v, (uint8_t)((uint16_t)v >> 8), 0 };
            b[2] = (uint8_t)(0xA5u ^ b[0] ^ b[1]);
            alink_usb_seed_load(&U[i].us, b);
        }
    }
    meas_reset(&r->m);
    const double dt = 1e-6;
    double period = 1e-3 / (1 + host_ppm * 1e-6);
    bool clicked = false;
    uint64_t click_frame = 0;
    double next_meas = 0;
    while (T < s->duration) {
        // host SOF
        bool gap = s->sof_gap_len > 0 && T >= s->sof_gap_at && T < s->sof_gap_at + s->sof_gap_len;
        if (s->host_step_at > 0 && T >= s->host_step_at) host_ppm = s->host_ppm2;
        if (s->host_step2_at > 0 && T >= s->host_step2_at) host_ppm = s->host_ppm3;
        period = 1e-3 / (1 + host_ppm * 1e-6);
        if (T >= next_sof) {
            next_sof += period;
            if (!gap) {
                host_n++;
                host_f11 = (host_f11 + 1) & 0x7FF;
                sof_time = T;
                for (int i = 0; i < nu; i++) {
                    unit_t *u = &U[i];
                    if (!u->booted && T >= u->boot_s) u->booted = true;
                    if (!u->booted) continue;
                    bool masked = i == 0 && s->mask_len > 0 && fmod(T, s->mask_period) < s->mask_len;
                    if (masked) {
                        // the SOF interrupt waits; SOF_RD and SOF_TIMESTAMP_LAST hold the latest SOF
                        u->pend_sof = true;
                        u->pend_f11 = host_f11;
                        u->pend_sof_true = T;
                    } else {
                        unit_sof(u);
                    }
                    bool st = T >= s->stream_start[i] &&
                              !(i == 1 && s->stop_len > 0 && T >= s->stop_at && T < s->stop_at + s->stop_len);
                    if (st != u->streaming) {
                        u->streaming = st;
                        unit_sel(u);
                        alink_usb_stream(&u->us, st);
                        unit_unsel();
                    }
                    bool skip = urand() < s->miss_rate;
                    if (i == 1 && s->stop_len > 0 && T >= s->stop_at - s->stop_lead && T < s->stop_at) skip = true;
                    if (i == 1 && s->burst_n > 0 && T >= s->burst_at && burst_done < s->burst_n) {
                        skip = true;
                        burst_done++;
                    }
                    host_packet(u, skip);
                }
                // CLICK on both units, bus frame 100 ms ahead (time base 2).
                if (s->click_at > 0 && !clicked && T >= s->click_at) {
                    clicked = true;
                    click_frame = (host_f11 + 100) & 0x7FF;
                    for (int i = 0; i < nu; i++) {
                        unit_sel(&U[i]);
                        uint8_t p[16] = { 0 };
                        uint32_t f = (uint32_t)click_frame;
                        memcpy(p, &f, 4);
                        p[8] = ALINK_OP_CLICK;
                        p[12] = 2;
                        if (!alink_usb_apply(&U[i].us, p)) printf("    click refused on unit %d\n", i);
                        unit_unsel();
                    }
                }
            } else if (s->gap_counts) {
                host_n++;                       // SOF not sent on this port, but the controller keeps counting
                host_f11 = (host_f11 + 1) & 0x7FF;
            } else if (s->sof_gap_len > 0 && T + period >= s->sof_gap_at + s->sof_gap_len && !s->no_jump) {
                // the host controller restarts with another frame number
                host_f11 = (host_f11 + 777) & 0x7FF;
                host_n += 777;
            }
        }
        for (int i = 0; i < nu; i++) {
            unit_t *u = &U[i];
            if (!u->booted) continue;
            bool masked = i == 0 && s->mask_len > 0 && fmod(T, s->mask_period) < s->mask_len;
            if (!masked && u->pend_sof) {
                // interrupts back: the SOF handler runs first (dcd_rp2040_irq), then buff status
                unit_sof_at(u, u->pend_f11, u->pend_sof_true, T);
                u->pend_sof = false;
            }
            if (i == 0 && s->skip_n && !u->skipped && T >= s->skip_at) {
                u->skipped = true;
                u->C += s->skip_n;                  // the output reader skips: the rings lose skip_n samples
                if (u->C > (double)u->W) u->W = (uint64_t)ceil(u->C);
            }
            if (i == 0 && s->reset_at > 0 && !u->reset_done && T >= s->reset_at) {
                // complete_pipeline_reset(): the rings restart empty at the stock target.
                u->reset_done = true;
                unit_sel(u);
                u->W = (uint64_t)ceil(u->C);
                alink_usb_outputs_reset(&u->us);
                sim_reset_dac(u);
                for (uint32_t k = 0; k < u->target; k++) write_one(NAN);
                unit_unsel();
            }
            if (s->gen && !gen_written && T >= s->gen->write_s) {
                gen_written = true;
                gen_host(s);
            }
            if (u->comp_t > 0 && T >= u->comp_t && !masked) unit_complete(u);
            if (u->start_at >= 0 && T >= u->start_at) unit_start(u);
            if (T >= u->next_pass) unit_pass(u);
            if (u->running) {
                double rate = fs * (1 + u->xtal_ppm * 1e-6) * (1 + u->vcxo * 1e-6);
                double c0 = u->C;
                u->C += rate * dt;
                if (u->cap_len) capture(u, c0, rate, dt);
                if (u->click_armed && u->C >= (double)u->click_w) {
                    u->click_t = T - dt + ((double)u->click_w - c0) / rate;
                    u->click_armed = false;
                }
                if (u->C > (double)u->W) {
                    // Counted on the phase timeline only: before the anchor the
                    // stock path's idle pump (not modelled) covers an empty ring.
                    if (u->us.started) u->underruns++;
                    uint64_t w = (uint64_t)ceil(u->C);  // the writer re-anchors to the reader
                    while (u->W < w) {
                        u->gen[u->W % RING] = 0.0f;
                        u->stag[u->W % RING] = -1;
                        u->content[u->W++ % RING] = NAN;
                    }
                }
            }
        }
        if (s->stop_len > 0) {
            if (!r->stop_sampled && T >= s->stop_at - s->stop_lead - 0.03) {
                r->gaps_pre = U[1].us.gaps;
                r->stop_sampled = true;
                r->gaps_at_stop = -1;
            }
            if (r->gaps_at_stop < 0 && T >= s->stop_at + 0.1) r->gaps_at_stop = (int)(U[1].us.gaps - r->gaps_pre);
        }
        if (T >= s->duration - 10.0 && T >= next_meas) {
            measure(&r->m);
            next_meas = T + 1e-3;
        }
        T += dt;
    }
    for (int i = 0; i < nu; i++) {
        unit_t *u = &U[i];
        r->anchors[i] = u->us.anchors;
        r->reanchors[i] = u->us.reanchors;
        r->missed[i] = u->us.missed;
        r->late[i] = u->us.late;
        r->state[i] = u->us.state;
        r->vcxo[i] = u->vcxo;
        r->ff[i] = u->us.ff_ppm;
        r->underruns[i] = u->underruns;
        r->overruns[i] = u->overruns;
        r->lost[i] = u->lost;
        r->holes[i] = u->holes;
        r->hole_samples[i] = u->hole_samples;
        r->muted_silence[i] = u->muted_silence;
        r->gap_events[i] = u->gap_events;
        r->events[i] = u->events;
        r->misstamps[i] = u->misstamps;
        r->stamped[i] = u->stamped;
        r->locked_at[i] = u->locked_at;
        r->rides[i] = u->us.diag.rides;
        r->relock[i] = s->ev_at > 0 && u->lock_t > s->ev_at && u->us.state == ALINK_USB_ST_PHASE_LOCK ? u->lock_t - s->ev_at : -1.0;
        r->audible_unlocked[i] = u->audible_unlocked;
        r->bad_sof[i] = u->us.diag.bad_sof;
        r->bad_given[i] = u->bad_given;
        r->disc[i] = u->us.diag.ends[ALINK_USB_END_DISCONT];
        r->lock_errors[i] = u->us.diag.lock_errors;
        r->lock_err[i] = u->us.lock_err;
        r->heard_lockerr[i] = u->heard_lockerr;
        if (getenv("SIM_DEBUG"))
            printf("    dbg unit %d: tb valid %d have_c %d n %u n_c %u | est seeded %d valid %d n0 %u last %u ppm %.3f | tb_ok %d rej %d ticked %d locked %d | e %.3f integ %.3f trim %.3f\n",
                   i, u->tb.valid, u->tb.have_c, u->tb.n, u->tb.n_c, u->us.est.seeded, u->us.est.valid,
                   u->us.est.n[0], u->us.est.last_n, u->us.est.ppm, u->us.tb_ok, u->us.rejected, u->us.ticked,
                   u->us.rate_locked, u->us.ph.e, u->us.ph.pi.integ, u->us.ph.pi.trim);
        unit_sel(u);
        alink_usb_get_status(&u->us, r->status[i]);
        if (u->g) {
            u->tl_end = sg_timeline(u->g);
            u->want_end = u->g->wants_output();
        }
        unit_unsel();
    }
    if (clicked && U[0].click_t > 0 && U[1].click_t > 0) {
        r->click_ok = true;
        r->click_skew = (U[0].click_t - U[1].click_t) * fs;
    }
}

static void report(const scen_t *s, const res_t *r) {
    printf("%s\n", s->name);
    for (int i = 0; i < nu; i++) {
        double mean = r->m.lat_n[i] ? r->m.lat_sum[i] / r->m.lat_n[i] : NAN;
        printf("    unit %d: state %u anchors %u reanchors %u  latency %.3f..%.3f (mean %.3f) samples  vcxo %.2f ff %.2f "
               "missed %u late %u underruns %u overruns %u lost %u\n",
               i, r->state[i], r->anchors[i], r->reanchors[i], r->m.lat_min[i], r->m.lat_max[i], mean, r->vcxo[i],
               r->ff[i], r->missed[i], r->late[i], r->underruns[i], r->overruns[i], r->lost[i]);
    }
    for (int i = 0; i < nu; i++)
        printf("    unit %d: holes %u (%u hard edges on the playing stream), muted/faded silence %u samples, "
               "GAP events %u, events %u, mis-stamped packets %u of %u\n", i, r->holes[i], r->hole_samples[i],
               r->muted_silence[i], r->gap_events[i], r->events[i], r->misstamps[i], r->stamped[i]);
    printf("    phase-locked %.2f s / %.2f s after boot\n", r->locked_at[0], r->locked_at[1]);
    printf("    stream samples audible while out of step (mode 2): %u / %u\n", r->audible_unlocked[0], r->audible_unlocked[1]);
    if (s->ev_at > 0) printf("    relock after the event at %.1f s: %.3f s / %.3f s\n", s->ev_at, r->relock[0], r->relock[1]);
    if (r->m.skew_n)
        printf("    skew (last 10 s): max %.4f mean %.4f samples (%.2f us)\n", r->m.skew_max, r->m.skew_sum / r->m.skew_n,
               r->m.skew_max * 1e6 / FS);
    if (r->click_ok) printf("    CLICK skew %.4f samples\n", r->click_skew);
}

static scen_t base(const char *name) {
    scen_t s;
    memset(&s, 0, sizeof(s));
    s.name = name;
    s.host_ppm = 35.0;
    s.xtal[0] = 12.0;
    s.xtal[1] = -18.0;
    s.boot[0] = 0.0;
    s.boot[1] = 3.7;            // different boot times: the extended frame counts differ
    s.hub_ns[0] = 150;
    s.hub_ns[1] = 160;
    s.mode = ALINK_USB_MODE_PHASE;
    s.d = 192;
    s.stream_start[0] = 1.0;
    s.stream_start[1] = 1.0;
    s.duration = 40.0;
    return s;
}

// Scenario knobs for expect_locked(): the latency tolerance below D, and
// whether ring underruns are part of the scenario (a main-loop stall).
static double lat_tol = 0.3;
static bool allow_underrun;

static void expect_locked(const scen_t *s, const res_t *r, double skew_max) {
    for (int i = 0; i < nu; i++) {
        // Owner decision 2026-11-14 (WP USB3): nothing of a stream is heard
        // while a mode-2 unit is out of step.
        if (s->mode == ALINK_USB_MODE_PHASE)
            CHECK(r->audible_unlocked[i] == 0, "%s: unit %d muted while out of step (%u stream samples heard)", s->name, i,
                  r->audible_unlocked[i]);
        // review usb1 F1: never a hard edge on the playing stream.
        CHECK(r->holes[i] == 0, "%s: unit %d no hard edge on the playing stream (holes %u)", s->name, i, r->holes[i]);
        double mean = r->m.lat_n[i] ? r->m.lat_sum[i] / r->m.lat_n[i] : 1e9;
        CHECK(r->state[i] == ALINK_USB_ST_PHASE_LOCK && r->m.lat_n[i] > 9000, "%s: unit %d phase-locked and playing",
              s->name, i);
        // A constant part of a sample below D is the same on every unit: the
        // output DMA's quarter-frame word in the fill reading.
        CHECK(mean > s->d - lat_tol && mean < s->d + 0.05 && r->m.lat_max[i] - r->m.lat_min[i] < 0.6,
              "%s: unit %d latency %.3f (spread %.3f), D %u", s->name, i, mean, r->m.lat_max[i] - r->m.lat_min[i], s->d);
        if (!allow_underrun)
            CHECK(r->underruns[i] == 0 && r->overruns[i] == 0, "%s: unit %d no ring underrun/overrun", s->name, i);
    }
    CHECK(r->m.skew_n > 9000 && r->m.skew_max < skew_max, "%s: skew %.4f < %.2f sample", s->name, r->m.skew_max, skew_max);
}

#define WANT(n) (!only || strstr((n), only))

// One LOCKTIME line: phase-locked after boot (cold) or after the event.
static void locktime(const scen_t *s, const res_t *r) {
    if (s->ev_at > 0)
        printf("LOCKTIME %-34s relock %.3f / %.3f s after %.1f s (anchors %u/%u reanchors %u/%u)\n", s->name, r->relock[0],
               r->relock[1], s->ev_at, r->anchors[0], r->anchors[1], r->reanchors[0], r->reanchors[1]);
    else
        printf("LOCKTIME %-34s locked %.3f / %.3f s after boot (anchors %u/%u)\n", s->name, r->locked_at[0],
               r->locked_at[1], r->anchors[0], r->anchors[1]);
}

// The lock-time scenarios check the skew too, with the margin of a 40 s run
// that locked early: the PI is still trimming the rate estimate's 0.1 ppm
// steps (the window rotates every 8 s), S1 and S4 are the skew's own cases.
#define LOCK_SKEW 0.08

static void lock_scenarios(const char *only) {
    res_t r;
    // Cold boot: phase-locked (s after boot) at most; unit 1 boots 3.7 s later.
    {
        scen_t s = base("L1 cold boot, host +35 ppm");
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, LOCK_SKEW);
            locktime(&s, &r);
            CHECK(r.locked_at[0] < 3.0 && r.locked_at[1] < 3.0, "L1: locked within 3 s of boot (%.2f / %.2f)",
                  r.locked_at[0], r.locked_at[1]);
        }
    }
    const double hp[] = { 20.0, 100.0, 290.0, -290.0 };
    const char *const tag[] = { "L2", "L3", "L4", "L4n" };
    const double lim[] = { 2.0, 5.5, 13.5, 13.5 };
    for (int k = 0; k < 4; k++) {
        for (int sd = 0; sd < 2; sd++) {
            static char nm[8][80];
            snprintf(nm[2 * k + sd], sizeof(nm[0]), "%s%s cold boot, host %+.0f ppm%s", tag[k], sd ? "s" : "", hp[k],
                     sd ? ", stored rate 0.4 ppm off" : "");
            scen_t s = base(nm[2 * k + sd]);
            s.host_ppm = hp[k];
            s.xtal[0] = s.xtal[1] = 0;
            s.duration = 40.0;
            if (sd) {
                s.seeded = true;
                s.seed = hp[k] - 0.4;       // the host drifted since the save
            }
            if (WANT(s.name)) {
                run(&s, &r);
                report(&s, &r);
                expect_locked(&s, &r, LOCK_SKEW);
                locktime(&s, &r);
                double l = sd ? 1.2 : lim[k];
                CHECK(r.locked_at[0] < l && r.locked_at[1] < l, "%s: locked within %.1f s of boot (%.2f / %.2f)", s.name,
                      l, r.locked_at[0], r.locked_at[1]);
            }
        }
    }
    {
        // The stored rate is another host's: the quick estimate takes over and
        // the muted fast slew brings the VCXO over (no worse than no seed by much).
        scen_t s = base("L9 cold boot, host +20 ppm, stored rate +290 (another host)");
        s.host_ppm = 20.0;
        s.xtal[0] = s.xtal[1] = 0;
        s.seeded = true;
        s.seed = 290.0;
        s.duration = 40.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, LOCK_SKEW);
            locktime(&s, &r);
            CHECK(r.locked_at[0] < 12.0 && r.locked_at[1] < 12.0, "L9: locked within 12 s (%.2f / %.2f)", r.locked_at[0],
                  r.locked_at[1]);
        }
    }
    {
        scen_t s = base("L5 SOF lost 15 s at 20 s (beyond the ride)");
        s.sof_gap_at = 20.0;
        s.sof_gap_len = 15.0;
        s.gap_counts = true;
        s.ev_at = 35.0;
        s.duration = 60.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, LOCK_SKEW);
            locktime(&s, &r);
            CHECK(r.relock[0] > 0 && r.relock[0] < 1.0 && r.relock[1] > 0 && r.relock[1] < 1.0,
                  "L5: relocked within 1 s of the SOF's return (%.2f / %.2f)", r.relock[0], r.relock[1]);
        }
    }
    {
        scen_t s = base("L6 host stream restart (unit 1, 2 s)");
        s.stop_at = 20.0;
        s.stop_len = 2.0;
        s.ev_at = 22.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, LOCK_SKEW);
            locktime(&s, &r);
            CHECK(r.anchors[1] == 1 && r.reanchors[1] == 0, "L6: the restart keeps the timeline (no relock needed)");
        }
    }
    {
        scen_t s = base("L7 re-anchor: ring skip 10 samples");
        s.skip_at = 25.0;
        s.skip_n = 10;
        s.ev_at = 25.0;
        s.duration = 55.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, LOCK_SKEW);
            locktime(&s, &r);
            CHECK(r.anchors[0] == 1 && r.reanchors[0] == 1 && r.relock[0] > 0 && r.relock[0] < 0.1,
                  "L7: a soft re-anchor (no drain), relocked in %.3f s", r.relock[0]);
        }
    }
    {
        scen_t s = base("L7b re-anchor: ring skip 100 samples (beyond the soft correction)");
        s.skip_at = 25.0;
        s.skip_n = 100;
        s.ev_at = 25.0;
        s.duration = 55.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            allow_underrun = true;          // 100 of the ring's ~130 samples gone: it runs dry until the anchor
            expect_locked(&s, &r, LOCK_SKEW);
            allow_underrun = false;
            locktime(&s, &r);
            CHECK(r.anchors[0] == 2 && r.relock[0] > 0 && r.relock[0] < 0.4, "L7b: a full re-anchor, relocked in %.3f s",
                  r.relock[0]);
        }
    }
    {
        scen_t s = base("L8 re-anchor: output rings reset");
        s.reset_at = 25.0;
        s.ev_at = 25.0;
        s.duration = 55.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, LOCK_SKEW);
            locktime(&s, &r);
            CHECK(r.anchors[0] == 2 && r.relock[0] > 0 && r.relock[0] < 0.1,
                  "L8: re-anchored without a second DAC hold, relocked in %.3f s", r.relock[0]);
        }
    }
}

// WP USB3b: bad RP2350 SOF readings, lock error and its action.
static void usb3b_scenarios(const char *only) {
    res_t r;
    {
        scen_t s = base("W1 bad RP2350 SOF readings (RAW 22.7 us behind LAST) every 7 s on unit 0");
        s.bad_period = 7.0;
        s.duration = 45.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            printf("    bad readings given %u, repaired %u, discontinuities %u\n", r.bad_given[0], r.bad_sof[0], r.disc[0]);
            expect_locked(&s, &r, 0.05);
            CHECK(r.bad_given[0] >= 5 && r.bad_sof[0] >= r.bad_given[0] - 1 && r.disc[0] == 0 && r.anchors[0] == 1 &&
                  r.reanchors[0] == 0, "W1: %u bad readings, %u repaired, no discontinuity, one anchor (anchors %u)",
                  r.bad_given[0], r.bad_sof[0], r.anchors[0]);
        }
    }
    for (int play = 0; play < 2; play++) {
        static char nm[2][80];
        snprintf(nm[play], sizeof(nm[0]), "E1%s host +300 ppm in mode 2 (lock error: host rejected), %s", play ? "p" : "",
                 play ? "action play" : "default action mute");
        scen_t s = base(nm[play]);
        s.host_ppm = 300.0;
        s.xtal[0] = s.xtal[1] = 0;
        s.lock_play = play;
        s.duration = 15.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            for (int i = 0; i < nu; i++) {
                CHECK(r.state[i] == ALINK_USB_ST_REJECTED && r.lock_err[i] && r.lock_errors[i] == 1,
                      "%s: unit %d rejected, in lock error, counted once", s.name, i);
                if (play)
                    CHECK(r.heard_lockerr[i] > 100000, "%s: unit %d plays unaligned (%u samples heard)", s.name, i,
                          r.heard_lockerr[i]);
                else
                    CHECK(r.audible_unlocked[i] == 0 && r.heard_lockerr[i] == 0, "%s: unit %d silent (heard %u)", s.name,
                          i, r.heard_lockerr[i]);
            }
        }
    }
    for (int play = 0; play < 2; play++) {
        static char nm[2][80];
        snprintf(nm[play], sizeof(nm[0]), "E2%s D 160 below d_min (lock error: CONFIG), %s", play ? "p" : "",
                 play ? "action play" : "default action mute");
        scen_t s = base(nm[play]);
        s.d = 160;
        s.lock_play = play;
        s.duration = 15.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            for (int i = 0; i < nu; i++) {
                CHECK(r.lock_err[i] && r.anchors[i] == 0, "%s: unit %d in lock error, no anchor", s.name, i);
                if (play)
                    CHECK(r.heard_lockerr[i] > 100000, "%s: unit %d plays rate-locked (%u samples heard)", s.name, i,
                          r.heard_lockerr[i]);
                else
                    CHECK(r.audible_unlocked[i] == 0 && r.heard_lockerr[i] == 0, "%s: unit %d silent (heard %u)", s.name,
                          i, r.heard_lockerr[i]);
            }
        }
    }
    {
        scen_t s = base("E3 PENDING (auto re-anchor off, ring skip 3 samples at 20 s): muted until pulled in");
        s.no_auto = true;
        s.skip_at = 20.0;
        s.skip_n = 3;
        s.ev_at = 20.0;
        s.duration = 60.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            // The PI that pulled 3 samples in (7 s in this run) is still unwinding
            // its integrator 30 s later (alink_phase's servo, unchanged here), so
            // not expect_locked()'s 0.05-sample bounds: within 0.5 sample of D.
            for (int i = 0; i < nu; i++) {
                double mean = r.m.lat_n[i] ? r.m.lat_sum[i] / r.m.lat_n[i] : 1e9;
                CHECK(r.state[i] == ALINK_USB_ST_PHASE_LOCK && r.holes[i] == 0 && r.audible_unlocked[i] == 0 &&
                      fabs(mean - s.d) < 0.5, "E3: unit %d phase-locked at the end, no hard edge, silent out of step "
                      "(latency %.3f)", i, mean);
            }
            CHECK(r.lock_errors[0] == 1 && !r.lock_err[0] && r.reanchors[0] == 0 && r.heard_lockerr[0] == 0,
                  "E3: lock error entered and left, no re-anchor, nothing heard meanwhile (%u)", r.heard_lockerr[0]);
        }
    }
}

// ==========================================================================
// WP FWTB3: the generator on the USB time base
// ==========================================================================

static const char *const TL_RES[] = { "none", "started", "refused late", "refused not ready", "cancelled loss",
                                      "cancelled config", "cancelled host", "stopped loss" };

static double cap_t_of(const unit_t *u, uint64_t f) {
    return f >= u->cap_lo && f < u->cap_lo + u->cap_len ? u->cap_t[f - u->cap_lo] : NAN;
}
static double cap_c_of(const unit_t *u, uint64_t f) {
    return f >= u->cap_lo && f < u->cap_lo + u->cap_len ? u->cap_c[f - u->cap_lo] : NAN;
}
static float cap_v_of(const unit_t *u, uint64_t f) {
    return f >= u->cap_lo && f < u->cap_lo + u->cap_len ? u->cap_v[f - u->cap_lo] : 0.0f;
}
static unsigned gen_nonzero(const unit_t *u, uint64_t *first, uint64_t *last) {
    unsigned n = 0;
    *first = *last = 0;
    for (uint32_t i = 0; i < u->cap_len; i++) {
        if (u->cap_v[i] == 0.0f) continue;
        if (!n) *first = u->cap_lo + i;
        *last = u->cap_lo + i;
        n++;
    }
    return n;
}

// The reference: copy 4, a plain START at once, blocks of 16.
static bool ref_ready(void) { return true; }
static uint16_t ref_lead(void) { return ARM_LEAD_MS; }
static uint32_t ref_zero(void) { return 0; }
static bool ref_ring(uint32_t *wr, uint32_t *played) { *wr = *played = 0; return false; }
static void ref_event(uint8_t tb, uint8_t st, uint8_t rs, uint32_t t) { (void)tb; (void)st; (void)rs; (void)t; }
static sg_port_t ref_port = { ref_ready, ref_lead, ref_zero, ref_zero, ref_ring, ref_event };
static float *gen_reference(const ugen_t *gp, uint32_t len) {
    const sg_unit_t *g = SGU[4];
    *g->port = &ref_port;
    SiggenConfig c = gen_cfg(gp);
    float *ref = calloc(len + 16u, sizeof(float));
    if (!g->stage_config(&c, sizeof c) || !g->control(SIGGEN_CTL_START)) return ref;
    for (uint32_t i = 0; i < len; i += 16) g->render(ref + i, GEN_OUT, 16, fs);
    return ref;
}

static void gen_checks(const scen_t *s) {
    const ugen_t *gp = s->gen;
    const uint32_t len = gen_len(gp);
    const float amp = powf(10.0f, -20.0f / 20.0f);
    // PLAN-AT FWTB3: under 0.1 sample on USB between two units; each unit
    // within half of it of the domain's timeline (the mean of the units).
    const double tol = 0.1;
    const double fsh = fs * (1.0 + host_ppm * 1e-6);    // the domain's rate: the host's
    float *ref = gen_reference(gp, len);
    uint32_t k0 = 0;
    while (k0 < len && ref[k0] == 0.0f) k0++;
    printf("    reference: the first nonzero sample is sample %u\n", k0);
    for (int i = 0; i < nu; i++) {
        unit_t *u = &U[i];
        const sg_timeline_t *tl = &u->tl_end;
        uint64_t f1, f2;
        unsigned nz = gen_nonzero(u, &f1, &f2);
        printf("    unit %d generator: T %llu (S origin %+lld wraps of 2048 frames from unit 0), result %s start %llu "
               "samples %u disc %u run_id %u; 0xE2 op 9 applied %u late %u cancelled %u refused %u; %u nonzero samples "
               "played; EXT_TIME_BASE t - output_frames %u reads, %u off; 0xE4 events %u\n",
               i, (unsigned long long)u->t_write,
               (long long)llround((double)(int64_t)(u->t_write - U[0].t_write - (uint64_t)(int64_t)gp->stagger[i] +
                                                    (uint64_t)(int64_t)gp->stagger[0]) / (2048.0 * (fs / 1000u))),
               tl->result < 8 ? TL_RES[tl->result] : "?", (unsigned long long)tl->start, tl->samples, tl->disc,
               tl->run_id, u->op9_res[0], u->op9_res[1], u->op9_res[2], u->op9_res[3], nz, u->tb_inv_reads,
               u->tb_inv_bad, u->tb_events);
        CHECK(u->tb_inv_reads > 1000 && u->tb_inv_bad == 0, "%s: unit %d t - output_frames constant while LOCKED (%u of %u)",
              s->name, i, u->tb_inv_bad, u->tb_inv_reads);
        CHECK(!u->want_end, "%s: unit %d the DAC is handed back at the end", s->name, i);
        if (gp->kind == G_LATE)
            CHECK(u->wr_past == TB_ERR_STATE && u->wr_short == TB_ERR_STATE && u->wr_bus == TB_ERR_VALUE,
                  "%s: unit %d refused at the write: T passed (0x%02x), inside the arm lead (0x%02x), bus-frame form "
                  "(0x%02x, R5)", s->name, i, u->wr_past, u->wr_short, u->wr_bus);
    }
    if (gp->kind == G_LOSS) {
        // SOF lost mid-run on the bus: unit 1's run stops (STOPPED_LOSS, the
        // discontinuity counted, silent from then), unit 0's later START is
        // cancelled (CANCELLED_LOSS, nothing played).
        unit_t *a = &U[0], *b = &U[1];
        uint64_t f1, f2;
        CHECK(a->wr_code == TB_OK && a->tl_end.result == TB_RES_CANCELLED_LOSS && a->op9_res[ALINK_APPLY_CANCELLED] == 1 &&
                  !a->tl_run_seen && gen_nonzero(a, &f1, &f2) == 0,
              "%s: unit 0 the pending START cancelled by the loss (%s), nothing played", s->name,
              TL_RES[a->tl_end.result & 7]);
        unsigned nz = gen_nonzero(b, &f1, &f2);
        double t_last = cap_t_of(b, f2);
        printf("    unit 1: last nonzero generator sample S %llu played at %.4f s (SOF lost from %.4f s)\n",
               (unsigned long long)f2, t_last, s->sof_gap_at);
        CHECK(b->wr_code == TB_OK && b->tl_run_seen && b->tl_run.start == b->t_write &&
                  b->tl_end.result == TB_RES_STOPPED_LOSS && b->tl_end.disc >= 1 && nz > 0 && f1 == b->t_write + k0 &&
                  f2 < b->t_write + len - 1,
              "%s: unit 1 started on T, stopped by the loss, the timeline counts it (%s, disc %u)", s->name,
              TL_RES[b->tl_end.result & 7], b->tl_end.disc);
        CHECK(t_last < s->sof_gap_at + 0.2, "%s: unit 1 silent soon after the SOF stopped (%.4f s)", s->name, t_last);
        free(ref);
        return;
    }
    double em_of[NU] = { 0 }, ex_of[NU] = { 0 };
    for (int i = 0; i < nu; i++) {
        unit_t *u = &U[i];
        uint64_t f1, f2;
        unsigned nz = gen_nonzero(u, &f1, &f2);
        CHECK(u->wr_code == TB_OK && u->tl_end.result == TB_RES_STARTED && u->tl_end.start == u->t_write &&
                  u->tl_end.tb == TB_ID_USB && u->tl_end.disc == 0 && u->op9_res[ALINK_APPLY_APPLIED] == 1 &&
                  u->tl_end.run_id == 1,
              "%s: unit %d op 9 by S accepted, started on T (start %llu, %s, disc %u)", s->name, i,
              (unsigned long long)u->tl_end.start, TL_RES[u->tl_end.result & 7], u->tl_end.disc);
        unsigned miss = 0, twice = 0;
        double vd = 0;
        for (uint32_t j = 0; j < len; j++) {
            uint64_t f = u->t_write + j;
            uint8_t c = f >= u->cap_lo && f < u->cap_lo + u->cap_len ? u->cap_n[f - u->cap_lo] : 0;
            miss += c == 0;
            twice += c > 1;
            vd = fmax(vd, fabs((double)cap_v_of(u, f) - ref[j]));
        }
        CHECK(miss == 0 && twice == 0, "%s: unit %d every sample of the run played once (%u missing, %u twice)", s->name,
              i, miss, twice);
        CHECK(nz > 0 && f1 == u->t_write + k0, "%s: unit %d no generator sample before T, the first on T + %u", s->name,
              i, k0);
        CHECK(vd <= (gp->type == SIGGEN_IMPULSE ? 0.0 : 1e-3), "%s: unit %d the reference's samples on T.. (%.3g)",
              s->name, i, vd);
        if (gp->type == SIGGEN_IMPULSE)
            CHECK(fabsf(cap_v_of(u, u->t_write) - amp) < 1e-6f && nz == gp->repeat,
                  "%s: unit %d the impulse train: sample 0 on T, %u impulses (%u)", s->name, i, gp->repeat, nz);
        if (i == 0) continue;
        // Against unit 0: the same programme sample (host S) under generator
        // sample 0 (plus the stagger), and the same instants.
        double c_i = cap_c_of(u, u->t_write), c_0 = cap_c_of(&U[0], U[0].t_write);
        double dc = c_i - c_0 - (double)(gp->stagger[i] - gp->stagger[0]);
        double sum = 0, mx = 0;
        unsigned k = 0;
        for (uint32_t j = 0; j < len; j++) {
            double t1 = cap_t_of(&U[0], U[0].t_write + j), t2 = cap_t_of(u, u->t_write + j);
            if (isnan(t1) || isnan(t2)) continue;
            double e = (t2 - t1) * fsh - (double)(gp->stagger[i] - gp->stagger[0]);
            sum += e;
            if (fabs(e) > mx) mx = fabs(e);
            k++;
        }
        em_of[i] = k ? sum / k : NAN;
        ex_of[i] = mx;
        printf("    unit %d against unit 0: generator sample 0 on host sample %+.0f of unit 0's (stagger %+d); play "
               "instants %+.4f sample mean, %.4f max\n", i, c_i - c_0, gp->stagger[i] - gp->stagger[0], em_of[i], mx);
        CHECK(!isnan(dc) && dc == 0.0, "%s: unit %d the generator's sample 0 sits on unit 0's programme sample (%+.0f)",
              s->name, i, dc);
    }
    double mean = 0;
    for (int i = 0; i < nu; i++) mean += em_of[i];
    mean /= nu;
    for (int i = 0; i < nu; i++) {
        double dev = em_of[i] - mean;
        printf("    unit %d: the run's instants %+.4f sample off the domain's (the mean of %d units)\n", i, dev, nu);
        CHECK(fabs(dev) < tol / 2 && ex_of[i] < tol,
              "%s: unit %d plays the run within the lock error (%+.4f off the timeline, under %.2f; any sample against "
              "unit 0 %.4f, under %.2f)", s->name, i, dev, tol / 2, ex_of[i], tol);
    }
    free(ref);
}

// A generator scenario in its own process (fresh generator copies).
static void gen_run(const scen_t *s) {
    fflush(NULL);
    int fd[2];
    if (pipe(fd) != 0) { perror("pipe"); exit(2); }
    pid_t pid = fork();
    if (pid < 0) { perror("fork"); exit(2); }
    if (pid == 0) {
        close(fd[0]);
        checks = failures = 0;
        static res_t r;
        run(s, &r);
        report(s, &r);
        gen_checks(s);
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
        printf("  FAIL: %s: the scenario's process died (status 0x%x)\n", s->name, st);
        v[0]++;
        v[1]++;
    }
    checks += v[0];
    failures += v[1];
}

static scen_t gen_base(const char *name, int n, const ugen_t *g) {
    scen_t s = base(name);
    s.nu = n;
    s.gen = g;
    // Units 2 and 3 (RP2350; RP2040 builds have no ops, OD-ATm).
    s.xtal[2] = 31.0;
    s.xtal[3] = -44.0;
    s.boot[2] = 1.6;
    s.boot[3] = 5.2;
    s.hub_ns[2] = 140;
    s.hub_ns[3] = 175;
    s.stream_start[2] = s.stream_start[3] = 1.0;
    s.duration = 32.0;
    return s;
}

static void gen_scenarios(const char *only) {
    // Ops 9 written about 0.5 s ahead at 30 s (units locked for 20 s or more).
    static const ugen_t g_imp = { G_COINCIDE, SIGGEN_IMPULSE, 5, 30.5, 30.0, { 0 } };
    static const ugen_t g_swp = { G_COINCIDE, SIGGEN_SWEEP_LOG, 3, 30.5, 30.0, { 0 } };
    static const ugen_t g_stag = { G_COINCIDE, SIGGEN_IMPULSE, 5, 30.5, 30.0, { 0, 4807, 9613, -3 } };
    static const ugen_t g_late = { G_LATE, SIGGEN_IMPULSE, 5, 30.5, 30.0, { 0 } };
    static const ugen_t g_loss = { G_LOSS, SIGGEN_SWEEP_LOG, 3, 30.5, 30.0, { 72000, 0 } };
    {
        scen_t s = gen_base("G1 two units, impulses at one S", 2, &g_imp);
        if (WANT(s.name)) gen_run(&s);
    }
    {
        scen_t s = gen_base("G2 two units, log sweeps at one S, host -60 ppm", 2, &g_swp);
        s.host_ppm = -60.0;
        if (WANT(s.name)) gen_run(&s);
    }
    {
        scen_t s = gen_base("G3 four units, impulses at one S (boots 0 / 3.7 / 1.6 / 5.2 s)", 4, &g_imp);
        if (WANT(s.name)) gen_run(&s);
    }
    {
        scen_t s = gen_base("G4 four units, log sweeps at one S, 96 kHz", 4, &g_swp);
        s.fs = 96000u;
        s.d = 336;          // d_min 324 at 96 kHz (S12)
        if (WANT(s.name)) gen_run(&s);
    }
    {
        scen_t s = gen_base("G5 four units, staggered starts (+4807, +9613, -3 samples)", 4, &g_stag);
        if (WANT(s.name)) gen_run(&s);
    }
    {
        scen_t s = gen_base("G6 SOF lost 0.3 s mid-run: unit 1's run stopped, unit 0's pending START cancelled", 2, &g_loss);
        s.sof_gap_at = 30.8;
        s.sof_gap_len = 0.3;
        s.no_jump = true;
        if (WANT(s.name)) gen_run(&s);
    }
    {
        scen_t s = gen_base("G7 late and bus-frame-form STARTs refused at the write, then on T", 2, &g_late);
        if (WANT(s.name)) gen_run(&s);
    }
}

int main(int argc, char **argv) {
    const char *only = argc > 1 ? argv[1] : NULL;
    res_t r;

    {
        scen_t s = base("S1 RP2350 pair, host +35 ppm, boots 3.7 s apart");
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[0] == 1 && r.anchors[1] == 1 && r.reanchors[0] == 0 && r.reanchors[1] == 0,
                  "S1: one anchor each, no re-anchor");
        }
    }
    {
        scen_t s = base("S2 RP2040 pair (ISR time base, 2 % late entries)");
        s.rp2040[0] = s.rp2040[1] = true;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.15);
        }
    }
    {
        scen_t s = base("S3 mixed RP2350 + RP2040");
        s.rp2040[1] = true;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.15);
        }
    }
    const double offs[] = { 100.0, -100.0 };
    for (int k = 0; k < 2; k++) {
        static char nm[2][80];
        snprintf(nm[k], sizeof(nm[k]), "S4 host %+.0f ppm", offs[k]);
        scen_t s = base(nm[k]);
        s.host_ppm = offs[k];
        s.duration = 60.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
        }
    }
    const double rej[] = { 300.0, -300.0, 400.0, -400.0 };
    for (int k = 0; k < 4; k++) {
        static char nm[4][80];
        snprintf(nm[k], sizeof(nm[k]), "S5 host %+.0f ppm (beyond the VCXO)", rej[k]);
        scen_t s = base(nm[k]);
        s.host_ppm = rej[k];
        s.xtal[0] = s.xtal[1] = 0;
        s.duration = 12.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            for (int i = 0; i < nu; i++)
                CHECK(r.state[i] == ALINK_USB_ST_REJECTED && r.vcxo[i] == 0.0 && r.anchors[i] == 0,
                      "%s: unit %d rejected, VCXO parked, stock (state %u)", s.name, i, r.state[i]);
        }
    }
    {
        scen_t s = base("S6 missing packets (0.5 % of frames skipped by the host)");
        s.miss_rate = 0.005;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.missed[0] > 0 && r.missed[1] > 0 && r.reanchors[0] == 0 && r.reanchors[1] == 0,
                  "S6: gaps filled with silence on the timeline, no re-anchor");
        }
    }
    {
        scen_t s = base("S7 host stream restart on unit 1 (2 s at alt 0)");
        s.stop_at = 15.0;
        s.stop_len = 2.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[1] == 1 && r.reanchors[1] == 0, "S7: the restarted stream is placed on the same timeline");
        }
    }
    {
        scen_t s = base("S8 SOF gap 120 ms, host frame number jumps (bus suspend / controller restart)");
        s.sof_gap_at = 15.0;
        s.sof_gap_len = 0.12;
        s.duration = 45.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[0] == 2 && r.anchors[1] == 2, "S8: one re-anchor each after the time base returns");
        }
    }
    {
        scen_t s = base("S9 staggered stream start (model A, unit 1 starts 3 frames later)");
        // Both units anchored first (on silence), then the streams start.
        s.stream_start[0] = 20.0;
        s.stream_start[1] = 20.003;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            uint16_t f0, f1;
            memcpy(&f0, r.status[0] + 32, 2);
            memcpy(&f1, r.status[1] + 32, 2);
            int d = ((int)f1 - (int)f0 + 2048) % 2048;
            CHECK(d >= 2 && d <= 4, "S9: status first-packet bus frames %u and %u, %d frames apart", f0, f1, d);
        }
    }
    {
        scen_t s = base("S10 CLICK at one bus frame on both units");
        s.click_at = 30.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            CHECK(r.click_ok && fabs(r.click_skew) < 0.05, "S10: click skew %.4f samples", r.click_skew);
        }
    }
    {
        scen_t s = base("S11 contrast: mode 1 (rate lock only)");
        s.mode = ALINK_USB_MODE_RATE;
        s.stream_start[1] = 12.3;           // B starts after both are rate-locked
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            for (int i = 0; i < nu; i++)
                CHECK(r.state[i] == ALINK_USB_ST_RATE_LOCK, "S11: unit %d rate-locked", i);
            // Rate-locked only, nothing holds the phase: the output follows each
            // packet's arrival and the fill servo (tens of samples of spread),
            // against hundredths of a sample in mode 2.
            double mean = r.m.skew_n ? r.m.skew_sum / r.m.skew_n : 0.0;
            CHECK(r.m.skew_n > 9000 && mean > 1.0, "S11: rate-locked only, mean skew %.2f samples (mode 2: < 0.05)", mean);
        }
    }
    {
        scen_t s = base("S12 96 kHz, D 336 (d_min 324 at 96 kHz)");
        s.fs = 96000;
        s.d = 336;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.1);
        }
    }
    // ----------------------------------------------------------------------
    // review usb1 probes
    // ----------------------------------------------------------------------
    {
        scen_t s = base("P1 single lost packets (0.5 %): are the holes muted?");
        s.miss_rate = 0.005;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.holes[0] + r.holes[1] == 0, "P1: no unmuted silence on a playing stream (holes %u / %u)", r.holes[0],
                  r.holes[1]);
        }
    }
    {
        scen_t s = base("P2 burst: unit 1 loses 5 consecutive packets at 20 s");
        s.burst_at = 20.0;
        s.burst_n = 5;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.reanchors[1] == 0 && r.gap_events[1] >= 1, "P2: a GAP event, no re-anchor (gaps %u, reanchors %u)",
                  r.gap_events[1], r.reanchors[1]);
        }
    }
    {
        scen_t s = base("P2b burst: unit 1 loses 2 consecutive packets at 20 s");
        s.burst_at = 20.0;
        s.burst_n = 2;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.holes[1] == 0, "P2b: the 2-packet gap is muted (holes %u, gaps %u)", r.holes[1], r.gap_events[1]);
        }
    }
    const double edge[] = { 294.0, -294.0, 296.0, -296.0 };
    for (int k = 0; k < 4; k++) {
        static char nm[4][80];
        snprintf(nm[k], sizeof(nm[k]), "P3 host %+.0f ppm (the 295 ppm boundary)", edge[k]);
        scen_t s = base(nm[k]);
        s.host_ppm = edge[k];
        s.xtal[0] = s.xtal[1] = 0;
        s.duration = 80.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            bool rej = fabs(edge[k]) > 295.0;
            for (int i = 0; i < nu; i++) {
                if (rej)
                    CHECK(r.state[i] == ALINK_USB_ST_REJECTED && r.anchors[i] == 0, "%s: unit %d rejected (state %u)",
                          s.name, i, r.state[i]);
                else
                    CHECK(r.state[i] == ALINK_USB_ST_PHASE_LOCK, "%s: unit %d accepted and phase-locked (state %u)",
                          s.name, i, r.state[i]);
            }
            if (!rej) expect_locked(&s, &r, 0.05);
        }
    }
    {
        scen_t s = base("P4 host drifts 280 -> 300 ppm at 30 s, back to 280 at 60 s");
        s.xtal[0] = s.xtal[1] = 0;
        s.host_step_at = 30.0;
        s.host_ppm2 = 300.0;
        s.host_step2_at = 60.0;
        s.host_ppm3 = 280.0;
        s.duration = 160.0;              // the pull-in back from 0 to 280 ppm takes 42 s
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[0] <= 3 && r.anchors[1] <= 3 && r.anchors[0] >= 2 && r.anchors[1] >= 2,
                  "P4: re-anchored at the step, rejected, anchored again; no anchor loop (anchors %u / %u)",
                  r.anchors[0], r.anchors[1]);
        }
    }
    {
        scen_t s = base("P5 suspend: SOF stops 3 s at 15 s, frame number continues");
        s.sof_gap_at = 15.0;
        s.sof_gap_len = 3.0;
        s.no_jump = true;
        s.duration = 45.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[0] == 2 && r.anchors[1] == 2, "P5: SOF_LOST, one new anchor each (anchors %u / %u)",
                  r.anchors[0], r.anchors[1]);
        }
    }
    {
        scen_t s = base("P6 USB port reset mid-stream: SOF gone 15 ms at 15 s (frame counter runs on), unit 1 stream down 0.4 s");
        s.sof_gap_at = 15.0;
        s.sof_gap_len = 0.015;
        s.gap_counts = true;
        s.stop_at = 15.0;
        s.stop_len = 0.4;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[0] == 1 && r.anchors[1] == 1 && r.reanchors[0] == 0 && r.reanchors[1] == 0,
                  "P6: the timeline survives the reset (anchors %u / %u, reanchors %u / %u)", r.anchors[0], r.anchors[1],
                  r.reanchors[0], r.reanchors[1]);
        }
    }
    {
        scen_t s = base("P6b SOF stalls 15 ms at 15 s with the frame number frozen (host hiccup), unit 1 stream down 0.4 s");
        s.sof_gap_at = 15.0;
        s.sof_gap_len = 0.015;
        s.no_jump = true;
        s.stop_at = 15.0;
        s.stop_len = 0.4;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.reanchors[0] <= 1 && r.reanchors[1] <= 1, "P6b: at most one re-anchor each (reanchors %u / %u)",
                  r.reanchors[0], r.reanchors[1]);
        }
    }
    {
        scen_t s = base("P8c interrupts masked 400 us every 100 ms on unit 0 (shared SOF / completion hold-off)");
        s.mask_period = 0.1;
        s.mask_len = 400e-6;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            // Documented limitation (review usb1 F5): a hold-off over 100 us
            // across a SOF can mis-stamp a packet on either chip; rare, faded.
            CHECK(r.misstamps[0] <= 8, "P8c: at most 8 mis-stamped packets in 400 hold-offs (mis-stamped %u, late %u missed %u holes %u)",
                  r.misstamps[0], r.late[0], r.missed[0], r.holes[0]);
        }
    }
    {
        scen_t s = base("P6c SOF stalls 1 ms at 15 s with the frame number frozen (RP2350 pair)");
        s.sof_gap_at = 15.0;
        s.sof_gap_len = 0.001;
        s.no_jump = true;
        s.duration = 60.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.reanchors[0] <= 1 && r.reanchors[1] <= 1, "P6c: at most one re-anchor each (reanchors %u / %u)",
                  r.reanchors[0], r.reanchors[1]);
        }
    }
    {
        scen_t s = base("P4b host steps +35 -> +60 ppm at 30 s (a 25 ppm feed-forward error while locked)");
        s.host_step_at = 30.0;
        s.host_ppm2 = 60.0;
        s.duration = 70.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            lat_tol = 0.5;                  // the PI still pulling the last tenths in after two re-anchors
            expect_locked(&s, &r, 0.05);
            lat_tol = 0.3;
            CHECK(r.reanchors[0] <= 2 && r.reanchors[1] <= 2, "P4b: at most two re-anchors each (reanchors %u / %u)",
                  r.reanchors[0], r.reanchors[1]);
        }
    }
    {
        scen_t s = base("P7 main-loop stall 10 ms on unit 0 at 20 s");
        s.stall_at = 20.0;
        s.stall_len = 0.010;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            // By design at D 192 (review usb1 F6): a stall longer than the ring
            // fill (3-4 ms) underruns and re-anchors once, muted.
            allow_underrun = true;
            expect_locked(&s, &r, 0.06);
            allow_underrun = false;
            CHECK(r.reanchors[0] <= 1, "P7: at most one re-anchor after a 10 ms stall (reanchors %u, gaps %u)",
                  r.reanchors[0], r.gap_events[0]);
        }
    }
    {
        scen_t s = base("P7b main-loop stall 100 ms on unit 0 at 20 s");
        s.stall_at = 20.0;
        s.stall_len = 0.100;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            allow_underrun = true;
            expect_locked(&s, &r, 0.05);
            allow_underrun = false;
            CHECK(r.reanchors[0] == 1, "P7b: a 100 ms stall re-anchors once (reanchors %u)", r.reanchors[0]);
        }
    }
    {
        scen_t s = base("P8 interrupts masked 300 us every 1 s on unit 0 (shared SOF / completion hold-off)");
        s.mask_period = 1.0;
        s.mask_len = 300e-6;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.misstamps[0] == 0 && r.late[0] == 0, "P8: no mis-stamped packets (late %u missed %u holes %u)", r.late[0],
                  r.missed[0], r.holes[0]);
        }
    }
    {
        scen_t s = base("P8b interrupts masked 300 us every 1 s, RP2040 pair");
        s.rp2040[0] = s.rp2040[1] = true;
        s.mask_period = 1.0;
        s.mask_len = 300e-6;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.15);
            CHECK(r.misstamps[0] == 0 && r.late[0] == 0, "P8b: no mis-stamped packets (late %u missed %u holes %u)", r.late[0],
                  r.missed[0], r.holes[0]);
        }
    }
    {
        scen_t s = base("P9 D at d_max (399)");
        s.d = 399;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
        }
    }
    {
        scen_t s = base("P10 D at d_min (170 at 48 kHz)");
        s.d = 170;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.late[0] == 0 && r.late[1] == 0, "P10: no late samples at d_min (late %u / %u)", r.late[0], r.late[1]);
        }
    }
    // ----------------------------------------------------------------------
    // Round 3 (hardware findings H1/H2, re-review R3)
    // ----------------------------------------------------------------------
    {
        scen_t s = base("H1 mode 1 with no stream: the VCXO settles on the host rate");
        s.mode = ALINK_USB_MODE_RATE;
        s.stream_start[0] = s.stream_start[1] = 1e9;
        s.duration = 40.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            for (int i = 0; i < nu; i++)
                CHECK(r.state[i] == ALINK_USB_ST_RATE_LOCK && fabs(r.vcxo[i] - r.ff[i]) < 1.0,
                      "H1: unit %d rate-locked, VCXO %.2f at the feed-forward %.2f (not pulled by an empty ring)", i,
                      r.vcxo[i], r.ff[i]);
        }
    }
    {
        // review usb1 N2: the alt stays set but no packet comes (a paused
        // application, a host xrun): the fill servo must not pull the VCXO.
        scen_t s = base("N2 mode 1, unit 1's stream stalls 25 s with the alt still set");
        s.mode = ALINK_USB_MODE_RATE;
        s.burst_at = 15.0;
        s.burst_n = 1000000;
        s.duration = 40.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            // Unit 0 keeps streaming (the fill servo trims it); unit 1, stalled,
            // stays on the feed-forward instead of being pulled by its empty ring.
            CHECK(r.state[0] == ALINK_USB_ST_RATE_LOCK, "N2: unit 0 (streaming) rate-locked");
            CHECK(r.state[1] == ALINK_USB_ST_RATE_LOCK && fabs(r.vcxo[1] - r.ff[1]) < 1.0,
                  "N2: unit 1 (stalled, alt set) rate-locked, VCXO %.2f at the feed-forward %.2f", r.vcxo[1], r.ff[1]);
        }
    }
    {
        scen_t s = base("H2 mode 2 with no stream: anchors on silence (DAC mute hold 1 s)");
        s.dac_hold = 1.0;
        s.stream_start[0] = s.stream_start[1] = 1e9;
        s.duration = 30.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            for (int i = 0; i < nu; i++)
                CHECK(r.state[i] == ALINK_USB_ST_PHASE_LOCK && r.anchors[i] == 1 && r.underruns[i] == 0 &&
                      fabs(r.vcxo[i] - r.ff[i]) < 1.0,
                      "H2: unit %d phase-locked without a stream (state %u, anchors %u)", i, r.state[i], r.anchors[i]);
        }
    }
    {
        scen_t s = base("H2b mode 2 anchored without a stream, streams start at 25 s");
        s.stream_start[0] = s.stream_start[1] = 25.0;
        s.duration = 45.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[0] == 1 && r.anchors[1] == 1, "H2b: anchored once, before the streams (anchors %u / %u)",
                  r.anchors[0], r.anchors[1]);
        }
    }
    {
        scen_t s = base("R3 stream stop: the last packet 5 ms before the alt 0 is no GAP");
        s.stop_at = 20.0;
        s.stop_len = 2.0;
        s.stop_lead = 0.005;
        // Only the stop's own gap: the model's random packet loss (1 % of the
        // passes take 900 us and miss the re-arm) can put a real GAP into the
        // 130 ms window and made this check depend on the random sequence.
        s.no_long_pass = true;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.gaps_at_stop == 0, "R3: GAPs counted around the stop: %d", r.gaps_at_stop);
        }
    }
    // ----------------------------------------------------------------------
    // WP USB2: ride through SOF absences with the host's frame counter running
    // ----------------------------------------------------------------------
    const double absent[] = { 0.1, 0.5, 2.0, 3.0 };
    for (int k = 0; k < 4; k++) {
        static char nm[4][100];
        snprintf(nm[k], sizeof(nm[k]), "U%d SOF absent %.1f s at 20 s, frame counter running: ridden through", k + 1, absent[k]);
        scen_t s = base(nm[k]);
        s.sof_gap_at = 20.0;
        s.sof_gap_len = absent[k];
        s.gap_counts = true;
        s.duration = 40.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[0] == 1 && r.anchors[1] == 1 && r.reanchors[0] == 0 && r.reanchors[1] == 0 &&
                  r.rides[0] >= 1 && r.rides[1] >= 1,
                  "%s: one anchor each, no re-anchor, ridden (rides %u / %u)", s.name, r.rides[0], r.rides[1]);
        }
    }
    {
        scen_t s = base("U5 SOF absent 12 s at 20 s (beyond the 10 s ride-through), counter running: one new anchor");
        s.sof_gap_at = 20.0;
        s.sof_gap_len = 12.0;
        s.gap_counts = true;
        s.duration = 55.0;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            expect_locked(&s, &r, 0.05);
            CHECK(r.anchors[0] == 2 && r.anchors[1] == 2, "U5: one new anchor each (anchors %u / %u)", r.anchors[0],
                  r.anchors[1]);
        }
    }
    // ----------------------------------------------------------------------
    // WP USB3: lock times (LOCKTIME lines) and out-of-step muting
    // ----------------------------------------------------------------------
    lock_scenarios(only);
    usb3b_scenarios(only);
    gen_scenarios(only);                // WP FWTB3
    printf("alink_usb_sim: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
