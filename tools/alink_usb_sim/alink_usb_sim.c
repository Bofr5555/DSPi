// alink_usb_sim.c - system simulation of alink USB-direct units (WP USB1).
//
// One USB host and two units on its bus.  Each unit runs the real engine
// (firmware/DSPi/alink_usb/alink_usb_core.c with link/link_phase.c) through the
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
// Build: cc -std=c11 -O2 -I firmware/DSPi tools/alink_usb_sim/alink_usb_sim.c -lm
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link/link_phase.c"
#include "alink_usb/alink_usb_core.c"

#define NU      2
#define RING    (1u << 16)
#define FS      48000u

alink_usb_tb_t alink_usb_tb;
const link_apply_limits_t alink_usb_apply_limits = { 10, 9, 2048 };

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
    // stream
    bool streaming;
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
} unit_t;

typedef struct scen scen_t;
static const scen_t *S;
static double g_stall_at, g_stall_len;

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
static void write_one(double c) {
    cur->content[cur->W % RING] = c;
    cur->W++;
    if (cur->running && ring_fill() > 448.0) cur->overruns++;
}
// Holes (review usb1 F1): a hard edge on the playing stream, not covered by
// the pipeline's soft mute: audio cut to silence without a fade-out, or
// silence followed by audio without a fade-in.
static bool playing(void) {
    return cur->us.started && cur->us.stream && cur->us.stream_seen && cur->us.ph.state == LINK_PHASE_LOCKED &&
           cur->W >= cur->mute_end_w;
}
void alink_usb_port_write_packet(const alink_usb_pkt_t *p, uint32_t off, uint32_t k, float g0, float g1) {
    const spkt_t *sp = (const spkt_t *)p->data;
    if (k && cur->last_gain == 0.0 && g0 > 0.1f && playing()) {
        cur->holes++;
        cur->hole_samples++;
    }
    for (uint32_t i = 0; i < k; i++) write_one(sp->c0 + off + i);
    if (k) cur->last_gain = g1;
}
void alink_usb_port_write_silence(uint32_t k) {
    if (k && cur->last_gain > 0.1 && playing()) {
        cur->holes++;
        cur->hole_samples++;
    } else if (k && playing()) {
        cur->muted_silence += k;
    }
    for (uint32_t i = 0; i < k; i++) write_one(NAN);
    if (k) cur->last_gain = 0.0;
}
const void *alink_usb_port_keep(const alink_usb_pkt_t *p) {
    cur->keep = *(const spkt_t *)p->data;
    return &cur->keep;
}
static void vcxo_cmd(double ppm) {
    if (ppm > cur->last_cmd + 0.5) ppm = cur->last_cmd + 0.5;
    if (ppm < cur->last_cmd - 0.5) ppm = cur->last_cmd - 0.5;
    cur->last_cmd = ppm;
    if (ppm > 300) ppm = 300;
    if (ppm < -300) ppm = -300;
    cur->vcxo = round(ppm / 0.11) * 0.11;
}
void alink_usb_port_actuate(float hz, float trim, bool ff_trim) {
    double ff = ((double)hz / fs - 1.0) * 1e6;
    if (ff_trim) {
        cur->fill_int = 0;
        cur->fill_smooth = cur->target / 48.0;
        vcxo_cmd(ff + trim);
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
    vcxo_cmd(ff + p + cur->fill_int);
}
void alink_usb_port_park(void) {
    cur->vcxo = cur->last_cmd = 0;
}
float alink_usb_port_vcxo_ppm(void) { return (float)cur->vcxo; }
uint8_t alink_usb_port_apply(const link_apply_op_t *op) {
    if (op->op == LINK_OP_CLICK) {
        cur->click_w = cur->W;          // the next sample written
        cur->click_armed = true;
        return LINK_APPLY_APPLIED;
    }
    return op->op == LINK_OP_SYNC_PULSE ? LINK_APPLY_REFUSED : LINK_APPLY_APPLIED;
}
void alink_usb_port_apply_result(uint8_t op, uint8_t r, uint32_t f) { (void)op; (void)r; (void)f; }
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
void alink_usb_port_ring_target(void) { cur->target = alink_usb_ring_target(&cur->us, fs, fs / 500u, cap(), 448); }

// --------------------------------------------------------------------------
// Unit and host model
// --------------------------------------------------------------------------

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
        if (alink_usb_engaged(&u->us)) {
            alink_usb_pkt_t ap = { p->frame, p->n, p, 0 };
            alink_usb_deliver(&u->us, &ap);
        } else {
            for (uint32_t i = 0; i < p->n; i++) write_one(p->c0 + i);
        }
        u->rt = (u->rt + 1) & 3;
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
        if (alink_usb_anchor_wanted(&u->us)) {
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
    alink_usb_service(&u->us, (uint32_t)(local_us(T) / 1000.0));
    if (!u->locked_at && u->us.state == ALINK_USB_ST_PHASE_LOCK) u->locked_at = T - u->boot_s;
    double pass = 60e-6 + urand() * 240e-6;
    if (urand() < 0.01) pass = 900e-6;
    if (g_stall_len > 0 && u == &U[0] && !u->stalled && T >= g_stall_at) {
        pass = g_stall_len;             // one long pass (a flash write, a long command)
        u->stalled = true;
    }
    u->next_pass = T + pass;
    unit_unsel();
}

static void unit_start(unit_t *u) {
    unit_sel(u);
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
    for (int i = 0; i < NU; i++) {
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
    for (int i = 0; i < NU; i++) {
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

struct scen {
    const char *name;
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
    double sof_gap_at, sof_gap_len;     // SOF stops (bus suspend / host controller restart), frame number jumps
    double click_at;            // CLICK on both units at the same bus frame
    uint32_t fs;                // 0: 48 kHz
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
    double lat_after_restart[NU];
    uint8_t first_bus[NU];
    uint8_t status[NU][ALINK_USB_STATUS_LEN];
} res_t;

static void run(const scen_t *s, res_t *r) {
    memset(U, 0, sizeof(U));
    memset(r, 0, sizeof(*r));
    S = s;
    g_stall_at = s->stall_at;
    g_stall_len = s->stall_len;
    int burst_done = 0;
    host_ppm = s->host_ppm;
    host_n = 1000000u + (uint64_t)(urand() * 2048);
    host_f11 = (uint32_t)host_n & 0x7FF;
    T = 0;
    next_sof = 0;
    sof_on = true;
    fs = s->fs ? s->fs : FS;
    for (int i = 0; i < NU; i++) {
        U[i].xtal_ppm = s->xtal[i];
        U[i].boot_s = s->boot[i];
        U[i].hub_ns = s->hub_ns[i];
        U[i].rp2040 = s->rp2040[i];
        U[i].mode = s->mode;
        U[i].d = s->d;
        U[i].stream_offset = s->stream_offset[i];
        unit_init(&U[i]);
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
                for (int i = 0; i < NU; i++) {
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
                    for (int i = 0; i < NU; i++) {
                        unit_sel(&U[i]);
                        uint8_t p[16] = { 0 };
                        uint32_t f = (uint32_t)click_frame;
                        memcpy(p, &f, 4);
                        p[8] = LINK_OP_CLICK;
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
        for (int i = 0; i < NU; i++) {
            unit_t *u = &U[i];
            if (!u->booted) continue;
            bool masked = i == 0 && s->mask_len > 0 && fmod(T, s->mask_period) < s->mask_len;
            if (!masked && u->pend_sof) {
                // interrupts back: the SOF handler runs first (dcd_rp2040_irq), then buff status
                unit_sof_at(u, u->pend_f11, u->pend_sof_true, T);
                u->pend_sof = false;
            }
            if (u->comp_t > 0 && T >= u->comp_t && !masked) unit_complete(u);
            if (u->start_at >= 0 && T >= u->start_at) unit_start(u);
            if (T >= u->next_pass) unit_pass(u);
            if (u->running) {
                double rate = fs * (1 + u->xtal_ppm * 1e-6) * (1 + u->vcxo * 1e-6);
                double c0 = u->C;
                u->C += rate * dt;
                if (u->click_armed && u->C >= (double)u->click_w) {
                    u->click_t = T - dt + ((double)u->click_w - c0) / rate;
                    u->click_armed = false;
                }
                if (u->C > (double)u->W) {
                    // Counted on the phase timeline only: before the anchor the
                    // stock path's idle pump (not modelled) covers an empty ring.
                    if (u->us.started) u->underruns++;
                    uint64_t w = (uint64_t)ceil(u->C);  // the writer re-anchors to the reader
                    while (u->W < w) u->content[u->W++ % RING] = NAN;
                }
            }
        }
        if (T >= s->duration - 10.0 && T >= next_meas) {
            measure(&r->m);
            next_meas = T + 1e-3;
        }
        T += dt;
    }
    for (int i = 0; i < NU; i++) {
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
        if (getenv("SIM_DEBUG"))
            printf("    dbg unit %d: tb valid %d have_c %d n %u n_c %u sofs %u | est seeded %d valid %d n0 %u last %u ppm %.3f | tb_ok %d rej %d ticked %d locked %d | e %.3f integ %.3f trim %.3f\n",
                   i, u->tb.valid, u->tb.have_c, u->tb.n, u->tb.n_c, u->tb.sofs, u->us.est.seeded, u->us.est.valid,
                   u->us.est.n[0], u->us.est.last_n, u->us.est.ppm, u->us.tb_ok, u->us.rejected, u->us.ticked,
                   u->us.rate_locked, u->us.ph.e, u->us.ph.pi.integ, u->us.ph.pi.trim);
        unit_sel(u);
        alink_usb_get_status(&u->us, r->status[i]);
        unit_unsel();
    }
    if (clicked && U[0].click_t > 0 && U[1].click_t > 0) {
        r->click_ok = true;
        r->click_skew = (U[0].click_t - U[1].click_t) * fs;
    }
}

static void report(const scen_t *s, const res_t *r) {
    printf("%s\n", s->name);
    for (int i = 0; i < NU; i++) {
        double mean = r->m.lat_n[i] ? r->m.lat_sum[i] / r->m.lat_n[i] : NAN;
        printf("    unit %d: state %u anchors %u reanchors %u  latency %.3f..%.3f (mean %.3f) samples  vcxo %.2f ff %.2f "
               "missed %u late %u underruns %u overruns %u lost %u\n",
               i, r->state[i], r->anchors[i], r->reanchors[i], r->m.lat_min[i], r->m.lat_max[i], mean, r->vcxo[i],
               r->ff[i], r->missed[i], r->late[i], r->underruns[i], r->overruns[i], r->lost[i]);
    }
    for (int i = 0; i < NU; i++)
        printf("    unit %d: holes %u (%u hard edges on the playing stream), muted/faded silence %u samples, "
               "GAP events %u, events %u, mis-stamped packets %u of %u\n", i, r->holes[i], r->hole_samples[i],
               r->muted_silence[i], r->gap_events[i], r->events[i], r->misstamps[i], r->stamped[i]);
    printf("    phase-locked %.2f s / %.2f s after boot\n", r->locked_at[0], r->locked_at[1]);
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
    for (int i = 0; i < NU; i++) {
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

int main(int argc, char **argv) {
    const char *only = argc > 1 ? argv[1] : NULL;
#define WANT(n) (!only || strstr((n), only))
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
            for (int i = 0; i < NU; i++)
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
        s.stream_start[1] = 1.2;
        if (WANT(s.name)) {
            run(&s, &r);
            report(&s, &r);
            for (int i = 0; i < NU; i++)
                CHECK(r.state[i] == ALINK_USB_ST_RATE_LOCK, "S11: unit %d rate-locked", i);
            double l0 = r.m.lat_sum[0] / r.m.lat_n[0], l1 = r.m.lat_sum[1] / r.m.lat_n[1];
            CHECK(r.m.lat_n[0] > 9000 && r.m.lat_n[1] > 9000 && fabs(l0 - l1) > 1.0,
                  "S11: rate-locked only, the latencies are whatever the start left: %.2f and %.2f samples", l0, l1);
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
            for (int i = 0; i < NU; i++) {
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
    printf("alink_usb_sim: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
