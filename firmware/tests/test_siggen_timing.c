// Host test for WP FWSIGGEN requirement (2): with no host stream the
// generator's zero-input pump emits exactly the scheduled frames, locked to
// the DAC's frame clock, with no insert or drop across cycles and gaps, under
// simulated main-loop jitter; and the one slip path (a ring that runs dry) is
// counted, frame for frame.
//
// Runs the real siggen.c engine and the real pump rule (siggen_pump.h).  The
// model around them mirrors the firmware: a DAC consumes one frame per
// sample period from a slot ring of 512 frames (448 writable, the ERASE_AHEAD
// margin), process_input_block() runs the pre-give recovery and the group
// admission before rendering (audio_pipeline.c output_rings_pre_give()), and
// renders the generator only into an admitted block (siggen_hold() otherwise).
// Delay lines are not modelled (a constant offset for every frame).
//
// Every run is compared sample for sample with a reference: the same config
// rendered in back-to-back blocks of the pump's size with no pump, no ring and
// no jitter.  (Same block size because the fade and window gains are
// interpolated per segment, so their rounding follows block edges; the
// sample timing does not.)
#define PICO_RP2350 1
#define USB_AUDIO_H              // siggen.c needs only audio_state from it
#include "config.h"
#include "notify.h"
#include <stdint.h>
typedef struct { uint32_t freq; } AudioStateStub;
static volatile AudioStateStub audio_state = { 48000u };
#include "siggen.c"
#include "siggen_pump.h"

#include <stdio.h>
#include <stdlib.h>

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

MatrixMixer matrix_mixer;
void notify_push_siggen_state(uint8_t s, uint8_t r, uint8_t t, uint8_t c) { (void)s; (void)r; (void)t; (void)c; }
static bool ready_now = true;
bool siggen_output_ready_now(void) { return ready_now; }

#define FS        48000u
#define TARGET       96u     // 2 ms output latency (the default option)
#define CAP          49u     // BLOCK_CAP(1000 us, 48 kHz)
#define SPAN        448u     // RING_FRAMES - ERASE_AHEAD
#define OUT_CH        2u     // output 3, as in the K340 measurement
#define MAXF   (FS * 40u)    // 40 s of played frames

static float bufs[NUM_OUTPUT_CHANNELS][AUDIO_BUFFER_SAMPLES];
static float *ref, *played;
static uint32_t ref_len;

// ---- the slot-0 ring and the DAC ---------------------------------------------
typedef struct {
    uint64_t now_ns;
    uint32_t wr;              // frames written (wr_frames)
    uint32_t prewrite_fill;   // recovery_prewrite_fill_frames (UINT32_MAX: none yet)
    uint32_t underruns;       // dry episodes recovered
    uint32_t underrun_frames; // frames skipped by recovery (the ring's underrun_frames)
    uint32_t refused;         // blocks the admission refused
    int      refuse_next;     // test hook: refuse the next N blocks
    bool     render_refused;  // the pre-FWSIGGEN order: render into a refused block
} sim_t;

static uint32_t consumed(const sim_t *s) { return (uint32_t)(s->now_ns * FS / 1000000000ull); }
static uint32_t fill(const sim_t *s) {
    int32_t f = (int32_t)(s->wr - consumed(s));
    return f > 0 ? (uint32_t)f : 0u;
}

// process_input_block(n) on zero input: pre-give, admission, render, write.
static void block(sim_t *s, uint32_t n) {
    int32_t deficit = (int32_t)(consumed(s) - s->wr);
    if (deficit > 0 && s->prewrite_fill != UINT32_MAX) {
        uint32_t skip = (uint32_t)deficit + s->prewrite_fill;   // output_rings_pre_give()
        s->wr += skip;
        s->underruns++;
        s->underrun_frames += skip;
    }
    bool admit = fill(s) + n <= SPAN;
    if (s->refuse_next > 0) { s->refuse_next--; admit = false; }
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++)
        for (uint32_t i = 0; i < n; i++) bufs[o][i] = 0.0f;
    if (siggen_running) {
        if (admit || s->render_refused) siggen_render(bufs, n, FS);
        else                            siggen_hold(bufs, n);
    }
    if (!admit) { s->refused++; return; }
    s->prewrite_fill = fill(s);
    for (uint32_t i = 0; i < n; i++)
        if (s->wr + i < MAXF) played[s->wr + i] = bufs[OUT_CH][i];
    s->wr += n;
}

// One main-loop pass: the pump exactly as siggen_pump() runs it.
static void pump(sim_t *s) {
    siggen_pump_gate_t g = { .running = siggen_running, .source_owns_rings = false, .preset_loading = false,
                             .source_is_usb = true, .slots_ready = true };
    if (!siggen_pump_may_run(&g)) return;
    for (uint32_t b = 0; siggen_pump_wants_block(fill(s), TARGET, b); b++) block(s, CAP);
}

static uint32_t rng = 12345u;
static uint32_t rnd(uint32_t n) { rng = rng * 1103515245u + 12345u; return (rng >> 8) % n; }

// Main-loop gaps: mostly short, with heavy tails up to max_us.
static uint64_t gap_ns(uint32_t max_us) {
    uint32_t r = rnd(100);
    uint32_t us = r < 80 ? 5 + rnd(60) : r < 97 ? 50 + rnd(600) : rnd(max_us + 1);
    return (uint64_t)us * 1000u;
}

static SiggenConfig mkcfg(uint8_t type, uint16_t repeat, uint16_t gap_ms, uint32_t dur_ms, float p1, float p2) {
    SiggenConfig c;
    memset(&c, 0, sizeof c);
    c.version = SIGGEN_CFG_VERSION;
    c.signal_type = type;
    c.channel_mask = 1u << OUT_CH;
    c.level_db = -6.0f;
    c.repeat = repeat;
    c.gap_ms = gap_ms;
    c.duration_ms = dur_ms;
    c.p1 = p1;
    c.p2 = p2;
    return c;
}

static void start(const SiggenConfig *c) {
    CHECK(siggen_stage_config(c, sizeof *c));
    CHECK(siggen_control(SIGGEN_CTL_START));
}

// The schedule rendered in back-to-back blocks: the ground truth.
static void make_ref(const SiggenConfig *c) {
    start(c);
    ref_len = 0;
    while (siggen_running && ref_len + CAP <= MAXF) {
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) memset(bufs[o], 0, sizeof bufs[o]);
        siggen_render(bufs, CAP, FS);
        memcpy(&ref[ref_len], bufs[OUT_CH], CAP * sizeof(float));
        ref_len += CAP;
    }
    siggen_service();
}

// Pump-driven run.  Returns the first ring frame of the generator's output.
static uint32_t run(sim_t *s, const SiggenConfig *c, uint32_t max_gap_us, uint64_t stall_at_ns, uint32_t stall_us) {
    memset(played, 0, MAXF * sizeof(float));
    s->now_ns = 777777u;              // the DAC has played a while: arbitrary phase
    s->wr = consumed(s) + TARGET;     // the idle ring as the idle pump left it
    s->prewrite_fill = 40;
    start(c);
    uint32_t first = s->wr;
    bool stalled = false;
    while (siggen_running && s->wr < MAXF - 4096) {
        pump(s);
        siggen_service();
        uint64_t g = gap_ns(max_gap_us);
        if (!stalled && stall_us && s->now_ns >= stall_at_ns) { g = (uint64_t)stall_us * 1000u; stalled = true; }
        s->now_ns += g;
    }
    siggen_service();
    return first;
}

static bool quiet;
static bool same(float a, float b) { return fabsf(a - b) <= 2e-6f; }

// Played (from `first`) equals the reference, with `ins` frames of silence
// inserted at played frame `at` (ins = 0: none).
static bool matches(uint32_t first, uint32_t at, uint32_t ins) {
    for (uint32_t i = 0; i < ref_len; i++) {
        uint32_t p = first + i + (i >= at ? ins : 0u);
        if (p >= MAXF || !same(played[p], ref[i])) {
            if (!quiet) printf("  mismatch at ref %u: played %g ref %g\n", i, p < MAXF ? played[p] : 0.0f, ref[i]);
            return false;
        }
    }
    for (uint32_t k = 0; k < ins; k++)
        if (played[first + at + k] != 0.0f) return false;
    return true;
}

// Played frames of the k-th impulse (single-sample clicks at cycle starts).
static uint32_t impulses(uint32_t first, uint32_t *pos, uint32_t max) {
    uint32_t n = 0;
    for (uint32_t i = first; i < MAXF && n < max; i++)
        if (fabsf(played[i]) > 0.25f) pos[n++] = i;
    return n;
}

int main(void) {
    ref = calloc(MAXF, sizeof(float));
    played = calloc(MAXF, sizeof(float));
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) matrix_mixer.outputs[o].enabled = 1;

    // 1. Impulses every 100 ms + 37 ms gap, 20 periods, main-loop jitter up to
    //    1.9 ms (below the 2 ms fill): every impulse on its scheduled DAC frame.
    {
        SiggenConfig c = mkcfg(SIGGEN_IMPULSE, 20, 37, 0, 100.0f, 0.0f);
        make_ref(&c);
        sim_t s = { 0 };
        uint32_t first = run(&s, &c, 1900, 0, 0);
        SiggenStatus st;
        siggen_get_status(&st);
        CHECK(st.cycles_done == 20 && st.stop_reason == SIGGEN_STOP_COMPLETED && !siggen_running);
        CHECK(s.underruns == 0 && s.refused == 0);
        uint32_t pos[32];
        uint32_t n = impulses(first, pos, 32);
        CHECK(n == 20);
        uint32_t period = FS * 100u / 1000u + FS * 37u / 1000u;   // cycle + gap
        bool exact = n == 20;
        for (uint32_t k = 0; k < n; k++) exact = exact && pos[k] == pos[0] + k * period;
        CHECK(exact);
        CHECK(pos[0] == first);                   // the first impulse at the first frame
        CHECK(matches(first, 0, 0));
    }

    // 2. Log sweeps 500 ms, 4 passes, 200 ms gaps (the P5 timetable shape):
    //    sample for sample the fixed-block schedule.
    SiggenConfig sweep = mkcfg(SIGGEN_SWEEP_LOG, 4, 200, 500, 20.0f, 20000.0f);
    {
        make_ref(&sweep);
        CHECK(ref_len >= FS * (4u * 500u + 3u * 200u) / 1000u);
        sim_t s = { 0 };
        uint32_t first = run(&s, &sweep, 1900, 0, 0);
        SiggenStatus st;
        siggen_get_status(&st);
        CHECK(st.cycles_done == 4 && st.stop_reason == SIGGEN_STOP_COMPLETED);
        CHECK(s.underruns == 0 && s.refused == 0);
        CHECK(matches(first, 0, 0));
    }

    // 3. Walked sweeps over 3 outputs with gaps, several jitter seeds: the
    //    schedule of the measured output is unchanged by the main loop.
    {
        SiggenConfig c = sweep;
        c.channel_mask = (1u << 0) | (1u << OUT_CH) | (1u << 3);
        c.flags = SIGGEN_FLAG_WALK;
        c.repeat = 2;
        make_ref(&c);
        for (uint32_t seed = 1; seed <= 5; seed++) {
            rng = seed * 7919u;
            sim_t s = { 0 };
            uint32_t first = run(&s, &c, 1900, 0, 0);
            CHECK(s.underruns == 0);
            CHECK(matches(first, 0, 0));
        }
    }

    // 4. A main-loop stall longer than the fill (5 ms against 2 ms): the ring
    //    runs dry once, recovery skips frames, and the slip is exactly the
    //    counted underrun_frames, inserted as silence where the ring ran dry.
    {
        make_ref(&sweep);
        rng = 99u;
        sim_t s = { 0 };
        uint64_t stall_at = 777777u + 1200000000ull;      // 1.2 s in, inside pass 2
        uint32_t first = run(&s, &sweep, 1900, stall_at, 5000);
        CHECK(s.underruns == 1);
        CHECK(s.underrun_frames >= 5000u * FS / 1000000u - TARGET);
        // Where it ran dry: the first played frame after the last real one.
        uint32_t at = 0;
        for (uint32_t i = FS; i < ref_len; i++)
            if (!same(played[first + i], ref[i])) { at = i; break; }
        CHECK(at > 0);
        CHECK(matches(first, at, s.underrun_frames));
    }

    // 5. A block the admission refuses (here forced, e.g. a PDM queue that
    //    cannot take it): the generator holds, so nothing drops.  The
    //    pre-FWSIGGEN order (render, then drop) loses exactly that block.
    {
        make_ref(&sweep);
        sim_t s = { 0 };
        s.refuse_next = 0;
        // Refuse one block 0.3 s in by arming the hook from the jitter loop.
        memset(played, 0, MAXF * sizeof(float));
        s.now_ns = 777777u;
        s.wr = consumed(&s) + TARGET;
        s.prewrite_fill = 40;
        start(&sweep);
        uint32_t first = s.wr;
        bool armed = false;
        while (siggen_running && s.wr < MAXF - 4096) {
            if (!armed && s.wr - first > FS * 3u / 10u) { s.refuse_next = 1; armed = true; }
            pump(&s);
            s.now_ns += gap_ns(1500);
        }
        CHECK(s.refused == 1 && s.underruns == 0);
        CHECK(matches(first, 0, 0));

        sim_t o = { 0 };
        o.render_refused = true;
        memset(played, 0, MAXF * sizeof(float));
        o.now_ns = 777777u;
        o.wr = consumed(&o) + TARGET;
        o.prewrite_fill = 40;
        start(&sweep);
        first = o.wr;
        armed = false;
        while (siggen_running && o.wr < MAXF - 4096) {
            if (!armed && o.wr - first > FS * 3u / 10u) { o.refuse_next = 1; armed = true; }
            pump(&o);
            o.now_ns += gap_ns(1500);
        }
        CHECK(o.refused == 1);
        quiet = true;
        CHECK(!matches(first, 0, 0));             // the old order slips by one block
        quiet = false;
    }

    // 6. A START that waits for the DAC (siggen_dac.h): status FADE_IN, the
    //    engine and the pump idle; the gate starts the full schedule intact.
    {
        make_ref(&sweep);
        ready_now = false;
        CHECK(siggen_stage_config(&sweep, sizeof sweep));
        CHECK(siggen_control(SIGGEN_CTL_START));
        SiggenStatus st;
        siggen_get_status(&st);
        CHECK(st.state == SIGGEN_STATE_FADE_IN && !siggen_running && siggen_wants_output());
        siggen_output_gate(false);
        CHECK(!siggen_running);
        siggen_output_gate(true);
        CHECK(siggen_running);
        uint32_t n = 0;
        bool ok = true;
        while (siggen_running && n + CAP <= ref_len) {
            for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) memset(bufs[o], 0, sizeof bufs[o]);
            siggen_render(bufs, CAP, FS);
            for (uint32_t i = 0; i < CAP; i++) ok = ok && same(bufs[OUT_CH][i], ref[n + i]);
            n += CAP;
        }
        CHECK(ok && n == ref_len);
        siggen_service();
        // A STOP while waiting cancels: idle, reason HOST, nothing started.
        CHECK(siggen_control(SIGGEN_CTL_START));
        CHECK(siggen_wants_output() && !siggen_running);
        CHECK(siggen_control(SIGGEN_CTL_STOP));
        siggen_get_status(&st);
        CHECK(!siggen_wants_output() && st.state == SIGGEN_STATE_IDLE && st.stop_reason == SIGGEN_STOP_HOST);
        siggen_output_gate(true);
        CHECK(!siggen_running);
        ready_now = true;
    }

    // 7. The pump's gate: a source that owns the rings (ADAT / alink locked, a
    //    USB phase timeline) excludes it; preset_loading only on non-USB.
    {
        siggen_pump_gate_t g = { .running = true, .source_is_usb = true, .slots_ready = true };
        CHECK(siggen_pump_may_run(&g));
        g.source_owns_rings = true;
        CHECK(!siggen_pump_may_run(&g));
        g.source_owns_rings = false;
        g.preset_loading = true;
        CHECK(siggen_pump_may_run(&g));
        g.source_is_usb = false;
        CHECK(!siggen_pump_may_run(&g));
        g.preset_loading = false;
        g.type_switch = true;
        CHECK(!siggen_pump_may_run(&g));
        CHECK(!siggen_pump_wants_block(TARGET, TARGET, 0));
        CHECK(siggen_pump_wants_block(TARGET - 1, TARGET, 1));
        CHECK(!siggen_pump_wants_block(0, TARGET, SIGGEN_PUMP_MAX_BLOCKS));
    }

    printf("test_siggen_timing: %d checks, %d failures\n", checks, failures);
    free(ref);
    free(played);
    return failures ? 1 : 0;
}
