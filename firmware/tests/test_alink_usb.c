// Host test for alink_usb/alink_usb_core.c (WP USB1): the USB-frame time base
// (frame extension, clean pairs, packet stamps), the host-rate estimator with
// SOF jitter, EXT_ALINK_USB validation and persistence, the D limits, the
// apply-at-frame time bases, the status layout, and that mode 0 leaves the
// stock behaviour alone (no actuation, no events, no engagement, async
// feedback).  The closed-loop behaviour (anchor, lock, restarts, two units)
// is in tools/alink_usb_sim.
// Built by run.sh: cc -std=c11 -Wall -Wextra -Werror -I ../DSPi test_alink_usb.c
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link/link_phase.c"
#include "alink_usb/alink_usb_core.c"

static int failures, checks;
#define CHECK(c, ...)                                                                                                \
    do {                                                                                                             \
        checks++;                                                                                                    \
        if (!(c)) {                                                                                                  \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);                                                     \
            fprintf(stderr, __VA_ARGS__);                                                                            \
            fprintf(stderr, "\n");                                                                                   \
            failures++;                                                                                              \
        }                                                                                                            \
    } while (0)

// ---------------------------------------------------------------------------
// Port stubs: a recorder
// ---------------------------------------------------------------------------

alink_usb_tb_t alink_usb_tb;
const link_apply_limits_t alink_usb_apply_limits = { 10, 9, 2048 };
static uint32_t now48, now_us, fs = 48000;
static int n_actuate, n_park, n_events, n_writes, n_results, n_mute;
static float vcxo;
static uint8_t last_result;
static uint32_t last_result_frame;
static float fill_now = 150.0f;

uint32_t alink_usb_port_now48(void) { return now48; }
uint32_t alink_usb_port_now_us(void) { return now_us; }
uint32_t alink_usb_port_irq_save(void) { return 0; }
void alink_usb_port_irq_restore(uint32_t s) { (void)s; }
uint32_t alink_usb_port_fs(void) { return fs; }
bool alink_usb_port_play(float *fill) { *fill = fill_now; return true; }
uint32_t alink_usb_port_room(void) { return 49; }
void alink_usb_port_write_packet(const alink_usb_pkt_t *p, uint32_t off, uint32_t k) { (void)p; (void)off; (void)k; n_writes++; }
void alink_usb_port_write_silence(uint32_t k) { (void)k; n_writes++; }
void alink_usb_port_actuate(float hz, float trim, bool ff) { (void)trim; (void)ff; n_actuate++; vcxo = (hz / (float)fs - 1.0f) * 1e6f; }
void alink_usb_port_park(void) { n_park++; vcxo = 0.0f; }
float alink_usb_port_vcxo_ppm(void) { return vcxo; }
uint8_t alink_usb_port_apply(const link_apply_op_t *op) { (void)op; return LINK_APPLY_APPLIED; }
void alink_usb_port_apply_result(uint8_t op, uint8_t r, uint32_t f) { (void)op; last_result = r; last_result_frame = f; n_results++; }
void alink_usb_port_event(uint8_t st, uint8_t rs, uint32_t f) { (void)st; (void)rs; (void)f; n_events++; }
void alink_usb_port_soft_mute(uint32_t s) { (void)s; n_mute++; }
bool alink_usb_port_pulse(uint32_t f) { (void)f; return false; }
void alink_usb_port_ring_target(void) {}

static void reset_counts(void) { n_actuate = n_park = n_events = n_writes = n_results = n_mute = 0; }

// ---------------------------------------------------------------------------

static void test_time_base(void) {
    alink_usb_tb_t tb;
    memset(&tb, 0, sizeof(tb));
    // RP2350 (exact): first SOF at bus frame 2040, then through the wrap.
    uint32_t t = 1000u;
    alink_usb_tb_sof(&tb, 2040, t + 96, 96);
    CHECK(tb.valid && tb.n == 2040 && tb.t == t, "first SOF n %u t %u", tb.n, tb.t);
    for (int i = 1; i <= 20; i++) alink_usb_tb_sof(&tb, (2040u + i) & 0x7FF, t + i * 48000u + 300, 300);
    CHECK(tb.n == 2060 && (tb.n & 0x7FF) == 12, "extended across the wrap: %u", tb.n);
    CHECK(tb.have_c && tb.n_c == 2060 && tb.t_c == t + 20u * 48000u, "exact clean pair %u %u", tb.n_c, tb.t_c);
    // Three SOFs missed (IRQ held off 3 ms): still exact on RP2350.
    alink_usb_tb_sof(&tb, (2064u) & 0x7FF, t + 24u * 48000u + 5000, 5000);
    CHECK(tb.n == 2064 && tb.n_c == 2064, "a 4-frame step counts and stays clean: %u", tb.n);
    // Over a second dark: flagged.
    alink_usb_tb_sof(&tb, (2064u + 1500u) & 0x7FF, t + (24u + 1500u) * 48000u, 0);
    CHECK(tb.long_gap, "a 1.5 s gap is flagged");

    // RP2040 (ISR time): clean only for 1-frame steps within 3 us.
    memset(&tb, 0, sizeof(tb));
    alink_usb_tb_sof(&tb, 5, 48000u * 5u, UINT32_MAX);
    alink_usb_tb_sof(&tb, 6, 48000u * 6u + 48u, UINT32_MAX);       // 1 us later than nominal
    CHECK(tb.have_c && tb.n_c == 6, "RP2040 clean within 3 us");
    alink_usb_tb_sof(&tb, 7, 48000u * 7u + 48u * 40u, UINT32_MAX);  // a 40 us late entry
    CHECK(tb.n_c == 6, "a late entry is not clean");
    CHECK(tb.t == 48000u * 7u + 48u, "a late entry's SOF time comes from the clean pair: %u", tb.t);

    // Stamps: a completion 150 us after the SOF is that frame's; 20 us after
    // is the previous frame's (interrupt held off across the SOF).
    memset(&tb, 0, sizeof(tb));
    alink_usb_tb_sof(&tb, 100, 4800000u, 0);
    alink_usb_tb_stamp(&tb, 4800000u + 150u * 48u);
    CHECK(tb.stamp == 100, "stamp in frame %u", tb.stamp);
    alink_usb_tb_stamp(&tb, 4800000u + 20u * 48u);
    CHECK(tb.stamp == 99, "stamp across the SOF %u", tb.stamp);
}

static uint32_t rng = 12345u;
static double urand(void) {
    rng = rng * 1664525u + 1013904223u;
    return (double)(rng >> 8) / 16777216.0;
}

static void est_run(double host_ppm, double jitter_us, int late_pct, float *ppm_out, bool *valid_out, int frames) {
    alink_usb_tb_t tb;
    alink_usb_est_t e;
    memset(&tb, 0, sizeof(tb));
    alink_usb_est_reset(&e);
    double period = 1000.0 / (1.0 + host_ppm * 1e-6);    // us of the unit's crystal
    for (int i = 0; i < frames; i++) {
        double t = 1000.0 + i * period;
        double lat = 1.0 + urand() * jitter_us;
        if ((int)(urand() * 100.0) < late_pct) lat += 30.0 + urand() * 200.0;
        // RP2040: the ISR reads time_us_32() (whole microseconds).
        uint32_t now = (uint32_t)(uint64_t)(t + lat) * 48u;
        alink_usb_tb_sof(&tb, (uint32_t)i & 0x7FF, now, UINT32_MAX);
        if (tb.have_c) alink_usb_est_sample(&e, tb.n_c, tb.t_c);
    }
    *ppm_out = e.ppm;
    *valid_out = e.valid;
}

static void test_estimator(void) {
    float ppm;
    bool valid;
    est_run(100.0, 2.0, 0, &ppm, &valid, 1900);
    CHECK(!valid, "not valid before a 2 s span");
    const double cases[] = { 0.0, 100.0, -100.0, 300.0, -400.0 };
    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        est_run(cases[c], 2.0, 2, &ppm, &valid, 20000);
        CHECK(valid && fabs(ppm - cases[c]) < 0.5, "RP2040 jitter 2 us + 2 %% late entries, host %+.0f ppm: %.3f",
              cases[c], ppm);
    }
    // RP2350-like exact timestamps (sub-microsecond).
    alink_usb_tb_t tb;
    alink_usb_est_t e;
    memset(&tb, 0, sizeof(tb));
    alink_usb_est_reset(&e);
    for (int i = 0; i < 9000; i++) {
        double t48 = 48.0 * (1000.0 + i * 1000.0 / (1.0 + 57.3e-6)) + (urand() - 0.5) * 2.0;   // +-21 ns hub jitter
        uint32_t since = 48u * 3u;
        alink_usb_tb_sof(&tb, (uint32_t)i & 0x7FF, (uint32_t)(uint64_t)t48 + since, since);
        if (tb.have_c) alink_usb_est_sample(&e, tb.n_c, tb.t_c);
    }
    CHECK(e.valid && fabs(e.ppm - 57.3) < 0.05, "exact timestamps: %.4f", e.ppm);
}

static void test_config(void) {
    alink_usb_t us;
    alink_usb_init(&us);
    uint8_t o[ALINK_USB_CFG_LEN];
    alink_usb_get_config(&us, o);
    const uint8_t def[8] = { 0, 1, 192, 0, 0, 0, 0, 0 };
    CHECK(!memcmp(o, def, 8), "defaults: mode 0, auto re-anchor, D 192");
    // 2 ms option at 48 kHz: block cap 49; span 448.
    alink_usb_limits(&us, 48000, 96, 49, 448);
    CHECK(us.d_min == 153 && us.d_max == 399, "48 kHz limits %u..%u", us.d_min, us.d_max);
    alink_usb_limits(&us, 96000, 192, 97, 448);
    CHECK(us.d_min == 273 && us.d_max == 351, "96 kHz limits %u..%u", us.d_min, us.d_max);
    alink_usb_limits(&us, 48000, 96, 49, 448);
    uint8_t p[8] = { 2, 1, 0x80, 0, 0, 0, 0, 0 };          // D 128 < 153
    CHECK(!alink_usb_set_config(&us, p), "mode 2 with D below d_min refused");
    p[0] = 1;
    CHECK(alink_usb_set_config(&us, p), "mode 1 takes any D");
    p[0] = 2; p[2] = 200;
    CHECK(alink_usb_set_config(&us, p) && us.mode == 2 && us.d == 200, "mode 2 D 200");
    uint8_t bad[][8] = { { 3, 1, 200, 0, 0, 0, 0, 0 }, { 2, 2, 200, 0, 0, 0, 0, 0 }, { 2, 1, 200, 0, 1, 0, 0, 0 },
                         { 1, 1, 0, 0, 0, 0, 0, 0 }, { 2, 1, 0x90, 1, 0, 0, 0, 0 } };
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        uint8_t before[8];
        alink_usb_get_config(&us, before);
        CHECK(!alink_usb_set_config(&us, bad[i]), "bad payload %u refused", i);
        alink_usb_get_config(&us, o);
        CHECK(!memcmp(o, before, 8), "a refused write changes nothing (%u)", i);
    }
    // IN-only bytes 6-7 are ignored on OUT.
    uint8_t in_only[8] = { 2, 1, 200, 0, 0, 0, 0x07, 0x05 };
    CHECK(alink_usb_set_config(&us, in_only), "IN-only bytes ignored on OUT");
    // Persistence.
    uint8_t st[ALINK_USB_STORE_LEN];
    alink_usb_store(&us, st);
    alink_usb_t u2;
    alink_usb_init(&u2);
    alink_usb_load(&u2, st);
    CHECK(u2.mode == 2 && u2.d == 200 && u2.flags == 1, "store / load round trip");
    uint8_t zero[4] = { 0 };
    alink_usb_init(&u2);
    alink_usb_load(&u2, zero);
    CHECK(u2.mode == 0 && u2.d == 192, "a never-saved block keeps the defaults");
    uint8_t junk[4] = { 9, 0, 1, 0 };
    alink_usb_load(&u2, junk);
    CHECK(u2.mode == 0, "implausible stored values keep the defaults");
    // Ring target: the phase prefill only when the phase lock can run.
    us.mode = 1;
    CHECK(alink_usb_ring_target(&us, 48000, 96, 49, 448) == 96, "mode 1 keeps the stock target");
    us.mode = 2;
    CHECK(alink_usb_ring_target(&us, 48000, 96, 49, 448) == 200 - 49 - 16, "mode 2 prefill D - cap - 16");
    CHECK(alink_usb_ring_target(&us, 44100, 88, 46, 448) == 88, "44.1 kHz: no phase lock, stock target");
}

// Mode 0 is stock: nothing actuates, nothing is sent, packets are not taken.
static void test_mode0_stock(void) {
    alink_usb_t us;
    alink_usb_init(&us);
    memset(&alink_usb_tb, 0, sizeof(alink_usb_tb));
    reset_counts();
    for (int i = 0; i < 5000; i++) {
        now48 = 48000u * (uint32_t)i;
        now_us = 1000u * (uint32_t)i;
        alink_usb_tb_sof(&alink_usb_tb, (uint32_t)i & 0x7FF, now48, 0);
        alink_usb_service(&us, (uint32_t)i);
    }
    CHECK(n_actuate == 0 && n_park == 0 && n_events == 0 && n_writes == 0 && n_mute == 0,
          "mode 0: no actuation %d, park %d, events %d, writes %d, mutes %d", n_actuate, n_park, n_events, n_writes, n_mute);
    CHECK(!alink_usb_nominal_feedback(&us) && !alink_usb_engaged(&us) && us.state == ALINK_USB_ST_OFF,
          "mode 0: asynchronous feedback, packets on the stock path, state off");
    uint8_t ap[16] = { 0 };
    ap[8] = LINK_OP_MUTE;
    ap[12] = 1;
    CHECK(!alink_usb_apply(&us, ap), "mode 0: apply refused");
    // Mode 1 then 0: the stop parks the VCXO and reports DISABLED.
    us.mode = 1;
    for (int i = 5000; i < 9000; i++) {
        now48 = 48000u * (uint32_t)i;
        now_us = 1000u * (uint32_t)i;
        alink_usb_tb_sof(&alink_usb_tb, (uint32_t)i & 0x7FF, now48, 0);
        alink_usb_service(&us, (uint32_t)i);
    }
    CHECK(n_actuate > 0 && us.rate_locked && alink_usb_nominal_feedback(&us) && us.state == ALINK_USB_ST_RATE_LOCK,
          "mode 1 locks (state %u, actuations %d)", us.state, n_actuate);
    us.mode = 0;
    alink_usb_service(&us, 10000);
    CHECK(n_park == 1 && us.state == ALINK_USB_ST_OFF && !alink_usb_nominal_feedback(&us), "back to mode 0: parked, off");
}

// Host rate beyond the VCXO's authority: rejected, stock feedback.
static void test_reject(void) {
    const double cases[] = { 290.0, 300.0, -400.0 };
    for (unsigned c = 0; c < 3; c++) {
        alink_usb_t us;
        alink_usb_init(&us);
        us.mode = 1;
        memset(&alink_usb_tb, 0, sizeof(alink_usb_tb));
        reset_counts();
        double period = 48000.0 / (1.0 + cases[c] * 1e-6);
        for (int i = 0; i < 4000; i++) {
            now48 = (uint32_t)(uint64_t)(i * period);
            now_us = now48 / 48u;
            alink_usb_tb_sof(&alink_usb_tb, (uint32_t)i & 0x7FF, now48, 0);
            alink_usb_service(&us, now_us / 1000u);
        }
        bool rej = fabs(cases[c]) > ALINK_USB_HOST_MAX_PPM;
        CHECK(us.rejected == rej && (us.state == ALINK_USB_ST_REJECTED) == rej && alink_usb_nominal_feedback(&us) == !rej,
              "host %+.0f ppm: rejected %d state %u", cases[c], us.rejected, us.state);
    }
}

static void test_apply_and_status(void) {
    alink_usb_t us;
    alink_usb_init(&us);
    us.mode = 2;
    us.d = 192;
    alink_usb_limits(&us, 48000, 96, 49, 448);
    memset(&alink_usb_tb, 0, sizeof(alink_usb_tb));
    // A time base at bus frame 1000 (extended 5096: 2 wraps + 1000) and a running timeline.
    alink_usb_tb.valid = alink_usb_tb.have_c = true;
    alink_usb_tb.n = alink_usb_tb.n_c = 2u * 2048u + 1000u;
    alink_usb_tb.t = alink_usb_tb.t_c = 0;
    now48 = 0;
    us.tb_ok = true;
    us.started = true;
    us.ph.timeline = true;
    us.ph.state = LINK_PHASE_LOCKED;
    us.ph.next_w = 48ull * alink_usb_tb.n;
    // Bus frame 1100, sample 5: 100 frames ahead.
    uint8_t p[16] = { 0 };
    uint32_t f = 1100u | 5u << 16;
    memcpy(p, &f, 4);
    p[8] = LINK_OP_MASTER_VOLUME;
    p[10] = 0x30; p[11] = 0xf8;                          // -2000 = -20 dB
    p[12] = 2;
    CHECK(alink_usb_apply(&us, p), "bus-frame op accepted");
    CHECK(us.aq.n == 1 && us.aq.q[0].frame == 48ull * (2u * 2048u + 1100u) + 5u, "mapped to S %llu",
          (unsigned long long)us.aq.q[0].frame);
    CHECK(alink_usb_bus_form(&us, us.aq.q[0].frame) == f, "bus form round trip");
    // Bus frame 990: 10 frames ago, inside the 64-frame window: past, kept (runs late).
    f = 990u;
    memcpy(p, &f, 4);
    p[8] = LINK_OP_MUTE; p[10] = p[11] = 0;
    CHECK(alink_usb_apply(&us, p) && us.aq.q[0].frame == 48ull * (2u * 2048u + 990u), "a past bus frame maps behind");
    // Sample offset beyond the frame, unknown time base, reserved bytes: refused.
    f = 1200u | 48u << 16;
    memcpy(p, &f, 4);
    CHECK(!alink_usb_apply(&us, p), "k >= spf refused");
    f = 1200u | 1u << 12;
    memcpy(p, &f, 4);
    CHECK(!alink_usb_apply(&us, p), "bits 11-15 refused");
    f = 1200u;
    memcpy(p, &f, 4);
    p[12] = 3;
    CHECK(!alink_usb_apply(&us, p), "time base 3 refused");
    p[12] = 1;
    p[13] = 1;
    CHECK(!alink_usb_apply(&us, p), "reserved byte refused");
    p[13] = 0;
    p[8] = LINK_OP_RATE;
    CHECK(!alink_usb_apply(&us, p), "RATE refused");
    // Time base 1: S directly.
    uint64_t s = 48ull * (2u * 2048u + 1050u);
    memcpy(p, &s, 8);
    p[8] = LINK_OP_CLICK; p[9] = 2;
    CHECK(alink_usb_apply(&us, p) && us.aq.n == 3, "S op accepted");
    p[8] = LINK_OP_MUTE; p[9] = 0;
    CHECK(alink_usb_apply(&us, p) && us.aq.n == 4, "4th op");
    CHECK(!alink_usb_apply(&us, p), "a full queue refuses");
    reset_counts();
    uint8_t c[16] = { 0 };
    c[12] = 1;
    CHECK(alink_usb_apply(&us, c) && us.aq.n == 0 && n_results == 4 && last_result == LINK_APPLY_CANCELLED,
          "CANCEL_ALL reports each op cancelled");
    // Not phase-locked: refused.
    us.ph.state = LINK_PHASE_ANCHORING;
    memcpy(p, &s, 8);
    CHECK(!alink_usb_apply(&us, p), "anchoring: refused");

    // Status layout.
    us.ph.state = LINK_PHASE_LOCKED;
    us.state = ALINK_USB_ST_PHASE_LOCK;
    us.est.valid = true;
    us.est.ppm = 12.5f;
    us.stream = us.stream_seen = true;
    us.stream_first = 2u * 2048u + 900u;
    us.stream_shift = -3;
    us.stream_samples = 4800;
    us.anchors = 1;
    uint8_t o[ALINK_USB_STATUS_LEN];
    alink_usb_get_status(&us, o);
    float hp;
    memcpy(&hp, o + 4, 4);
    uint64_t snow;
    memcpy(&snow, o + 16, 8);
    uint16_t bus, first;
    memcpy(&bus, o + 24, 2);
    memcpy(&first, o + 32, 2);
    uint32_t age;
    int32_t shift;
    memcpy(&age, o + 36, 4);
    memcpy(&shift, o + 40, 4);
    CHECK(o[0] == ALINK_USB_ST_PHASE_LOCK && o[1] == LINK_PHASE_LOCKED && o[2] == 1 && (o[3] & 0x0A) == 0x0A,
          "status head %u %u %u %02x", o[0], o[1], o[2], o[3]);
    CHECK(hp == 12.5f && snow == 48ull * (2u * 2048u + 1000u) && bus == 1000, "status S %llu bus %u",
          (unsigned long long)snow, bus);
    CHECK(first == 900 && age == 100 && shift == -3, "stream first %u age %u shift %d", first, age, shift);
    uint16_t dmin, dmax;
    memcpy(&dmin, o + 26, 2);
    memcpy(&dmax, o + 28, 2);
    CHECK(dmin == 153 && dmax == 399, "status limits");
}

int main(void) {
    test_time_base();
    test_estimator();
    test_config();
    test_mode0_stock();
    test_reject();
    test_apply_and_status();
    printf("test_alink_usb: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
