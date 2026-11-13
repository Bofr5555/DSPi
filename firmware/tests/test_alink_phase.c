// Host test for link/link_phase.c (WP LK6): EXT_LINK_PHASE validation and
// limits, the anchor / verification / re-anchor decisions, holdover and
// resync placement, the phase servo's closed loop on a modelled plant, and
// the apply-at-frame queue (parse, order, depth, superframe intake, the
// examples of extension_channel_spec.md 2.8.5 and 2.8.7).
// Built by run.sh: cc -std=c11 -Wall -Wextra -Werror -I ../DSPi test_link_phase.c
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "link/link_phase.c"

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

static const link_apply_limits_t LIM = { 10, 9, 2048 };

// 1 ms option at 48 kHz: target 48, block cap 17; the rings' writable span 448.
static void limits_1ms(link_phase_t *ph) { link_phase_limits(ph, 48, 17, 448); }

static void test_phase_config(void) {
    link_phase_t ph;
    link_phase_init(&ph);
    limits_1ms(&ph);
    CHECK(ph.d_min == 81 && ph.d_max == 431, "LK6 limits %u..%u", ph.d_min, ph.d_max);
    CHECK(link_phase_active(&ph), "LK6 default 128 active under the 1 ms option");
    CHECK(link_phase_prefill(&ph, 48, 17, 448) == 95, "LK6 prefill %u", link_phase_prefill(&ph, 48, 17, 448));
    // 4 ms option: 192 + 49 + 16 = 257 > 128.
    link_phase_limits(&ph, 192, 49, 448);
    CHECK(ph.d_min == 257 && ph.d_max == 399, "LK6 limits 4 ms %u..%u", ph.d_min, ph.d_max);
    CHECK(!link_phase_active(&ph), "LK6 128 below the 4 ms minimum is not active");
    uint8_t o[8];
    link_phase_get(&ph, o);
    CHECK(o[6] == LINK_PHASE_FLAG_BELOW_MIN, "LK6 D_BELOW_MIN flag %02x", o[6]);
    CHECK(link_phase_prefill(&ph, 192, 49, 448) == 192, "LK6 inactive prefill is the option's target");
    limits_1ms(&ph);

    // The 2.8.5 example: phase lock, D_local 128 (state is IN only).
    const uint8_t ex[8] = { 0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x02 };
    CHECK(link_phase_set(&ph, ex) == 0, "LK6 rewriting the same values is no change");
    ph.state = LINK_PHASE_LOCKED;
    link_phase_get(&ph, o);
    CHECK(memcmp(o, ex, 8) == 0, "LK6 PHASE IN reads the example");
    uint8_t p[8];
    memcpy(p, ex, 8); p[2] = 0x90;                     // 144
    CHECK(link_phase_set(&ph, p) == 1 && ph.d_local == 144, "LK6 a new delay is a change");
    memcpy(p, ex, 8); p[0] = 2;
    CHECK(link_phase_set(&ph, p) == -1 && ph.d_local == 144, "LK6 mode 2 refused, nothing changed");
    memcpy(p, ex, 8); p[1] = 1;
    CHECK(link_phase_set(&ph, p) == -1, "LK6 reserved byte refused");
    memcpy(p, ex, 8); p[6] = 1;
    CHECK(link_phase_set(&ph, p) == -1, "LK6 OUT flags refused");
    memcpy(p, ex, 8); p[4] = 65;
    CHECK(link_phase_set(&ph, p) == -1, "LK6 hops_offset 65 refused");
    memcpy(p, ex, 8); p[4] = 0xC0; p[5] = 0xFF;        // 128 - 64 = 64 < 81
    CHECK(link_phase_set(&ph, p) == -1 && link_phase_delay(&ph) == 144, "LK6 effective delay checked with hops");
    memcpy(p, ex, 8); p[2] = 0xC0; p[4] = 0xC0; p[5] = 0xFF;   // 192 - 64 = 128
    CHECK(link_phase_set(&ph, p) == 1 && ph.hops_offset == -64 && link_phase_delay(&ph) == 128,
          "LK6 hops_offset -64: effective %d", (int)link_phase_delay(&ph));
    link_phase_init(&ph);
    limits_1ms(&ph);
    memcpy(p, ex, 8); p[2] = 80;
    CHECK(link_phase_set(&ph, p) == -1, "LK6 effective delay 80 below d_local_min refused");
    memcpy(p, ex, 8); p[2] = 0xB0; p[3] = 0x01;        // 432
    CHECK(link_phase_set(&ph, p) == -1, "LK6 effective delay 432 above d_local_max refused");
    memcpy(p, ex, 8); p[2] = 100; p[4] = 0xFC; p[5] = 0xFF;   // 100 - 4 = 96
    CHECK(link_phase_set(&ph, p) == 1 && link_phase_delay(&ph) == 96, "LK6 d_local + hops_offset");
    memcpy(p, ex, 8); p[0] = 0;
    CHECK(link_phase_set(&ph, p) == 1 && !link_phase_active(&ph), "LK6 mode 0 is a change and not active");
}

static void test_check(void) {
    link_phase_t ph;
    link_phase_init(&ph);
    limits_1ms(&ph);
    int32_t k = 0;
    link_phase_drain(&ph);
    CHECK(ph.state == LINK_PHASE_ANCHORING && !ph.timeline, "LK6 drain anchors");
    link_phase_written(&ph, 1000, 95);
    CHECK(ph.first == 1000 && ph.next_w == 1095, "LK6 first/next_w");
    link_phase_written(&ph, 1095, 17);
    CHECK(ph.first == 1000 && ph.next_w == 1112, "LK6 next_w advances");
    // Anchor wait: F_arr - first = 120 of 128.
    CHECK(fabsf(link_phase_anchor_wait(&ph, 120.0f) - 8.0f) < 1e-6f, "LK6 anchor wait");
    link_phase_started(&ph);
    CHECK(link_phase_muted(&ph), "LK6 muted until verified");
    CHECK(link_phase_check(&ph, 0.2f, true, &k) == LINK_PHASE_ACT_RESUME && ph.state == LINK_PHASE_LOCKED,
          "LK6 verified within half a frame");
    CHECK(!link_phase_muted(&ph), "LK6 unmuted after verification");
    CHECK(link_phase_check(&ph, 1.9f, true, &k) == LINK_PHASE_ACT_NONE, "LK6 locked tolerates 1.9");
    CHECK(link_phase_check(&ph, -2.5f, true, &k) == LINK_PHASE_ACT_REANCHOR, "LK6 locked re-anchors at 2.5");
    CHECK(link_phase_check(&ph, -2.5f, false, &k) == LINK_PHASE_ACT_NONE && ph.state == LINK_PHASE_PENDING,
          "LK6 automatic re-anchor off: pending");
    CHECK(link_phase_check(&ph, 0.3f, false, &k) == LINK_PHASE_ACT_NONE && ph.state == LINK_PHASE_LOCKED,
          "LK6 pending to locked when the servo pulled it in");

    // Late start: corrections first, still muted.
    link_phase_drain(&ph);
    link_phase_written(&ph, 5000, 95);
    link_phase_started(&ph);
    CHECK(link_phase_check(&ph, 3.4f, true, &k) == LINK_PHASE_ACT_CORRECT && k == 3, "LK6 correct +3 (%d)", (int)k);
    CHECK(link_phase_check(&ph, -0.6f, true, &k) == LINK_PHASE_ACT_CORRECT && k == -1, "LK6 correct -1 (%d)", (int)k);
    CHECK(link_phase_check(&ph, 0.6f, true, &k) == LINK_PHASE_ACT_REANCHOR, "LK6 corrections exhausted");
    link_phase_started(&ph);
    CHECK(link_phase_check(&ph, 100.0f, true, &k) == LINK_PHASE_ACT_REANCHOR, "LK6 beyond the correction range");
    link_phase_started(&ph);
    ph.corrections = LINK_PHASE_MAX_CORRECTIONS;
    CHECK(link_phase_check(&ph, 0.8f, false, &k) == LINK_PHASE_ACT_RESUME && ph.state == LINK_PHASE_PENDING,
          "LK6 automatic re-anchor off: resume pending");

    // Mode 0: the timeline is kept for apply-at-frame, the lock is free.
    uint8_t p[8] = { 0, 0, 0x80, 0, 0, 0, 0, 0 };
    link_phase_set(&ph, p);
    link_phase_drain(&ph);
    CHECK(ph.state == LINK_PHASE_FREE, "LK6 mode 0 drain is free");
    link_phase_written(&ph, 1, 10);
    link_phase_started(&ph);
    CHECK(!link_phase_loss(&ph, 40), "LK6 mode 0: no holdover");
}

static void test_holdover(void) {
    link_phase_t ph;
    link_phase_init(&ph);
    limits_1ms(&ph);
    CHECK(!link_phase_loss(&ph, 10), "LK6 no timeline, no holdover");
    link_phase_drain(&ph);
    link_phase_written(&ph, 100, 112);
    link_phase_started(&ph);
    int32_t k;
    link_phase_check(&ph, 0.1f, true, &k);
    CHECK(link_phase_loss(&ph, 110) && ph.hold && link_phase_muted(&ph), "LK6 holdover");
    CHECK(link_phase_hold_frames(&ph, 100, 17) == 10, "LK6 hold frames 10");
    CHECK(link_phase_hold_frames(&ph, 80, 17) == 17, "LK6 hold frames capped");
    CHECK(link_phase_hold_frames(&ph, 115, 17) == 0, "LK6 hold frames none above the fill");
    link_phase_written(&ph, ph.next_w, 50);
    CHECK(ph.next_w == 262, "LK6 silence advances the timeline");
    CHECK(link_phase_relock(&ph, 1000, 1010, 448), "LK6 relock near the held F");
    CHECK(!ph.hold && ph.resync && ph.state == LINK_PHASE_ANCHORING, "LK6 relock resyncs");
    CHECK(link_phase_gap(&ph, 250) == -12 && link_phase_gap(&ph, 270) == 8 && link_phase_gap(&ph, 262) == 0,
          "LK6 gap");
    CHECK(link_phase_gap_ok(-448, 448) && link_phase_gap_ok(448, 448) && !link_phase_gap_ok(-449, 448) &&
          !link_phase_gap_ok(449, 448), "LK6 gap within the ring span");
}

// Review lk6 F1: a sender power cycle restarts F at 0.  The relock compares
// the stream's F with the holdover's extrapolation and refuses to place rows
// on the held timeline when they are further apart than the rings hold, in
// either direction; the glue then takes the full reset.
static void test_relock_far(void) {
    const uint64_t held = 172800000ull;             // an hour at 48 kHz
    const struct { uint64_t f; bool ok; } cases[] = {
        { held, true }, { held - 448, true }, { held + 448, true },
        { 0, false }, { 68, false }, { held - 449, false }, { held + 449, false }, { held + 10000000ull, false },
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        link_phase_t ph;
        link_phase_init(&ph);
        limits_1ms(&ph);
        link_phase_drain(&ph);
        link_phase_written(&ph, held - 300, 112);
        link_phase_started(&ph);
        int32_t k;
        link_phase_check(&ph, 0.0f, true, &k);
        CHECK(link_phase_loss(&ph, 100), "LK6 F1 holdover %u", i);
        bool ok = link_phase_relock(&ph, cases[i].f, held, 448);
        CHECK(ok == cases[i].ok, "LK6 F1 relock at F %llu: %d", (unsigned long long)cases[i].f, ok);
        CHECK(!ph.hold, "LK6 F1 holdover ends either way (%u)", i);
        if (!ok) {
            CHECK(!ph.resync && !ph.started, "LK6 F1 refused relock: no placement, no verification (%u)", i);
            CHECK(!link_phase_loss(&ph, 100), "LK6 F1 refused relock: no timeline to hold (%u)", i);
        } else {
            CHECK(ph.resync && ph.started, "LK6 F1 accepted relock places and verifies (%u)", i);
        }
    }
}

// Phase error arithmetic: F_arr from a reference 7 frames and 160 bits past
// it at 320 bits per frame (L1, 48 kHz) = ref + 7.5.
static void test_error(void) {
    link_phase_t ph;
    link_phase_init(&ph);
    limits_1ms(&ph);
    link_phase_drain(&ph);
    link_phase_written(&ph, 1000000000000ull, 100);       // next_w = 10^12 + 100
    float since = link_phase_since(1000000000000ull + 93, ph.next_w, 7 * 320 + 160, 320.0f);
    CHECK(fabsf(since - 0.5f) < 1e-6f, "LK6 since %f", since);
    // 0.5 in flight + 127.75 in the ring - 128 = 0.25.
    float e = link_phase_error(&ph, since, 127.75f);
    CHECK(fabsf(e - 0.25f) < 1e-6f, "LK6 error %f", e);
    CHECK(fabsf(link_phase_since(5, 9, -320, 320.0f) + 5.0f) < 1e-6f, "LK6 negative since");
}

// The servo against a plant: e' = Fs * 1e-6 * (sender ppm - output ppm),
// with the rate feed-forward off by 0.3 ppm, the slew limit and the VCXO's
// 0.11 ppm step.  From 0.45 frame it settles without overshooting half a
// frame; after 60 s the VCXO step's limit cycle stays under 0.05 frame (the
// soak criterion of SYS2 8f), at 44.1, 48 and 96 kHz.
static void test_servo(void) {
    const float rates[3] = { 44100.0f, 48000.0f, 96000.0f };
    for (int r = 0; r < 3; r++) {
        link_phase_t ph;
        link_phase_init(&ph);
        float fs = rates[r], ff = 0.3f, cmd = 0.0f, e = 0.45f, worst = 0.0f, after30 = 0.0f;
        for (int tick = 0; tick < 1600; tick++) {          // 120 s
            float trim = link_phase_servo(&ph, e);
            float want = ff + trim;
            if (want > cmd + 0.5f) want = cmd + 0.5f;
            if (want < cmd - 0.5f) want = cmd - 0.5f;
            cmd = want;
            float q = cmd / 0.11f;
            float vcxo = 0.11f * (float)(int)(q < 0.0f ? q - 0.5f : q + 0.5f);
            e -= fs * 1e-6f * vcxo * 0.075f;              // outputs faster than the sender: latency shrinks
            if (fabsf(e) > worst && tick > 0) worst = fabsf(e);
            if (tick >= 800 && fabsf(e) > after30) after30 = fabsf(e);
        }
        CHECK(worst < 0.5f, "LK6 servo %g Hz overshoot %f", fs, worst);
        CHECK(after30 < 0.05f, "LK6 servo %g Hz residual after 60 s %f", fs, after30);
        printf("  LK6 servo %5.0f Hz: worst %.3f frame, residual after 60 s %.3f frame\n", fs, worst, after30);
    }
}

static void test_apply(void) {
    link_apply_op_t op;
    // 2.8.7 example: MASTER_VOLUME -20 dB at frame 1 002 400.
    const uint8_t ex[16] = { 0xa0, 0x4b, 0x0f, 0, 0, 0, 0, 0, 0x04, 0x00, 0x30, 0xf8, 0, 0, 0, 0 };
    CHECK(link_apply_parse(&LIM, ex, &op) && op.frame == 1002400u && op.op == 4 && op.value == -2000,
          "LK6 apply example parses");
    uint8_t p[16];
    memcpy(p, ex, 16); p[8] = 6; p[10] = 0; p[11] = 0;
    CHECK(!link_apply_parse(&LIM, p, &op), "LK6 RATE refused on the per-unit path");
    memcpy(p, ex, 16); p[8] = 9;
    CHECK(!link_apply_parse(&LIM, p, &op), "LK6 unknown op refused");
    memcpy(p, ex, 16); p[13] = 1;
    CHECK(!link_apply_parse(&LIM, p, &op), "LK6 reserved byte refused");
    memcpy(p, ex, 16); p[10] = 1; p[11] = 0;
    CHECK(!link_apply_parse(&LIM, p, &op), "LK6 volume +0.01 dB refused");
    memcpy(p, ex, 16); p[8] = 3; p[9] = 10; p[10] = 0; p[11] = 0;
    CHECK(!link_apply_parse(&LIM, p, &op), "LK6 preset slot 10 refused");
    memcpy(p, ex, 16); p[8] = 5; p[9] = 2; p[10] = 0x00; p[11] = 0x08;
    CHECK(!link_apply_parse(&LIM, p, &op), "LK6 OUT_DELAY 2048 refused");
    p[10] = 0xFF; p[11] = 0x07;
    CHECK(link_apply_parse(&LIM, p, &op) && op.value == 2047, "LK6 OUT_DELAY 2047 accepted");
    memcpy(p, ex, 16); p[8] = 7; p[9] = 9; p[10] = 0; p[11] = 0;
    CHECK(!link_apply_parse(&LIM, p, &op), "LK6 CLICK output 9 refused");
    memcpy(p, ex, 16); p[8] = 0; p[9] = 0; p[10] = 0; p[11] = 0;
    CHECK(link_apply_parse(&LIM, p, &op) && op.op == LINK_APPLY_CANCEL_ALL, "LK6 CANCEL_ALL parses");
    p[9] = 1;
    CHECK(!link_apply_parse(&LIM, p, &op), "LK6 CANCEL_ALL with an argument refused");
    memcpy(p, ex, 16); p[8] = 8; p[10] = 0; p[11] = 0;
    CHECK(link_apply_parse(&LIM, p, &op), "LK6 SYNC_PULSE parses");

    link_apply_q_t q;
    link_apply_init(&q);
    CHECK(link_apply_next(&q) == UINT64_MAX, "LK6 empty queue");
    const uint64_t fr[5] = { 500, 300, 400, 300, 100 };
    for (int i = 0; i < 4; i++) {
        link_apply_op_t o = { fr[i], (uint8_t)(1 + i), 0, 0 };
        CHECK(link_apply_push(&q, &o), "LK6 push %d", i);
    }
    link_apply_op_t o5 = { fr[4], 1, 0, 0 };
    CHECK(!link_apply_push(&q, &o5), "LK6 a fifth op is refused (depth 4)");
    CHECK(link_apply_next(&q) == 300, "LK6 earliest first");
    const uint8_t want_ops[4] = { 2, 4, 3, 1 };          // 300 (pushed 2nd), 300 (4th), 400, 500
    for (int i = 0; i < 4; i++) {
        link_apply_pop(&q, &op);
        CHECK(op.op == want_ops[i], "LK6 pop %d: op %u", i, op.op);
    }
    CHECK(q.n == 0, "LK6 drained");
    uint8_t in[16];
    link_apply_in(&q, in);
    for (int i = 0; i < 16; i++) CHECK(in[i] == 0, "LK6 IN all zero before the first op");
    link_apply_done(&q, 4, LINK_APPLY_APPLIED, 1002400u);
    link_apply_in(&q, in);
    const uint8_t want_in[16] = { 0, 4, 0, 0, 0xa0, 0x4b, 0x0f, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    CHECK(memcmp(in, want_in, 16) == 0, "LK6 IN reads the 2.8.7 example");

    // The superframe's apply: taken once, RATE and none ignored, refused when full.
    link_sf_t sf;
    memset(&sf, 0, sizeof(sf));
    CHECK(link_apply_take_sf(&q, &LIM, &sf) == 0, "LK6 no apply in the superframe");
    sf.apply_frame = 64000; sf.apply_op = LINK_OP_RATE; sf.apply_arg = 2;
    CHECK(link_apply_take_sf(&q, &LIM, &sf) == 0 && q.n == 0, "LK6 RATE is the decoder's, not the queue's");
    sf.apply_op = LINK_OP_MUTE; sf.apply_arg = 0;
    CHECK(link_apply_take_sf(&q, &LIM, &sf) == 1 && q.n == 1, "LK6 superframe MUTE queued");
    CHECK(link_apply_take_sf(&q, &LIM, &sf) == 0 && q.n == 1, "LK6 the same apply again is a repeat");
    sf.apply_frame = 64032;
    CHECK(link_apply_take_sf(&q, &LIM, &sf) == 1 && q.n == 2, "LK6 a new superframe apply queued");
    sf.apply_frame = 64064; sf.apply_op = LINK_OP_MASTER_VOLUME; sf.apply_value = 5;
    CHECK(link_apply_take_sf(&q, &LIM, &sf) == 2 && q.n == 2, "LK6 bad superframe arguments: refused");
    for (int i = 0; i < 2; i++) {
        link_apply_op_t o = { (uint64_t)(70000 + i), 1, 0, 0 };
        link_apply_push(&q, &o);
    }
    sf.apply_frame = 64096; sf.apply_value = -100;
    CHECK(link_apply_take_sf(&q, &LIM, &sf) == 2 && q.n == 4, "LK6 full queue: superframe apply refused");
}

int main(void) {
    test_phase_config();
    test_check();
    test_holdover();
    test_relock_far();
    test_error();
    test_servo();
    test_apply();
    printf("test_link_phase: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
