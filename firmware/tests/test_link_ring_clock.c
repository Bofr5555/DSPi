// Host test for link_ring_clock.h (WP LK45F, review lk45 findings 1 and 5):
//  - the wire-rate measurement fed from (time, position) pairs published by
//    a decode service that runs at its own moments (core 1 between EQ
//    blocks, 1-3 ms apart) while core 0 polls at other moments: with the
//    published pairs every window is right; with the poll's own time (the
//    old pairing) the fast windows are off by percent, which the servo
//    rejects;
//  - the stall rule and the ring wrap count: service gaps up to the rule's
//    limit keep the absolute position exact at every class with the line up
//    to 1000 ppm fast, and the limit is at least 4 ms at L3 (the review saw
//    2.1 ms).
// Built by run.sh: cc -std=c11 -Wall -Wextra -Werror -I ../DSPi test_link_ring_clock.c
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "link/link_ring_clock.h"

static int failures;
#define CHECK(c, ...)                                                                                                \
    do {                                                                                                             \
        if (!(c)) {                                                                                                  \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);                                                     \
            fprintf(stderr, __VA_ARGS__);                                                                            \
            fprintf(stderr, "\n");                                                                                   \
            failures++;                                                                                              \
        }                                                                                                            \
    } while (0)

static uint64_t xs = 0x2545F4914F6CDD1Dull;
static uint64_t rnd(void) { xs ^= xs << 13; xs ^= xs >> 7; xs ^= xs << 17; return xs; }
static uint32_t rnd_range(uint32_t lo, uint32_t hi) { return lo + (uint32_t)(rnd() % (hi - lo + 1u)); }

#define RING_WORDS 8192u                // link_input.c: INPUT_ARENA_LINK_WORDS
#define RING_BITS  (RING_WORDS * 32u)
#define GUARD_BITS (64u * 32u)          // link_fast.h: LINK_FAST_GUARD_BITS

static const uint32_t baud[4] = { 0, 15360000u, 30720000u, 61440000u };

// Bits the DMA has written by time us on a line running ppm fast of baud
// (whole words, as the DMA's write address shows them).
static uint64_t wire_bits(unsigned cls, double ppm, uint64_t us) {
    double bits = (double)baud[cls] * (1.0 + ppm * 1e-6) * (double)us / 1e6;
    return ((uint64_t)bits / 32u) * 32u;
}

// One 20 s run: the service publishes every svc_lo..svc_hi us, core 0 polls
// every 200..1000 us and feeds the measurement the last published pair
// (good) and the same position with its own time (old).
static void run_meas(unsigned cls, uint32_t fs, double ppm, uint32_t svc_lo, uint32_t svc_hi, double *good_fast,
                     double *good_long, double *old_fast) {
    link_meas_t good, old;
    link_meas_reset(&good);
    link_meas_reset(&old);
    float k = 1e6f * (float)fs / (float)baud[cls];
    double truth = fs * (1.0 + ppm * 1e-6);
    uint64_t t0 = 1000000u, t = t0, next_svc = t0, next_poll = t0 + 137u;
    uint64_t pub_us = 0, pub_bits = 0;
    *good_fast = *good_long = *old_fast = 0.0;
    uint64_t long_from = 0;
    while (t < t0 + 20000000u) {
        if (next_svc <= next_poll) {
            t = next_svc;
            pub_us = t;
            pub_bits = wire_bits(cls, ppm, t);
            next_svc += rnd_range(svc_lo, svc_hi);
        } else {
            t = next_poll;
            next_poll += rnd_range(200u, 1000u);
            if (!pub_us) continue;
            uint64_t old_win = good.win_us;
            link_meas_sample(&good, pub_us, pub_bits, k);
            if (good.long_valid && !long_from) long_from = t;
            link_meas_sample(&old, t, pub_bits, k);
            // After a second of settling: worst error of the smoothed fast
            // window and of the long window, in ppm.
            if (t - t0 > 1000000u && good.win_us != old_win) {
                double e = fabs(good.hz_smooth / truth - 1.0) * 1e6;
                if (e > *good_fast) *good_fast = e;
                if (good.long_valid) {
                    e = fabs(good.hz_long / truth - 1.0) * 1e6;
                    if (e > *good_long) *good_long = e;
                }
            }
            if (t - t0 > 1000000u && old.hz_smooth != 0.0f) {
                double e = fabs(old.hz_smooth / truth - 1.0) * 1e6;
                if (e > *old_fast) *old_fast = e;
            }
        }
    }
    CHECK(good.long_valid, "L%u: long window valid after 20 s", cls);
    // -- WP LK6 -- valid from a 2 s span (link_ring_clock.h LINK_MEAS_LONG_MIN_US).
    CHECK(long_from > 0 && long_from - t0 <= LINK_MEAS_LONG_MIN_US + LINK_MEAS_WINDOW_US + 5000u,
          "L%u: long window valid after %.3f s", cls, (double)(long_from - t0) * 1e-6);
    CHECK(link_meas_hz(&good) == good.hz_long, "L%u: the servo takes the long window", cls);
}

static void test_meas(void) {
    static const struct { unsigned cls; uint32_t fs; double ppm; } cases[] = {
        { 1, 48000u, 0.0 }, { 1, 44100u, -180.0 }, { 2, 96000u, 250.0 }, { 3, 48000u, 120.0 }, { 3, 96000u, -400.0 },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        double gf, gl, of;
        // Core 1 (between EQ blocks, 1-3 ms apart).
        run_meas(cases[i].cls, cases[i].fs, cases[i].ppm, 1000u, 3000u, &gf, &gl, &of);
        printf("  L%u %u Hz %+.0f ppm, core 1: fast %.1f ppm, long %.2f ppm; poll-time pairing: fast %.0f ppm\n",
               cases[i].cls, cases[i].fs, cases[i].ppm, gf, gl, of);
        CHECK(gf < 100.0, "core 1: fast window off by %.1f ppm", gf);
        CHECK(gl < 2.0, "core 1: long window off by %.2f ppm", gl);
        CHECK(of > 2000.0, "the old pairing should exceed the servo's 2000 ppm limit (%.0f)", of);
        // Core 0 (the service runs inside each poll): the same accuracy.
        run_meas(cases[i].cls, cases[i].fs, cases[i].ppm, 200u, 1000u, &gf, &gl, &of);
        CHECK(gf < 100.0 && gl < 2.0, "core 0: fast %.1f long %.2f ppm", gf, gl);
    }

    // A pair that is not newer than the window start changes nothing.
    link_meas_t m;
    link_meas_reset(&m);
    link_meas_sample(&m, 5000u, 1000u, 1.0f);
    link_meas_sample(&m, 5000u + LINK_MEAS_WINDOW_US - 1u, 9999u, 1.0f);
    CHECK(m.win_us == 5000u && m.hz_smooth == 0.0f && link_meas_hz(&m) == 0.0f, "short window ignored");
    link_meas_sample(&m, 0u, 1u, 1.0f);
    CHECK(m.win_us == 5000u, "no time: ignored");
    printf("  rate measurement from published pairs: OK\n");
}

static void test_stall(void) {
    for (unsigned cls = 1; cls <= 3; cls++) {
        uint32_t pass_us = (uint32_t)((uint64_t)RING_BITS * 1000000u / baud[cls]);     // link_input.c load_program()
        uint32_t max_us = link_ring_stall_max_us(pass_us);
        printf("  L%u: ring pass %u us, rides through service gaps up to %u us\n", cls, pass_us, max_us);
        CHECK(max_us < pass_us && link_ring_gap_ok(max_us, pass_us) && !link_ring_gap_ok(max_us + 1u, pass_us),
              "L%u: rule", cls);
        // Line 1000 ppm fast: a gap at the limit stays inside one pass and
        // inside the decoder's overrun limit (guard, comma window, a frame).
        uint64_t adv = wire_bits(cls, 1000.0, max_us) + 32u;
        CHECK(adv + GUARD_BITS + 12u + 400u < RING_BITS, "L%u: %" PRIu64 " bits in the longest gap", cls, adv);

        // Random service gaps up to the limit, the line -1000..+1000 ppm:
        // the counted position follows the writer exactly.
        for (int ppm = -1000; ppm <= 1000; ppm += 500) {
            uint32_t last_idx = 0;
            uint64_t passes = 0, t = 0, bad = 0;
            for (int n = 0; n < 200000; n++) {
                t += n % 7 == 0 ? max_us : rnd_range(1u, max_us);
                uint64_t w = wire_bits(cls, ppm, t) / 32u;
                uint64_t got = link_ring_count((uint32_t)(w % RING_WORDS), RING_WORDS, &last_idx, &passes);
                if (got != w) bad++;
            }
            CHECK(bad == 0, "L%u %+d ppm: %" PRIu64 " miscounted positions", cls, ppm, bad);
        }
    }
    CHECK(link_ring_stall_max_us((uint32_t)((uint64_t)RING_BITS * 1000000u / baud[3])) >= 4000u,
          "L3 rides through at least 4 ms");
    printf("  stall rule and wrap count: OK\n");
}

int main(void) {
    test_meas();
    test_stall();
    if (failures) {
        fprintf(stderr, "test_link_ring_clock: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_link_ring_clock: OK\n");
    return 0;
}
