// alink_spdif_sim.c - an S/PDIF source and two followers on time base 3 (WP FWSP2)
//
// The world is firmware/tests/alink_spdif_world.h: the real roles
// (alink_spdif.c), phase lock (alink/alink_phase.c) and carriage codec behind
// a model of the source's crystal, the wire (each subframe word lands when its
// last bit has arrived, plus white jitter: the receiver's quantisation is one
// word, half a frame), the receive FIFO with the live reader, each follower's
// crystal, soft VCXO and rate feed-forward error, main-loop pass jitter and
// slot 0's output ring.  The audio carries F, so each output shows the F it
// plays.
//
// Checks (PLAN-AT FWSP2): the skew between the two followers under 0.1 frame
// at 48 and 96 kHz from 10 s after the start; the source-to-follower latency
// identical (within 0.1 frame) after a follower relock (a 200 ms carrier cut,
// ridden through in holdover), a source epoch change, and a re-anchor (a D
// write and back); and the latency equal to D plus the constant quarter frame
// of the word quantisation (within half a frame).
//
// The 10 s figure holds at this seed and these feed-forward errors only
// (review fwbc N1: about 0.2 frame per ppm of feed-forward residual the
// integrator has not removed yet).  The robust criterion is the settled skew:
// settled() runs several seeds with +-0.5 ppm feed-forward errors (the line's
// long rate window measures 0.13 to 0.81 ppm) and checks the skew 40-45 s
// after the start, with the 10-15 s figure printed for reference.
//
//   tools/alink_spdif_sim/run.sh       (exit 1 on a failure)

#include <inttypes.h>

#include "alink_spdif_world.h"

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

// Observation: skew between the followers and each latency, over a window.
static double skew_max, lat_sum[2], lat_min[2], lat_max[2];
static int n_obs, n_silent;
static void obs_reset(void) {
    skew_max = 0;
    n_obs = n_silent = 0;
    for (int i = 0; i < 2; i++) { lat_sum[i] = 0; lat_min[i] = 1e9; lat_max[i] = -1e9; }
}
static void obs(double t) {
    (void)t;
    double p0 = sw_fol_playing(&SW.fol[0]), p1 = sw_fol_playing(&SW.fol[1]);
    if (p0 < 0 || p1 < 0) { n_silent++; return; }
    double s = fabs(p0 - p1);
    if (s > skew_max) skew_max = s;
    double src = sw_src_playing();
    double l[2] = { src - p0, src - p1 };
    for (int i = 0; i < 2; i++) {
        lat_sum[i] += l[i];
        if (l[i] < lat_min[i]) lat_min[i] = l[i];
        if (l[i] > lat_max[i]) lat_max[i] = l[i];
    }
    n_obs++;
}

// Run [t0, t1] observing every 0.37 ms (incommensurate with the frame).
static void window(double t1) {
    obs_reset();
    sw_run(t1, 0.00037, obs);
}

static void lat(double out[2]) { for (int i = 0; i < 2; i++) out[i] = n_obs ? lat_sum[i] / n_obs : NAN; }

static void scenario(uint32_t fs, uint16_t d) {
    printf("== %u Hz, D %u: source +3 ppm, followers +6 / -4 ppm, feed-forward errors +0.3 / -0.2 ppm\n", fs, d);
    sw_world_init(fs, 3.0, 0xC0FFEEull + fs);
    SW.word_jit_us = 0.05;
    SW.nfol = 2;
    sw_fol_init(&SW.fol[0], 6.0, d, 650.0);
    sw_fol_init(&SW.fol[1], -4.0, d, 900.0);
    SW.fol[0].ff_err = 0.3;
    SW.fol[1].ff_err = -0.2;
    sw_run(10.0, 0, NULL);
    for (int i = 0; i < 2; i++)
        CHECK(alink_spdif_state(&SW.fol[i].u) == TB_ST_LOCKED, "follower %d LOCKED at 10 s (state %u)", i,
              alink_spdif_state(&SW.fol[i].u));
    window(15.0);
    double l0[2];
    lat(l0);
    printf("  10-15 s: skew max %.4f frame, latency %.4f / %.4f (spread %.3f / %.3f), %d samples\n", skew_max, l0[0],
           l0[1], lat_max[0] - lat_min[0], lat_max[1] - lat_min[1], n_obs);
    CHECK(n_obs > 1000 && n_silent == 0, "observed (%d, silent %d)", n_obs, n_silent);
    CHECK(skew_max < 0.1, "skew between the followers under 0.1 frame (%.4f)", skew_max);
    for (int i = 0; i < 2; i++)
        CHECK(fabs(l0[i] - (double)d - 0.25) < 0.5, "follower %d plays F at F + D (latency %.3f)", i, l0[i]);

    // 1. A follower relock: a 200 ms carrier cut on follower 0, ridden in holdover.
    uint32_t ra = SW.fol[0].u.reanchors;
    SW.fol[0].wire = false;
    sw_run(SW.t + 0.2, 0, NULL);
    bool held = SW.fol[0].u.ph.hold;
    SW.fol[0].wire = true;
    sw_run(SW.t + 6.0, 0, NULL);
    window(SW.t + 3.0);
    double l1[2];
    lat(l1);
    printf("  after a 200 ms cut (holdover %d, re-anchors %u): skew %.4f, latency %.4f / %.4f\n", held,
           SW.fol[0].u.reanchors - ra, skew_max, l1[0], l1[1]);
    CHECK(held && SW.fol[0].u.reanchors == ra, "the cut was a holdover, no re-anchor");
    CHECK(fabs(l1[0] - l0[0]) < 0.1 && fabs(l1[1] - l0[1]) < 0.1, "latency identical after the relock");
    CHECK(skew_max < 0.1, "skew after the relock (%.4f)", skew_max);

    // 2. A source epoch change (its outputs restart): both re-anchor (EPOCH).
    uint32_t r0 = SW.fol[0].u.reanchors, r1 = SW.fol[1].u.reanchors;
    sw_src_restart(SW.t, 0.008);
    sw_run(SW.t + 8.0, 0, NULL);
    window(SW.t + 3.0);
    double l2[2];
    lat(l2);
    printf("  after an epoch change (re-anchors %u / %u, epoch %u): skew %.4f, latency %.4f / %.4f\n",
           SW.fol[0].u.reanchors - r0, SW.fol[1].u.reanchors - r1, SW.fol[0].u.src_epoch, skew_max, l2[0], l2[1]);
    CHECK(SW.fol[0].u.reanchors > r0 && SW.fol[1].u.reanchors > r1, "both followers re-anchored");
    CHECK(SW.fol[0].u.src_epoch == SW.src.u.epoch && SW.fol[1].u.src_epoch == SW.src.u.epoch, "on the new epoch");
    CHECK(fabs(l2[0] - l0[0]) < 0.1 && fabs(l2[1] - l0[1]) < 0.1, "latency identical after the epoch change");
    CHECK(skew_max < 0.1, "skew after the epoch change (%.4f)", skew_max);

    // 3. A re-anchor: follower 1's D is written to D + 7 and back.
    uint8_t c[8] = { 1, 1, (uint8_t)(d + 7), (uint8_t)((d + 7) >> 8), 0, 0, 0, 0 };
    SWF = &SW.fol[1];
    CHECK(alink_spdif_set_config(&SW.fol[1].u, c, 0) == TB_OK, "D + 7 written");
    sw_run(SW.t + 4.0, 0, NULL);
    c[2] = (uint8_t)d;
    c[3] = (uint8_t)(d >> 8);
    SWF = &SW.fol[1];
    CHECK(alink_spdif_set_config(&SW.fol[1].u, c, 0) == TB_OK, "D written back");
    sw_run(SW.t + 6.0, 0, NULL);
    window(SW.t + 3.0);
    double l3[2];
    lat(l3);
    printf("  after a re-anchor (follower 1 re-anchors %u): skew %.4f, latency %.4f / %.4f\n",
           SW.fol[1].u.reanchors - r1, skew_max, l3[0], l3[1]);
    CHECK(fabs(l3[0] - l0[0]) < 0.1 && fabs(l3[1] - l0[1]) < 0.1, "latency identical after the re-anchor");
    CHECK(skew_max < 0.1, "skew after the re-anchor (%.4f)", skew_max);
    for (int i = 0; i < 2; i++)
        CHECK(SW.fol[i].underruns == 0 && SW.fol[i].overflows == 0, "follower %d: no underrun or overflow (%u %u)", i,
              SW.fol[i].underruns, SW.fol[i].overflows);
    printf("  sf_errors %u / %u, phase error %.3f / %.3f\n", SW.fol[0].u.sf_errors, SW.fol[1].u.sf_errors,
           SW.fol[0].u.ph.e, SW.fol[1].u.ph.e);
}

// review fwbc N1: the settled skew over several seeds, feed-forward +-0.5 ppm.
static void settled(uint32_t fs, uint16_t d) {
    static const uint64_t seeds[] = { 0xC0FFEEull, 0xBADC0DEull, 0x1234567ull, 0x5EEDull };
    double worst_early = 0, worst = 0;
    for (unsigned k = 0; k < sizeof seeds / sizeof seeds[0]; k++) {
        sw_world_init(fs, 3.0, seeds[k] + fs);
        SW.word_jit_us = 0.05;
        SW.nfol = 2;
        sw_fol_init(&SW.fol[0], 6.0, d, 650.0);
        sw_fol_init(&SW.fol[1], -4.0, d, 900.0);
        SW.fol[0].ff_err = 0.5;
        SW.fol[1].ff_err = -0.5;
        sw_run(10.0, 0, NULL);
        window(15.0);
        double early = skew_max;
        sw_run(40.0, 0, NULL);
        window(45.0);
        printf("  seed %#" PRIx64 ": skew max %.4f frame at 10-15 s, %.4f at 40-45 s (%d samples, silent %d)\n",
               seeds[k], early, skew_max, n_obs, n_silent);
        CHECK(n_obs > 1000 && n_silent == 0, "settled seed %#" PRIx64 ": observed (%d, silent %d)", seeds[k], n_obs,
              n_silent);
        CHECK(skew_max < 0.1, "settled seed %#" PRIx64 ": skew 40-45 s under 0.1 frame (%.4f)", seeds[k], skew_max);
        if (early > worst_early) worst_early = early;
        if (skew_max > worst) worst = skew_max;
    }
    printf("== %u Hz settled: worst skew %.4f frame at 40-45 s (%.4f at 10-15 s, not checked)\n", fs, worst,
           worst_early);
}

int main(void) {
    scenario(48000, 192);
    scenario(96000, 320);
    printf("== settled skew (review fwbc N1): seeds x feed-forward errors +0.5 / -0.5 ppm\n");
    settled(48000, 192);
    settled(96000, 320);
    printf("alink_spdif_sim: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
