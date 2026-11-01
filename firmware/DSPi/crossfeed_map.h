#ifndef CROSSFEED_MAP_H
#define CROSSFEED_MAP_H

// Crossfeed output pair map: which two outputs each crossfeed pair processes.
//
// Pure, dependency-free helpers (stdint/stdbool only) so they can be compiled
// and unit-tested on the host (firmware/tests/test_crossfeed_map.c).
//
// ENCODING (one byte per pair, CROSSFEED_MAP_PAIRS bytes, little-endian packed
// into a uint32_t in RAM and on the wire / in flash):
//
//   byte p == 0x00          pair p uses its default outputs (2p, 2p+1)
//   byte p == (R << 4) | L  pair p processes left output L (low nibble) and
//                           right output R (high nibble), both 0-based output
//                           indices
//
// 0x00 can never be an explicit pair (L == R == 0 is invalid), so it is free
// to mean "default".  An all-zero map is the historical fixed pairing, which
// makes the field backwards compatible: the bytes it occupies (the former
// WireCrossfeedParams.reserved2) were always written as zero.  Firmware stores
// and reports the map normalized (an explicit default byte becomes 0x00), so a
// default configuration is byte-identical to firmware without the feature.
//
// VALIDITY (crossfeed_map_valid): for every pair p < num_pairs the decoded L
// and R are both < 2 * num_pairs (stereo S/PDIF/I2S outputs only; the mono PDM
// sub, index 2 * num_pairs, is never part of a pair), L != R, and no output is
// used by more than one pair, i.e. the map is a permutation of the stereo
// outputs into pairs.  Bytes for pairs p >= num_pairs must be 0x00.  Requiring
// disjoint pairs for the WHOLE map (not only the pairs enabled in the
// output_pair_mask) keeps the pair mask independent: any mask is valid with
// any valid map, and no output can ever be written by two pairs (or two cores)
// in one packet.
//
// The filter itself is symmetric (one coefficient set for both sides), so
// which output is called "left" only matters for presentation.
//
// Example, K340 (one DAC per ear; out0 = R woofer, out1 = R tweeter,
// out2 = L woofer, out3 = L tweeter): pair 0 = woofers (L=2, R=0) -> 0x02,
// pair 1 = tweeters (L=3, R=1) -> 0x13; map = 0x00001302 (bytes 02 13 00 00).

#include <stdint.h>
#include <stdbool.h>

#define CROSSFEED_MAP_PAIRS   4   // wire/flash width; NUM_SPDIF_INSTANCES <= this

static inline uint8_t crossfeed_map_byte(uint32_t map, int p) {
    return (uint8_t)(map >> (8 * p));
}

// Decode pair p's outputs (applies the 0x00 = default rule).
static inline void crossfeed_map_pair(uint32_t map, int p, uint8_t *l, uint8_t *r) {
    uint8_t b = crossfeed_map_byte(map, p);
    if (b == 0) {
        *l = (uint8_t)(2 * p);
        *r = (uint8_t)(2 * p + 1);
    } else {
        *l = b & 0x0F;
        *r = b >> 4;
    }
}

// Encode one explicit pair (no normalization).
static inline uint8_t crossfeed_map_encode_pair(uint8_t l, uint8_t r) {
    return (uint8_t)(((r & 0x0F) << 4) | (l & 0x0F));
}

static inline bool crossfeed_map_valid(uint32_t map, int num_pairs) {
    if (num_pairs < 1 || num_pairs > CROSSFEED_MAP_PAIRS) return false;
    uint32_t used = 0;
    for (int p = 0; p < CROSSFEED_MAP_PAIRS; p++) {
        if (p >= num_pairs) {
            if (crossfeed_map_byte(map, p) != 0) return false;
            continue;
        }
        uint8_t l, r;
        crossfeed_map_pair(map, p, &l, &r);
        if (l >= 2 * num_pairs || r >= 2 * num_pairs || l == r) return false;
        uint32_t bits = (1u << l) | (1u << r);
        if (used & bits) return false;
        used |= bits;
    }
    return true;
}

// Rewrite explicit default bytes (L = 2p, R = 2p+1) as 0x00.  Apply to a map
// that passed crossfeed_map_valid().
static inline uint32_t crossfeed_map_normalize(uint32_t map) {
    uint32_t out = 0;
    for (int p = 0; p < CROSSFEED_MAP_PAIRS; p++) {
        uint8_t b = crossfeed_map_byte(map, p);
        if (b == crossfeed_map_encode_pair((uint8_t)(2 * p), (uint8_t)(2 * p + 1))) b = 0;
        out |= (uint32_t)b << (8 * p);
    }
    return out;
}

// Which core runs a pair in the dual-core (Core 1 EQ worker) pipeline.
// Core 1 owns outputs [core1_first, core1_last], Core 0 the outputs below.
//   CORE0: both outputs on Core 0 -> processed by Core 0 after the dispatch
//   CORE1: both outputs on Core 1 -> processed inside eq_worker_loop
//   SPLIT: one output on each core -> processed by Core 0 BEFORE it hands the
//          packet to Core 1, so both outputs are final before either core's
//          per-output chain (EQ/crossover/gain) reads them.
// The default pairs (2p, 2p+1) never split (core1_first is even), so the
// default configuration keeps its original per-core work split.
typedef enum {
    XF_OWNER_ALL   = 0,   // single-core path: every pair (selector only)
    XF_OWNER_CORE0 = 1,
    XF_OWNER_CORE1 = 2,
    XF_OWNER_SPLIT = 3,
} CrossfeedPairOwner;

static inline CrossfeedPairOwner crossfeed_pair_owner(uint8_t l, uint8_t r,
                                                      int core1_first, int core1_last) {
    bool l1 = (l >= core1_first && l <= core1_last);
    bool r1 = (r >= core1_first && r <= core1_last);
    if (l1 && r1) return XF_OWNER_CORE1;
    if (!l1 && !r1) return XF_OWNER_CORE0;
    return XF_OWNER_SPLIT;
}

#endif // CROSSFEED_MAP_H
