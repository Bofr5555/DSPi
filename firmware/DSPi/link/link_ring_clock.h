/*
 * link_ring_clock.h - the link receiver's view of the RX ring in time
 * (WP LK5, fixups LK45F)
 *
 *  - The stall rule: how long the decode service may go uncalled before the
 *    ring position count (DMA write index plus counted wraps) can no longer
 *    be trusted.
 *  - The wire-rate measurement that feeds the clock servo: frames per second
 *    from (time, bits written) pairs.  Each pair must be taken together, by
 *    whoever read the DMA position (core 0's poll or core 1's service): a
 *    position paired with a later poll time skews every window by the delay
 *    between the two (review lk45 finding 1).
 *
 * Portable C11, header only, no SDK; link_input.c and the host tests include it.
 */

#ifndef LINK_RING_CLOCK_H
#define LINK_RING_CLOCK_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Stall rule
// ---------------------------------------------------------------------------

// The position count stays exact while service passes are less than one ring
// pass apart (a wrap shows as a smaller write index).  15/16 of a pass leaves
// 6 % for the line running fast of its nominal baud and for timer jitter, and
// stays inside the decoder's own overrun limit (the ring less
// LINK_FAST_GUARD_BITS, 0.8 % of the 32 KB ring).  Longer gaps restart the
// decoder: the bits are overwritten by then anyway.
static inline uint32_t link_ring_stall_max_us(uint32_t pass_us) {
    return pass_us - pass_us / 16u;
}

static inline bool link_ring_gap_ok(uint64_t gap_us, uint32_t pass_us) {
    return gap_us <= link_ring_stall_max_us(pass_us);
}

// Absolute words written from the DMA's write index idx (0..ring_words-1),
// counting a wrap whenever the index went down since the last call.  Exact
// while calls are less than one ring pass apart (link_ring_gap_ok()).
static inline uint64_t link_ring_count(uint32_t idx, uint32_t ring_words, uint32_t *last_idx, uint64_t *passes) {
    if (idx < *last_idx) (*passes)++;
    *last_idx = idx;
    return *passes * ring_words + idx;
}

// ---------------------------------------------------------------------------
// Wire-rate measurement
// ---------------------------------------------------------------------------

#define LINK_MEAS_WINDOW_US     32000u      // fast window
#define LINK_MEAS_LONG_HALF_US  8000000ull  // long window: anchors rotate every 8 s, span 8-16 s

typedef struct {
    uint64_t win_us, win_bits;      // fast window start; win_us 0: no sample yet
    float    hz_smooth;             // fast windows, smoothed; 0 until the first one
    uint64_t long_us[2], long_bits[2];
    bool     long_valid;
    float    hz_long;
} link_meas_t;

static inline void link_meas_reset(link_meas_t *m) {
    memset(m, 0, sizeof(*m));
}

// One (us, bits) pair: bits the RX DMA had written at time us.  k converts
// line bits per microsecond to frames per second (1e6 * Fs / baud).  A pair
// that is not newer than the last one (nobody published since) is ignored.
static inline void link_meas_sample(link_meas_t *m, uint64_t us, uint64_t bits, float k) {
    if (!us) return;
    if (!m->win_us) {
        m->win_us = us;
        m->win_bits = bits;
        m->long_us[0] = m->long_us[1] = us;
        m->long_bits[0] = m->long_bits[1] = bits;
        m->hz_smooth = 0.0f;
        return;
    }
    if ((int64_t)(us - m->win_us) < (int64_t)LINK_MEAS_WINDOW_US) return;
    uint64_t span = us - m->win_us;
    float hz = (float)(bits - m->win_bits) * k / (float)span;
    m->win_us = us;
    m->win_bits = bits;
    m->hz_smooth = m->hz_smooth == 0.0f ? hz : m->hz_smooth + 0.25f * (hz - m->hz_smooth);
    if (us - m->long_us[1] >= LINK_MEAS_LONG_HALF_US) {
        m->long_us[0] = m->long_us[1];
        m->long_bits[0] = m->long_bits[1];
        m->long_us[1] = us;
        m->long_bits[1] = bits;
    }
    uint64_t long_span = us - m->long_us[0];
    if (long_span >= LINK_MEAS_LONG_HALF_US) {
        m->hz_long = (float)(bits - m->long_bits[0]) * k / (float)long_span;
        m->long_valid = true;
    }
}

// The servo's input: the long window once it spans 8 s, the fast one before;
// 0 while nothing is measured.
static inline float link_meas_hz(const link_meas_t *m) {
    return m->long_valid ? m->hz_long : m->hz_smooth;
}

#endif // LINK_RING_CLOCK_H
