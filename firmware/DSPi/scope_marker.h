/*
 * scope_marker.h - WP SCOPE1: a GPIO marker on the shared time base, for
 * measuring the alink timing between units with a logic analyser / MSO.
 *
 * TEST BUILDS ONLY.  Compiled in only with -DALINK_SCOPE_MARKER_GPIO=<pin>
 * (CMake cache variable of the same name); a normal build has none of this
 * and RELEASE builds refuse the option.
 *
 * The marker pin rises when output sample index M, a multiple of
 * ALINK_SCOPE_MARKER_N, is fetched from the output ring by slot 0's DMA, M on
 * the shared time base: the USB sample index S (EXT_ALINK_USB mode 2,
 * extension_channel_spec.md 2.9.1) or the alink frame F (alink receiver,
 * 2.8).  The edge is taken by spinning on the DMA read address, so it is
 * locked to the hardware fetch to well under a microsecond; the fetch leads
 * the sample's first bit on the wire by the PIO TX FIFO depth (I2S: 8 words =
 * 4 frames, S/PDIF: 8 words = 2 frames), the same on every unit with the same
 * output type and rate.  So the time between two units' edges equals the
 * offset of their audio.  Derivation: Documentation/Bench/scope_marker.md.
 *
 * ALINK_SCOPE_MARKER_N must be a power of two up to 32768: the USB time base
 * of two units differs by a multiple of 2048 frames = 2048 * spf samples
 * (2^15 * 3 at 48 kHz), so only a divisor of that marks the same bus instant
 * on every unit.  The default 4096 is 11.7 Hz at 48 kHz, 23.4 Hz at 96 kHz.
 *
 * ALINK_SCOPE_MARKER_LOCK_GPIO (optional): high while the unit is
 * phase-locked on that time base (USB state 5 / alink phase LOCKED, not in
 * holdover), low otherwise: the per-unit sanity signal.
 */

#ifndef SCOPE_MARKER_H
#define SCOPE_MARKER_H

#include <stdbool.h>
#include <stdint.h>

#if defined(ALINK_SCOPE_MARKER_GPIO)

#ifndef ALINK_SCOPE_MARKER_N
#define ALINK_SCOPE_MARKER_N 4096u
#endif

_Static_assert(ALINK_SCOPE_MARKER_N >= 256u && ALINK_SCOPE_MARKER_N <= 32768u &&
               (ALINK_SCOPE_MARKER_N & (ALINK_SCOPE_MARKER_N - 1u)) == 0u,
               "ALINK_SCOPE_MARKER_N must be a power of two in 256..32768 (a divisor of 2048 * spf)");

typedef struct {
    uint32_t edges;         // rising edges placed
    uint32_t skipped;       // targets given up: preempted, passed or timed out in the spin
    uint32_t missed;        // targets the main loop saw too late to arm
    uint32_t armed;         // alarms armed
} scope_marker_stats_t;

extern volatile scope_marker_stats_t scope_marker_stats;

void scope_marker_init(void);
// Main loop, every pass.
void scope_marker_service(void);

#if DSPI_ALINK
// alink_input.c: the receiver's timeline (false: none running).
bool alink_input_scope_timeline(uint64_t *next_w, bool *locked);
#endif

#else

static inline void scope_marker_init(void) {}
static inline void scope_marker_service(void) {}

#endif // ALINK_SCOPE_MARKER_GPIO

#endif // SCOPE_MARKER_H
