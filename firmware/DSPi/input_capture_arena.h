/*
 * input_capture_arena.h; shared storage for the input capture buffers.
 *
 * The SPDIF RX FIFO, the I2S RX rings and the ADAT RX ring are mutually
 * exclusive (one input source runs at a time), so they overlay one arena.
 * See Documentation/current_architecture.md "Input Capture Arena" for the
 * ownership invariant and the per-platform sizes.
 */

#ifndef INPUT_CAPTURE_ARENA_H
#define INPUT_CAPTURE_ARENA_H

#include <stdint.h>
#include "pico.h"

// Geometry mirrors the owning modules' own ring constants; each module
// static-asserts its constants against these, so a size change there breaks
// the build instead of silently truncating a member.
#if PICO_RP2350
#define INPUT_ARENA_ALIGN        8192u
#define INPUT_ARENA_ADAT_WORDS   2048u
#define INPUT_ARENA_I2S_PAIRS    4u
#define INPUT_ARENA_I2S_WORDS    2048u
#else
#define INPUT_ARENA_ALIGN        4096u
#define INPUT_ARENA_I2S_PAIRS    1u
#define INPUT_ARENA_I2S_WORDS    1024u
#endif
#define INPUT_ARENA_SPDIF_WORDS  3072u
#if PICO_RP2350
// The alink receiver's raw ring (alink_input.c): 32 KB, the whole arena.  The
// arena is only INPUT_ARENA_ALIGN-aligned, so the ring wraps by a second DMA
// channel that reloads the write address, not by the DMA ring-wrap.
#define INPUT_ARENA_ALINK_WORDS  8192u
#endif

// Alignment is the largest DMA write-wrap requirement of any member.  Each
// I2S row is its own wrap region, so row bytes == arena alignment is what
// keeps every row self-aligned; do not shrink a row without re-checking it.
typedef union {
#if PICO_RP2350
    uint32_t adat_ring[INPUT_ARENA_ADAT_WORDS];
#endif
    uint32_t i2s_ring[INPUT_ARENA_I2S_PAIRS][INPUT_ARENA_I2S_WORDS];
    uint32_t spdif_fifo[INPUT_ARENA_SPDIF_WORDS];
#if PICO_RP2350
    uint32_t alink_ring[INPUT_ARENA_ALINK_WORDS];
#endif
} __attribute__((aligned(INPUT_ARENA_ALIGN))) InputCaptureArena;

#if PICO_RP2350
// The alink ring takes no more than the I2S rows already do.
_Static_assert(INPUT_ARENA_ALINK_WORDS <= INPUT_ARENA_I2S_PAIRS * INPUT_ARENA_I2S_WORDS,
               "the alink ring must not grow the arena");
#endif

extern InputCaptureArena input_capture_arena;

typedef enum {
    INPUT_ARENA_FREE = 0,
    INPUT_ARENA_SPDIF,
    INPUT_ARENA_I2S,
    INPUT_ARENA_ADAT,
    INPUT_ARENA_ALINK,
} InputArenaOwner;

// Claim panics unless the arena is free or already held by the same owner:
// two live receivers would write the same bytes.  Release by a non-owner is
// a no-op.  USB claims nothing.
void input_arena_claim(InputArenaOwner owner);
void input_arena_release(InputArenaOwner owner);
InputArenaOwner input_arena_get_owner(void);

#endif // INPUT_CAPTURE_ARENA_H
