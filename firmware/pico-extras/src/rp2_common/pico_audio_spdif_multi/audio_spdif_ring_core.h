/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// audio_spdif_ring_core.h - the ring stampers of audio_spdif.c (internal).
//
// The per-frame IEC 60958 encoders, the silence pump, the skip and the audio
// writer, and (PICO_AUDIO_SPDIF_UBITS, WP FWSP1) the user-bit superframe table
// they stamp from.  Split out of audio_spdif.c, unchanged otherwise, so the
// host test firmware/tests/test_spdif_ubits.c runs exactly this code; it has
// no hardware access of its own (the reader position comes from
// audio_spdif_ring_consumed_frames()).  Include once, after pico/audio_spdif.h,
// pico/audio_spdif/sample_encoding.h and hardware/sync.h, with
// SPDIF_TIME_CRITICAL defined.
//
// User bits (alink_protocol_spec.md 12.1, 12.4).  A table of 4 entries keyed
// by block index (block = frames since the ring reset / 192, entry block & 3)
// holds the 48-byte superframe of each block.  Every stamper (silence pump,
// skip, audio writer) takes a block's bits from it by the block index of the
// frame it stamps, and an entry is frozen once any frame of its block is
// stamped, so a block's 384 user bits can never mix two contents however the
// stampers interleave.  An entry not filled for its block when its first frame
// is stamped stamps (and stays) zero.  The optional filler is called right
// before a block's first stamp, in the same pass (the 64-frame window of
// 12.4 a); it runs inside the stampers, so it must be RAM-resident.  The block
// phase is continuous across the 32-bit frame counter wrap (the frame counter
// is unwrapped against a per-instance block reference), so an output that
// runs 24.8 h keeps 192-frame blocks (spec 12.3).  Instances not set to carry
// stamp all-zero user bits, byte-identical to the stampers before FWSP1 (the
// 2^32 wrap excepted, where the old phase stepped by 64).

#ifndef AUDIO_SPDIF_RING_CORE_H
#define AUDIO_SPDIF_RING_CORE_H

#include <stdint.h>
#include <stdbool.h>

_Static_assert((PICO_AUDIO_SPDIF_RING_FRAMES & (PICO_AUDIO_SPDIF_RING_FRAMES - 1)) == 0,
    "ring frame count must be a power of two");

#define RING_FRAMES      PICO_AUDIO_SPDIF_RING_FRAMES
#define RING_MASK        (RING_FRAMES - 1u)
#define RING_WORDS       (RING_FRAMES * 4u)
#define RING_BYTES       PICO_AUDIO_SPDIF_RING_BYTES
// Writable window: capacity minus the silence margin the writer maintains.
#define WRITABLE_FRAMES  (RING_FRAMES - PICO_AUDIO_SPDIF_ERASE_AHEAD)

// Instance registry (starvation getters, group operations, user-bit checks)
static audio_spdif_instance_t *spdif_instances[PICO_AUDIO_SPDIF_MAX_INSTANCES];
static uint spdif_instance_count = 0;

// ---------------------------------------------------------------------------
// S/PDIF constants
// ---------------------------------------------------------------------------

#define PREAMBLE_X 0b11001001
#define PREAMBLE_Y 0b01101001
#define PREAMBLE_Z 0b00111001

// IEC 60958-3 consumer channel status (5 bytes = 40 bits)
// Byte values verified against Linux kernel include/sound/asoundef.h
static uint8_t spdif_channel_status[5] = {
    0x04,  // Byte 0: consumer, PCM, copy permitted
    0x00,  // Byte 1: general category
    0x00,  // Byte 2: source/channel
    0x00,  // Byte 3: sample rate (set dynamically via update_pio_frequency)
    0x0B,  // Byte 4: max=24bit, word length=24bit (0x01 | 0x0A)
};

static inline uint get_channel_status_bit(uint subframe_idx) {
    if (subframe_idx >= 40) return 0;
    return (spdif_channel_status[subframe_idx / 8] >> (subframe_idx % 8)) & 1u;
}

// PIO-encoder words (audio_spdif_bmc.pio). Control word: preamble as lead
// and trail low-run counts, [2:0] and [5:3]. Data word: sample [23:0], V, U,
// C at bits 24..26; parity comes from the PIO.
#define BMC_CTRL_X  (4u | (1u << 3))
#define BMC_CTRL_Y  (2u | (3u << 3))
#define BMC_CTRL_Z  (0u | (5u << 3))

#if !PICO_AUDIO_SPDIF_UBITS
// ===========================================================================
// Stampers without user bits (RP2040 and builds without DSPI_ALINK_SPDIF):
// the code before WP FWSP1, unchanged.
// ===========================================================================

// Encode one stereo frame at ring position `p` with IEC phase `phase`.
static inline void stamp_frame(spdif_subframe_t *p, uint phase,
                               int32_t l, int32_t r) {
    uint c = get_channel_status_bit(phase);
    p[0].l = (phase == 0) ? PREAMBLE_Z : PREAMBLE_X;
    p[0].h = 0x55000000u | (c << 29u);
    spdif_update_subframe(&p[0], l);
    p[1].l = PREAMBLE_Y;
    p[1].h = 0x55000000u | (c << 29u);
    spdif_update_subframe(&p[1], r);
}

// Zero PCM encodes to alternating bits, with only the preamble, channel
// status and its matching parity bit varying. Write those words directly
// instead of running the sample lookup/parity encoder twice per frame.
// Always inline: the silence pump also runs while flash is unavailable.
static inline __attribute__((always_inline))
void stamp_silence_frame(spdif_subframe_t *p, uint phase) {
    uint32_t c = get_channel_status_bit(phase);
    uint32_t h = 0x55555555u | (c << 29u) | (c << 31u);
    p[0].l = 0x55555500u | ((phase == 0) ? PREAMBLE_Z : PREAMBLE_X);
    p[0].h = h;
    p[1].l = 0x55555500u | PREAMBLE_Y;
    p[1].h = h;
}

static inline __attribute__((always_inline))
void stamp_frame_bmc(spdif_subframe_t *p, uint phase, int32_t l, int32_t r) {
    uint32_t c = (uint32_t)get_channel_status_bit(phase) << 26u;
    p[0].l = (phase == 0) ? BMC_CTRL_Z : BMC_CTRL_X;
    p[0].h = ((uint32_t)l & 0xFFFFFFu) | c;
    p[1].l = BMC_CTRL_Y;
    p[1].h = ((uint32_t)r & 0xFFFFFFu) | c;
}

static inline __attribute__((always_inline))
void stamp_silence_frame_bmc(spdif_subframe_t *p, uint phase) {
    uint32_t c = (uint32_t)get_channel_status_bit(phase) << 26u;
    p[0].l = (phase == 0) ? BMC_CTRL_Z : BMC_CTRL_X;
    p[0].h = c;
    p[1].l = BMC_CTRL_Y;
    p[1].h = c;
}

// Stamp n silence frames at absolute frames [e, e+n) in the instance's word
// format. Always inline: callers run while flash may be unavailable.
static inline __attribute__((always_inline))
void stamp_silence_run(audio_spdif_instance_t *inst, uint32_t e, uint32_t n) {
    spdif_subframe_t *ring = (spdif_subframe_t *)inst->ring;
    uint phase = e % PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT;
    if (inst->pio_bmc) {
        while (n--) {
            stamp_silence_frame_bmc(&ring[(e++ & RING_MASK) * 2u], phase);
            if (++phase == PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT) phase = 0;
        }
    } else {
        while (n--) {
            stamp_silence_frame(&ring[(e++ & RING_MASK) * 2u], phase);
            if (++phase == PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT) phase = 0;
        }
    }
}

// Stamp n audio frames from interleaved L/R samples at absolute frames [w, w+n).
static inline __attribute__((always_inline))
void stamp_audio_run(audio_spdif_instance_t *inst, uint32_t w,
                     const int32_t *lr, uint32_t n) {
    spdif_subframe_t *ring = (spdif_subframe_t *)inst->ring;
    uint phase = w % PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT;
    if (inst->pio_bmc) {
        for (uint32_t i = 0; i < n; i++) {
            stamp_frame_bmc(&ring[(w++ & RING_MASK) * 2u], phase, lr[i * 2u], lr[i * 2u + 1u]);
            if (++phase == PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT) phase = 0;
        }
    } else {
        for (uint32_t i = 0; i < n; i++) {
            stamp_frame(&ring[(w++ & RING_MASK) * 2u], phase, lr[i * 2u], lr[i * 2u + 1u]);
            if (++phase == PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT) phase = 0;
        }
    }
}

static inline void spdif_ubits_ring_reset(audio_spdif_instance_t *inst) { (void)inst; }

#else  // PICO_AUDIO_SPDIF_UBITS
// ===========================================================================
// Stampers with user bits (WP FWSP1)
// ===========================================================================

#define UB_BLOCK      PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT
#define UB_TABLE      PICO_AUDIO_SPDIF_UBITS_TABLE_BLOCKS
#define UB_HAS_DATA   0x01u
#define UB_FROZEN     0x02u

typedef struct {
    uint32_t block;                         // block index the entry holds
    uint8_t  state;                         // 0 empty, UB_HAS_DATA | UB_FROZEN
    uint8_t  sf[PICO_AUDIO_SPDIF_UBITS_LEN];
} spdif_ubits_entry_t;

// RAM (.bss), never const: the stampers read these while flash is unavailable.
static spdif_ubits_entry_t spdif_ubits_table[UB_TABLE];
static uint8_t spdif_ubits_zero[PICO_AUDIO_SPDIF_UBITS_LEN];
static audio_spdif_ubits_filler_t spdif_ubits_filler;

// Block index and phase of frame e: the frame counter unwrapped against the
// instance's block reference (a block start), which then moves to e's block.
// Every stamped frame lies within a ring or so of the last one, so the signed
// distance never nears 2^31 and the phase stays continuous at the 2^32 wrap.
static inline __attribute__((always_inline))
uint32_t ub_locate(audio_spdif_instance_t *inst, uint32_t e, uint *phase) {
    int32_t d = (int32_t)(e - inst->ub_ref_frame);
    int32_t q = d / (int32_t)UB_BLOCK;
    int32_t r = d - q * (int32_t)UB_BLOCK;
    if (r < 0) { r += (int32_t)UB_BLOCK; q--; }
    inst->ub_ref_frame += (uint32_t)q * UB_BLOCK;
    inst->ub_ref_block += (uint32_t)q;
    *phase = (uint)r;
    return inst->ub_ref_block;
}

// The bits a stamper stamps into frames of block `blk`.  The first stamp of a
// block newer than its entry's gives the filler its chance (the same pass,
// immediately before the stamp), then freezes the entry for blk, filled or
// not.  A block older than the entry's lies behind the reader: zeros, and the
// entry is left alone.
SPDIF_TIME_CRITICAL
static const uint8_t *ub_enter(uint32_t blk) {
    spdif_ubits_entry_t *t = &spdif_ubits_table[blk & (UB_TABLE - 1u)];
    if (t->state == 0 || (int32_t)(blk - t->block) > 0) {
        audio_spdif_ubits_filler_t fn = spdif_ubits_filler;
        if (fn) fn(blk);
        if (t->state == 0 || t->block != blk) {
            t->block = blk;
            t->state = 0;
        }
    } else if (t->block != blk) {
        return spdif_ubits_zero;
    }
    t->state |= UB_FROZEN;
    return (t->state & UB_HAS_DATA) ? t->sf : spdif_ubits_zero;
}

// Locate frame e and fetch its block's bits (zeros for a slot not carrying).
static inline __attribute__((always_inline))
const uint8_t *ub_segment(audio_spdif_instance_t *inst, uint32_t e, uint *phase) {
    uint32_t blk = ub_locate(inst, e, phase);
    return inst->ub_carry ? ub_enter(blk) : spdif_ubits_zero;
}

// The two user bits of frame `phase` of a block: bit 0 channel A, bit 1 B.
#define UB_FRAME_BITS(ub, phase) (((uint32_t)(ub)[(phase) >> 2] >> (((phase) & 3u) * 2u)) & 3u)

static inline __attribute__((always_inline))
void stamp_frame(spdif_subframe_t *p, uint phase, uint32_t u, int32_t l, int32_t r) {
    uint32_t c = get_channel_status_bit(phase);
    p[0].l = (phase == 0) ? PREAMBLE_Z : PREAMBLE_X;
    p[0].h = 0x55000000u | (c << 29u) | ((u & 1u) << 27u);
    spdif_update_subframe(&p[0], l);
    p[1].l = PREAMBLE_Y;
    p[1].h = 0x55000000u | (c << 29u) | ((u >> 1) << 27u);
    spdif_update_subframe(&p[1], r);
}

// Zero PCM: alternating cells; preamble, U, C and the matching parity vary.
static inline __attribute__((always_inline))
void stamp_silence_frame(spdif_subframe_t *p, uint phase, uint32_t u) {
    uint32_t c = get_channel_status_bit(phase);
    uint32_t ua = u & 1u, ub = u >> 1;
    p[0].l = 0x55555500u | ((phase == 0) ? PREAMBLE_Z : PREAMBLE_X);
    p[0].h = 0x55555555u | (c << 29u) | (ua << 27u) | ((c ^ ua) << 31u);
    p[1].l = 0x55555500u | PREAMBLE_Y;
    p[1].h = 0x55555555u | (c << 29u) | (ub << 27u) | ((c ^ ub) << 31u);
}

static inline __attribute__((always_inline))
void stamp_frame_bmc(spdif_subframe_t *p, uint phase, uint32_t u, int32_t l, int32_t r) {
    uint32_t c = (uint32_t)get_channel_status_bit(phase) << 26u;
    p[0].l = (phase == 0) ? BMC_CTRL_Z : BMC_CTRL_X;
    p[0].h = ((uint32_t)l & 0xFFFFFFu) | c | ((u & 1u) << 25u);
    p[1].l = BMC_CTRL_Y;
    p[1].h = ((uint32_t)r & 0xFFFFFFu) | c | ((u >> 1) << 25u);
}

static inline __attribute__((always_inline))
void stamp_silence_frame_bmc(spdif_subframe_t *p, uint phase, uint32_t u) {
    uint32_t c = (uint32_t)get_channel_status_bit(phase) << 26u;
    p[0].l = (phase == 0) ? BMC_CTRL_Z : BMC_CTRL_X;
    p[0].h = c | ((u & 1u) << 25u);
    p[1].l = BMC_CTRL_Y;
    p[1].h = c | ((u >> 1) << 25u);
}

// Stamp n silence frames at absolute frames [e, e+n), block by block.  One
// RAM copy for its three callers (pump, skip, writer): callers run while
// flash may be unavailable.
SPDIF_TIME_CRITICAL
static void stamp_silence_run(audio_spdif_instance_t *inst, uint32_t e, uint32_t n) {
    spdif_subframe_t *ring = (spdif_subframe_t *)inst->ring;
    while (n) {
        uint phase;
        const uint8_t *ub = ub_segment(inst, e, &phase);
        uint32_t seg = UB_BLOCK - phase;
        if (seg > n) seg = n;
        n -= seg;
        if (inst->pio_bmc) {
            for (uint32_t i = 0; i < seg; i++, phase++)
                stamp_silence_frame_bmc(&ring[(e++ & RING_MASK) * 2u], phase, UB_FRAME_BITS(ub, phase));
        } else {
            for (uint32_t i = 0; i < seg; i++, phase++)
                stamp_silence_frame(&ring[(e++ & RING_MASK) * 2u], phase, UB_FRAME_BITS(ub, phase));
        }
    }
}

// Stamp n audio frames from interleaved L/R samples at absolute frames [w, w+n).
static inline __attribute__((always_inline))
void stamp_audio_run(audio_spdif_instance_t *inst, uint32_t w,
                     const int32_t *lr, uint32_t n) {
    spdif_subframe_t *ring = (spdif_subframe_t *)inst->ring;
    while (n) {
        uint phase;
        const uint8_t *ub = ub_segment(inst, w, &phase);
        uint32_t seg = UB_BLOCK - phase;
        if (seg > n) seg = n;
        n -= seg;
        if (inst->pio_bmc) {
            for (uint32_t i = 0; i < seg; i++, phase++, lr += 2)
                stamp_frame_bmc(&ring[(w++ & RING_MASK) * 2u], phase, UB_FRAME_BITS(ub, phase), lr[0], lr[1]);
        } else {
            for (uint32_t i = 0; i < seg; i++, phase++, lr += 2)
                stamp_frame(&ring[(w++ & RING_MASK) * 2u], phase, UB_FRAME_BITS(ub, phase), lr[0], lr[1]);
        }
    }
}

static void spdif_ubits_clear_locked(void) {
    for (uint i = 0; i < UB_TABLE; i++) {
        spdif_ubits_table[i].state = 0;
        spdif_ubits_table[i].block = 0;
    }
}

// Ring reset: block 0 starts at frame 0 again.  A carrying slot drops every
// entry of the old numbering (block >= 4) but keeps blocks 0-3: a sibling slot
// reset in the same restart has already stamped them, and a ring must never
// see two contents for one block.  A source that starts a new epoch calls
// audio_spdif_ubits_clear() once before the group restart (then a run shorter
// than four blocks cannot leave its blocks 0-3 behind either).
static inline void spdif_ubits_ring_reset(audio_spdif_instance_t *inst) {
    inst->ub_ref_frame = 0;
    inst->ub_ref_block = 0;
    if (!inst->ub_carry) return;
    uint32_t save = save_and_disable_interrupts();
    for (uint i = 0; i < UB_TABLE; i++) {
        if (spdif_ubits_table[i].block >= UB_TABLE) {
            spdif_ubits_table[i].state = 0;
            spdif_ubits_table[i].block = 0;
        }
    }
    restore_interrupts(save);
}

void audio_spdif_ubits_clear(void) {
    uint32_t save = save_and_disable_interrupts();
    spdif_ubits_clear_locked();
    restore_interrupts(save);
}

void audio_spdif_ubits_set_filler(audio_spdif_ubits_filler_t fn) {
    spdif_ubits_filler = fn;
}

void audio_spdif_ubits_set_carry(audio_spdif_instance_t *inst, bool carry) {
    inst->ub_carry = carry;
}

// Block index of frame f of an instance, without moving its reference.
static inline uint32_t ub_block_of(const audio_spdif_instance_t *inst, uint32_t f) {
    int32_t d = (int32_t)(f - inst->ub_ref_frame);
    int32_t q = d / (int32_t)UB_BLOCK;
    if (d - q * (int32_t)UB_BLOCK < 0) q--;
    return inst->ub_ref_block + (uint32_t)q;
}

// The reader's block (the oldest over carrying slots) and the newest stamped
// block; false with no carrying slot set up and enabled.
SPDIF_TIME_CRITICAL
static bool ub_span(uint32_t *reader_blk, uint32_t *stamped_blk) {
    bool any = false;
    for (uint i = 0; i < spdif_instance_count; i++) {
        audio_spdif_instance_t *inst = spdif_instances[i];
        if (!inst->ub_carry || !inst->ring || !inst->enabled) continue;
        uint32_t rb = ub_block_of(inst, audio_spdif_ring_consumed_frames(inst));
        // Every frame before erased_frames is stamped (audio ends at wr <=
        // erased).  Not wr: a skip advances wr before it stamps the gap.
        uint32_t sb = ub_block_of(inst, inst->erased_frames - 1u);
        if (!any || (int32_t)(rb - *reader_blk) < 0) *reader_blk = rb;
        if (!any || (int32_t)(sb - *stamped_blk) > 0) *stamped_blk = sb;
        any = true;
    }
    return any;
}

SPDIF_TIME_CRITICAL
int audio_spdif_ubits_fill(uint32_t block, const uint8_t sf[PICO_AUDIO_SPDIF_UBITS_LEN]) {
    int res = AUDIO_SPDIF_UBITS_OK;
    uint32_t save = save_and_disable_interrupts();
    spdif_ubits_entry_t *t = &spdif_ubits_table[block & (UB_TABLE - 1u)];
    uint32_t rb, sb;
    bool live = ub_span(&rb, &sb);
    if (t->state != 0 && (int32_t)(block - t->block) < 0) {
        res = AUDIO_SPDIF_UBITS_FROZEN;           // an older block: gone
    } else if (t->state != 0 && t->block == block && (t->state & UB_FROZEN)) {
        res = AUDIO_SPDIF_UBITS_FROZEN;           // already (partly) stamped
    } else if (live && (int32_t)(block - sb) <= 0) {
        res = AUDIO_SPDIF_UBITS_FROZEN;           // inside the stamped span (erase-ahead)
    } else if (live && (int32_t)(block - rb) >= (int32_t)UB_TABLE) {
        res = AUDIO_SPDIF_UBITS_EARLY;            // its entry still serves a block not yet played
    } else {
        t->block = block;
        for (uint i = 0; i < PICO_AUDIO_SPDIF_UBITS_LEN; i++) t->sf[i] = sf[i];
        t->state = UB_HAS_DATA;
    }
    restore_interrupts(save);
    return res;
}

bool audio_spdif_ubits_window(uint32_t *first, uint32_t *last) {
    uint32_t rb, sb;
    uint32_t save = save_and_disable_interrupts();
    bool live = ub_span(&rb, &sb);
    restore_interrupts(save);
    if (!live) return false;
    *first = sb + 1u;
    *last = rb + UB_TABLE - 1u;
    return (int32_t)(*last - *first) >= 0;
}

uint32_t audio_spdif_ubits_block_of(const audio_spdif_instance_t *inst, uint32_t frame) {
    return ub_block_of(inst, frame);
}

#endif // PICO_AUDIO_SPDIF_UBITS

// ---------------------------------------------------------------------------
// Ring writer, skip and silence pump
// ---------------------------------------------------------------------------

SPDIF_TIME_CRITICAL
void audio_spdif_ring_skip(audio_spdif_instance_t *inst, uint32_t frames) {
    inst->underrun_frames += frames;
    inst->wr_frames += frames;
    // Restamp silence through the new margin: the reader immediately
    // traverses the skipped span, which beyond the old erase-ahead may
    // still hold stale audio from the previous lap.
    uint32_t target = inst->wr_frames + PICO_AUDIO_SPDIF_ERASE_AHEAD;
    uint32_t e = inst->erased_frames;
    if ((int32_t)(target - e) > (int32_t)RING_FRAMES) e = target - RING_FRAMES;
    if ((int32_t)(target - e) > 0) {
        stamp_silence_run(inst, e, target - e);
        e = target;
    }
    inst->erased_frames = e;
}

// Extend the silence region toward one full ring ahead of the reader so a
// stalled producer can never loop stale content. Never touches unplayed
// audio (the stamped span is exactly the free region). Bounded per call.
SPDIF_TIME_CRITICAL
void audio_spdif_ring_silence_pump(audio_spdif_instance_t *inst, uint32_t max_frames) {
    uint32_t consumed = audio_spdif_ring_consumed_frames(inst);
    uint32_t target = consumed + RING_FRAMES;
    uint32_t e = inst->erased_frames;
    if ((int32_t)(e - inst->wr_frames) < 0) e = inst->wr_frames;
    int32_t need = (int32_t)(target - e);
    if (need <= 0) return;
    uint32_t n = ((uint32_t)need > max_frames) ? max_frames : (uint32_t)need;
    stamp_silence_run(inst, e, n);
    inst->erased_frames = e + n;
}

SPDIF_TIME_CRITICAL
void audio_spdif_ring_write_s32(audio_spdif_instance_t *inst,
                                const int32_t *lr, uint32_t frames) {
    uint32_t consumed = audio_spdif_ring_consumed_frames(inst);
    // Reader ahead of the writer is a deficit, not a full ring: it can
    // overtake between the give path's admission and this write, and a
    // reject then would advance every peer's wr but not this slot's.
    int32_t signed_fill = (int32_t)(inst->wr_frames - consumed);
    uint32_t fill = (signed_fill > 0) ? (uint32_t)signed_fill : 0u;

    // All-or-nothing: a partial write would advance this slot's wr by a
    // different amount than its peers', permanently skewing alignment. The
    // DSPi give path admits a block only when EVERY enabled slot has space,
    // and space only grows afterwards, so this reject is unreachable there.
    uint32_t writable = (fill < WRITABLE_FRAMES) ? (WRITABLE_FRAMES - fill) : 0;
    if (frames > writable) {
        inst->drop_frames += frames;
        inst->snap_produced_frames = inst->wr_frames;
        inst->snap_consumed_frames = consumed;
        return;
    }

#if !PICO_AUDIO_SPDIF_UBITS
    // Phase = w mod 192. Known limit: at the uint32 wrap (~24.8 h of
    // uninterrupted streaming, and any ring reset restarts the clock) the
    // phase steps by 64 for one block; receivers resync on the next Z.
#else
    // Block phase from the unwrapped block reference: continuous at the wrap.
#endif
    uint32_t w = inst->wr_frames;
    stamp_audio_run(inst, w, lr, frames);
    w += frames;
    inst->wr_frames = w;

    // Extend the erase-ahead silence margin.
    // Signed ordering: raw compares break at the uint32 counter wrap.
    uint32_t e = inst->erased_frames;
    if ((int32_t)(e - w) < 0) e = w;
    uint32_t target = w + PICO_AUDIO_SPDIF_ERASE_AHEAD;
    if ((int32_t)(target - e) > 0) {
        stamp_silence_run(inst, e, target - e);
        e = target;
    }
    inst->erased_frames = e;

    // Recovery needs the fill before THIS block, not the post-write servo
    // snapshot below. If the producer stalls, these `frames` remain real
    // audio ahead of PDM's first inserted zero; counting them as silence
    // would move the slot timeline by one block. Clamped at 0: frames the
    // reader already passed are lost to this slot whatever the skip.
    inst->recovery_prewrite_fill_frames = fill;

    // Latch the coherent produced/consumed pair (constant production phase;
    // the clock servo's cumulative-count observable).
    inst->snap_produced_frames = w;
    inst->snap_consumed_frames = consumed;
}

#endif // AUDIO_SPDIF_RING_CORE_H
