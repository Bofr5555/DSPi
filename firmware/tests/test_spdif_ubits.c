// Host test for the S/PDIF user-bit carriage in and out (WP FWSP1,
// alink_protocol_spec.md 12.1 and 12.4): the transmit library's own stampers
// (pico_audio_spdif_multi/audio_spdif_ring_core.h: audio writer, silence pump,
// skip, the 4-entry superframe table and its filler) driven by a model of the
// DSPi main loop, the give path and the flash-blackout timer interrupt,
// interleaved and overwriting the same frames, against a model DMA reader that
// captures what goes on the wire; the wire is decoded into the receiver's raw
// subframe words and read back block by block through the receiver's own
// user-bit collector (pico_spdif_rx/spdif_rx_ubits.h).  Checks: every block's
// 384 user bits are exactly one content (no mixed block) and the content the
// table held for it; blocks are exactly 192 frames apart (Z preamble), also
// across the 32-bit frame counter wrap; parity holds with U set; the fill
// rules (erase-ahead refused, too early refused); the stamper's same-pass
// filler is never refused outside a blackout; a blackout emits zeros; an
// emptied table emits zeros; a slot not carrying is byte-identical to the
// stampers before FWSP1; a broadcast at the minimum lead (896 frames) is
// carried by at least one complete superframe.  At 44.1, 48 and 96 kHz, both
// word formats (software NRZI words and the PIO biphase encoder's words).
// Built by run.sh: cc -std=c11 -Wall -Wextra -Werror -I ../DSPi -I stubs test_spdif_ubits.c
#define PICO_AUDIO_SPDIF_UBITS 1
#define SPDIF_TIME_CRITICAL
#include "pico.h"
#include "../pico-extras/src/rp2_common/pico_audio_spdif_multi/include/pico/audio_spdif.h"
#include "../pico-extras/src/rp2_common/pico_audio_spdif_multi/include/pico/audio_spdif/sample_encoding.h"
#include "hardware/sync.h"

uint32_t spdif_lookup[256];
uint32_t audio_spdif_ring_consumed_frames(audio_spdif_instance_t *inst);

#include "../pico-extras/src/rp2_common/pico_audio_spdif_multi/audio_spdif_ring_core.h"
#include "../pico-extras/src/rp2_common/pico_spdif_rx/spdif_rx_ubits.h"
#include "alink/alink_spdif_codec.c"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { if (failures++ < 40) { \
    fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } } while (0)

// ---------------------------------------------------------------------------
// The model reader (the DMA): consumed_frames is the instance's own field
// ---------------------------------------------------------------------------

uint32_t audio_spdif_ring_consumed_frames(audio_spdif_instance_t *inst) {
    return inst->consumed_frames;
}

static uint32_t rng_state = 1;
static uint32_t rnd(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}
static uint32_t rnd_in(uint32_t lo, uint32_t hi) { return lo + rnd() % (hi - lo + 1u); }

static void build_lookup(void) {
    for (uint i = 0; i < 256; i++) {
        uint32_t v = 0x5555;
        uint p = 0;
        for (uint j = 0; j < 8; j++) {
            if (i & (1u << j)) {
                p ^= 1;
                v |= (2u << (j * 2));
            }
        }
        spdif_lookup[i] = v | (p << 16u);
    }
}

// ---------------------------------------------------------------------------
// The stampers before WP FWSP1, verbatim (byte-identity of a slot not carrying)
// ---------------------------------------------------------------------------

static void old_stamp_frame(spdif_subframe_t *p, uint phase, int32_t l, int32_t r) {
    uint c = get_channel_status_bit(phase);
    p[0].l = (phase == 0) ? PREAMBLE_Z : PREAMBLE_X;
    p[0].h = 0x55000000u | (c << 29u);
    spdif_update_subframe(&p[0], l);
    p[1].l = PREAMBLE_Y;
    p[1].h = 0x55000000u | (c << 29u);
    spdif_update_subframe(&p[1], r);
}

static void old_stamp_silence_frame(spdif_subframe_t *p, uint phase) {
    uint32_t c = get_channel_status_bit(phase);
    uint32_t h = 0x55555555u | (c << 29u) | (c << 31u);
    p[0].l = 0x55555500u | ((phase == 0) ? PREAMBLE_Z : PREAMBLE_X);
    p[0].h = h;
    p[1].l = 0x55555500u | PREAMBLE_Y;
    p[1].h = h;
}

static void old_stamp_frame_bmc(spdif_subframe_t *p, uint phase, int32_t l, int32_t r) {
    uint32_t c = (uint32_t)get_channel_status_bit(phase) << 26u;
    p[0].l = (phase == 0) ? BMC_CTRL_Z : BMC_CTRL_X;
    p[0].h = ((uint32_t)l & 0xFFFFFFu) | c;
    p[1].l = BMC_CTRL_Y;
    p[1].h = ((uint32_t)r & 0xFFFFFFu) | c;
}

// ---------------------------------------------------------------------------
// The wire: frames the reader passed, decoded into receiver raw words
// ---------------------------------------------------------------------------

#define SYNC_B 0xFu
#define SYNC_M 0xBu
#define SYNC_W 0x7u

#define MAX_WIRE (1u << 20)   // frames per instance per scenario

typedef struct {
    audio_spdif_instance_t inst;
    uint32_t ring[PICO_AUDIO_SPDIF_RING_FRAMES * 4u];
    spdif_subframe_t *wire;           // 2 subframes per captured frame
    uint32_t wire_n;
    uint32_t wire_first;              // frame counter of wire[0]
    bool carry;
} slot_t;

#define NSLOTS 3   // 0: carrying, NRZI; 1: carrying, PIO encoder; 2: not carrying, NRZI

static slot_t slots[NSLOTS];

// Raw receiver word of one subframe (bits 0-3 sync, bit s = time slot s).
static uint32_t rx_word(const spdif_subframe_t *sf, bool bmc, bool *ok) {
    uint32_t w = 0;
    if (bmc) {
        uint32_t ctrl = sf->l;
        w = ctrl == BMC_CTRL_Z ? SYNC_B : ctrl == BMC_CTRL_X ? SYNC_M : ctrl == BMC_CTRL_Y ? SYNC_W : 0;
        if (!w) *ok = false;
        w |= (sf->h & 0x7FFFFFFu) << 4;               // sample, V, U, C at slots 4..30
        uint32_t par = __builtin_popcount(w & 0x7FFFFFF0u) & 1u;
        w |= par << 31;                               // the PIO makes even parity
    } else {
        uint32_t pre = sf->l & 0xFFu;
        w = pre == PREAMBLE_Z ? SYNC_B : pre == PREAMBLE_X ? SYNC_M : pre == PREAMBLE_Y ? SYNC_W : 0;
        if (!w) *ok = false;
        for (uint s = 4; s < 32; s++) {
            uint32_t cell = s < 16 ? (sf->l >> (2u * s)) & 3u : (sf->h >> (2u * (s - 16u))) & 3u;
            if (!(cell & 1u)) *ok = false;            // every cell starts with a transition
            w |= ((cell >> 1) & 1u) << s;
        }
    }
    return w;
}

// The model DMA: advance every slot's reader by n frames, capturing the ring.
static uint32_t stamped_min_gap;   // smallest (stamped frontier - reader) seen, model sanity
static void reader_advance(uint32_t n) {
    for (int k = 0; k < NSLOTS; k++) {
        slot_t *s = &slots[k];
        audio_spdif_instance_t *inst = &s->inst;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t f = inst->consumed_frames;
            uint32_t e = inst->erased_frames;
            if ((int32_t)(e - inst->wr_frames) < 0) e = inst->wr_frames;
            int32_t gap = (int32_t)(e - f);
            if (gap <= 0) stamped_min_gap = 0;
            spdif_subframe_t *p = &((spdif_subframe_t *)s->ring)[(f & RING_MASK) * 2u];
            if (s->wire_n < MAX_WIRE) {
                s->wire[2u * s->wire_n] = p[0];
                s->wire[2u * s->wire_n + 1u] = p[1];
                s->wire_n++;
            }
            inst->consumed_frames = f + 1u;
        }
    }
}

// ---------------------------------------------------------------------------
// The source model (the FWSP2 role, reduced): F = origin + 192 * block
// ---------------------------------------------------------------------------

#define MAX_BLOCKS (MAX_WIRE / 192u + 64u)

static struct {
    alink_spdif_src_t src;
    uint64_t origin;            // F of block 0
    uint32_t block0;            // block index the expectation arrays start at
    bool blackout;              // flash unavailable: the filler declines
    bool filler_on;
    uint8_t expect[MAX_BLOCKS][48];
    bool filled[MAX_BLOCKS];
    uint32_t asked, asked_refused, fills, prefills;
} S;

static void block_sf(uint32_t block, uint8_t out[48]) {
    alink_spdif_sf_build(&S.src, S.origin + 192u * (uint64_t)block, out);
}

static void record_fill(uint32_t block, const uint8_t sf[48]) {
    uint32_t i = block - S.block0;
    if (i < MAX_BLOCKS) {
        memcpy(S.expect[i], sf, 48);
        S.filled[i] = true;
    }
}

static void test_filler(uint32_t block) {
    S.asked++;
    if (S.blackout) return;
    uint8_t sf[48];
    block_sf(block, sf);
    int r = audio_spdif_ubits_fill(block, sf);
    if (r == AUDIO_SPDIF_UBITS_OK) {
        record_fill(block, sf);
        S.fills++;
    } else {
        S.asked_refused++;
        if (S.asked_refused < 4) {
            uint32_t rb, sb;
            bool live = ub_span(&rb, &sb);
            spdif_ubits_entry_t *t = &spdif_ubits_table[block & 3u];
            fprintf(stderr, "  filler refused %d: block %u live %d reader %u stamped %u entry %u/%u\n", r,
                    block, live, rb, sb, t->block, t->state);
        }
    }
}

// Fills of any block near the reader with a different content (another
// epoch): the rules must refuse every block already (partly) stamped; an
// accepted one becomes that block's content.
static uint32_t rogue_ok, rogue_refused;
static void rogue_fills(void) {
    uint32_t rb = audio_spdif_ubits_block_of(&slots[0].inst, slots[0].inst.consumed_frames);
    for (uint32_t b = rb - 1u; b != rb + 6u; b++) {
        alink_spdif_src_t keep = S.src;
        S.src.epoch = (uint8_t)(S.src.epoch + 1u + (rnd() & 7u));
        uint8_t sf[48];
        block_sf(b, sf);
        S.src = keep;
        if (audio_spdif_ubits_fill(b, sf) == AUDIO_SPDIF_UBITS_OK) { record_fill(b, sf); rogue_ok++; }
        else rogue_refused++;
    }
}

// The flash-resident way: fill the whole window from the main loop.
static void prefill_window(void) {
    uint32_t first, last;
    if (S.blackout || !audio_spdif_ubits_window(&first, &last)) return;
    for (uint32_t b = first; (int32_t)(b - last) <= 0; b++) {
        uint8_t sf[48];
        block_sf(b, sf);
        int r = audio_spdif_ubits_fill(b, sf);
        CHECK(r == AUDIO_SPDIF_UBITS_OK, "window fill of block %" PRIu32 " refused (%d)", b, r);
        if (r == AUDIO_SPDIF_UBITS_OK) { record_fill(b, sf); S.prefills++; }
    }
}

// ---------------------------------------------------------------------------
// Ring operations applied to every slot (all start in sync)
// ---------------------------------------------------------------------------

static int32_t sample_of(uint32_t frame, unsigned ch) {
    uint32_t h = frame * 0x9E3779B1u ^ (ch + 1u) * 0x85EBCA77u;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 13;
    return (int32_t)(h << 8) >> 8;
}

static uint8_t *audio_mark;      // per wire frame: 1 if written as audio
static uint32_t mark_first;

static void write_all(uint32_t frames) {
    static int32_t lr[2 * 512];
    uint32_t w = slots[0].inst.wr_frames;
    for (uint32_t i = 0; i < frames; i++) {
        lr[2 * i] = sample_of(w + i, 0);
        lr[2 * i + 1] = sample_of(w + i, 1);
        uint32_t m = w + i - mark_first;
        if (m < MAX_WIRE) audio_mark[m] = 1;
    }
    for (int k = 0; k < NSLOTS; k++) audio_spdif_ring_write_s32(&slots[k].inst, lr, frames);
}

static uint32_t fill_of(void) {
    int32_t f = (int32_t)(slots[0].inst.wr_frames - slots[0].inst.consumed_frames);
    return f > 0 ? (uint32_t)f : 0u;
}

static void pump_all(uint32_t max) {
    for (int k = 0; k < NSLOTS; k++) audio_spdif_ring_silence_pump(&slots[k].inst, max);
}

// The give path's re-anchor (output_rings_pre_give): a reader past the writer.
static void recover_all(uint32_t target) {
    int32_t deficit = (int32_t)(slots[0].inst.consumed_frames - slots[0].inst.wr_frames);
    if (deficit <= 0) return;
    for (int k = 0; k < NSLOTS; k++) audio_spdif_ring_skip(&slots[k].inst, (uint32_t)deficit + target);
}

static void slots_init(uint32_t start_frame, bool bmc1) {
    spdif_instance_count = 0;
    for (int k = 0; k < NSLOTS; k++) {
        slot_t *s = &slots[k];
        spdif_subframe_t *keep = s->wire;
        memset(&s->inst, 0, sizeof(s->inst));
        s->wire = keep;
        s->inst.ring = s->ring;
        s->inst.pio_bmc = (k == 1) && bmc1;
        s->inst.enabled = true;
        s->inst.freq = 48000;
        s->carry = k < 2;
        audio_spdif_ubits_set_carry(&s->inst, s->carry);
        spdif_instances[spdif_instance_count++] = &s->inst;
    }
    audio_spdif_ubits_clear();
    for (int k = 0; k < NSLOTS; k++) {
        audio_spdif_instance_t *inst = &slots[k].inst;
        // audio_spdif_ring_reset() without the hardware: anchor, then (below)
        // a ring of silence, the reset's own stamp (the filler may fill).
        inst->wr_frames = inst->erased_frames = inst->consumed_frames = 0;
        inst->recovery_prewrite_fill_frames = UINT32_MAX;
        spdif_ubits_ring_reset(inst);
        if (start_frame) {
            // Jump the counters (the 2^32 wrap scenario): the block reference
            // is a block start, so the block phase is continuous from here.
            inst->wr_frames = inst->erased_frames = inst->consumed_frames = start_frame;
            inst->ub_ref_frame = start_frame;
            inst->ub_ref_block = 1000u;
        }
    }
    for (int k = 0; k < NSLOTS; k++) {
        audio_spdif_instance_t *inst = &slots[k].inst;
        audio_spdif_ring_silence_pump(inst, PICO_AUDIO_SPDIF_RING_FRAMES);
        slots[k].wire_n = 0;
        slots[k].wire_first = inst->consumed_frames;
    }
    mark_first = slots[0].inst.consumed_frames;
    memset(audio_mark, 0, MAX_WIRE);
}

// ---------------------------------------------------------------------------
// Reading the wire back through the collector
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t blocks, data_blocks, zero_blocks, mixed, bad_parity, bad_cells, bad_spacing, mismatched;
    uint32_t carried_ok;          // blocks carrying the op under test
} readback_t;

static uint32_t rxw[2 * MAX_WIRE];

// Decode slot k's wire into rxw; returns the parity errors, *cells_ok false on a bad cell.
static uint32_t decode_wire(int k, bool *cells_ok) {
    slot_t *s = &slots[k];
    uint32_t bad = 0;
    *cells_ok = true;
    for (uint32_t i = 0; i < 2u * s->wire_n; i++) {
        bool ok = true;
        rxw[i] = rx_word(&s->wire[i], s->inst.pio_bmc, &ok);
        if (!ok) *cells_ok = false;
        if (__builtin_popcount(rxw[i] & 0xFFFFFFF0u) & 1) bad++;
    }
    return bad;
}

static void read_back(int k, readback_t *rb, uint64_t want_apply, bool *apply_seen) {
    slot_t *s = &slots[k];
    bool cells_ok;
    rb->bad_parity += decode_wire(k, &cells_ok);
    if (!cells_ok) rb->bad_cells++;
    // Block starts: subframe with the B preamble.  The wire begins at a
    // reader position; the first Z marks block `blk0`.
    int64_t prev_z = -1;
    for (uint32_t i = 0; i + 1u < 2u * s->wire_n; i += 2) {
        if ((rxw[i] & 0xFu) != SYNC_B) continue;
        uint32_t frame = s->wire_first + i / 2u;
        if (prev_z >= 0 && (int64_t)i - prev_z != 384) rb->bad_spacing++;
        prev_z = i;
        if (i + 384u > 2u * s->wire_n) break;
        bool aligned = true;
        for (uint32_t j = 0; j < 384u; j++) {
            uint32_t sy = rxw[i + j] & 0xFu;
            if ((j & 1u) ? sy != SYNC_W : (j ? sy != SYNC_M : sy != SYNC_B)) aligned = false;
        }
        if (!aligned) { rb->bad_spacing++; continue; }
        uint8_t ub[SPDIF_RX_UBITS_LEN];
        spdif_rx_collect_ubits(&rxw[i], ub);
        uint32_t blk = audio_spdif_ubits_block_of(&s->inst, frame);
        uint32_t bi = blk - S.block0;
        rb->blocks++;
        static const uint8_t zero[48];
        bool is_zero = memcmp(ub, zero, 48) == 0;
        if (!s->carry) {
            if (!is_zero) rb->mismatched++;
            rb->zero_blocks++;
            continue;
        }
        const uint8_t *want = (bi < MAX_BLOCKS && S.filled[bi]) ? S.expect[bi] : zero;
        if (memcmp(ub, want, 48) != 0) {
            rb->mismatched++;
            // Mixed: neither the expected content nor zeros, nor any whole
            // superframe (a valid CRC).
            alink_spdif_sf_t sf;
            if (!is_zero && alink_spdif_sf_unpack(ub, &sf) != ALINK_SPDIF_OK) rb->mixed++;
            if (failures < 40)
                fprintf(stderr, "  slot %d block %" PRIu32 " frame %" PRIu32 ": got %s\n", k, blk, frame,
                        is_zero ? "zeros" : "other content");
        }
        if (is_zero) { rb->zero_blocks++; continue; }
        rb->data_blocks++;
        alink_spdif_sf_t sf;
        int r = alink_spdif_sf_unpack(ub, &sf);
        CHECK(r == ALINK_SPDIF_OK, "slot %d block %" PRIu32 ": superframe invalid (%d)", k, blk, r);
        CHECK(sf.frame == S.origin + 192u * (uint64_t)blk, "F of block %" PRIu32, blk);
        if (want_apply && sf.apply_frame == want_apply && sf.frame + 192u <= want_apply) {
            rb->carried_ok++;
            if (apply_seen) *apply_seen = true;
        }
    }
}

// The bytes of a slot not carrying equal the old stampers' for every frame.
static uint32_t check_legacy_identity(int k) {
    slot_t *s = &slots[k];
    uint32_t bad = 0;
    for (uint32_t i = 0; i < s->wire_n; i++) {
        uint32_t f = s->wire_first + i;
        spdif_subframe_t want[2];
        uint phase = f % 192u;
        bool audio = (f - mark_first) < MAX_WIRE && audio_mark[f - mark_first];
        if (s->inst.pio_bmc) {
            old_stamp_frame_bmc(want, phase, audio ? sample_of(f, 0) : 0, audio ? sample_of(f, 1) : 0);
        } else if (audio) {
            old_stamp_frame(want, phase, sample_of(f, 0), sample_of(f, 1));
        } else {
            old_stamp_silence_frame(want, phase);
        }
        if (memcmp(want, &s->wire[2u * i], sizeof(want)) != 0) bad++;
    }
    return bad;
}

// ---------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------

typedef enum { MODE_FILLER, MODE_PREFILL } fill_mode_t;

typedef struct {
    uint32_t rate;
    fill_mode_t mode;
    bool bmc;
    bool blackouts;
    bool irq_pumps;          // stall-bracket pumps between main-loop steps
    bool ops;                // broadcast ops at the minimum lead
    uint32_t start_frame;    // nonzero: near the 2^32 wrap
    uint32_t seconds_x10;
} scen_t;

static uint32_t ops_scheduled, ops_carried;

static void run_scenario(const scen_t *sc, const char *label) {
    memset(&S, 0, sizeof(S));
    S.src = (alink_spdif_src_t){ .rate = sc->rate == 44100 ? 0 : sc->rate == 48000 ? 1 : 2,
                                 .sender = 0x42, .sender2 = 0x37, .epoch = 3 };
    S.origin = 192u * 5000u;
    S.filler_on = sc->mode == MODE_FILLER;
    audio_spdif_ubits_set_filler(S.filler_on ? test_filler : NULL);
    stamped_min_gap = UINT32_MAX;
    // block0: the block index of the first frame (expectation arrays start there).
    S.block0 = sc->start_frame ? 1000u : 0u;
    slots_init(sc->start_frame, sc->bmc);
    CHECK(S.block0 == audio_spdif_ubits_block_of(&slots[0].inst, slots[0].inst.consumed_frames), "block0");

    uint64_t total_us = (uint64_t)sc->seconds_x10 * 100000u;
    uint64_t t_us = 0;
    double owed = 0.0, frac = 0.0;
    uint32_t pkt_acc = 0;
    bool streaming = true;
    uint64_t phase_end = 300000;      // alternate streaming / idle
    uint64_t blackout_end = 0, next_blackout = 150000;
    uint32_t target = sc->rate * 3u / 1000u;    // 3 ms: the reader never passes the erase-ahead in a 2 ms pass
    uint64_t pending_n = 0;
    uint64_t pending_set_at_frame = 0;
    bool pending_seen = false;
    uint32_t ops_before = ops_scheduled;
    (void)pending_set_at_frame;

    while (t_us < total_us && slots[0].wire_n + 4096u < MAX_WIRE) {
        // Phase changes.
        if (t_us >= phase_end) {
            streaming = !streaming;
            phase_end = t_us + rnd_in(80000, 400000);
            if (streaming) owed = 0.0;
        }
        bool in_blackout = sc->blackouts && t_us < blackout_end;
        if (sc->blackouts && !in_blackout && t_us >= next_blackout) {
            blackout_end = t_us + rnd_in(3000, 60000);
            next_blackout = blackout_end + rnd_in(50000, 250000);
            in_blackout = true;
        }
        S.blackout = in_blackout;
        uint32_t dt;
        if (in_blackout) {
            // The stall alarm: every 1 ms a full ring of silence ahead.
            pump_all(PICO_AUDIO_SPDIF_RING_FRAMES);
            dt = 1000;
        } else {
            // One main-loop pass: the source's service, the give path, the pumps.
            if (sc->mode == MODE_PREFILL) {
                if (rnd() % 2u) rogue_fills();
                prefill_window();
            }
            if (sc->ops && !pending_n && rnd() % 64u == 0) {
                // A broadcast at exactly the minimum lead from the frame played now.
                uint32_t f = slots[0].inst.consumed_frames;
                uint ph;
                uint32_t blk = audio_spdif_ubits_block_of(&slots[0].inst, f);
                ph = (uint)(f - slots[0].inst.ub_ref_frame - 192u * (blk - slots[0].inst.ub_ref_block));
                uint64_t now_F = S.origin + 192u * (uint64_t)blk + ph;
                pending_n = now_F + ALINK_SPDIF_MIN_LEAD_FRAMES;
                CHECK(alink_spdif_lead_ok(now_F, pending_n), "lead rule");
                S.src.apply_frame = pending_n;
                S.src.apply_op = ALINK_SPDIF_OP_CLICK;
                S.src.apply_arg = 1;
                pending_seen = false;
                ops_scheduled++;
            }
            if (sc->irq_pumps && rnd() % 4u == 0) pump_all(PICO_AUDIO_SPDIF_RING_FRAMES);
            recover_all(target);
            if (streaming) {
                uint32_t pkt = sc->rate / 1000u;
                if (sc->rate == 44100) { pkt_acc += 100; pkt = 44u + (pkt_acc % 1000u < 100u ? 1u : 0u); }
                while (owed >= pkt) {
                    if (sc->irq_pumps && rnd() % 8u == 0) pump_all(PICO_AUDIO_SPDIF_RING_FRAMES);
                    if (fill_of() + pkt <= WRITABLE_FRAMES) write_all(pkt);
                    owed -= pkt;
                }
            } else if ((int32_t)(slots[0].inst.wr_frames - slots[0].inst.consumed_frames) < 0) {
                pump_all(192u);
            }
            if (sc->irq_pumps && rnd() % 4u == 0) pump_all(PICO_AUDIO_SPDIF_RING_FRAMES);
            // Streaming passes up to 2 ms; idle ones as the DSPi's (the idle
            // pump acts once the reader is past the writer: within 64 frames).
            dt = streaming ? rnd_in(50, 2000) : rnd_in(20, 500);
        }
        // The reader plays dt.
        double adv = (double)sc->rate * dt / 1e6 + frac;
        uint32_t n = (uint32_t)adv;
        frac = adv - n;
        if (streaming && !in_blackout) owed += (double)sc->rate * dt / 1e6;
        reader_advance(n);
        t_us += dt;
        // A pending op whose target has been played is done.
        if (pending_n) {
            uint32_t f = slots[0].inst.consumed_frames;
            uint32_t blk = audio_spdif_ubits_block_of(&slots[0].inst, f);
            uint64_t now_F = S.origin + 192u * (uint64_t)blk;
            if (now_F > pending_n + 192u) {
                S.src.apply_op = 0;
                S.src.apply_frame = 0;
                pending_n = 0;
            }
        }
    }
    (void)pending_seen;
    CHECK(stamped_min_gap > 0, "%s: model: the reader overran the stamped span", label);

    // Read back every slot.
    for (int k = 0; k < NSLOTS; k++) {
        readback_t rb = { 0 };
        read_back(k, &rb, 0, NULL);
        CHECK(rb.blocks > 50, "%s slot %d: only %" PRIu32 " blocks", label, k, rb.blocks);
        CHECK(rb.mismatched == 0 && rb.mixed == 0, "%s slot %d: %" PRIu32 " blocks differ from the table (%" PRIu32 " mixed)",
              label, k, rb.mismatched, rb.mixed);
        CHECK(rb.bad_parity == 0, "%s slot %d: %" PRIu32 " parity errors", label, k, rb.bad_parity);
        CHECK(rb.bad_cells == 0, "%s slot %d: bad biphase cells", label, k);
        CHECK(rb.bad_spacing == 0, "%s slot %d: %" PRIu32 " blocks not 192 frames apart", label, k, rb.bad_spacing);
        if (k < 2 && S.filler_on && !sc->blackouts)
            CHECK(rb.zero_blocks <= 1, "%s slot %d: %" PRIu32 " zero blocks with the filler on", label, k, rb.zero_blocks);
        if (k < 2 && sc->blackouts)
            CHECK(rb.zero_blocks > 0 && rb.data_blocks > 0, "%s slot %d: blackout zeros %" PRIu32 " data %" PRIu32,
                  label, k, rb.zero_blocks, rb.data_blocks);
        if (k == 0)
            printf("  %-34s %6" PRIu32 " blocks: %6" PRIu32 " carried, %4" PRIu32 " zero; asked %" PRIu32
                   " (refused %" PRIu32 "), prefilled %" PRIu32 "\n", label, rb.blocks, rb.data_blocks,
                   rb.zero_blocks, S.asked, S.asked_refused, S.prefills);
    }
    if (S.filler_on && !sc->blackouts)
        CHECK(S.asked_refused == 0, "%s: the same-pass filler was refused %" PRIu32 " times", label, S.asked_refused);
    if (!sc->start_frame) {
        uint32_t bad = check_legacy_identity(2);
        CHECK(bad == 0, "%s: slot not carrying differs from the old stampers in %" PRIu32 " frames", label, bad);
    }
    // Ops at the minimum lead: every scheduled op is carried by at least one
    // complete superframe on the wire.
    if (sc->ops) {
        uint32_t n_ops = ops_scheduled - ops_before;
        // Collect every carried apply_frame from the wire of slot 0.
        uint32_t carried = 0;
        slot_t *s = &slots[0];
        bool cells_ok;
        (void)decode_wire(0, &cells_ok);
        uint64_t seen[4096];
        uint32_t nseen = 0;
        for (uint32_t i = 0; i + 384u <= 2u * s->wire_n; i += 2) {
            if ((rxw[i] & 0xFu) != SYNC_B) continue;
            uint8_t ub[48];
            spdif_rx_collect_ubits(&rxw[i], ub);
            alink_spdif_sf_t sf;
            if (alink_spdif_sf_unpack(ub, &sf) != ALINK_SPDIF_OK || !sf.apply_frame) continue;
            if (sf.frame + 192u > sf.apply_frame) continue;
            if (nseen == 0 || seen[nseen - 1] != sf.apply_frame) {
                if (nseen < 4096) seen[nseen++] = sf.apply_frame;
                carried++;
            }
        }
        // The last op may still be pending past the end of the capture.
        CHECK(carried + 1u >= n_ops && n_ops > 5, "%s: %" PRIu32 " of %" PRIu32 " ops at the minimum lead carried",
              label, carried, n_ops);
        ops_carried += carried;
    }
}

// The fill rules in a steady stream.
static void test_fill_rules(void) {
    memset(&S, 0, sizeof(S));
    S.src = (alink_spdif_src_t){ .rate = 1, .sender = 1, .epoch = 1 };
    audio_spdif_ubits_set_filler(NULL);
    slots_init(0, false);
    // Stream a while: 48-frame packets, fill ~2 ms.
    for (int i = 0; i < 400; i++) {
        recover_all(96);
        while (fill_of() < 96) write_all(48);
        reader_advance(48);
    }
    audio_spdif_instance_t *inst = &slots[0].inst;
    uint32_t rb = audio_spdif_ubits_block_of(inst, inst->consumed_frames);
    uint32_t wb = audio_spdif_ubits_block_of(inst, inst->wr_frames);
    uint32_t sb = audio_spdif_ubits_block_of(inst, inst->erased_frames - 1u);
    uint8_t sf[48];
    memset(sf, 0x5A, sizeof(sf));
    CHECK(audio_spdif_ubits_fill(rb, sf) == AUDIO_SPDIF_UBITS_FROZEN, "reader's block refused");
    CHECK(audio_spdif_ubits_fill(wb, sf) == AUDIO_SPDIF_UBITS_FROZEN, "writer's block refused");
    CHECK(audio_spdif_ubits_fill(sb, sf) == AUDIO_SPDIF_UBITS_FROZEN, "erase-ahead block refused");
    CHECK(audio_spdif_ubits_fill(rb + 4u, sf) == AUDIO_SPDIF_UBITS_EARLY, "block reader + 4 too early");
    CHECK(audio_spdif_ubits_fill(rb - 1u, sf) == AUDIO_SPDIF_UBITS_FROZEN, "an older block refused");
    uint32_t first, last;
    CHECK(audio_spdif_ubits_window(&first, &last) && first == sb + 1u && last == rb + 3u,
          "window %" PRIu32 "..%" PRIu32 " (stamped %" PRIu32 ", reader %" PRIu32 ")", first, last, sb, rb);
    uint32_t rdr = ~0u;
    CHECK(audio_spdif_ubits_reader(&rdr) && rdr == rb, "reader's block %" PRIu32 " (%" PRIu32 ")", rdr, rb);
    CHECK(audio_spdif_ubits_fill(sb + 1u, sf) == AUDIO_SPDIF_UBITS_OK, "first unstamped block filled");
    CHECK(audio_spdif_ubits_fill(sb + 1u, sf) == AUDIO_SPDIF_UBITS_OK, "refill before its stamp");
    // Its first stamp freezes it: then refused, and every stamper uses it.
    while (audio_spdif_ubits_block_of(inst, inst->erased_frames - 1u) != sb + 1u) { write_all(48); reader_advance(48); }
    CHECK(audio_spdif_ubits_fill(sb + 1u, sf) == AUDIO_SPDIF_UBITS_FROZEN, "frozen once stamped");
    // A full-ring pump (the idle pump / blackout): the stamped frontier passes
    // the table's end, so the window closes.
    pump_all(PICO_AUDIO_SPDIF_RING_FRAMES);
    uint32_t f2, l2;
    bool open = audio_spdif_ubits_window(&f2, &l2);
    rb = audio_spdif_ubits_block_of(inst, inst->consumed_frames);
    sb = audio_spdif_ubits_block_of(inst, inst->erased_frames - 1u);
    CHECK((sb == rb + 3u && !open) || (sb == rb + 2u && open && f2 == l2 && f2 == rb + 3u),
          "full ring ahead: frontier block %" PRIu32 " reader %" PRIu32 " window %d", sb, rb, open);
    // Review fwbc F4: the reader's block is known whether or not the window
    // is open (a source stages ahead of it in every pass), at every reader
    // phase of a block under a full-ring pump.
    int closed = 0;
    for (int i = 0; i < 2 * 192; i++) {
        reader_advance(1);
        pump_all(PICO_AUDIO_SPDIF_RING_FRAMES);
        rb = audio_spdif_ubits_block_of(inst, inst->consumed_frames);
        rdr = ~0u;
        bool live = audio_spdif_ubits_reader(&rdr);
        if (!audio_spdif_ubits_window(&f2, &l2)) closed++;
        if (!live || rdr != rb) { CHECK(false, "reader's block with the window closed: %" PRIu32 " (%" PRIu32 ")", rdr, rb); break; }
    }
    CHECK(closed > 192, "the window is closed for most of a block under a full-ring pump (%d of 384 frames)", closed);
    // No carrying slot enabled: the frontier checks are off, entries still freeze.
    for (int k = 0; k < NSLOTS; k++) slots[k].inst.enabled = false;
    CHECK(!audio_spdif_ubits_window(&f2, &l2), "no window without an enabled carrying slot");
    CHECK(!audio_spdif_ubits_reader(&rdr), "no reader's block without an enabled carrying slot");
    CHECK(audio_spdif_ubits_fill(sb, sf) == AUDIO_SPDIF_UBITS_FROZEN, "a stamped entry stays frozen");
    for (int k = 0; k < NSLOTS; k++) slots[k].inst.enabled = true;
}

// An emptied table emits zero user bits; an output with no table emits zeros.
static void test_cleared_table(void) {
    memset(&S, 0, sizeof(S));
    S.src = (alink_spdif_src_t){ .rate = 1, .sender = 2, .epoch = 9 };
    S.origin = 0;
    audio_spdif_ubits_set_filler(test_filler);
    slots_init(0, true);
    for (int i = 0; i < 200; i++) {
        recover_all(96);
        while (fill_of() < 96) write_all(48);
        reader_advance(48);
    }
    audio_spdif_ubits_set_filler(NULL);
    audio_spdif_ubits_clear();
    audio_spdif_instance_t *inst = &slots[0].inst;
    uint32_t frontier = audio_spdif_ubits_block_of(inst, inst->erased_frames - 1u);
    for (int i = 0; i < 300; i++) {
        recover_all(96);
        while (fill_of() < 96) write_all(48);
        reader_advance(48);
    }
    // Every block whose first frame came after the clear is all zero.
    for (int k = 0; k < 2; k++) {
        slot_t *s = &slots[k];
        uint32_t nz = 0, z = 0;
        for (uint32_t i = 0; i + 1u < s->wire_n; i++) {
            uint32_t f = s->wire_first + i;
            uint32_t blk = audio_spdif_ubits_block_of(&s->inst, f);
            if ((int32_t)(blk - frontier) <= 0) continue;
            bool ok = true;
            uint32_t a = rx_word(&s->wire[2u * i], s->inst.pio_bmc, &ok);
            uint32_t b = rx_word(&s->wire[2u * i + 1u], s->inst.pio_bmc, &ok);
            if (((a | b) >> 29) & 1u) nz++; else z++;
        }
        CHECK(nz == 0 && z > 1000, "slot %d: %" PRIu32 " frames with U set after the clear (%" PRIu32 " zero)", k, nz, z);
    }
}

// The collector against the codec's bit order (spec 12.1): subframe i's bit 29.
static void test_collector_order(void) {
    alink_spdif_src_t src = { .rate = 1, .sender = 0x42, .sender2 = 0x37, .epoch = 3,
                              .apply_frame = 1000000, .apply_op = 9 };
    uint8_t sf[48], got[48];
    alink_spdif_sf_build(&src, 960000, sf);
    uint32_t words[384];
    for (unsigned i = 0; i < 384; i++)
        words[i] = (rnd() & ~(1u << 29)) | (alink_spdif_ubit(sf, i) << 29);
    spdif_rx_collect_ubits(words, got);
    CHECK(memcmp(got, sf, 48) == 0, "collector order");
    static const unsigned first16[16] = { 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
    for (unsigned i = 0; i < 16; i++) CHECK(((words[i] >> 29) & 1u) == first16[i], "first bits %u", i);
}

int main(void) {
    printf("test_spdif_ubits\n");
    build_lookup();
    for (int k = 0; k < NSLOTS; k++) slots[k].wire = calloc(2u * MAX_WIRE, sizeof(spdif_subframe_t));
    audio_mark = calloc(MAX_WIRE, 1);
    if (!audio_mark) return 2;
    test_collector_order();
    test_fill_rules();
    test_cleared_table();

    static const uint32_t rates[3] = { 44100, 48000, 96000 };
    char label[64];
    for (int r = 0; r < 3; r++) {
        for (int bmc = 0; bmc < 2; bmc++) {
            rng_state = 0x1234567u + (uint32_t)r * 77u + (uint32_t)bmc;
            scen_t a = { .rate = rates[r], .mode = MODE_FILLER, .bmc = bmc, .irq_pumps = true, .ops = true,
                         .seconds_x10 = 30 };
            snprintf(label, sizeof(label), "%u filler%s irq-pumps ops", rates[r], bmc ? " bmc" : "");
            run_scenario(&a, label);
            scen_t b = { .rate = rates[r], .mode = MODE_FILLER, .bmc = bmc, .blackouts = true, .irq_pumps = true,
                         .seconds_x10 = 30 };
            snprintf(label, sizeof(label), "%u filler%s blackouts", rates[r], bmc ? " bmc" : "");
            run_scenario(&b, label);
            scen_t c = { .rate = rates[r], .mode = MODE_PREFILL, .bmc = bmc, .blackouts = true, .irq_pumps = true,
                         .seconds_x10 = 30 };
            snprintf(label, sizeof(label), "%u prefill%s blackouts", rates[r], bmc ? " bmc" : "");
            run_scenario(&c, label);
        }
        // Across the 32-bit frame counter wrap (24.8 h at 48 kHz).
        rng_state = 0xC0FFEEu + (uint32_t)r;
        scen_t w = { .rate = rates[r], .mode = MODE_FILLER, .bmc = true, .irq_pumps = true,
                     .start_frame = 0u - 3u * rates[r] / 10u, .seconds_x10 = 8 };
        snprintf(label, sizeof(label), "%u filler bmc 2^32 wrap", rates[r]);
        run_scenario(&w, label);
    }
    printf("  rogue fills: %" PRIu32 " accepted (unstamped), %" PRIu32 " refused\n", rogue_ok, rogue_refused);
    CHECK(rogue_ok > 100 && rogue_refused > 1000, "rogue fills exercised");
    printf("  ops at the minimum lead carried: %" PRIu32 " of %" PRIu32 " scheduled\n", ops_carried, ops_scheduled);
    if (failures) {
        printf("FAILED: %d\n", failures);
        return 1;
    }
    printf("  all OK\n");
    return 0;
}
