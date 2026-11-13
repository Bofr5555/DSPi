/*
 * sender_link.h - DSPi link sender (WP LK4): the portable encoder core
 *
 * The sender emits one link stream (link_protocol_spec.md) that every link
 * output's DMA channel reads from the same word ring.  This is the part of
 * the sender that has no Pico SDK in it, so the host tests compile it as is:
 *
 *  - a table-driven frame encoder that produces exactly the bits of LK1's
 *    reference link_tx_frame() (same superframe, apply and RATE rules, the
 *    state is LK1's link_tx_t), at a cost a core can afford at L3;
 *  - the word sink the encoder writes into: a power-of-two ring, optionally
 *    with a parallel ring for the SYNC pin (a pulse that starts on the first
 *    bit of a chosen frame's comma, sent by its own PIO state machine in
 *    step with the links);
 *  - a log of frame start positions, so the TX side can tell which frame is
 *    on the wire from the DMA read position;
 *  - the sender's scheduling rules (RATE at a multiple of 32, at least
 *    SND_RATE_LEAD frames ahead, value = new C, one apply at a time) and the
 *    byte layouts and refusal rules of EXT_LINK_TX and EXT_LINK_CTRL_MSG
 *    (extension_channel_spec.md 2.8.8, 2.8.9);
 *  - the 1 kHz test tone.
 *
 * Portable C11, no heap.  The tables are built at run time into a struct
 * the caller places (RAM on the target), so nothing here adds .data.
 */

#ifndef SENDER_LINK_H
#define SENDER_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "link_codec.h"

// Frames between the next encoded frame and a RATE the sender schedules
// itself (SYS2 3.3: 50 ms at 48 kHz; a multiple of 32).
#define SND_RATE_LEAD           2400u

// Link outputs on the v1 carrier (EXT_LINK_INFO.max_links).
#define SND_MAX_LINKS           8u

// Receiver-side argument limits of the apply ops (extension_channel_spec.md
// 2.8.7; the receivers are RP2350 DSPis).
#define SND_OP_OUTPUTS          9u      // 8 output slots + the PDM sub
#define SND_OP_DELAY_MAX        2047    // MAX_DELAY_SAMPLES - 1 on RP2350
#define SND_OP_PRESET_SLOTS     10u

// ---------------------------------------------------------------------------
// Tables
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t enc[2][256];       // data byte at RD-/RD+: symbol (bits 9..0) | RD after (bit 15)
    uint16_t comma[2];          // K28.5 at RD-/RD+, same format
    uint16_t crc16[256];        // CRC-16/CCITT-FALSE, MSB first
    uint8_t  crc8[256];         // CRC-8 poly 0x07
    uint8_t  prbs[3u * LINK_MAX_CHANNELS];  // the PRBS-7 payload (spec 5.8)
} snd_tables_t;

void snd_tables_init(snd_tables_t *t);

// ---------------------------------------------------------------------------
// Word sink
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t *ring;             // power-of-two ring of words, MSB first on the line
    uint32_t  mask;             // ring words - 1
    uint32_t *sync;             // parallel ring for the SYNC pin, NULL = none
    uint64_t  words;            // whole words emitted (absolute)
    uint32_t  cur;              // pending bits, the first one in bit 31
    unsigned  n;                // pending bit count, 0..31
    uint64_t  pulse_from;       // SYNC high on absolute bits [pulse_from, pulse_to)
    uint64_t  pulse_to;
} snd_sink_t;

void snd_sink_init(snd_sink_t *s, uint32_t *ring, uint32_t ring_words, uint32_t *sync);

// Absolute bits emitted, the pending ones included.
static inline uint64_t snd_sink_bits(const snd_sink_t *s) {
    return s->words * 32u + s->n;
}

static inline uint32_t snd_pulse_mask(const snd_sink_t *s, uint64_t b0) {
    if (s->pulse_to <= b0 || s->pulse_from >= b0 + 32u) return 0;
    unsigned lo = s->pulse_from > b0 ? (unsigned)(s->pulse_from - b0) : 0u;
    unsigned hi = s->pulse_to < b0 + 32u ? (unsigned)(s->pulse_to - b0) : 32u;
    return (0xFFFFFFFFu >> lo) & (hi >= 32u ? 0xFFFFFFFFu : ~(0xFFFFFFFFu >> hi));
}

static inline void snd_sink_emit(snd_sink_t *s, uint32_t w) {
    uint32_t i = (uint32_t)s->words & s->mask;
    s->ring[i] = w;
    if (s->sync) {
        // Every word is rewritten (the ring is reused); the mask is worked
        // out only while a pulse is due.
        uint64_t b0 = s->words * 32u;
        s->sync[i] = b0 < s->pulse_to ? snd_pulse_mask(s, b0) : 0u;
    }
    s->words++;
}

// Append the low nb bits of v (1 <= nb <= 32; v < 2^nb), first bit first.
static inline void snd_put(snd_sink_t *s, uint32_t v, unsigned nb) {
    unsigned fr = 32u - s->n;
    if (nb < fr) {
        s->cur |= v << (fr - nb);
        s->n += nb;
    } else {
        unsigned r = nb - fr;
        snd_sink_emit(s, s->cur | (r < 32u ? v >> r : 0u));
        s->cur = r ? v << (32u - r) : 0u;
        s->n = r;
    }
}

// nbits of D21.5 idles (1010..., starting with 1).
void snd_put_idles(snd_sink_t *s, unsigned nbits);

// Write the SYNC ring for absolute words [from, to) (no-op without one).
void snd_sync_fill(snd_sink_t *s, uint64_t from, uint64_t to);

// The hot-path writer: the sink's state in locals for one frame, words
// stored without the per-word SYNC work, which SND_W_END() then does for
// the frame's words in one pass.  nb <= 30.
#define SND_W_BEGIN(s)                                                  \
    uint32_t *const w_ring_ = (s)->ring;                                \
    const uint32_t w_mask_ = (s)->mask;                                 \
    const uint64_t w_start_ = (s)->words;                               \
    uint32_t w_idx_ = (uint32_t)w_start_, w_cur_ = (s)->cur;            \
    unsigned w_n_ = (s)->n
#define SND_W_PUT(v, nb) do {                                           \
        uint32_t v_ = (v);                                              \
        unsigned nb_ = (nb), fr_ = 32u - w_n_;                          \
        if (nb_ < fr_) {                                                \
            w_cur_ |= v_ << (fr_ - nb_);                                \
            w_n_ += nb_;                                                \
        } else {                                                        \
            unsigned r_ = nb_ - fr_;                                    \
            w_ring_[w_idx_++ & w_mask_] = w_cur_ | (v_ >> r_);          \
            w_cur_ = r_ ? v_ << (32u - r_) : 0u;                        \
            w_n_ = r_;                                                  \
        }                                                               \
    } while (0)
#define SND_W_END(s) do {                                               \
        (s)->cur = w_cur_;                                              \
        (s)->n = w_n_;                                                  \
        (s)->words = w_start_ + (uint32_t)(w_idx_ - (uint32_t)w_start_);\
        snd_sync_fill((s), w_start_, (s)->words);                       \
    } while (0)

// ---------------------------------------------------------------------------
// Frame start log (which frame is on the wire)
// ---------------------------------------------------------------------------

#define SND_LOG_LEN 512u        // a power of two; more frames than the ring holds

typedef struct {
    uint64_t frame[SND_LOG_LEN];
    uint64_t bit[SND_LOG_LEN];  // absolute sink bit of the frame's first bit
    uint32_t head;              // entries written
    uint32_t tail;              // entries consumed by snd_log_wire()
    uint64_t wire_frame;        // last result of snd_log_wire()
} snd_log_t;

void snd_log_init(snd_log_t *l, uint64_t first_frame);

static inline void snd_log_push(snd_log_t *l, uint64_t frame, uint64_t bit) {
    uint32_t i = l->head & (SND_LOG_LEN - 1u);
    l->frame[i] = frame;
    l->bit[i] = bit;
    l->head++;
}

// The frame whose bits include absolute bit `wire_bit` (the frame on the
// wire when the DMA has read that far).  Frames the wire has passed are
// dropped from the log.
uint64_t snd_log_wire(snd_log_t *l, uint64_t wire_bit);

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

// Encode frame tx->frame into s: the bits link_tx_frame() produces for the
// same state and samples (tx->channels samples, ignored while the PRBS flag
// is in force), and advance tx exactly as it does.  Records the frame's
// start in log (may be NULL).  Never fails: the caller keeps the ring ahead
// of its reader.
void snd_frame(link_tx_t *tx, const snd_tables_t *t, const int32_t *samples, snd_sink_t *s,
               snd_log_t *log);

// The "idle only" test pattern: frame tx->frame as D21.5 only (no comma, no
// head), with the encoder state advanced as for a real frame.
void snd_idle_frame(link_tx_t *tx, snd_sink_t *s, snd_log_t *log);

// Bits of frame tx->frame.
static inline unsigned snd_frame_bits(const link_tx_t *tx) {
    return 10u * link_frame_symbols(tx->cls, tx->rate, tx->frame);
}

// ---------------------------------------------------------------------------
// Scheduling
// ---------------------------------------------------------------------------

// The sender's own layout change (EXT_LINK_TX rate_code / channels, or a USB
// rate change): RATE at the first superframe boundary at least SND_RATE_LEAD
// frames after the next encoded frame, value = channels when it changes, else
// 0.  False when an apply is pending or the layout does not fit the class
// (nothing scheduled); *at is the target frame on success.
bool snd_schedule_layout(link_tx_t *tx, unsigned rate, unsigned channels, uint64_t *at);

// Whether (op, arg, value) is a valid apply for the receivers (ops 1-5, 7, 8
// of extension_channel_spec.md 2.8.7; RATE is checked by link_tx_schedule).
bool snd_op_args_ok(uint8_t op, uint8_t arg, int16_t value);

// ---------------------------------------------------------------------------
// EXT_LINK_TX (0x8007), 16 bytes
// ---------------------------------------------------------------------------

#define SND_TX_LEN          16u
#define SND_CTRL_OUT_LEN    16u
#define SND_CTRL_IN_LEN     24u

#define SND_TP_AUDIO        0u
#define SND_TP_PRBS         1u
#define SND_TP_IDLE         2u
#define SND_TP_TONE         3u

typedef struct {
    uint16_t link_enable;
    uint8_t  cls;
    uint8_t  channels;
    uint8_t  rate;
    uint8_t  test_pattern;
    uint8_t  slot_map_id;
    uint8_t  sync_pin;
    uint16_t txdis_mask;
    uint8_t  flags;             // bit 0 system mute
} snd_txcfg_t;

// The build defaults of 2.8.8.
void snd_txcfg_default(snd_txcfg_t *c);

// A Pico 2 header GPIO (0-22, 26-28), or 0xFF.
static inline bool snd_pin_ok(uint8_t p) {
    return p == 0xFF || p <= 22u || (p >= 26u && p <= 28u);
}

// Parse and check an OUT payload on its own (ranges, reserved bytes, the
// protocol capacity of 5.7, bits at or above max_links).  0 = accepted.
int snd_txcfg_parse(const uint8_t p[SND_TX_LEN], unsigned max_links, snd_txcfg_t *c);

// The IN form: cfg plus the IN-only los_mask and usb_alt.
void snd_txcfg_pack(const snd_txcfg_t *c, uint16_t los_mask, uint8_t usb_alt, uint8_t out[SND_TX_LEN]);

// ---------------------------------------------------------------------------
// EXT_LINK_CTRL_MSG (0x8008)
// ---------------------------------------------------------------------------

typedef struct {
    uint64_t frame;
    uint8_t  op, arg;
    int16_t  value;
    uint8_t  system_flags;
} snd_ctrl_t;

// Parse an OUT payload; checks the op's arguments, system_flags and the
// reserved bytes.  The lead, one-at-a-time and RATE rules are those of
// link_tx_schedule(), applied by the caller.  0 = accepted.
int snd_ctrl_parse(const uint8_t p[SND_CTRL_OUT_LEN], snd_ctrl_t *c);

// The IN form on a sender: the superframe content as the encoder would pack
// it now (layout, flags with the pending bits, the pending apply) and the
// low 32 bits of `sf_frame`, the first frame of the superframe on the wire.
void snd_ctrl_pack_in(const link_tx_t *tx, uint64_t sf_frame, uint8_t out[SND_CTRL_IN_LEN]);

// The superframe flags byte the encoder packs for tx (pending bits derived).
uint8_t snd_sf_flags(const link_tx_t *tx);

// ---------------------------------------------------------------------------
// Test tone
// ---------------------------------------------------------------------------

// 1 kHz at about -20 dBFS (24-bit), the same on every slot.
typedef struct {
    uint32_t phase;             // Q32 turns
    uint32_t step;
} snd_tone_t;

void snd_tone_set_rate(snd_tone_t *t, uint32_t fs);
int32_t snd_tone_next(snd_tone_t *t);

#endif // SENDER_LINK_H
