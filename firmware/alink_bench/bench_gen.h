/*
 * bench_gen.h - alink bench: real-time frame generator and bit sink
 *
 * The bench image (WP LK3) sends and checks alink streams at line rate. The
 * reference encoder alink_tx_frame() packs bit by bit and recomputes PRBS and
 * CRC every frame, which costs far more than a core has at L2/L3.  This
 * generator produces the same bits, using only LK1's primitives: the bench
 * payload (PRBS-7 or a fixed test vector) is the same in every frame, so its
 * banks are encoded once per starting running disparity and copied, and only
 * the comma and the head (HDR, CTL, CRC8) are encoded per frame.  The host
 * tests prove the output identical to alink_tx_frame() at every class, rate
 * and channel count.
 *
 * It also injects TX-side impairments at exact frame positions (comma
 * damage, payload bit flips, bit slips, idle frames) so that the receiver's
 * flywheel, slip window and loss rules can be exercised on real wires.
 *
 * Portable C11, no SDK: the firmware and the host tests compile it alike.
 */

#ifndef BENCH_GEN_H
#define BENCH_GEN_H

#include <stdbool.h>
#include <stdint.h>

#include "alink_codec.h"

// ---------------------------------------------------------------------------
// Bit sink: MSB-first bits into a power-of-two ring of 32-bit words
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t *ring;
    uint32_t  mask;         // ring words - 1 (ring size a power of two)
    uint64_t  words;        // whole words emitted so far (absolute)
    uint32_t  cur;          // partial word, MSB first
    unsigned  cur_n;        // bits in cur (0..31)
} bench_sink_t;

void bench_sink_init(bench_sink_t *s, uint32_t *ring, uint32_t ring_words);

// Absolute bits emitted, the partial word included.
static inline uint64_t bench_sink_bits(const bench_sink_t *s) {
    return s->words * 32u + s->cur_n;
}

// Append the low n bits of v (0 <= n <= 32), most significant first.
void bench_sink_put(bench_sink_t *s, uint32_t v, unsigned n);

// Append nbits bits from src, MSB first from src[0].
void bench_sink_put_bits(bench_sink_t *s, const uint32_t *src, unsigned nbits);

// ---------------------------------------------------------------------------
// Generator
// ---------------------------------------------------------------------------

#define BENCH_PAY_PRBS      0u      // superframe flag b4, PRBS-7 payload (spec 5.8)
#define BENCH_PAY_VECTOR    1u      // fixed test vector bench_vector_sample()

// TX-side impairments (bench_gen_inject).  Each one acts on the next n frames
// the generator encodes, or once for SLIP.
#define BENCH_IMP_COMMA     1u      // flip one bit of the comma: a flywheel frame each
#define BENCH_IMP_FLIP      2u      // flip one bit of bank 0's first symbol: a CRC-16 failure each
#define BENCH_IMP_SLIP      3u      // n = -3..+3: drop (n < 0) or add (n > 0) |n| bits before the next comma
#define BENCH_IMP_IDLE      4u      // send n whole frames as idles (no comma, no head)

// Longest payload: 8 banks of 26 symbols.
#define BENCH_PAY_MAX_BITS  (10u * 8u * ALINK_BANK_SYMBOLS(ALINK_BANK_CHANNELS))
#define BENCH_PAY_WORDS     ((BENCH_PAY_MAX_BITS + 31u) / 32u)

typedef struct {
    uint32_t comma;         // frames whose comma was damaged
    uint32_t flip;          // frames with a flipped payload bit
    uint32_t slip;          // slips applied
    int32_t  slip_bits;     // sum of applied slips (bits)
    uint32_t idle;          // frames sent as idles
} bench_imp_stats_t;

typedef struct {
    // Stream description (superframe content).
    uint8_t  cls, rate, channels, pay;
    uint8_t  sender, hops;
    uint8_t  flags;         // superframe flags; PRBS is set from pay
    // Cached payload (banks and their CRCs), one per running disparity at
    // its start, and the disparity after it.
    uint32_t pay_bits[2][BENCH_PAY_WORDS];
    unsigned pay_nbits;
    uint8_t  pay_rd_end[2];
    unsigned layout;        // alink_layout_symbols(channels)
    unsigned sym_const;     // symbols per frame when constant (48/96 kHz), else 0
    // State.
    uint64_t frame;         // F of the next frame
    unsigned rd;
    bool     started;
    uint8_t  sf[ALINK_SF_LEN];
    // Pending impairments.
    uint16_t imp_comma, imp_flip, imp_idle;
    int8_t   imp_slip;
    bench_imp_stats_t imp;
} bench_gen_t;

// The test-vector sample of channel ch (24-bit, both signs, distinct per
// channel and byte): what BENCH_PAY_VECTOR sends and the receiver expects.
int32_t bench_vector_sample(unsigned ch);

// Set up a stream starting at frame `first` with running disparity rd.
// False for an unknown class or rate or a C that does not fit.
bool bench_gen_init(bench_gen_t *g, unsigned cls, unsigned rate, unsigned channels,
                    unsigned pay, uint64_t first, uint8_t sender);

// Restart at frame `first` with running disparity rd (receiver side: the
// state recovered from a decoded comma), keeping the stream description.
void bench_gen_seek(bench_gen_t *g, uint64_t first, unsigned rd);

// Bits in the next frame (without impairments).
unsigned bench_gen_frame_bits(const bench_gen_t *g);

// Encode the next frame into s (s == NULL: advance the state only) and
// return the number of bits it emitted.
unsigned bench_gen_frame(bench_gen_t *g, bench_sink_t *s);

// The next frame as the receiver-side checker compares it: 40 head bits
// (comma, HDR, CTL, CRC8, MSB first in head[0..1]), the cached payload for
// the disparity after the head, then idle_nbits of D21.5 (alternating bits
// from 1).  Advances the state like bench_gen_frame() and returns the
// frame's bits.  Impairments are not applied (they belong to a sender).
typedef struct {
    uint32_t head[2];
    unsigned comma_rd;
    const uint32_t *pay;
    unsigned pay_nbits;
    unsigned idle_nbits;
} bench_parts_t;

unsigned bench_gen_parts(bench_gen_t *g, bench_parts_t *p);

// Queue an impairment.  False for an unknown kind or a slip outside -3..+3.
bool bench_gen_inject(bench_gen_t *g, unsigned kind, int n);

#endif // BENCH_GEN_H
