/*
 * bench_check.h - alink bench: receiver-side checks
 *
 * Three consumers of the raw bit ring an alink_rx_N program fills:
 *
 *  - bench_acquire(): runs LK1's reference receiver (alink_decoder.c,
 *    unchanged) over a snapshot of the ring until it is locked, and returns
 *    where frame F starts, its running disparity and the superframe.
 *  - bench_bert_*: from that state, regenerates the stream the bench
 *    transmitter sends (bench_gen.c) and compares every received bit with
 *    it: a bit error rate tester at line rate.  It follows bit slips of up
 *    to 3 bits and declares loss of sync after a run of bad frames.
 *  - bench_ref_*: feeds the same ring to a reference receiver as fast as
 *    the CPU allows, for the protocol counters of spec 8.9 (EXT_ALINK_STATUS
 *    and EXT_ALINK_DIAG). When it falls a ring behind it restarts at the
 *    oldest data and counts a gap, so its counters cover a known fraction
 *    of the stream.
 *
 * Positions are absolute bit numbers since the ring started (bit 0 = the
 * MSB of the first word the DMA wrote).  Portable C11, no SDK.
 */

#ifndef BENCH_CHECK_H
#define BENCH_CHECK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bench_gen.h"
#include "alink_decoder.h"

// ---------------------------------------------------------------------------
// Ring view
// ---------------------------------------------------------------------------

typedef struct {
    const volatile uint32_t *ring;
    uint32_t mask;          // ring words - 1
} bench_ring_t;

// The 32 bits starting at absolute bit `pos` (MSB first).
static inline uint32_t bench_ring_bits32(const bench_ring_t *r, uint64_t pos) {
    uint32_t wi = (uint32_t)(pos >> 5), o = (uint32_t)pos & 31u;
    uint32_t w0 = r->ring[wi & r->mask];
    if (!o) return w0;
    return (w0 << o) | (r->ring[(wi + 1u) & r->mask] >> (32u - o));
}

// ---------------------------------------------------------------------------
// Acquisition
// ---------------------------------------------------------------------------

typedef struct {
    uint64_t frame;         // F of the frame found
    uint64_t start;         // absolute bit of its comma's first bit
    unsigned rd;            // running disparity before the comma
    alink_sf_t sf;          // the superframe in force
    alink_rx_stats_t stats; // the snapshot decoder's counters
} bench_acq_t;

// Bits a snapshot needs to guarantee one lock: 64 frames plus a search
// margin, at the longest frame of the class.
uint32_t bench_acquire_bits(unsigned cls);

// Decode `nbits` bits (words[0] MSB = absolute bit first_bit) with a fresh
// reference receiver of class cls; true with *out filled at the first
// locked frame that starts at a comma with a good head.  rx is scratch.
bool bench_acquire(const uint32_t *words, uint64_t first_bit, size_t nbits, unsigned cls,
                   alink_rx_t *rx, bench_acq_t *out);

// ---------------------------------------------------------------------------
// Bit error rate tester
// ---------------------------------------------------------------------------

#define BENCH_BERT_BAD_LIMIT    16u     // consecutive bad frames that mean loss of sync
#define BENCH_BERT_SLIP_MAX     3

typedef struct {
    uint64_t bits;          // bits compared
    uint64_t errors;        // bits that differed
    uint64_t frames;        // frames compared
    uint64_t err_frames;    // frames with at least one error
    uint64_t bad_frames;    // frames too damaged to follow (more than 1/8 of bits wrong)
    uint64_t skipped_frames;// frames that left the ring before they were compared
    uint32_t slips;         // re-alignments by 1..3 bits
    int32_t  slip_bits;     // their sum
    uint32_t syncs;         // acquisitions taken
    uint32_t losses;        // losses of sync (BENCH_BERT_BAD_LIMIT bad frames)
} bench_bert_stats_t;

typedef struct {
    bench_gen_t gen;        // the expected stream
    bool     synced;
    uint64_t pos;           // absolute bit where the next expected frame starts
    unsigned bad_run;
    bench_bert_stats_t st;
} bench_bert_t;

void bench_bert_init(bench_bert_t *b);

// Start comparing at an acquired frame.  The stream description comes from
// the superframe; false if the superframe does not describe a bench stream
// (an apply in flight, or an unknown layout).
bool bench_bert_start(bench_bert_t *b, const bench_acq_t *a);

// Compare the frames that lie wholly in [oldest, newest), at most
// max_frames; frames older than `oldest` are skipped (counted, not
// compared).  Returns the frames consumed.  Does nothing unless synced.
unsigned bench_bert_run(bench_bert_t *b, const bench_ring_t *r, uint64_t oldest, uint64_t newest,
                        unsigned max_frames);

// ---------------------------------------------------------------------------
// Reference receiver runner
// ---------------------------------------------------------------------------

typedef struct {
    alink_rx_t rx;
    uint8_t  cls, pay;
    uint64_t base;          // absolute bit of the current decoder's bit 0
    uint64_t pos;           // next absolute bit to push (a word boundary)
    bool     restart;       // restart at the next run (bench_ref_restart)
    alink_rx_stats_t acc;   // counters of finished decoder instances
    uint32_t gaps;          // restarts after falling a ring behind
    uint64_t bits;          // bits pushed in total
    uint32_t sample_errors; // OK banks whose samples differ from the bench payload
    uint32_t events_lock, events_loss;
    uint64_t last_frame;    // F of the last delivered frame
    uint8_t  comma_offset;  // bit offset of the last comma in the 32-bit words
    int32_t  expect[ALINK_MAX_CHANNELS];
} bench_ref_t;

void bench_ref_init(bench_ref_t *r, unsigned cls, unsigned pay);

// Push whole words from [pos, newest), at most max_bits.  If pos has
// fallen below `oldest`, the current instance's counters are kept, a gap is
// counted and a fresh receiver starts halfway between oldest and newest.
// Returns the bits pushed.  bench_ref_restart() forces such a restart (the
// caller found that the writer overtook data it had just pushed).
size_t bench_ref_run(bench_ref_t *r, const bench_ring_t *ring, uint64_t oldest, uint64_t newest,
                     size_t max_bits);

static inline void bench_ref_restart(bench_ref_t *r) { r->restart = true; }

// Counters of every instance so far.
void bench_ref_stats(const bench_ref_t *r, alink_rx_stats_t *out);

void bench_stats_add(alink_rx_stats_t *acc, const alink_rx_stats_t *s);

#endif // BENCH_CHECK_H
