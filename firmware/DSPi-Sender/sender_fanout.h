/*
 * sender_fanout.h - DSPi alink sender (WP LK4): fan-out (slaved) mode core
 *
 * A fan-out is a sender slaved to an alink input (SYS2 1.3): it re-sends the
 * stream it receives on all its alink outputs, frame for frame, with the
 * same F, payload and layout, the superframe's hop count + 1 and the class
 * of the outgoing line, re-encoded with its own running disparity and
 * padding (alink_protocol_spec.md 7.2). Until it holds a valid superframe it
 * sends only D21.5 idles.
 *
 * This is the portable part: it follows the received frames in the raw bit
 * ring an alink_rx_N program fills (LK1's reference receiver acquires the
 * first lock, then a word-wise parser keeps up at line rate with the same
 * flywheel, slip and loss rules, spec 8.4-8.8) and writes the re-encoded
 * frames into the sender's word sink.  The engine (sender_tx.c) decides
 * where in the output stream the first frame goes, which sets the delay,
 * and servoes the clock to keep it.
 *
 * The superframe is re-sent byte by byte as it arrives, one byte per frame,
 * so the output never waits for a whole superframe: byte 5 becomes the
 * output class, byte 6 hops + 1, bytes 30-31 the CRC over the rewritten
 * bytes.  Corruption is never laundered: a byte that arrived in a bad head
 * is replaced by the previous superframe's, and if any byte was replaced or
 * the received CRC does not match, the re-sent CRC is made wrong too, so the
 * downstream receivers reject that superframe as they would have.  Banks are
 * copied with their received CRC bytes; a bank with a symbol that is not a
 * data code group goes out with a CRC that cannot match.
 *
 * Portable C11, no SDK, no heap.
 */

#ifndef SENDER_FANOUT_H
#define SENDER_FANOUT_H

#include <stdbool.h>
#include <stdint.h>

#include "alink_codec.h"
#include "alink_decoder.h"
#include "sender_alink.h"

// 10-bit symbol -> data byte (bits 7..0) | SND_DEC_DATA, or 0 for a symbol
// that is not a data code group (K codes and invalid symbols).
#define SND_DEC_DATA    0x100u

typedef struct {
    uint16_t dec[1024];
} snd_dectab_t;

void snd_dectab_init(snd_dectab_t *d);

// The input ring: absolute bit b is bit 31 - (b & 31) of word (b >> 5) & mask.
typedef struct {
    const volatile uint32_t *ring;
    uint32_t mask;              // ring words - 1
} snd_ring_t;

static inline uint32_t snd_ring_bits32(const snd_ring_t *r, uint64_t pos) {
    uint32_t wi = (uint32_t)(pos >> 5), o = (uint32_t)pos & 31u;
    uint32_t w0 = r->ring[wi & r->mask];
    if (!o) return w0;
    return (w0 << o) | (r->ring[(wi + 1u) & r->mask] >> (32u - o));
}

// States (EXT_ALINK_STATUS.state on a sender, describing the input).
#define SND_FO_OFF      0u
#define SND_FO_SEARCH   1u      // no lock: acquire with the reference receiver
#define SND_FO_LOCKED   2u

// Frame results.
#define SND_FO_R_OK         0x00u
#define SND_FO_R_COMMA      0x01u   // comma missing: flywheel
#define SND_FO_R_SLIP       0x02u   // comma 1-2 bits off: re-aligned
#define SND_FO_R_HEAD       0x04u   // bad head (non-data symbol or CRC-8)
#define SND_FO_R_COUNTER    0x08u   // HDR != F & 0xFF
#define SND_FO_R_LOSS       0x10u   // third failed frame in a row: lock lost
#define SND_FO_R_SF         0x20u   // a superframe was accepted at this frame
#define SND_FO_R_SF_ERR     0x40u   // a superframe was rejected at this frame

typedef struct {
    // Configuration.
    uint8_t  in_cls, out_cls;
    const snd_tables_t *t;
    const snd_dectab_t *d;

    // Input.
    uint8_t  state;
    uint64_t pos;               // absolute bit of the current frame's comma
    uint64_t frame;             // F of that frame
    uint8_t  rate, channels;    // its layout
    unsigned fails;             // consecutive failed frames
    bool     sf_valid;          // a valid superframe has been received
    alink_sf_t sf;              // the last valid one
    uint8_t  sf_bytes[ALINK_SF_LEN];    // this superframe's bytes as received
    uint32_t sf_good;           // bit k: byte k arrived in a good head
    bool     sf_skip;           // joined this superframe late: not judged
    uint8_t  last_sf[ALINK_SF_LEN];     // the last valid superframe's bytes
    // A RATE carried by the last valid superframe (spec 6.5, 8.5).
    bool     rate_pending;
    uint64_t rate_at;
    uint8_t  rate_new, c_new;

    // Output.
    bool     out_on;            // re-sending (else the engine sends idles)
    unsigned out_rd;
    uint16_t out_crc;           // CRC-16 over this superframe's re-sent bytes 0..29
    bool     out_taint;         // a re-sent byte was a substitute
    uint8_t  out_sf[ALINK_SF_LEN];
    uint32_t out_frames;        // frames re-sent
    uint32_t out_unfit;         // frames whose layout does not fit the output class (sent as idles)

    // Counters (alink_protocol_spec.md 8.9; EXT_ALINK_STATUS / DIAG on a sender).
    alink_rx_stats_t st;
} snd_fo_t;

void snd_fo_init(snd_fo_t *f, unsigned in_cls, unsigned out_cls, const snd_tables_t *t,
                 const snd_dectab_t *d);

// Bits a snapshot needs for one acquisition at in_cls: 68 of the longest
// frames (a superframe lock takes 32-63 frames).
uint32_t snd_fo_acquire_bits(unsigned in_cls);

// Acquire from a snapshot (words[0] MSB = absolute bit first_bit, nbits
// bits) with LK1's reference receiver: true when it locked, with the input
// state set to the first locked frame that started at a comma with a good
// head.  rx is scratch.  Keeps the counters.
bool snd_fo_acquire(snd_fo_t *f, const uint32_t *words, uint64_t first_bit, uint32_t nbits, alink_rx_t *rx);

// Move the input state forward to the first frame whose comma is at or after
// bit `from` (the snapshot is older than the ring's oldest data once the
// acquisition took longer than a ring).  False if a layout change lies in
// between (re-acquire).
bool snd_fo_project(snd_fo_t *f, uint64_t from);

// Bits the current frame needs in the ring before snd_fo_frame() may run:
// the frame, the slip window around the next comma, and the comma itself.
uint64_t snd_fo_need(const snd_fo_t *f);

// Frame bits of frame F at the given class and rate.
static inline unsigned snd_fo_frame_bits(unsigned cls, unsigned rate, uint64_t frame) {
    return 10u * alink_frame_symbols(cls, rate, frame);
}

// Parse the current frame from the ring and, when out_on, re-send it into s
// (log may be NULL).  Returns SND_FO_R_* flags.  On SND_FO_R_LOSS the state
// is SND_FO_SEARCH and out_on is false.
unsigned snd_fo_frame(snd_fo_t *f, const snd_ring_t *r, snd_sink_t *s, snd_log_t *log);

// Start re-sending at the current frame (the engine has put the sink where
// that frame belongs).  The frame must start a superframe.
void snd_fo_start_output(snd_fo_t *f);

// Delay arithmetic (engine): the output bit at which a frame whose comma
// started at input bit `in_bit` must start, given a simultaneous sample of
// the input bits received (`in_now`) and the output bit on the wire
// (`out_now`), the cycles per bit of both classes and the delay in output
// bits.  Signed 64-bit; the inputs are absolute bit counts.
static inline int64_t snd_fo_target_bit(uint64_t in_bit, uint64_t in_now, uint64_t out_now,
                                        unsigned in_cycles_per_bit, unsigned out_cycles_per_bit,
                                        uint64_t delay_bits) {
    int64_t ago_in = (int64_t)(in_now - in_bit);
    int64_t ago_out = ago_in * (int64_t)in_cycles_per_bit / (int64_t)out_cycles_per_bit;
    return (int64_t)out_now - ago_out + (int64_t)delay_bits;
}

#endif // SENDER_FANOUT_H
