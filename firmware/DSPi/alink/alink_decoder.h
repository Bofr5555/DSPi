/*
 * alink_decoder.h - DSPi alink protocol v1: reference receiver
 *
 * The bit-exact receiver of Documentation/Features/alink_protocol_spec.md
 * section 8: comma search, alignment, superframe lock, the flywheel and the
 * slip window, bank hold/zero and the error counters.  It consumes line bits
 * (MSB first in 32-bit words, as alink_codec.h) in chunks of any size; its
 * output depends only on the bits, never on how they were chunked.
 *
 * Portable C11, no SDK, no heap.  The state lives in the caller's
 * alink_rx_t (about 1 KB), so nothing here costs .data or .bss until a
 * caller instantiates it.
 */

#ifndef ALINK_DECODER_H
#define ALINK_DECODER_H

#include "alink_codec.h"

#define ALINK_RX_BUF_WORDS  128u    // 4096 bits; a step never needs more than 1430
#define ALINK_RX_SLIP_MAX   2       // a comma this many bits off its place is a slip
#define ALINK_RX_FAIL_LIMIT 3u      // consecutive failed frames that mean loss

typedef enum {
    ALINK_RX_SEARCH = 0,    // looking for a comma followed by a good head
    ALINK_RX_ALIGNED = 1,   // symbol-aligned, waiting for a superframe
    ALINK_RX_LOCKED = 2,    // superframe known: F, rate and C known, banks decoded
} alink_rx_state_t;

// Frame flags (alink_rx_event_t.flags for ALINK_RX_EV_FRAME).
#define ALINK_RXF_COMMA_MISSING 0x01u   // no comma within the slip window: flywheel
#define ALINK_RXF_SLIP          0x02u   // comma found alink_rx_event_t.slip bits off
#define ALINK_RXF_HEAD_BAD      0x04u   // HDR/CTL/CRC8 not code groups, or CRC-8 wrong
#define ALINK_RXF_COUNTER       0x08u   // head good but HDR != F & 0xFF

// Bank status.
#define ALINK_BANK_SKIPPED  0u      // not decoded (not in the bank mask, or not in the frame)
#define ALINK_BANK_OK       1u
#define ALINK_BANK_HOLD     2u      // first consecutive failure: last samples repeated
#define ALINK_BANK_ZERO     3u      // second and later consecutive failure: zeros

typedef enum {
    ALINK_RX_EV_ALIGN = 0,  // entered ALIGNED; bitpos = comma end
    ALINK_RX_EV_LOCK,       // entered LOCKED; frame = F of the frame that completed the superframe
    ALINK_RX_EV_FRAME,      // a frame decoded in LOCKED
    ALINK_RX_EV_SUPERFRAME, // a superframe accepted in LOCKED (sf valid)
    ALINK_RX_EV_SF_ERROR,   // a superframe rejected (sf_error = ALINK_SF_ERR_* or ALINK_RX_SF_*)
    ALINK_RX_EV_LOSS,        // alink loss; back to SEARCH
} alink_rx_event_type_t;

// Superframe rejections beyond ALINK_SF_ERR_*.
#define ALINK_RX_SF_INCOMPLETE  16  // a byte of it was lost (LOCKED only)
#define ALINK_RX_SF_CLASS       17  // class byte differs from the receiver's class
#define ALINK_RX_SF_COUNTER     18  // F field does not match the frames' HDR bytes

typedef struct {
    alink_rx_event_type_t type;
    uint64_t bitpos;                // ALIGN: comma end (bits since the first bit pushed)
    uint64_t frame;                 // LOCK, FRAME, SUPERFRAME
    uint8_t  flags;                 // FRAME: ALINK_RXF_*
    int8_t   slip;                  // FRAME with ALINK_RXF_SLIP: comma offset in bits
    uint8_t  channels;              // FRAME: C
    uint8_t  bank_status[ALINK_MAX_BANKS];  // FRAME
    const int32_t *samples;         // FRAME: C samples; 0 in skipped banks
    const alink_sf_t *sf;           // SUPERFRAME, LOCK
    int      sf_error;              // SF_ERROR
} alink_rx_event_t;

typedef void (*alink_rx_cb_t)(void *ctx, const alink_rx_event_t *ev);

typedef struct {
    uint32_t lock_count;        // entries into LOCKED
    uint32_t loss_count;        // losses from LOCKED (a loss before lock is not counted)
    uint32_t crc8_errors;       // bad heads (non-code-group symbol or CRC-8)
    uint32_t crc16_errors;      // failed banks (non-code-group symbol or CRC-16)
    uint32_t bit_slips;         // commas found inside the slip window but off place
    uint32_t superframe_errors; // rejected superframes
    uint32_t comma_errors;      // flywheel frames (comma missing)
    uint32_t counter_errors;    // HDR != F & 0xFF with a good head, or F corrected by a superframe
    uint32_t code_errors;       // symbols that are not data code groups in head or banks
    uint32_t idle_errors;       // non-idle symbols in the padding
    uint32_t frames;            // frames decoded in LOCKED
} alink_rx_stats_t;

typedef struct {
    // Configuration.
    uint8_t  cls;               // class of the PIO sampler (1..3)
    uint8_t  bank_mask;         // banks to decode (bit b = bank b)
    alink_rx_cb_t cb;
    void    *ctx;

    // Bit buffer: buf holds absolute bits [base, avail).
    uint32_t buf[ALINK_RX_BUF_WORDS];
    uint64_t base;
    uint64_t avail;

    // State.
    alink_rx_state_t state;
    uint64_t q;                 // SEARCH: next comma end to test
    uint64_t p;                 // ALIGNED/LOCKED: comma end of the current frame
    bool     body_done;         // current frame's body processed, next comma not yet found
    bool     comma_ok;          // current frame started at a comma (not the flywheel)
    int8_t   slip;              // current frame's comma offset
    unsigned fails;             // consecutive failed frames
    uint64_t frame;             // LOCKED: F of the current frame
    uint8_t  rate, channels;    // LOCKED: layout of the current frame
    int      prev_hdr;          // ALIGNED: HDR of the previous frame, -1 if its head was bad

    // Superframe assembly.
    uint8_t  sf_bytes[ALINK_SF_LEN];
    unsigned sf_count;          // bytes 0..sf_count-1 collected from consecutive good heads
    alink_sf_t sf;              // last accepted superframe
    bool     apply_pending;     // sf carries a RATE op not yet reached

    // Bank output.
    uint8_t  bank_fails[ALINK_MAX_BANKS];
    int32_t  samples[ALINK_MAX_CHANNELS];

    alink_rx_stats_t stats;
} alink_rx_t;

// Reset to SEARCH at bit 0 with zeroed counters.  bank_mask 0xFF decodes all banks.
void alink_rx_init(alink_rx_t *rx, unsigned cls, uint8_t bank_mask, alink_rx_cb_t cb, void *ctx);

// Push nbits line bits, MSB first from words[0], and process every step the
// bits allow.  Returns the number of bits taken (always nbits).
size_t alink_rx_push(alink_rx_t *rx, const uint32_t *words, size_t nbits);

#endif // ALINK_DECODER_H
