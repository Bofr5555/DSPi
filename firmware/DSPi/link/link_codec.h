/*
 * link_codec.h - DSPi link protocol v1: reference encoder and primitives
 *
 * Normative text: Documentation/Features/link_protocol_spec.md.  This is the
 * bit-exact reference: 8b/10b, CRC-8 and CRC-16, the frame layout and its
 * length at every rate (44.1 kHz padding included), the 32-byte superframe,
 * PRBS-7 and the sender-side frame encoder.  The receiver is link_decoder.h.
 *
 * Portable C11: no Pico SDK, no heap, no floating point, no writable static
 * data (every table is const, so it lands in flash, not in .data).  The host
 * tests (firmware/tests/test_link_codec.c) compile it directly; the Python
 * twin is tools/link/link_codec.py and both check
 * firmware/tests/link_vectors.json.
 *
 * Conventions.  A symbol is a 10-bit code group "abcdei fghj" held in the
 * low 10 bits of a uint16_t with 'a' in bit 9; it goes on the line 'a'
 * first.  Running disparity is 0 for RD- and 1 for RD+.  Line bits are packed
 * MSB first into 32-bit words (the first bit on the line is bit 31 of the
 * first word), which is what a PIO "out pins,1" with autopull 32 and shift
 * left sends, and what "in pins,1" with autopush 32 and shift left receives.
 */

#ifndef LINK_CODEC_H
#define LINK_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_PROTOCOL_VERSION   1u

// Line-rate classes (superframe byte 5) and their rates in baud.
#define LINK_CLASS_L1           1u      // 15.36 MBd, 20 PIO cycles per bit at 307.2 MHz
#define LINK_CLASS_L2           2u      // 30.72 MBd, 10 cycles
#define LINK_CLASS_L3           3u      // 61.44 MBd, 5 cycles

// Sample-rate codes (superframe byte 3).
#define LINK_RATE_44K1          0u
#define LINK_RATE_48K           1u
#define LINK_RATE_96K           2u

// Frame layout.
#define LINK_MAX_CHANNELS       64u
#define LINK_BANK_CHANNELS      8u
#define LINK_MAX_BANKS          8u      // ceil(64 / 8)
#define LINK_HEAD_SYMBOLS       4u      // K28.5, HDR, CTL, CRC8
#define LINK_BANK_SYMBOLS(k)    (3u * (k) + 2u)
#define LINK_MAX_FRAME_SYMBOLS  140u    // longest frame of any class/rate (L3 at 44.1 kHz)

// Special symbols.
#define LINK_SYM_K28_5_RDN      0x0FAu  // 001111 1010, sent when RD is -
#define LINK_SYM_K28_5_RDP      0x305u  // 110000 0101, sent when RD is +
#define LINK_SYM_IDLE           0x2AAu  // D21.5 101010 1010 (same in both columns)
#define LINK_BYTE_IDLE          0xB5u   // D21.5 = (5 << 5) | 21
#define LINK_BYTE_K28_5         0xBCu   // K28.5 = (5 << 5) | 28
#define LINK_SYM_INVALID        0xFFFFu

// Superframe (CTL sub-channel, one byte per frame, 32 frames).
#define LINK_SF_LEN             32u
#define LINK_SF_MAGIC           0xA5u
#define LINK_SF_FLAG_MUTE       0x01u   // b0 system mute
#define LINK_SF_FLAG_RATE_PEND  0x02u   // b1 a RATE apply is pending (set by the encoder)
#define LINK_SF_FLAG_APPLY_PEND 0x04u   // b2 an apply is pending (set by the encoder)
#define LINK_SF_FLAG_USB_LIVE   0x08u   // b3 sender has a live USB stream
#define LINK_SF_FLAG_PRBS       0x10u   // b4 payload is PRBS-7 (test pattern)
#define LINK_SF_FLAG_SLAVED     0x20u   // b5 sender is slaved (fan-out)
#define LINK_SF_FLAGS_DEFINED   0x3Fu

// Apply ops (superframe byte 24, EXT_APPLY_AT_FRAME).
#define LINK_OP_NONE            0u
#define LINK_OP_MUTE            1u
#define LINK_OP_UNMUTE          2u
#define LINK_OP_PRESET_LOAD     3u      // arg = slot
#define LINK_OP_MASTER_VOLUME   4u      // value
#define LINK_OP_OUT_DELAY       5u      // arg = output, value = samples
#define LINK_OP_RATE            6u      // arg = rate code, value = channels C (0 = unchanged)
#define LINK_OP_CLICK           7u      // arg = output
#define LINK_OP_SYNC_PULSE      8u
#define LINK_OP_MAX             8u

// link_sf_unpack() results.
#define LINK_SF_OK              0
#define LINK_SF_ERR_CRC         1
#define LINK_SF_ERR_MAGIC       2
#define LINK_SF_ERR_VERSION     3
#define LINK_SF_ERR_FRAME       4       // frame counter not a multiple of 32
#define LINK_SF_ERR_LAYOUT      5       // class, rate or channel count out of range
#define LINK_SF_ERR_APPLY       6       // unknown op, or op without target / target without op

// ---------------------------------------------------------------------------
// Classes, rates and the frame layout
// ---------------------------------------------------------------------------

// Line rate of a class in baud, 0 for an unknown class.
uint32_t link_class_baud(unsigned cls);

// Sample rate of a rate code in Hz, 0 for an unknown code.
uint32_t link_rate_hz(unsigned rate);

// Symbols in frame F (comma to the symbol before the next comma), including
// idle padding.  Constant at 48 and 96 kHz; at 44.1 kHz it follows the
// fractional accumulator of spec section 5.6, which depends on F mod 147.
// 0 for an unknown class or rate.
unsigned link_frame_symbols(unsigned cls, unsigned rate, uint64_t frame);

// Fewest and most symbols any frame has at this class and rate.
unsigned link_frame_symbols_min(unsigned cls, unsigned rate);
unsigned link_frame_symbols_max(unsigned cls, unsigned rate);

// Symbols the head and the banks of a C-channel frame take (no padding).
unsigned link_layout_symbols(unsigned channels);

// Largest C (<= 64) whose layout fits every frame at this class and rate.
unsigned link_max_channels(unsigned cls, unsigned rate);

// Banks of a C-channel frame, channels in bank b, and the symbol index of
// bank b's first byte (the comma is symbol 0).
static inline unsigned link_bank_count(unsigned channels) {
    return (channels + LINK_BANK_CHANNELS - 1u) / LINK_BANK_CHANNELS;
}
static inline unsigned link_bank_channels(unsigned channels, unsigned bank) {
    unsigned first = bank * LINK_BANK_CHANNELS;
    if (first >= channels) return 0;
    unsigned k = channels - first;
    return k > LINK_BANK_CHANNELS ? LINK_BANK_CHANNELS : k;
}
static inline unsigned link_bank_symbol(unsigned bank) {
    return LINK_HEAD_SYMBOLS + bank * LINK_BANK_SYMBOLS(LINK_BANK_CHANNELS);
}

// ---------------------------------------------------------------------------
// 8b/10b (IEEE 802.3 clause 36 code groups)
// ---------------------------------------------------------------------------

// Encode one byte at running disparity *rd (0 = RD-, 1 = RD+) and update
// *rd.  k selects the control code group; only K28.0-7, K23.7, K27.7, K29.7
// and K30.7 exist.  Returns the 10-bit symbol, or LINK_SYM_INVALID (with *rd
// unchanged) for a K code that does not exist.
uint16_t link_8b10b_encode(uint8_t byte, bool k, unsigned *rd);

// Decode a 10-bit symbol.  Returns a mask of the running disparities it is a
// valid code group for (bit 0: RD-, bit 1: RD+), 0 if it is not a code group
// at all; on a non-zero return *byte and *k hold the decoded value.  The set
// of valid symbols is exactly the image of link_8b10b_encode().
unsigned link_8b10b_decode(uint16_t sym, uint8_t *byte, bool *k);

// Running disparity after sending sym at running disparity rd.
unsigned link_8b10b_rd_after(uint16_t sym, unsigned rd);

static inline bool link_sym_is_comma(uint16_t sym) {
    return sym == LINK_SYM_K28_5_RDN || sym == LINK_SYM_K28_5_RDP;
}

// ---------------------------------------------------------------------------
// CRCs and samples
// ---------------------------------------------------------------------------

// CRC-8, poly 0x07, init 0x00, no reflection, no final XOR (check 0xF4).
uint8_t link_crc8(const uint8_t *data, size_t len);

// CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF, no reflection, no final XOR
// (check 0x29B1), the same CRC as uart_control.h.
uint16_t link_crc16(const uint8_t *data, size_t len);

// 24-bit two's complement, MSB first.  Writing takes bits 23..0 of the
// sample; reading sign-extends to 32 bits.
static inline void link_put_s24(uint8_t *out, int32_t sample) {
    uint32_t u = (uint32_t)sample;
    out[0] = (uint8_t)(u >> 16);
    out[1] = (uint8_t)(u >> 8);
    out[2] = (uint8_t)u;
}
static inline int32_t link_get_s24(const uint8_t *in) {
    uint32_t u = ((uint32_t)in[0] << 16) | ((uint32_t)in[1] << 8) | in[2];
    return (int32_t)(u ^ 0x800000u) - 0x800000;
}

// PRBS-7 (x^7 + x^6 + 1), state reset to all ones, bits packed MSB first:
// the payload of every frame whose superframe has LINK_SF_FLAG_PRBS.
void link_prbs7_fill(uint8_t *out, size_t len);

// ---------------------------------------------------------------------------
// Superframe
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  flags;         // byte 2, LINK_SF_FLAG_*
    uint8_t  rate;          // byte 3, LINK_RATE_*
    uint8_t  channels;      // byte 4, C (1..64)
    uint8_t  cls;           // byte 5, class of this line (1..3)
    uint8_t  hops;          // byte 6, 0 at the master sender, +1 per repeater/fan-out
    uint8_t  sender;        // byte 7, low byte of the sender's serial
    uint64_t frame;         // bytes 8-15, F of the superframe's first frame (multiple of 32)
    uint64_t apply_frame;   // bytes 16-23, target frame, 0 = none
    uint8_t  apply_op;      // byte 24, LINK_OP_*
    uint8_t  apply_arg;     // byte 25
    int16_t  apply_value;   // bytes 26-27
} link_sf_t;

// Serialise (version 1, reserved bytes 0, CRC-16 little-endian in 30-31).
void link_sf_pack(const link_sf_t *sf, uint8_t out[LINK_SF_LEN]);

// Parse and validate; LINK_SF_OK or LINK_SF_ERR_*.  *sf is filled whenever
// the CRC, magic and version are good.
int link_sf_unpack(const uint8_t in[LINK_SF_LEN], link_sf_t *sf);

// The superframe a repeater or fan-out re-emits on a line of class out_cls:
// the same content with hops + 1 (saturating at 255) and the new class.
void link_sf_repeat(const link_sf_t *in, uint8_t out_cls, link_sf_t *out);

// ---------------------------------------------------------------------------
// Bit packing
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t *words;
    size_t    cap_words;
    size_t    nbits;        // bits written so far
} link_bitw_t;

static inline void link_bitw_init(link_bitw_t *w, uint32_t *words, size_t cap_words) {
    w->words = words;
    w->cap_words = cap_words;
    w->nbits = 0;
}

// Append the low n bits of v (n <= 32), most significant first.  False (and
// nothing written) when the buffer is full.
bool link_bitw_put(link_bitw_t *w, uint32_t v, unsigned n);

// ---------------------------------------------------------------------------
// Frame encoder (sender reference)
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  cls;
    uint8_t  rate;          // layout in effect
    uint8_t  channels;
    unsigned rd;            // running disparity carried from frame to frame
    uint64_t frame;         // F of the next frame to encode
    // Superframe content the caller owns: flags b0/b3/b4/b5, sender, hops.
    // Changes take effect at the next superframe (or at the first frame).
    uint8_t  flags;
    uint8_t  sender;
    uint8_t  hops;
    // One pending apply, carried in every superframe that completes before
    // its target (spec 6.4).
    uint64_t apply_frame;
    uint8_t  apply_op;
    uint8_t  apply_arg;
    int16_t  apply_value;
    uint8_t  sf[LINK_SF_LEN];   // the superframe being sent
    bool     started;           // sf packed (first frame encoded)
} link_tx_t;

// Start a stream at frame `first` with running disparity RD-.  False for an
// unknown class or rate, or a C that does not fit.
bool link_tx_init(link_tx_t *tx, unsigned cls, unsigned rate, unsigned channels,
                  uint64_t first, uint8_t sender);

// Schedule an apply op at frame `at`.  A RATE op (arg = rate code, value = new
// C or 0) also switches the encoder's layout at `at`.  False if `at` is too
// close for one complete superframe to carry it, an op is already pending,
// or the op is invalid (RATE: `at` not a multiple of 32, or C does not fit).
bool link_tx_schedule(link_tx_t *tx, uint64_t at, uint8_t op, uint8_t arg, int16_t value);

// Encode frame tx->frame into w (K28.5, head, banks, idles: exactly
// link_frame_symbols() symbols) and advance to the next frame.  samples
// holds tx->channels 24-bit samples; it is ignored while the PRBS flag is
// set.  False if w is full (the frame is then incomplete).
bool link_tx_frame(link_tx_t *tx, const int32_t *samples, link_bitw_t *w);

#endif // LINK_CODEC_H
