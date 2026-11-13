/*
 * alink_fast.h - DSPi alink protocol v1: production receiver (WP LK5)
 *
 * The receiver of Documentation/Features/alink_protocol_spec.md section 8,
 * bit-exact with the reference (alink_decoder.c: same frames, samples, events
 * and counters on the same bits), rewritten for speed on the RP2350:
 *
 *  - it reads the line bits in place from the raw DMA ring (32-bit words,
 *    MSB first, any power-of-two size) instead of copying them bit by bit;
 *  - it searches for commas only while unlocked (bit-parallel, 32 positions
 *    per step), and once locked it looks only where the layout puts the next
 *    comma (and the +-2-bit slip window), with the 44.1 kHz frame lengths
 *    kept incrementally from F mod 147;
 *  - symbols decode through a 1024-entry table (byte, K flag, running
 *    disparities, "not a data code group" flag) built at init;
 *  - only the banks in the bank mask are decoded; the others are stepped
 *    over by symbol count; a full bank is decoded by an unrolled routine
 *    from 9 realigned words;
 *  - CRC-8 and CRC-16 by 256-entry tables; idle padding is checked a ring
 *    word at a time against the D21.5 bit pattern;
 *  - the superframe is assembled from the CTL bytes and parsed once per
 *    32 frames.
 *
 * Portable C11, no SDK, no heap, no writable static data: the state and the
 * tables live in the caller's alink_fast_t. The firmware places the hot
 * functions in RAM by defining ALINK_FAST_HOT (DSPi: the .time_critical
 * section, like DSP_TIME_CRITICAL); on the host it is empty.
 *
 * Events are alink_decoder.h's alink_rx_event_t, delivered through the same
 * callback type, so a consumer works with either receiver.
 */

#ifndef ALINK_FAST_H
#define ALINK_FAST_H

#include "alink_decoder.h"

#ifndef ALINK_FAST_HOT
#define ALINK_FAST_HOT
#endif

// ALINK_FAST_M33 1 (RP2350 builds): the full-bank decoder runs as the
// generated Thumb-2 of alink_fast_m33.S instead of the C bank8()/bank8_dma().
#ifndef ALINK_FAST_M33
#define ALINK_FAST_M33 0
#endif

// ALINK_FAST_M33_SW 1: also the assembly bank decoder with the table CRC-16
// (lf_bank8_sw_m33).  Off in DSPi, where the DMA sniffer does the CRC and the
// C decoder in flash is the fallback when the sniffer fails its self-test;
// on in the bench image, which measures both.
#ifndef ALINK_FAST_M33_SW
#define ALINK_FAST_M33_SW 0
#endif

// What the assembly bank decoder reads (offsets fixed: alink_fast_m33.S).
typedef struct {
    const uint32_t *dec;
    const uint16_t *crc16;
    volatile uint32_t *dma_ch;      // sniffer channel's registers (ALINK_FAST_CRC_DMA)
} lf_m33_ctx_t;

// What the assembly clean-frame loop reads (offsets fixed: alink_fast_m33_loop.S).
typedef struct {
    const uint32_t *wp;
    uint32_t n, step, o, ro, xw, xs, xro, iw0, iw1, fm, lm, pat, mid;
    const uint32_t *dec;
    const uint8_t *crc8;
    uint32_t bw, bo;
    bool (*bankfn)(const uint32_t *r, unsigned o, int32_t *out, const lf_m33_ctx_t *c);
    const lf_m33_ctx_t *m33;
    int32_t *samples;
    uint8_t *fails;
    int32_t (*out)[8];
    const int32_t *src;
    uint32_t flo;
    uint8_t *sf_bytes;
    uint32_t sf_count;
    int32_t prev_hdr;
} lf_clean_ctx_t;

// ALINK_FAST_CRC_DMA 1 (RP2350 builds): full banks can have their CRC-16
// computed by the DMA sniffer (alink_fast_t.crc_dma) instead of the table.
#ifndef ALINK_FAST_CRC_DMA
#define ALINK_FAST_CRC_DMA 0
#endif

// Decode table entry (alink_fast_t.dec): bits 0-7 the byte, bit 24 K, bits
// 25-26 the running disparities the code group is valid at (bit 25 RD-,
// bit 26 RD+).  A data code group has nothing else set.  Anything that is
// not a data code group (a K code, or no code group at all) also has
// ALINK_FAST_DEC_BAD = bits 31, 23 and 15: three entries make a sample as
// e2 | e1 << 8 | e0 << 16 (then a sign extension from bit 23), and bit 31 of
// that word is set exactly when one of the three is bad, so a bank is
// checked with one OR per sample.  Version 1 receivers do not check running
// disparity (spec 4.2); those bits are for diagnostics.
#define ALINK_FAST_DEC_K    0x01000000u
#define ALINK_FAST_DEC_RDN  0x02000000u
#define ALINK_FAST_DEC_RDP  0x04000000u
#define ALINK_FAST_DEC_BAD  0x80808000u

// Bits kept free between the writer and the oldest bit the receiver still
// needs; closer than this counts as an overrun (alink_fast_t.overruns).
#define ALINK_FAST_GUARD_BITS   (64u * 32u)

typedef struct {
    // Configuration.
    const volatile uint32_t *ring;  // raw ring, MSB first; absolute bit b is in word (b / 32) & ring_mask
    uint32_t ring_mask;             // ring words - 1 (power of two)
    uint8_t  cls;
    uint8_t  bank_mask;
    alink_rx_cb_t cb;
    void    *ctx;

    // Writer position: absolute bits written (64-bit for the ALIGN event,
    // 32-bit for the arithmetic, which is modular).
    uint64_t avail64;
    uint32_t avail;

    // State, as alink_rx_t. Positions are absolute bit numbers mod 2^32.
    alink_rx_state_t state;
    uint32_t q;                     // SEARCH: next comma end to test
    uint32_t p;                     // ALIGNED/LOCKED: comma end of the current frame
    bool     body_done;
    bool     comma_ok;
    int8_t   slip;
    uint8_t  fails;
    int16_t  prev_hdr;              // ALIGNED/LOCKED superframe assembly: previous HDR, -1 none
    uint64_t frame;                 // LOCKED: F of the current frame
    uint8_t  rate, channels;        // LOCKED: layout of the current frame

    // Layout cache for (cls, rate, channels).
    uint16_t num, den;              // symbols per frame = num / den (den 1 at 48/96 kHz)
    uint16_t fmod;                  // F mod den
    uint8_t  layout;                // alink_layout_symbols(channels)
    uint8_t  nbanks;                // banks in the frame
    uint8_t  decode_banks;          // bank_mask limited to the frame's banks

    // Superframe assembly.
    uint8_t  sf_count;
    bool     apply_pending;
    uint8_t  sf_bytes[ALINK_SF_LEN];
    alink_sf_t sf;

    // Bank output (the hold buffer).
    uint8_t  bank_fails[ALINK_MAX_BANKS];
    int32_t  samples[ALINK_MAX_CHANNELS];

    // Delivery: per LOCKED frame, out[out_n++][i] = the sample of the alink
    // channel assigned to output i (EXT_ALINK_SLOTS), 24-bit sign-extended,
    // 0 for an unassigned output or a channel at or above C.  The run stops
    // at a frame boundary when the block is full; the caller consumes the
    // out_n frames and sets out_n to 0.
    int32_t (*out)[8];
    uint16_t out_cap, out_n;
    const int32_t *slot_src[8];
    int8_t   slot_run;              // the 8 outputs are channels slot_run .. +7 of one bank (else -1)
    int32_t  zero;                  // what an unassigned output reads
    bool     frame_events;          // deliver ALINK_RX_EV_FRAME to cb as well (tests, tools)
    int8_t   crc_dma;               // ALINK_FAST_CRC_DMA: sniffer channel for full banks, -1 = table CRC
    lf_m33_ctx_t m33;               // ALINK_FAST_M33: the assembly decoder's view of the tables

    uint32_t lin[ALINK_MAX_FRAME_SYMBOLS * 10u / 32u + 4u]; // a frame that crosses the ring's wrap

    alink_rx_stats_t stats;
    uint32_t overruns;              // times the writer came within ALINK_FAST_GUARD_BITS of unread bits
    alink_rx_event_t ev;            // the event being delivered

    // Tables built by alink_fast_init().
    uint8_t  cap[4][3];             // alink_max_channels(cls, rate)
    uint16_t ratio[4][3][2];        // symbols per frame as num, den
    uint8_t  crc8[256];
    uint16_t crc16[256];
    uint32_t dec[1024];
} alink_fast_t;

// Start in SEARCH at absolute bit start_bit (the writer's position when the
// ring starts, normally 0) with zeroed counters.  ring_words is a power of
// two; bank_mask 0xFF decodes every bank.
void alink_fast_init(alink_fast_t *f, const volatile uint32_t *ring, uint32_t ring_words, unsigned cls,
                    uint8_t bank_mask, alink_rx_cb_t cb, void *ctx, uint64_t start_bit);

// Process every step the bits below avail_bits (absolute, as the writer's
// position) allow.  search_budget_bits > 0 bounds the comma search: when
// more than that many bits are waiting while in SEARCH, the search skips
// ahead to the newest bits (a real-time receiver catching up; the reference
// would scan them all and lock no earlier on a live line).  0 = never skip
// (bit-exact with the reference on any input).  If the writer has overrun
// bits the receiver still needs, it counts an overrun, declares a loss when
// it was aligned or locked, and searches again from the newest bits.
// Returns the number of frames delivered.
ALINK_FAST_HOT unsigned alink_fast_run(alink_fast_t *f, uint64_t avail_bits, uint32_t search_budget_bits);

// Start over in SEARCH at start_bit with class cls and zeroed counters,
// keeping the tables, the ring, the callback, the bank mask, the slots, the
// output block and the CRC channel (alink_fast_init() without rebuilding the
// tables).  Safe on whichever core decodes.
void alink_fast_restart(alink_fast_t *f, unsigned cls, uint64_t start_bit);

// Change the banks to decode (takes effect at the next frame).
void alink_fast_set_bank_mask(alink_fast_t *f, uint8_t bank_mask);

// Alink channel (0..63) per output, 0xFF (anything >= 64) for silence. The
// bank mask is the caller's business (the banks of the assigned channels).
void alink_fast_set_slots(alink_fast_t *f, const uint8_t slots[8]);

// Deliver LOCKED frames into out[0..cap) (NULL: no delivery, no limit).
// Inline: the decode service sets it on every call.
static inline void alink_fast_set_output(alink_fast_t *f, int32_t (*out)[8], uint16_t cap) {
    f->out = out;
    f->out_cap = cap;
    f->out_n = 0;
}

#if ALINK_FAST_M33
// Clean frames only (48/96 kHz, one full bank, the assembly path): what a
// second core runs without ever reaching the cold paths in flash.  Returns
// the frames taken.  alink_fast_needs_general() then says whether it stopped
// at a frame that needs alink_fast_run() (bits and output room were there),
// rather than at the end of the bits or the output block.
ALINK_FAST_HOT unsigned alink_fast_run_clean(alink_fast_t *f, uint64_t avail_bits);
ALINK_FAST_HOT bool alink_fast_needs_general(const alink_fast_t *f);
#endif

#if ALINK_FAST_CRC_DMA
// Use DMA channel ch (configured by the platform for the sniffer: 32-bit,
// read increment, unpaced, SNIFF_EN, SNIFF_CTRL on ch with CRC-16-CCITT) for
// the CRC-16 of full banks; ch < 0 goes back to the table.
void alink_fast_set_crc_dma(alink_fast_t *f, int ch);

// Run the sniffer CRC over 6 words (24 bytes, MSB first) on f->crc_dma and
// return it: the platform's self-test compares it with alink_crc16() before
// it enables the DMA path (and picks SNIFF_CTRL.BSWAP if it has to).
uint32_t alink_fast_crc_dma_probe(alink_fast_t *f, const uint32_t words[6]);
#endif

#endif // ALINK_FAST_H
