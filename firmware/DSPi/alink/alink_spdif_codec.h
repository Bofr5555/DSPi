/*
 * alink_spdif_codec.h - the alink time base carried on S/PDIF (WP FWSP1)
 *
 * Normative text: Documentation/Features/alink_protocol_spec.md section 12
 * ("Carriage on S/PDIF").  The bit-exact reference for the 48-byte S/PDIF
 * superframe a source puts into the user bits of every IEC 60958 block: its
 * layout, validity (in the line's order, with class 0x10 and F mod 192), the
 * user-bit order, the apply carriage rule and the broadcast lead.  The roles
 * (source counter, epochs, follower) are WP FWSP2 (alink_spdif.{c,h}).
 *
 * Portable C11 like alink_codec: no Pico SDK, no heap, no floating point, no
 * writable static data.  It does NOT include alink_codec.h: the S/PDIF
 * carriage has its own constants and its own allowed-op set (PLAN-AT R9b), so
 * a change to the line's op range never widens what S/PDIF carries.  The host
 * test is firmware/tests/test_alink_spdif_codec.c, the Python twin
 * tools/alink/alink_spdif.py; both check firmware/tests/alink_spdif_vectors.json.
 *
 * ALINK_SPDIF_HOT marks the functions a RAM-resident transmit filler may call
 * (alink_spdif_sf_build() and what it uses); the firmware build defines it to
 * a .time_critical section, the host build leaves it empty.
 */

#ifndef ALINK_SPDIF_CODEC_H
#define ALINK_SPDIF_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef ALINK_SPDIF_HOT
#define ALINK_SPDIF_HOT
#endif

#define ALINK_SPDIF_VERSION          1u

// Geometry (spec 12.1, 12.4).  Spec names in the comments.
#define ALINK_SPDIF_BLOCK_FRAMES     192u   // SPDIF_BLOCK_FRAMES: frames per IEC 60958 block
#define ALINK_SPDIF_USER_BITS        384u   // SPDIF_USER_BITS: one U bit per subframe
#define ALINK_SPDIF_SF_LEN           48u    // SPDIF_SUPERFRAME_LEN: 384 bits
#define ALINK_SPDIF_SF_CRC_LEN       30u    // CRC over bytes 0-29, stored at 30-31
#define ALINK_SPDIF_CLASS            0x10u  // SPDIF_CARRIER_CLASS, byte 5
#define ALINK_SPDIF_CHANNELS         2u     // byte 4
#define ALINK_SPDIF_TX_TABLE_BLOCKS  4u     // SPDIF_TX_TABLE_BLOCKS
#define ALINK_SPDIF_TX_RING_FRAMES   512u   // SPDIF_TX_RING_FRAMES
#define ALINK_SPDIF_MIN_LEAD_FRAMES  896u   // SPDIF_CTRL_MIN_LEAD_FRAMES = ring + 2 blocks
#define ALINK_SPDIF_COAST_BLOCKS     8u     // SPDIF_COAST_BLOCKS (follower, FWSP2)

_Static_assert(ALINK_SPDIF_USER_BITS / 8u == ALINK_SPDIF_SF_LEN, "user bits are the superframe");
_Static_assert(ALINK_SPDIF_MIN_LEAD_FRAMES ==
               ALINK_SPDIF_TX_RING_FRAMES + 2u * ALINK_SPDIF_BLOCK_FRAMES, "lead = ring + 2 blocks");

// Rate codes (byte 3), as on the line.
#define ALINK_SPDIF_RATE_44K1        0u
#define ALINK_SPDIF_RATE_48K         1u
#define ALINK_SPDIF_RATE_96K         2u
#define ALINK_SPDIF_RATE_MAX         2u

// Flags (byte 2).  b2 is derived (set exactly when an apply is carried); b0 is
// sent 0 in this version but honoured by followers; b3 from the source's state.
#define ALINK_SPDIF_FLAG_MUTE        0x01u
#define ALINK_SPDIF_FLAG_APPLY_PEND  0x04u
#define ALINK_SPDIF_FLAG_USB_LIVE    0x08u

// Apply ops carried on S/PDIF (byte 24): its own set, spec 12.2.  The numbers
// are the line's (6.4); 3 PRESET_LOAD, 5 OUT_DELAY, 6 RATE and 8 SYNC_PULSE
// are never carried on S/PDIF.
#define ALINK_SPDIF_OP_NONE          0u
#define ALINK_SPDIF_OP_MUTE          1u
#define ALINK_SPDIF_OP_UNMUTE        2u
#define ALINK_SPDIF_OP_MASTER_VOLUME 4u
#define ALINK_SPDIF_OP_CLICK         7u
#define ALINK_SPDIF_OP_SIGGEN_START  9u
#define ALINK_SPDIF_OP_SIGGEN_STOP   10u
#define ALINK_SPDIF_OPS_ALLOWED      ((1u << ALINK_SPDIF_OP_MUTE) | (1u << ALINK_SPDIF_OP_UNMUTE) | \
                                      (1u << ALINK_SPDIF_OP_MASTER_VOLUME) | (1u << ALINK_SPDIF_OP_CLICK) | \
                                      (1u << ALINK_SPDIF_OP_SIGGEN_START) | (1u << ALINK_SPDIF_OP_SIGGEN_STOP))

// alink_spdif_sf_unpack() results: the first failing check, the line's codes (6.3).
#define ALINK_SPDIF_OK               0
#define ALINK_SPDIF_ERR_CRC          1
#define ALINK_SPDIF_ERR_MAGIC        2
#define ALINK_SPDIF_ERR_VERSION      3
#define ALINK_SPDIF_ERR_FRAME        4      // frame not a multiple of 192
#define ALINK_SPDIF_ERR_LAYOUT       5      // class not 0x10, rate not 0..2 or channels not 2
#define ALINK_SPDIF_ERR_APPLY        6      // op not in the S/PDIF set, or op without target / target without op

#define ALINK_SPDIF_SF_MAGIC         0xA5u

static inline bool alink_spdif_op_allowed(unsigned op) {
    return op < 32u && ((ALINK_SPDIF_OPS_ALLOWED >> op) & 1u);
}

// ---------------------------------------------------------------------------
// The superframe (spec 12.2)
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  flags;         // byte 2, ALINK_SPDIF_FLAG_*
    uint8_t  rate;          // byte 3, ALINK_SPDIF_RATE_*
    uint8_t  channels;      // byte 4, 2
    uint8_t  cls;           // byte 5, ALINK_SPDIF_CLASS
    uint8_t  hops;          // byte 6, 0 (reserved for the relay)
    uint8_t  sender;        // byte 7, serial byte 0 (unique board id byte 7)
    uint64_t frame;         // bytes 8-15, F of the block's first frame, F mod 192 = 0
    uint64_t apply_frame;   // bytes 16-23, target N, 0 = none
    uint8_t  apply_op;      // byte 24
    uint8_t  apply_arg;     // byte 25
    int16_t  apply_value;   // bytes 26-27
    uint8_t  epoch;         // byte 28
    uint8_t  sender2;       // byte 29, serial byte 1 (unique board id byte 6)
} alink_spdif_sf_t;

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR):
// the line's CRC (alink_codec.h alink_crc16), kept here so this codec stands alone.
uint16_t alink_spdif_crc16(const uint8_t *data, size_t len) ALINK_SPDIF_HOT;

// Serialise as given (version 1, flags as given, padding 0, CRC low byte first).
void alink_spdif_sf_pack(const alink_spdif_sf_t *sf, uint8_t out[ALINK_SPDIF_SF_LEN]) ALINK_SPDIF_HOT;

// Parse and validate in the order of 12.2; ALINK_SPDIF_OK or ALINK_SPDIF_ERR_*.
// *sf is filled whenever the CRC, magic and version are good.  arg and value are
// not judged (the unit refuses a bad argument when it takes the op).
int alink_spdif_sf_unpack(const uint8_t in[ALINK_SPDIF_SF_LEN], alink_spdif_sf_t *sf);

// The source id (EXT_ALINK_SPDIF_STATUS.source_id): sender | sender2 << 8.
static inline uint16_t alink_spdif_source_id(const alink_spdif_sf_t *sf) {
    return (uint16_t)(sf->sender | ((unsigned)sf->sender2 << 8));
}

// User bit i (0..383) of a block carrying superframe sf: bit i mod 8 of byte
// i / 8, LSB first.  Subframe 2k is channel A of frame k, 2k + 1 channel B.
static inline unsigned alink_spdif_ubit(const uint8_t sf[ALINK_SPDIF_SF_LEN], unsigned i) {
    return (sf[i >> 3] >> (i & 7u)) & 1u;
}

// The two user bits of frame k (0..191): bit 0 channel A, bit 1 channel B.
static inline unsigned alink_spdif_frame_ubits(const uint8_t sf[ALINK_SPDIF_SF_LEN], unsigned k) {
    return (sf[k >> 2] >> ((k & 3u) * 2u)) & 3u;
}

// ---------------------------------------------------------------------------
// The source's block superframe (transmit reference, spec 12.2-12.4)
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  flags;         // b0 and b3 only (b2 is derived per block)
    uint8_t  rate;          // rate code of the source's outputs
    uint8_t  sender;        // serial byte 0
    uint8_t  sender2;       // serial byte 1
    uint8_t  epoch;
    // The one pending broadcast apply (none: apply_op 0).
    uint64_t apply_frame;
    uint8_t  apply_op;
    uint8_t  apply_arg;
    int16_t  apply_value;
} alink_spdif_src_t;

// The superframe of the block whose first frame is `frame` (a multiple of
// 192): the source's fields, class 0x10, 2 channels, hops 0, and the pending
// apply exactly when the block completes before it (frame + 192 <= N), with
// flag b2 set exactly then.  RAM-safe (ALINK_SPDIF_HOT, 32-bit arithmetic and
// no library calls), so a RAM-resident filler can build a block's entry in the
// same pass as its first stamp.
void alink_spdif_sf_build(const alink_spdif_src_t *src, uint64_t frame,
                          uint8_t out[ALINK_SPDIF_SF_LEN]) ALINK_SPDIF_HOT;

// Whether a block starting at `frame` carries an apply targeted at `apply_frame`:
// frame + 192 <= N, written so that it cannot overflow at the top of the u64.
static inline bool alink_spdif_carries(uint64_t frame, uint64_t apply_frame) {
    return apply_frame != 0 && frame < apply_frame &&
           apply_frame - frame >= ALINK_SPDIF_BLOCK_FRAMES;
}

// The broadcast lead rule (12.4 b, EXT_ALINK_SPDIF_CTRL): a target at least
// 896 frames after the frame the source plays now.
static inline bool alink_spdif_lead_ok(uint64_t now_frame, uint64_t target) {
    return target >= now_frame && target - now_frame >= ALINK_SPDIF_MIN_LEAD_FRAMES;
}

#endif // ALINK_SPDIF_CODEC_H
