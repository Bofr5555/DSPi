/*
 * midi_frame.h - pure codec for the USB MIDI control channel (DSPI_USB_MIDI)
 *
 * One SysEx message carries exactly one UART control frame
 * (uart_control.h), from the type byte through the CRC16, 7-bit packed:
 *
 *   F0 <mfr...> 'D' 'S' 'P' <ver=0x01> <tag> <packed frame> F7
 *
 * See Documentation/Features/usb_midi_control_spec.md for the normative
 * text.  This module has no Pico SDK or TinyUSB dependency so the host unit
 * tests (firmware/tests/test_midi_frame.c) compile it directly; the TinyUSB
 * glue and the dispatch live in midi_control.c.
 *
 * Packing ("8-into-7"): every group of up to 7 frame bytes becomes one MSB
 * byte (bit i = bit 7 of group byte i) followed by the low 7 bits of each
 * group byte.  A full group is 8 SysEx bytes, a final partial group of n
 * bytes is n + 1 SysEx bytes.  Unused MSB bits of a partial group are 0.
 */

#ifndef MIDI_FRAME_H
#define MIDI_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MF_SYSEX_START      0xF0u
#define MF_SYSEX_END        0xF7u
#define MF_VERSION          0x01u
#define MF_TAG_NOTIFY       0x00u   // device-initiated notification
#define MF_TAG_UNKNOWN      0x7Fu   // error reply when the request tag was not read

// UART frame types (uart_control.h), carried verbatim.
#define MF_TYPE_SET         0x01u
#define MF_TYPE_GET         0x02u
#define MF_TYPE_NOTIFY      0x40u
#define MF_RESP_SET         0x81u
#define MF_RESP_GET         0x82u

// CTRL_STATUS_* codes this module produces itself (same values as
// vendor_commands.h; repeated here so the module stays SDK-free).
#define MF_STATUS_OK          0x00u
#define MF_STATUS_CRC_ERROR   0x05u
#define MF_STATUS_FRAME_ERROR 0x07u

// USB-MIDI 1.0 Code Index Numbers used for SysEx.
#define MF_CIN_SYSEX_START  0x4u    // SysEx starts or continues (3 bytes)
#define MF_CIN_SYSEX_END1   0x5u    // SysEx ends with 1 byte (or 1-byte common)
#define MF_CIN_SYSEX_END2   0x6u
#define MF_CIN_SYSEX_END3   0x7u
#define MF_CIN_SINGLE_BYTE  0xFu

// Manufacturer SysEx ID: 1 byte, or 3 bytes starting with 0x00.
typedef struct {
    uint8_t bytes[3];
    uint8_t len;
} mf_mfr_id_t;

// ---------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------

// CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF), one byte at a time.
uint16_t mf_crc16_step(uint16_t crc, uint8_t b);
uint16_t mf_crc16(const uint8_t *p, size_t n);

// Packed length of n raw bytes.
size_t mf_packed_len(size_t n);
// Pack n raw bytes into out (mf_packed_len(n) bytes); returns that length.
size_t mf_pack(const uint8_t *raw, size_t n, uint8_t *out);
// Unpack n SysEx data bytes; returns the raw length or -1 if malformed (a
// byte >= 0x80, a lone trailing MSB byte, or a set unused MSB bit).
long mf_unpack(const uint8_t *in, size_t n, uint8_t *out);

// SysEx header length (F0 through tag) for an ID.
static inline uint8_t mf_header_len(const mf_mfr_id_t *id) {
    return (uint8_t)(1u + id->len + 3u + 1u + 1u);
}

// Extract the bytes of one USB-MIDI event packet that matter to the SysEx
// decoder: CIN 0x4-0x7 (SysEx) and 0xF (single byte) on `cable`.  Returns
// the byte count (0..3); every other packet yields 0.
uint8_t mf_packet_bytes(const uint8_t pkt[4], uint8_t cable, uint8_t out[3]);

// ---------------------------------------------------------------------------
// Encoder: streams one frame as SysEx bytes / USB-MIDI packets.
//
// The frame is fhead[] + payload[] + CRC16 (LE, over fhead and payload).
// The payload pointer is read lazily, so a large payload (REQ_GET_ALL_PARAMS
// from bulk_param_buf) is never copied.
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t        hdr[10];    // F0 mfr.. 'D' 'S' 'P' ver tag
    uint8_t        hdr_len, hdr_pos;
    uint8_t        fhead[9];
    uint8_t        fhead_len;
    const uint8_t *payload;
    uint32_t       plen;
    uint32_t       fpos;       // frame bytes consumed into groups
    uint32_t       flen;       // fhead_len + plen + 2
    uint16_t       crc;
    uint8_t        grp[8];
    uint8_t        grp_len, grp_pos;
    uint8_t        phase;      // 0 header, 1 data, 2 F7, 3 done
} mf_enc_t;

void mf_enc_begin(mf_enc_t *e, const mf_mfr_id_t *id, uint8_t tag,
                  const uint8_t *fhead, uint8_t fhead_len,
                  const uint8_t *payload, uint32_t plen);

// Device-to-host frame: type status lenL lenH payload crc.  The payload is
// only sent when status == OK (as on UART).
void mf_enc_begin_response(mf_enc_t *e, const mf_mfr_id_t *id, uint8_t tag,
                           uint8_t type, uint8_t status,
                           const uint8_t *payload, uint16_t len);

// Host-to-device request (used by the tests and as the reference encoder):
// type bReq wValue wIndex wLength payload crc.
void mf_enc_begin_request(mf_enc_t *e, const mf_mfr_id_t *id, uint8_t tag,
                          uint8_t type, uint8_t bReq, uint16_t wValue,
                          uint16_t wIndex, uint16_t wLength,
                          const uint8_t *payload);

static inline bool mf_enc_done(const mf_enc_t *e) { return e->phase == 3; }

// Payload bytes already read out of e->payload (for bulk-lock refresh).
static inline uint32_t mf_enc_payload_pos(const mf_enc_t *e) {
    uint32_t p = e->fpos > e->fhead_len ? e->fpos - e->fhead_len : 0;
    return p > e->plen ? e->plen : p;
}

// Copy up to `max` SysEx bytes; returns the count (0 once done).
size_t mf_enc_read(mf_enc_t *e, uint8_t *out, size_t max);

// Next USB-MIDI event packet; false once the message is complete.
bool mf_enc_packet(mf_enc_t *e, uint8_t cable, uint8_t pkt[4]);

// Total SysEx length of a frame with `flen` frame bytes.
static inline size_t mf_sysex_len(const mf_mfr_id_t *id, size_t flen) {
    return mf_header_len(id) + mf_packed_len(flen) + 1u;
}

// ---------------------------------------------------------------------------
// Decoder: SysEx bytes in, request events out.  Push one byte at a time.
// ---------------------------------------------------------------------------

typedef enum {
    MF_EV_NONE = 0,
    // A SET header with wLength > 0 was parsed.  Before the next feed the
    // caller sets d->dst (payload destination with room for wLength bytes,
    // or NULL to discard) and may set d->deferred_status (answered instead
    // of dispatching once the message completes).
    MF_EV_SET_HEADER,
    // A complete, CRC-valid request.  Answer d->deferred_status if it is not
    // OK, otherwise dispatch d->is_get / bReq / wValue / wIndex / wLength.
    MF_EV_REQUEST,
    // A message addressed to us was malformed.  Answer d->err_status with
    // tag d->err_tag and response type d->err_type; nothing is dispatched.
    MF_EV_ERROR,
} mf_event_t;

typedef struct {
    mf_mfr_id_t id;
    // SysEx layer
    uint8_t  st;
    uint8_t  hpos;
    uint8_t  tag;            // MF_TAG_UNKNOWN until read
    uint8_t  skip_status;    // pending error while skipping to F7
    uint8_t  msbs;
    uint8_t  gi;             // 0 = next byte is a group's MSB byte, else 1..7
    // frame layer
    uint8_t  fst;
    uint8_t  ftype;
    uint8_t  fhdr[7];
    uint8_t  fhpos;
    uint8_t  crc_pos;
    uint8_t  crc_lo;
    bool     crc_ok;
    uint16_t crc;
    uint32_t ppos;           // payload bytes received
    // request (valid from MF_EV_SET_HEADER / MF_EV_REQUEST)
    bool     is_get;
    uint8_t  bReq;
    uint16_t wValue, wIndex, wLength;
    uint8_t *dst;
    uint8_t  deferred_status;
    // error (valid from MF_EV_ERROR)
    uint8_t  err_status, err_tag, err_type;
} mf_dec_t;

void mf_dec_init(mf_dec_t *d, const mf_mfr_id_t *id);
// Drop any message in progress (no event).
void mf_dec_reset(mf_dec_t *d);
// True between F0 and F7 of any SysEx message.
bool mf_dec_busy(const mf_dec_t *d);
// True while a SET payload is being received.
bool mf_dec_in_payload(const mf_dec_t *d);
mf_event_t mf_dec_feed(mf_dec_t *d, uint8_t b);

#endif // MIDI_FRAME_H
