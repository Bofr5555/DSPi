/*
 * midi_frame.c - pure codec for the USB MIDI control channel
 *
 * See midi_frame.h.  No SDK headers: compiled into the firmware only when
 * DSPI_USB_MIDI is ON, and directly by firmware/tests/test_midi_frame.c.
 */

#include "midi_frame.h"
#include <string.h>

// ---------------------------------------------------------------------------
// CRC16-CCITT-FALSE.  Same table as uart_control.c (kept separate so the
// UART translation unit, and therefore the default build, stays untouched).
// ---------------------------------------------------------------------------

static const uint16_t mf_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

uint16_t mf_crc16_step(uint16_t crc, uint8_t b) {
    return (uint16_t)((crc << 8) ^ mf_crc16_table[(crc >> 8) ^ b]);
}

uint16_t mf_crc16(const uint8_t *p, size_t n) {
    uint16_t c = 0xFFFF;
    while (n--) c = mf_crc16_step(c, *p++);
    return c;
}

// ---------------------------------------------------------------------------
// 7-bit packing (whole buffers; the encoder/decoder below stream the same
// scheme byte by byte)
// ---------------------------------------------------------------------------

size_t mf_packed_len(size_t n) {
    size_t r = n % 7u;
    return (n / 7u) * 8u + (r ? r + 1u : 0u);
}

size_t mf_pack(const uint8_t *raw, size_t n, uint8_t *out) {
    size_t o = 0;
    for (size_t i = 0; i < n; i += 7) {
        size_t g = (n - i < 7u) ? n - i : 7u;
        uint8_t msbs = 0;
        for (size_t k = 0; k < g; k++)
            msbs |= (uint8_t)(((raw[i + k] >> 7) & 1u) << k);
        out[o++] = msbs;
        for (size_t k = 0; k < g; k++) out[o++] = raw[i + k] & 0x7Fu;
    }
    return o;
}

long mf_unpack(const uint8_t *in, size_t n, uint8_t *out) {
    size_t o = 0;
    for (size_t i = 0; i < n; i += 8) {
        size_t g = (n - i < 8u) ? n - i : 8u;   // MSB byte + up to 7 data
        if (g < 2) return -1;                   // lone MSB byte
        uint8_t msbs = in[i];
        if (msbs & 0x80u) return -1;
        if (msbs >> (g - 1)) return -1;         // bit set for a missing byte
        for (size_t k = 1; k < g; k++) {
            if (in[i + k] & 0x80u) return -1;
            out[o++] = (uint8_t)(in[i + k] | (((msbs >> (k - 1)) & 1u) << 7));
        }
    }
    return (long)o;
}

uint8_t mf_packet_bytes(const uint8_t pkt[4], uint8_t cable, uint8_t out[3]) {
    if ((pkt[0] >> 4) != cable) return 0;
    uint8_t n;
    switch (pkt[0] & 0x0Fu) {
        case MF_CIN_SYSEX_START: n = 3; break;
        case MF_CIN_SYSEX_END1:  n = 1; break;
        case MF_CIN_SYSEX_END2:  n = 2; break;
        case MF_CIN_SYSEX_END3:  n = 3; break;
        case MF_CIN_SINGLE_BYTE: n = 1; break;
        default: return 0;
    }
    memcpy(out, &pkt[1], n);
    return n;
}

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

static void enc_header(mf_enc_t *e, const mf_mfr_id_t *id, uint8_t tag) {
    uint8_t n = 0;
    e->hdr[n++] = MF_SYSEX_START;
    for (uint8_t i = 0; i < id->len && i < 3; i++) e->hdr[n++] = id->bytes[i];
    e->hdr[n++] = 'D';
    e->hdr[n++] = 'S';
    e->hdr[n++] = 'P';
    e->hdr[n++] = MF_VERSION;
    e->hdr[n++] = tag & 0x7Fu;
    e->hdr_len = n;
    e->hdr_pos = 0;
}

void mf_enc_begin(mf_enc_t *e, const mf_mfr_id_t *id, uint8_t tag,
                  const uint8_t *fhead, uint8_t fhead_len,
                  const uint8_t *payload, uint32_t plen) {
    enc_header(e, id, tag);
    if (fhead_len > sizeof(e->fhead)) fhead_len = sizeof(e->fhead);
    memcpy(e->fhead, fhead, fhead_len);
    e->fhead_len = fhead_len;
    e->payload = payload;
    e->plen = payload ? plen : 0;
    e->fpos = 0;
    e->flen = (uint32_t)fhead_len + e->plen + 2u;
    e->crc = 0xFFFF;
    e->grp_len = 0;
    e->grp_pos = 0;
    e->phase = 0;
}

void mf_enc_begin_response(mf_enc_t *e, const mf_mfr_id_t *id, uint8_t tag,
                           uint8_t type, uint8_t status,
                           const uint8_t *payload, uint16_t len) {
    if (status != MF_STATUS_OK) len = 0;
    uint8_t h[4] = { type, status, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    mf_enc_begin(e, id, tag, h, 4, len ? payload : NULL, len);
}

void mf_enc_begin_request(mf_enc_t *e, const mf_mfr_id_t *id, uint8_t tag,
                          uint8_t type, uint8_t bReq, uint16_t wValue,
                          uint16_t wIndex, uint16_t wLength,
                          const uint8_t *payload) {
    uint8_t h[8] = {
        type, bReq,
        (uint8_t)(wValue & 0xFF), (uint8_t)(wValue >> 8),
        (uint8_t)(wIndex & 0xFF), (uint8_t)(wIndex >> 8),
        (uint8_t)(wLength & 0xFF), (uint8_t)(wLength >> 8),
    };
    bool has_payload = (type == MF_TYPE_SET) && wLength;
    mf_enc_begin(e, id, tag, h, 8, has_payload ? payload : NULL,
                 has_payload ? wLength : 0);
}

// Next raw frame byte (CRC accumulated over everything before the CRC).
static uint8_t enc_frame_byte(mf_enc_t *e) {
    uint32_t p = e->fpos++;
    uint8_t b;
    if (p < e->fhead_len) {
        b = e->fhead[p];
    } else if (p < (uint32_t)e->fhead_len + e->plen) {
        b = e->payload[p - e->fhead_len];
    } else if (p == (uint32_t)e->fhead_len + e->plen) {
        return (uint8_t)(e->crc & 0xFF);
    } else {
        return (uint8_t)(e->crc >> 8);
    }
    e->crc = mf_crc16_step(e->crc, b);
    return b;
}

// Load the next packed group (MSB byte + up to 7 data bytes).
static void enc_fill_group(mf_enc_t *e) {
    uint32_t left = e->flen - e->fpos;
    uint8_t g = (uint8_t)(left < 7u ? left : 7u);
    uint8_t msbs = 0;
    for (uint8_t k = 0; k < g; k++) {
        uint8_t b = enc_frame_byte(e);
        msbs |= (uint8_t)(((b >> 7) & 1u) << k);
        e->grp[1 + k] = b & 0x7Fu;
    }
    e->grp[0] = msbs;
    e->grp_len = (uint8_t)(g + 1u);
    e->grp_pos = 0;
}

static bool enc_next(mf_enc_t *e, uint8_t *out) {
    for (;;) {
        switch (e->phase) {
            case 0:
                if (e->hdr_pos < e->hdr_len) { *out = e->hdr[e->hdr_pos++]; return true; }
                e->phase = 1;
                break;
            case 1:
                if (e->grp_pos < e->grp_len) { *out = e->grp[e->grp_pos++]; return true; }
                if (e->fpos >= e->flen) { e->phase = 2; break; }
                enc_fill_group(e);
                break;
            case 2:
                *out = MF_SYSEX_END;
                e->phase = 3;
                return true;
            default:
                return false;
        }
    }
}

size_t mf_enc_read(mf_enc_t *e, uint8_t *out, size_t max) {
    size_t n = 0;
    while (n < max && enc_next(e, &out[n])) n++;
    return n;
}

bool mf_enc_packet(mf_enc_t *e, uint8_t cable, uint8_t pkt[4]) {
    uint8_t b[3] = { 0, 0, 0 };
    size_t n = mf_enc_read(e, b, 3);
    if (n == 0) return false;
    uint8_t cin = (b[n - 1] == MF_SYSEX_END) ? (uint8_t)(MF_CIN_SYSEX_END1 + n - 1u)
                                              : MF_CIN_SYSEX_START;
    pkt[0] = (uint8_t)((cable << 4) | cin);
    pkt[1] = b[0];
    pkt[2] = b[1];
    pkt[3] = b[2];
    return true;
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

enum {
    D_IDLE = 0,   // waiting for F0
    D_MFR,        // matching the manufacturer ID
    D_MAGIC,      // matching 'D' 'S' 'P'
    D_VER,
    D_TAG,
    D_DATA,       // packed frame bytes
    D_IGNORE,     // not ours: skip silently to F7
    D_SKIP_ERR,   // ours but malformed: skip to F7, then report skip_status
};

enum { F_TYPE = 0, F_HDR, F_PAYLOAD, F_CRC, F_DONE };

static const uint8_t mf_magic[3] = { 'D', 'S', 'P' };

void mf_dec_reset(mf_dec_t *d) {
    d->st = D_IDLE;
    d->hpos = 0;
    d->tag = MF_TAG_UNKNOWN;
    d->skip_status = MF_STATUS_FRAME_ERROR;
    d->gi = 0;
    d->msbs = 0;
    d->fst = F_TYPE;
    d->ftype = 0;
    d->fhpos = 0;
    d->crc_pos = 0;
    d->crc_ok = false;
    d->crc = 0xFFFF;
    d->ppos = 0;
    d->dst = NULL;
    d->deferred_status = MF_STATUS_OK;
}

void mf_dec_init(mf_dec_t *d, const mf_mfr_id_t *id) {
    memset(d, 0, sizeof(*d));
    d->id = *id;
    mf_dec_reset(d);
}

bool mf_dec_busy(const mf_dec_t *d) { return d->st != D_IDLE; }

bool mf_dec_in_payload(const mf_dec_t *d) {
    return d->st == D_DATA && d->fst == F_PAYLOAD;
}

// The message is ours (header matched) from D_VER on.
static bool dec_ours(const mf_dec_t *d) {
    return d->st == D_VER || d->st == D_TAG || d->st == D_DATA || d->st == D_SKIP_ERR;
}

static uint8_t dec_resp_type(const mf_dec_t *d) {
    return (d->ftype == MF_TYPE_GET) ? MF_RESP_GET : MF_RESP_SET;
}

static mf_event_t dec_error(mf_dec_t *d, uint8_t status) {
    d->err_status = status;
    d->err_tag = d->tag;
    d->err_type = dec_resp_type(d);
    return MF_EV_ERROR;
}

static void dec_skip(mf_dec_t *d, uint8_t status) {
    d->skip_status = status;
    d->st = D_SKIP_ERR;
}

static void dec_start(mf_dec_t *d) {
    mf_dec_reset(d);
    d->st = D_MFR;
}

// One unpacked frame byte.
static mf_event_t dec_frame_byte(mf_dec_t *d, uint8_t b) {
    switch (d->fst) {
        case F_TYPE:
            d->ftype = b;
            if (b != MF_TYPE_SET && b != MF_TYPE_GET) {
                dec_skip(d, MF_STATUS_FRAME_ERROR);
                return MF_EV_NONE;
            }
            d->is_get = (b == MF_TYPE_GET);
            d->crc = mf_crc16_step(0xFFFF, b);
            d->fhpos = 0;
            d->fst = F_HDR;
            return MF_EV_NONE;

        case F_HDR:
            d->fhdr[d->fhpos++] = b;
            d->crc = mf_crc16_step(d->crc, b);
            if (d->fhpos < 7) return MF_EV_NONE;
            d->bReq    = d->fhdr[0];
            d->wValue  = (uint16_t)(d->fhdr[1] | (d->fhdr[2] << 8));
            d->wIndex  = (uint16_t)(d->fhdr[3] | (d->fhdr[4] << 8));
            d->wLength = (uint16_t)(d->fhdr[5] | (d->fhdr[6] << 8));
            d->ppos = 0;
            d->dst = NULL;
            d->deferred_status = MF_STATUS_OK;
            if (d->is_get || d->wLength == 0) {
                d->crc_pos = 0;
                d->fst = F_CRC;
                return MF_EV_NONE;
            }
            d->fst = F_PAYLOAD;
            return MF_EV_SET_HEADER;

        case F_PAYLOAD:
            d->crc = mf_crc16_step(d->crc, b);
            if (d->dst) d->dst[d->ppos] = b;
            if (++d->ppos == d->wLength) { d->crc_pos = 0; d->fst = F_CRC; }
            return MF_EV_NONE;

        case F_CRC:
            if (d->crc_pos == 0) { d->crc_lo = b; d->crc_pos = 1; return MF_EV_NONE; }
            d->crc_ok = ((uint16_t)(d->crc_lo | (b << 8)) == d->crc);
            d->fst = F_DONE;
            return MF_EV_NONE;

        default:   // trailing bytes after the CRC
            dec_skip(d, MF_STATUS_FRAME_ERROR);
            return MF_EV_NONE;
    }
}

// F7 closing a message in D_DATA.
static mf_event_t dec_finish(mf_dec_t *d) {
    bool group_ok = true;
    if (d->gi == 1) group_ok = false;                    // lone MSB byte
    else if (d->gi > 1 && (d->msbs >> (d->gi - 1)))      // bit for a missing byte
        group_ok = false;
    if (d->fst != F_DONE || !group_ok) return dec_error(d, MF_STATUS_FRAME_ERROR);
    if (!d->crc_ok) return dec_error(d, MF_STATUS_CRC_ERROR);
    return MF_EV_REQUEST;
}

mf_event_t mf_dec_feed(mf_dec_t *d, uint8_t b) {
    if (b >= 0x80u) {
        if (b >= 0xF8u) return MF_EV_NONE;   // real-time: may interleave, ignore

        if (b == MF_SYSEX_END) {
            mf_event_t ev = MF_EV_NONE;
            if (d->st == D_DATA) {
                ev = dec_finish(d);
            } else if (d->st == D_SKIP_ERR) {
                ev = dec_error(d, d->skip_status);
            } else if (d->st == D_VER || d->st == D_TAG) {
                ev = dec_error(d, MF_STATUS_FRAME_ERROR);   // truncated header
            }
            // Back to idle; the request / error fields stay readable until
            // the next F0 starts a new message.
            d->st = D_IDLE;
            return ev;
        }

        // F0 (new message) or a system-common status byte (ends SysEx).
        mf_event_t ev = dec_ours(d) ? dec_error(d, MF_STATUS_FRAME_ERROR) : MF_EV_NONE;
        if (b == MF_SYSEX_START) dec_start(d);
        else                     mf_dec_reset(d);
        return ev;
    }

    switch (d->st) {
        case D_IDLE:
        case D_IGNORE:
        case D_SKIP_ERR:
            return MF_EV_NONE;

        case D_MFR:
            if (b != d->id.bytes[d->hpos]) { d->st = D_IGNORE; return MF_EV_NONE; }
            if (++d->hpos == d->id.len) { d->hpos = 0; d->st = D_MAGIC; }
            return MF_EV_NONE;

        case D_MAGIC:
            if (b != mf_magic[d->hpos]) { d->st = D_IGNORE; return MF_EV_NONE; }
            if (++d->hpos == 3) d->st = D_VER;
            return MF_EV_NONE;

        case D_VER:
            if (b != MF_VERSION) { dec_skip(d, MF_STATUS_FRAME_ERROR); return MF_EV_NONE; }
            d->st = D_TAG;
            return MF_EV_NONE;

        case D_TAG:
            if (b == MF_TAG_NOTIFY) { dec_skip(d, MF_STATUS_FRAME_ERROR); return MF_EV_NONE; }
            d->tag = b;
            d->gi = 0;
            d->fst = F_TYPE;
            d->st = D_DATA;
            return MF_EV_NONE;

        case D_DATA:
            if (d->gi == 0) { d->msbs = b; d->gi = 1; return MF_EV_NONE; }
            {
                uint8_t raw = (uint8_t)(b | (((d->msbs >> (d->gi - 1)) & 1u) << 7));
                d->gi = (d->gi == 7) ? 0 : (uint8_t)(d->gi + 1);
                return dec_frame_byte(d, raw);
            }

        default:
            return MF_EV_NONE;
    }
}
