/*
 * alink_spdif_codec.c - the alink time base carried on S/PDIF (WP FWSP1)
 *
 * See alink_spdif_codec.h and Documentation/Features/alink_protocol_spec.md 12.
 */

#include "alink_spdif_codec.h"

ALINK_SPDIF_HOT
uint16_t alink_spdif_crc16(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? ((crc << 1) ^ 0x1021u) & 0xFFFFu : (crc << 1) & 0xFFFFu;
    }
    return (uint16_t)crc;
}

// Little-endian u64 through 32-bit halves: no 64-bit shift helpers.
static inline void spdif_put_u64le(uint8_t *p, uint64_t v) {
    uint32_t lo = (uint32_t)v, hi = (uint32_t)(v >> 32);
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(lo >> (8 * i));
        p[4 + i] = (uint8_t)(hi >> (8 * i));
    }
}

static inline uint64_t spdif_get_u64le(const uint8_t *p) {
    uint32_t lo = 0, hi = 0;
    for (int i = 0; i < 4; i++) {
        lo |= (uint32_t)p[i] << (8 * i);
        hi |= (uint32_t)p[4 + i] << (8 * i);
    }
    return ((uint64_t)hi << 32) | lo;
}

ALINK_SPDIF_HOT
void alink_spdif_sf_pack(const alink_spdif_sf_t *sf, uint8_t out[ALINK_SPDIF_SF_LEN]) {
    for (unsigned i = 0; i < ALINK_SPDIF_SF_LEN; i++) out[i] = 0;
    out[0] = ALINK_SPDIF_SF_MAGIC;
    out[1] = ALINK_SPDIF_VERSION;
    out[2] = sf->flags;
    out[3] = sf->rate;
    out[4] = sf->channels;
    out[5] = sf->cls;
    out[6] = sf->hops;
    out[7] = sf->sender;
    spdif_put_u64le(out + 8, sf->frame);
    spdif_put_u64le(out + 16, sf->apply_frame);
    out[24] = sf->apply_op;
    out[25] = sf->apply_arg;
    out[26] = (uint8_t)((uint16_t)sf->apply_value);
    out[27] = (uint8_t)((uint16_t)sf->apply_value >> 8);
    out[28] = sf->epoch;
    out[29] = sf->sender2;
    uint16_t crc = alink_spdif_crc16(out, ALINK_SPDIF_SF_CRC_LEN);
    out[30] = (uint8_t)crc;
    out[31] = (uint8_t)(crc >> 8);
}

// F mod 192 = 0 without a 64-bit division: 192 = 64 * 3, and 2^32 = 1 (mod 3).
static inline bool spdif_frame_aligned(uint64_t f) {
    uint32_t lo = (uint32_t)f, hi = (uint32_t)(f >> 32);
    if (lo & 63u) return false;
    return ((lo % 3u) + (hi % 3u)) % 3u == 0;
}

int alink_spdif_sf_unpack(const uint8_t in[ALINK_SPDIF_SF_LEN], alink_spdif_sf_t *sf) {
    uint16_t crc = (uint16_t)(in[30] | (in[31] << 8));
    if (alink_spdif_crc16(in, ALINK_SPDIF_SF_CRC_LEN) != crc) return ALINK_SPDIF_ERR_CRC;
    if (in[0] != ALINK_SPDIF_SF_MAGIC) return ALINK_SPDIF_ERR_MAGIC;
    if (in[1] != ALINK_SPDIF_VERSION) return ALINK_SPDIF_ERR_VERSION;
    sf->flags = in[2];
    sf->rate = in[3];
    sf->channels = in[4];
    sf->cls = in[5];
    sf->hops = in[6];
    sf->sender = in[7];
    sf->frame = spdif_get_u64le(in + 8);
    sf->apply_frame = spdif_get_u64le(in + 16);
    sf->apply_op = in[24];
    sf->apply_arg = in[25];
    sf->apply_value = (int16_t)(uint16_t)(in[26] | (in[27] << 8));
    sf->epoch = in[28];
    sf->sender2 = in[29];
    if (!spdif_frame_aligned(sf->frame)) return ALINK_SPDIF_ERR_FRAME;
    if (sf->cls != ALINK_SPDIF_CLASS || sf->rate > ALINK_SPDIF_RATE_MAX ||
        sf->channels != ALINK_SPDIF_CHANNELS)
        return ALINK_SPDIF_ERR_LAYOUT;
    if ((sf->apply_op != ALINK_SPDIF_OP_NONE && !alink_spdif_op_allowed(sf->apply_op)) ||
        (sf->apply_op == ALINK_SPDIF_OP_NONE) != (sf->apply_frame == 0))
        return ALINK_SPDIF_ERR_APPLY;
    return ALINK_SPDIF_OK;
}

ALINK_SPDIF_HOT
void alink_spdif_sf_build(const alink_spdif_src_t *src, uint64_t frame,
                          uint8_t out[ALINK_SPDIF_SF_LEN]) {
    alink_spdif_sf_t sf = {
        .flags = (uint8_t)(src->flags & (ALINK_SPDIF_FLAG_MUTE | ALINK_SPDIF_FLAG_USB_LIVE)),
        .rate = src->rate,
        .channels = ALINK_SPDIF_CHANNELS,
        .cls = ALINK_SPDIF_CLASS,
        .sender = src->sender,
        .frame = frame,
        .epoch = src->epoch,
        .sender2 = src->sender2,
    };
    if (src->apply_op != ALINK_SPDIF_OP_NONE && alink_spdif_carries(frame, src->apply_frame)) {
        sf.flags |= ALINK_SPDIF_FLAG_APPLY_PEND;
        sf.apply_frame = src->apply_frame;
        sf.apply_op = src->apply_op;
        sf.apply_arg = src->apply_arg;
        sf.apply_value = src->apply_value;
    }
    alink_spdif_sf_pack(&sf, out);
}
