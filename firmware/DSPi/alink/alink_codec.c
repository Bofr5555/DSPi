/*
 * alink_codec.c - DSPi alink protocol v1: reference encoder and primitives
 *
 * See alink_codec.h and Documentation/Features/alink_protocol_spec.md.
 */

#include "alink_codec.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Classes, rates and the frame layout
// ---------------------------------------------------------------------------

uint32_t alink_class_baud(unsigned cls) {
    switch (cls) {
    case ALINK_CLASS_L1: return 15360000u;
    case ALINK_CLASS_L2: return 30720000u;
    case ALINK_CLASS_L3: return 61440000u;
    default: return 0;
    }
}

uint32_t alink_rate_hz(unsigned rate) {
    switch (rate) {
    case ALINK_RATE_44K1: return 44100u;
    case ALINK_RATE_48K: return 48000u;
    case ALINK_RATE_96K: return 96000u;
    default: return 0;
    }
}

static uint32_t gcd_u32(uint32_t a, uint32_t b) {
    while (b) { uint32_t t = a % b; a = b; b = t; }
    return a;
}

// Symbols per frame as the reduced fraction num/den (baud / (10 * Fs)).
static bool frame_ratio(unsigned cls, unsigned rate, uint32_t *num, uint32_t *den) {
    uint32_t baud = alink_class_baud(cls), fs = alink_rate_hz(rate);
    if (!baud || !fs) return false;
    uint32_t d = 10u * fs, g = gcd_u32(baud, d);
    *num = baud / g;
    *den = d / g;
    return true;
}

unsigned alink_frame_symbols(unsigned cls, unsigned rate, uint64_t frame) {
    uint32_t num, den;
    if (!frame_ratio(cls, rate, &num, &den)) return 0;
    uint64_t m = frame % den;
    return (unsigned)(((m + 1u) * num) / den - (m * num) / den);
}

unsigned alink_frame_symbols_min(unsigned cls, unsigned rate) {
    uint32_t num, den;
    if (!frame_ratio(cls, rate, &num, &den)) return 0;
    return (unsigned)(num / den);
}

unsigned alink_frame_symbols_max(unsigned cls, unsigned rate) {
    uint32_t num, den;
    if (!frame_ratio(cls, rate, &num, &den)) return 0;
    return (unsigned)((num + den - 1u) / den);
}

unsigned alink_layout_symbols(unsigned channels) {
    unsigned n = ALINK_HEAD_SYMBOLS;
    for (unsigned b = 0; b < alink_bank_count(channels); b++)
        n += ALINK_BANK_SYMBOLS(alink_bank_channels(channels, b));
    return n;
}

unsigned alink_max_channels(unsigned cls, unsigned rate) {
    unsigned room = alink_frame_symbols_min(cls, rate), c = 0;
    while (c < ALINK_MAX_CHANNELS && alink_layout_symbols(c + 1u) <= room) c++;
    return c;
}

// ---------------------------------------------------------------------------
// 8b/10b
// ---------------------------------------------------------------------------

// 5b/6b, "abcdei" with a in bit 5, index x = EDCBA; [x][0] RD-, [x][1] RD+.
static const uint8_t enc6[32][2] = {
    {0x27, 0x18}, {0x1D, 0x22}, {0x2D, 0x12}, {0x31, 0x31}, // D.00-03
    {0x35, 0x0A}, {0x29, 0x29}, {0x19, 0x19}, {0x38, 0x07}, // D.04-07
    {0x39, 0x06}, {0x25, 0x25}, {0x15, 0x15}, {0x34, 0x34}, // D.08-11
    {0x0D, 0x0D}, {0x2C, 0x2C}, {0x1C, 0x1C}, {0x17, 0x28}, // D.12-15
    {0x1B, 0x24}, {0x23, 0x23}, {0x13, 0x13}, {0x32, 0x32}, // D.16-19
    {0x0B, 0x0B}, {0x2A, 0x2A}, {0x1A, 0x1A}, {0x3A, 0x05}, // D.20-23
    {0x33, 0x0C}, {0x26, 0x26}, {0x16, 0x16}, {0x36, 0x09}, // D.24-27
    {0x0E, 0x0E}, {0x2E, 0x11}, {0x1E, 0x21}, {0x2B, 0x14}, // D.28-31
};
#define ENC6_K28_RDN 0x0Fu  // 001111
#define ENC6_K28_RDP 0x30u  // 110000

// 3b/4b, "fghj" with f in bit 3, index y = HGF; [y][0] RD-, [y][1] RD+.
// Data y = 7 is the primary P7 here; A7 is used per the alternate rule.
static const uint8_t enc4d[8][2] = {
    {0xB, 0x4}, {0x9, 0x9}, {0x5, 0x5}, {0xC, 0x3},
    {0xD, 0x2}, {0xA, 0xA}, {0x6, 0x6}, {0xE, 0x1},
};
static const uint8_t enc4k[8][2] = {
    {0xB, 0x4}, {0x6, 0x9}, {0xA, 0x5}, {0xC, 0x3},
    {0xD, 0x2}, {0x5, 0xA}, {0x9, 0x6}, {0x7, 0x8},
};
#define ENC4_A7_RDN 0x7u    // 0111
#define ENC4_A7_RDP 0x8u    // 1000

static unsigned popcount_u32(uint32_t v) {
    unsigned n = 0;
    while (v) { v &= v - 1u; n++; }
    return n;
}

// Running disparity after a 6-bit sub-block.
static unsigned rd_after6(uint8_t s6, unsigned rd) {
    unsigned ones = popcount_u32(s6);
    if (ones > 3) return 1;
    if (ones < 3) return 0;
    if (s6 == 0x07) return 1;   // 000111
    if (s6 == 0x38) return 0;   // 111000
    return rd;
}

// Running disparity after a 4-bit sub-block.
static unsigned rd_after4(uint8_t s4, unsigned rd) {
    unsigned ones = popcount_u32(s4);
    if (ones > 2) return 1;
    if (ones < 2) return 0;
    if (s4 == 0x3) return 1;    // 0011
    if (s4 == 0xC) return 0;    // 1100
    return rd;
}

uint16_t alink_8b10b_encode(uint8_t byte, bool k, unsigned *rd) {
    unsigned x = byte & 0x1Fu, y = byte >> 5, r = *rd & 1u;
    uint8_t s6, s4;
    if (k) {
        bool k28 = x == 28;
        if (!k28 && !(y == 7 && (x == 23 || x == 27 || x == 29 || x == 30)))
            return ALINK_SYM_INVALID;
        s6 = k28 ? (r ? ENC6_K28_RDP : ENC6_K28_RDN) : enc6[x][r];
        r = rd_after6(s6, r);
        s4 = enc4k[y][r];
    } else {
        s6 = enc6[x][r];
        r = rd_after6(s6, r);
        if (y == 7 && ((r == 0 && (x == 17 || x == 18 || x == 20)) ||
                       (r == 1 && (x == 11 || x == 13 || x == 14))))
            s4 = r ? ENC4_A7_RDP : ENC4_A7_RDN;
        else
            s4 = enc4d[y][r];
    }
    *rd = rd_after4(s4, r);
    return (uint16_t)(((unsigned)s6 << 4) | s4);
}

unsigned alink_8b10b_rd_after(uint16_t sym, unsigned rd) {
    return rd_after4((uint8_t)(sym & 0xFu), rd_after6((uint8_t)((sym >> 4) & 0x3Fu), rd & 1u));
}

// Decoding: the sub-block tables below only propose a candidate (byte, k);
// the candidate is accepted for a running disparity when re-encoding it there
// gives the symbol back, so the decoder accepts exactly the encoder's image.
// dec6: 6-bit sub-block -> x (0..31), 28 | 0x40 for the K28 sub-blocks, 0xFF none.
static const uint8_t dec6[64] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x17, 0x08, 0x07, 0xFF, 0x1B, 0x04, 0x14, 0x18, 0x0C, 0x1C, 0x5C,
    0xFF, 0x1D, 0x02, 0x12, 0x1F, 0x0A, 0x1A, 0x0F, 0x00, 0x06, 0x16, 0x10, 0x0E, 0x01, 0x1E, 0xFF,
    0xFF, 0x1E, 0x01, 0x11, 0x10, 0x09, 0x19, 0x00, 0x0F, 0x05, 0x15, 0x1F, 0x0D, 0x02, 0x1D, 0xFF,
    0x5C, 0x03, 0x13, 0x18, 0x0B, 0x04, 0x1B, 0xFF, 0x07, 0x08, 0x17, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};
// dec4: 4-bit data sub-block -> y (P7 and A7 both give 7), 0xFF none.
static const uint8_t dec4[16] = {
    0xFF, 0x07, 0x04, 0x03, 0x00, 0x02, 0x06, 0x07, 0x07, 0x01, 0x05, 0x00, 0x03, 0x04, 0x07, 0xFF,
};

static unsigned accept_rds(uint16_t sym, uint8_t byte, bool k) {
    unsigned mask = 0;
    for (unsigned r = 0; r < 2; r++) {
        unsigned rd = r;
        if (alink_8b10b_encode(byte, k, &rd) == sym) mask |= 1u << r;
    }
    return mask;
}

unsigned alink_8b10b_decode(uint16_t sym, uint8_t *byte, bool *k) {
    if (sym > 0x3FFu) return 0;
    uint8_t x = dec6[sym >> 4], y = dec4[sym & 0xFu];
    if (x == 0xFF || y == 0xFF) return 0;
    if (x & 0x40) {
        // K28.y: after 110000 (RD- follows) the K column swaps 1<->6 and 2<->5.
        if ((sym >> 4) == ENC6_K28_RDP) {
            static const uint8_t swap[8] = { 0, 6, 5, 3, 4, 2, 1, 7 };
            y = swap[y];
        }
        uint8_t b = (uint8_t)((y << 5) | 28u);
        unsigned m = accept_rds(sym, b, true);
        if (m) { *byte = b; *k = true; }
        return m;
    }
    uint8_t b = (uint8_t)((y << 5) | x);
    unsigned m = accept_rds(sym, b, false);
    if (m) { *byte = b; *k = false; return m; }
    if (y == 7 && (x == 23 || x == 27 || x == 29 || x == 30)) {
        m = accept_rds(sym, b, true);
        if (m) { *byte = b; *k = true; }
    }
    return m;
}

// ---------------------------------------------------------------------------
// CRCs, PRBS-7
// ---------------------------------------------------------------------------

uint8_t alink_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (uint8_t)((crc & 0x80u) ? ((unsigned)crc << 1) ^ 0x07u : (unsigned)crc << 1);
    }
    return crc;
}

uint16_t alink_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (uint16_t)((crc & 0x8000u) ? ((unsigned)crc << 1) ^ 0x1021u : (unsigned)crc << 1);
    }
    return crc;
}

void alink_prbs7_fill(uint8_t *out, size_t len) {
    unsigned s = 0x7Fu;
    for (size_t i = 0; i < len; i++) {
        uint8_t v = 0;
        for (int b = 0; b < 8; b++) {
            unsigned nb = ((s >> 6) ^ (s >> 5)) & 1u;
            s = ((s << 1) | nb) & 0x7Fu;
            v = (uint8_t)((v << 1) | nb);
        }
        out[i] = v;
    }
}

// ---------------------------------------------------------------------------
// Superframe
// ---------------------------------------------------------------------------

static void put_u64le(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i));
}

static uint64_t get_u64le(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | p[i];
    return v;
}

void alink_sf_pack(const alink_sf_t *sf, uint8_t out[ALINK_SF_LEN]) {
    memset(out, 0, ALINK_SF_LEN);
    out[0] = ALINK_SF_MAGIC;
    out[1] = ALINK_PROTOCOL_VERSION;
    out[2] = sf->flags;
    out[3] = sf->rate;
    out[4] = sf->channels;
    out[5] = sf->cls;
    out[6] = sf->hops;
    out[7] = sf->sender;
    put_u64le(out + 8, sf->frame);
    put_u64le(out + 16, sf->apply_frame);
    out[24] = sf->apply_op;
    out[25] = sf->apply_arg;
    out[26] = (uint8_t)((uint16_t)sf->apply_value);
    out[27] = (uint8_t)((uint16_t)sf->apply_value >> 8);
    uint16_t crc = alink_crc16(out, 30);
    out[30] = (uint8_t)crc;
    out[31] = (uint8_t)(crc >> 8);
}

int alink_sf_unpack(const uint8_t in[ALINK_SF_LEN], alink_sf_t *sf) {
    return alink_sf_unpack_ops(in, sf, ALINK_OP_MAX);
}

int alink_sf_unpack_ops(const uint8_t in[ALINK_SF_LEN], alink_sf_t *sf, unsigned op_max) {
    uint16_t crc = (uint16_t)(in[30] | (in[31] << 8));
    if (alink_crc16(in, 30) != crc) return ALINK_SF_ERR_CRC;
    if (in[0] != ALINK_SF_MAGIC) return ALINK_SF_ERR_MAGIC;
    if (in[1] != ALINK_PROTOCOL_VERSION) return ALINK_SF_ERR_VERSION;
    sf->flags = in[2];
    sf->rate = in[3];
    sf->channels = in[4];
    sf->cls = in[5];
    sf->hops = in[6];
    sf->sender = in[7];
    sf->frame = get_u64le(in + 8);
    sf->apply_frame = get_u64le(in + 16);
    sf->apply_op = in[24];
    sf->apply_arg = in[25];
    sf->apply_value = (int16_t)(uint16_t)(in[26] | (in[27] << 8));
    if (sf->frame % ALINK_SF_LEN) return ALINK_SF_ERR_FRAME;
    if (!alink_class_baud(sf->cls) || !alink_rate_hz(sf->rate) || sf->channels < 1 ||
        sf->channels > alink_max_channels(sf->cls, sf->rate))
        return ALINK_SF_ERR_LAYOUT;
    if (sf->apply_op > op_max || (sf->apply_op == ALINK_OP_NONE) != (sf->apply_frame == 0))
        return ALINK_SF_ERR_APPLY;
    return ALINK_SF_OK;
}

void alink_sf_repeat(const alink_sf_t *in, uint8_t out_cls, alink_sf_t *out) {
    *out = *in;
    out->hops = in->hops == 0xFFu ? 0xFFu : (uint8_t)(in->hops + 1u);
    out->cls = out_cls;
}

// ---------------------------------------------------------------------------
// Bit packing
// ---------------------------------------------------------------------------

bool alink_bitw_put(alink_bitw_t *w, uint32_t v, unsigned n) {
    if (n > 32 || w->nbits + n > w->cap_words * 32u) return false;
    for (unsigned i = n; i-- > 0;) {
        size_t wi = w->nbits >> 5;
        uint32_t bit = 0x80000000u >> (w->nbits & 31u);
        if ((w->nbits & 31u) == 0) w->words[wi] = 0;
        if ((v >> i) & 1u) w->words[wi] |= bit;
        w->nbits++;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Frame encoder
// ---------------------------------------------------------------------------

static void tx_pack_sf(alink_tx_t *tx, uint64_t first) {
    alink_sf_t sf = {
        .flags = (uint8_t)(tx->flags & (ALINK_SF_FLAGS_DEFINED & ~(ALINK_SF_FLAG_RATE_PEND |
                                                                   ALINK_SF_FLAG_APPLY_PEND))),
        .rate = tx->rate, .channels = tx->channels, .cls = tx->cls,
        .hops = tx->hops, .sender = tx->sender, .frame = first,
        .apply_frame = tx->apply_frame, .apply_op = tx->apply_op,
        .apply_arg = tx->apply_arg, .apply_value = tx->apply_value,
    };
    if (sf.apply_op != ALINK_OP_NONE) sf.flags |= ALINK_SF_FLAG_APPLY_PEND;
    if (sf.apply_op == ALINK_OP_RATE) sf.flags |= ALINK_SF_FLAG_RATE_PEND;
    alink_sf_pack(&sf, tx->sf);
}

bool alink_tx_init(alink_tx_t *tx, unsigned cls, unsigned rate, unsigned channels,
                  uint64_t first, uint8_t sender) {
    memset(tx, 0, sizeof(*tx));
    if (!alink_class_baud(cls) || !alink_rate_hz(rate) || channels < 1 ||
        channels > alink_max_channels(cls, rate))
        return false;
    tx->cls = (uint8_t)cls;
    tx->rate = (uint8_t)rate;
    tx->channels = (uint8_t)channels;
    tx->frame = first;
    tx->sender = sender;
    return true;
}

bool alink_tx_schedule(alink_tx_t *tx, uint64_t at, uint8_t op, uint8_t arg, int16_t value) {
    uint64_t next_sf = (tx->frame + ALINK_SF_LEN - 1u) & ~(uint64_t)(ALINK_SF_LEN - 1u);
    if (op == ALINK_OP_NONE || op > ALINK_OP_MAX || tx->apply_op != ALINK_OP_NONE) return false;
    if (at < next_sf + ALINK_SF_LEN) return false;
    if (op == ALINK_OP_RATE) {
        unsigned c = value ? (unsigned)value : tx->channels;
        if (at % ALINK_SF_LEN || value < 0 || !alink_rate_hz(arg) || c < 1 ||
            c > alink_max_channels(tx->cls, arg))
            return false;
    }
    tx->apply_frame = at;
    tx->apply_op = op;
    tx->apply_arg = arg;
    tx->apply_value = value;
    return true;
}

static bool put_sym(alink_tx_t *tx, alink_bitw_t *w, uint8_t byte, bool k) {
    return alink_bitw_put(w, alink_8b10b_encode(byte, k, &tx->rd), 10);
}

bool alink_tx_frame(alink_tx_t *tx, const int32_t *samples, alink_bitw_t *w) {
    uint64_t f = tx->frame;
    // Pack at every superframe start, and at the first frame even when the
    // stream starts inside a superframe (that partial one is still valid).
    if (f % ALINK_SF_LEN == 0 || !tx->started) {
        uint64_t first = f & ~(uint64_t)(ALINK_SF_LEN - 1u);
        // Carried while the superframe completes before the target.
        if (tx->apply_op != ALINK_OP_NONE && first + ALINK_SF_LEN > tx->apply_frame) {
            tx->apply_frame = 0;
            tx->apply_op = ALINK_OP_NONE;
            tx->apply_arg = 0;
            tx->apply_value = 0;
        }
        tx_pack_sf(tx, first);
        tx->started = true;
    }

    uint8_t head[3];
    head[0] = (uint8_t)f;
    head[1] = tx->sf[f % ALINK_SF_LEN];
    head[2] = alink_crc8(head, 2);
    bool ok = put_sym(tx, w, ALINK_BYTE_K28_5, true);
    for (int i = 0; i < 3; i++) ok = ok && put_sym(tx, w, head[i], false);

    unsigned c = tx->channels, used = alink_layout_symbols(c);
    uint8_t payload[3 * ALINK_MAX_CHANNELS];
    if (tx->sf[2] & ALINK_SF_FLAG_PRBS) {
        alink_prbs7_fill(payload, 3u * c);
    } else {
        for (unsigned ch = 0; ch < c; ch++) alink_put_s24(payload + 3u * ch, samples[ch]);
    }
    for (unsigned b = 0; b < alink_bank_count(c); b++) {
        const uint8_t *p = payload + 3u * ALINK_BANK_CHANNELS * b;
        unsigned n = 3u * alink_bank_channels(c, b);
        uint16_t crc = alink_crc16(p, n);
        for (unsigned i = 0; i < n; i++) ok = ok && put_sym(tx, w, p[i], false);
        ok = ok && put_sym(tx, w, (uint8_t)crc, false);
        ok = ok && put_sym(tx, w, (uint8_t)(crc >> 8), false);
    }
    unsigned total = alink_frame_symbols(tx->cls, tx->rate, f);
    for (unsigned i = used; i < total; i++) ok = ok && put_sym(tx, w, ALINK_BYTE_IDLE, false);
    tx->frame = f + 1u;
    // A RATE switches the layout here, at the end of the frame before its
    // target, so tx->rate and tx->channels describe the next frame before the
    // caller fills its samples (the receiver switches at the same point).
    if (tx->apply_op == ALINK_OP_RATE && tx->frame == tx->apply_frame) {
        tx->rate = tx->apply_arg;
        if (tx->apply_value) tx->channels = (uint8_t)tx->apply_value;
    }
    return ok;
}
