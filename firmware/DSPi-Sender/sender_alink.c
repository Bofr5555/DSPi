/*
 * sender_alink.c - DSPi alink sender (WP LK4): the portable encoder core
 *
 * See sender_alink.h. The encoder below is alink_tx_frame() (alink_codec.c)
 * with the per-bit packing and the per-symbol 8b/10b and CRC arithmetic
 * replaced by tables; the host tests compare the two bit for bit.
 */

#include "sender_alink.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Tables
// ---------------------------------------------------------------------------

void snd_tables_init(snd_tables_t *t) {
    for (unsigned r = 0; r < 2; r++) {
        for (unsigned b = 0; b < 256; b++) {
            unsigned rd = r;
            uint16_t sym = alink_8b10b_encode((uint8_t)b, false, &rd);
            t->enc[r][b] = (uint16_t)(sym | (rd << 15));
        }
        unsigned rd = r;
        uint16_t sym = alink_8b10b_encode(ALINK_BYTE_K28_5, true, &rd);
        t->comma[r] = (uint16_t)(sym | (rd << 15));
    }
    for (unsigned i = 0; i < 256; i++) {
        uint8_t b = (uint8_t)i;
        t->crc8[i] = alink_crc8(&b, 1);
        uint16_t crc = (uint16_t)(i << 8);
        for (int k = 0; k < 8; k++)
            crc = (uint16_t)((crc & 0x8000u) ? ((unsigned)crc << 1) ^ 0x1021u : (unsigned)crc << 1);
        t->crc16[i] = crc;
    }
    alink_prbs7_fill(t->prbs, sizeof(t->prbs));
}

// ---------------------------------------------------------------------------
// Sink and log
// ---------------------------------------------------------------------------

void snd_sink_init(snd_sink_t *s, uint32_t *ring, uint32_t ring_words, uint32_t *sync) {
    s->ring = ring;
    s->mask = ring_words - 1u;
    s->sync = sync;
    s->words = 0;
    s->cur = 0;
    s->n = 0;
    s->pulse_from = s->pulse_to = 0;
}

#define IDLE30 0x2AAAAAAAu      // 30 bits of D21.5: 1010...10

void snd_put_idles(snd_sink_t *s, unsigned nbits) {
    SND_W_BEGIN(s);
    while (nbits >= 30u) {
        SND_W_PUT(IDLE30, 30);
        nbits -= 30u;
    }
    if (nbits) SND_W_PUT(IDLE30 >> (30u - nbits), nbits);
    SND_W_END(s);
}

void snd_sync_fill(snd_sink_t *s, uint64_t from, uint64_t to) {
    if (!s->sync) return;
    uint32_t *sy = s->sync, m = s->mask;
    if (s->pulse_to <= from * 32u) {
        // No pulse in reach (nearly always): 32-bit indices suffice.
        for (uint32_t i = (uint32_t)from, e = (uint32_t)to; i != e; i++) sy[i & m] = 0;
        return;
    }
    // Words before the pulse, the pulse's own, then words after it.
    uint64_t p0 = s->pulse_from / 32u, p1 = (s->pulse_to + 31u) / 32u;
    uint64_t w = from;
    for (; w < to && w < p0; w++) sy[(uint32_t)w & m] = 0;
    for (; w < to && w < p1; w++) sy[(uint32_t)w & m] = snd_pulse_mask(s, w * 32u);
    for (; w < to; w++) sy[(uint32_t)w & m] = 0;
}

void snd_log_init(snd_log_t *l, uint64_t first_frame) {
    l->head = l->tail = 0;
    l->wire_frame = first_frame;
}

uint64_t snd_log_wire(snd_log_t *l, uint64_t wire_bit) {
    if (l->head - l->tail > SND_LOG_LEN) l->tail = l->head - SND_LOG_LEN;
    while (l->head - l->tail >= 2u &&
           l->bit[(l->tail + 1u) & (SND_LOG_LEN - 1u)] <= wire_bit)
        l->tail++;
    if (l->head != l->tail) {
        uint32_t i = l->tail & (SND_LOG_LEN - 1u);
        if (l->bit[i] <= wire_bit) l->wire_frame = l->frame[i];
    }
    return l->wire_frame;
}

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

uint8_t snd_sf_flags(const alink_tx_t *tx) {
    uint8_t f = (uint8_t)(tx->flags & (ALINK_SF_FLAGS_DEFINED &
                                       ~(ALINK_SF_FLAG_RATE_PEND | ALINK_SF_FLAG_APPLY_PEND)));
    if (tx->apply_op != ALINK_OP_NONE) f |= ALINK_SF_FLAG_APPLY_PEND;
    if (tx->apply_op == ALINK_OP_RATE) f |= ALINK_SF_FLAG_RATE_PEND;
    return f;
}

// The superframe bookkeeping at the start of frame tx->frame, exactly as
// alink_tx_frame() does it: pack at every boundary and at the first frame,
// and drop an apply no complete superframe can carry any more.
static void frame_begin(alink_tx_t *tx) {
    uint64_t f = tx->frame;
    if (f % ALINK_SF_LEN == 0 || !tx->started) {
        uint64_t first = f & ~(uint64_t)(ALINK_SF_LEN - 1u);
        if (tx->apply_op != ALINK_OP_NONE && first + ALINK_SF_LEN > tx->apply_frame) {
            tx->apply_frame = 0;
            tx->apply_op = ALINK_OP_NONE;
            tx->apply_arg = 0;
            tx->apply_value = 0;
        }
        alink_sf_t sf = {
            .flags = snd_sf_flags(tx), .rate = tx->rate, .channels = tx->channels,
            .cls = tx->cls, .hops = tx->hops, .sender = tx->sender, .frame = first,
            .apply_frame = tx->apply_frame, .apply_op = tx->apply_op,
            .apply_arg = tx->apply_arg, .apply_value = tx->apply_value,
        };
        alink_sf_pack(&sf, tx->sf);
        tx->started = true;
    }
}

// A RATE switches the layout at the end of the frame before its target.
static void frame_end(alink_tx_t *tx) {
    tx->frame++;
    if (tx->apply_op == ALINK_OP_RATE && tx->frame == tx->apply_frame) {
        tx->rate = tx->apply_arg;
        if (tx->apply_value) tx->channels = (uint8_t)tx->apply_value;
    }
}

static unsigned frame_symbols(const alink_tx_t *tx) {
    // Constant at 48 and 96 kHz; the 64-bit modulo only at 44.1 kHz.
    if (tx->rate != ALINK_RATE_44K1) return (unsigned)(alink_class_baud(tx->cls) / (10u * alink_rate_hz(tx->rate)));
    return alink_frame_symbols(tx->cls, tx->rate, tx->frame);
}

#define PUT(v, nb) SND_W_PUT(v, nb)

// One byte through 8b/10b: e = table entry, rd updated.
#define ENC(b) (e = t->enc[rd][(b)], rd = e >> 15, (uint32_t)(e & 0x3FFu))

void snd_frame(alink_tx_t *tx, const snd_tables_t *t, const int32_t *samples, snd_sink_t *s,
               snd_log_t *log) {
    frame_begin(tx);
    uint64_t f = tx->frame;
    if (log) snd_log_push(log, f, snd_sink_bits(s));

    SND_W_BEGIN(s);
    unsigned rd = tx->rd, e;

    // Comma and head (HDR, CTL, CRC8): 40 bits.
    uint8_t hdr = (uint8_t)f, ctl = tx->sf[f % ALINK_SF_LEN];
    uint8_t crc8 = t->crc8[t->crc8[hdr] ^ ctl];
    e = t->comma[rd];
    rd = e >> 15;
    PUT(e & 0x3FFu, 10);
    uint32_t h0 = ENC(hdr), h1 = ENC(ctl), h2 = ENC(crc8);
    PUT((h0 << 20) | (h1 << 10) | h2, 30);

    // Banks: 3 bytes per channel, MSB first, then the bank's CRC-16 low byte
    // first.
    unsigned c = tx->channels;
    const uint8_t *prbs = (tx->sf[2] & ALINK_SF_FLAG_PRBS) ? t->prbs : NULL;
    for (unsigned ch = 0; ch < c;) {
        unsigned end = ch + ALINK_BANK_CHANNELS < c ? ch + ALINK_BANK_CHANNELS : c;
        uint16_t crc = 0xFFFFu;
        for (; ch < end; ch++) {
            uint8_t b0, b1, b2;
            if (prbs) {
                b0 = prbs[3u * ch]; b1 = prbs[3u * ch + 1u]; b2 = prbs[3u * ch + 2u];
            } else {
                uint32_t u = (uint32_t)samples[ch];
                b0 = (uint8_t)(u >> 16); b1 = (uint8_t)(u >> 8); b2 = (uint8_t)u;
            }
            crc = (uint16_t)((crc << 8) ^ t->crc16[(crc >> 8) ^ b0]);
            crc = (uint16_t)((crc << 8) ^ t->crc16[(crc >> 8) ^ b1]);
            crc = (uint16_t)((crc << 8) ^ t->crc16[(crc >> 8) ^ b2]);
            uint32_t s0 = ENC(b0), s1 = ENC(b1), s2 = ENC(b2);
            PUT((s0 << 20) | (s1 << 10) | s2, 30);
        }
        uint32_t c0 = ENC((uint8_t)crc), c1 = ENC((uint8_t)(crc >> 8));
        PUT((c0 << 10) | c1, 20);
    }

    // Idles to the frame length.  D21.5 is neutral, rd is unchanged.
    unsigned idle = 10u * (frame_symbols(tx) - alink_layout_symbols(c));
    while (idle >= 30u) { PUT(IDLE30, 30); idle -= 30u; }
    if (idle) PUT(IDLE30 >> (30u - idle), idle);

    SND_W_END(s);
    tx->rd = rd;
    frame_end(tx);
}

void snd_idle_frame(alink_tx_t *tx, snd_sink_t *s, snd_log_t *log) {
    frame_begin(tx);
    if (log) snd_log_push(log, tx->frame, snd_sink_bits(s));
    snd_put_idles(s, 10u * frame_symbols(tx));
    frame_end(tx);
}

// ---------------------------------------------------------------------------
// Scheduling
// ---------------------------------------------------------------------------

bool snd_schedule_layout(alink_tx_t *tx, unsigned rate, unsigned channels, uint64_t *at) {
    uint64_t next_sf = (tx->frame + ALINK_SF_LEN - 1u) & ~(uint64_t)(ALINK_SF_LEN - 1u);
    uint64_t n = next_sf + SND_RATE_LEAD;
    int16_t value = channels != tx->channels ? (int16_t)channels : 0;
    if (rate > 0xFFu || !alink_tx_schedule(tx, n, ALINK_OP_RATE, (uint8_t)rate, value)) return false;
    *at = n;
    return true;
}

bool snd_op_args_ok(uint8_t op, uint8_t arg, int16_t value) {
    switch (op) {
    case ALINK_OP_MUTE:
    case ALINK_OP_UNMUTE:
    case ALINK_OP_SYNC_PULSE:
        return arg == 0 && value == 0;
    case ALINK_OP_PRESET_LOAD:
        return arg < SND_OP_PRESET_SLOTS && value == 0;
    case ALINK_OP_MASTER_VOLUME:
        return arg == 0 && value >= -12800 && value <= 0;
    case ALINK_OP_OUT_DELAY:
        return arg < SND_OP_OUTPUTS && value >= 0 && value <= SND_OP_DELAY_MAX;
    case ALINK_OP_CLICK:
        return arg < SND_OP_OUTPUTS && value == 0;
    default:
        return false;
    }
}

// ---------------------------------------------------------------------------
// EXT_ALINK_TX
// ---------------------------------------------------------------------------

static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

void snd_txcfg_default(snd_txcfg_t *c) {
    memset(c, 0, sizeof(*c));
    c->alink_enable = 0;
    c->cls = ALINK_CLASS_L1;
    c->channels = 8;
    c->rate = ALINK_RATE_48K;
    c->test_pattern = SND_TP_AUDIO;
    c->slot_map_id = 0;
    c->sync_pin = 0xFF;
    c->txdis_mask = 0x00FF;
    c->flags = 0;
}

int snd_txcfg_parse(const uint8_t p[SND_TX_LEN], unsigned max_alinks, snd_txcfg_t *c) {
    snd_txcfg_t v;
    v.alink_enable = get_u16(p);
    v.cls = p[2];
    v.channels = p[3];
    v.rate = p[4];
    v.test_pattern = p[5];
    v.slot_map_id = p[6];
    v.sync_pin = p[7];
    v.txdis_mask = get_u16(p + 8);
    v.flags = p[13];
    uint16_t over = (uint16_t)(0xFFFFu << max_alinks);
    if (v.alink_enable & over) return 1;
    if (v.cls < ALINK_CLASS_L1 || v.cls > ALINK_CLASS_L3) return 2;
    if (v.rate > ALINK_RATE_96K) return 3;
    if (v.channels < 1 || v.channels > alink_max_channels(v.cls, v.rate)) return 4;
    if (v.test_pattern > SND_TP_TONE) return 5;
    if (v.slot_map_id) return 6;
    if (!snd_pin_ok(v.sync_pin)) return 7;
    if (v.txdis_mask & over) return 8;
    if (v.flags & ~1u) return 9;
    if (p[14] || p[15]) return 10;
    *c = v;
    return 0;
}

void snd_txcfg_pack(const snd_txcfg_t *c, uint16_t los_mask, uint8_t usb_alt, uint8_t out[SND_TX_LEN]) {
    memset(out, 0, SND_TX_LEN);
    put_u16(out, c->alink_enable);
    out[2] = c->cls;
    out[3] = c->channels;
    out[4] = c->rate;
    out[5] = c->test_pattern;
    out[6] = c->slot_map_id;
    out[7] = c->sync_pin;
    put_u16(out + 8, c->txdis_mask);
    put_u16(out + 10, los_mask);
    out[12] = usb_alt;
    out[13] = c->flags;
}

// ---------------------------------------------------------------------------
// EXT_ALINK_CTRL_MSG
// ---------------------------------------------------------------------------

int snd_ctrl_parse(const uint8_t p[SND_CTRL_OUT_LEN], snd_ctrl_t *c) {
    snd_ctrl_t v;
    v.frame = 0;
    for (int i = 7; i >= 0; i--) v.frame = (v.frame << 8) | p[i];
    v.op = p[8];
    v.arg = p[9];
    v.value = (int16_t)get_u16(p + 10);
    v.system_flags = p[12];
    if (v.op > ALINK_OP_MAX) return 1;
    if (v.system_flags & ~1u) return 2;
    if (p[13] || p[14] || p[15]) return 3;
    if (v.op == ALINK_OP_RATE) {
        if (v.arg > ALINK_RATE_96K || v.value != 0 || v.frame % ALINK_SF_LEN) return 4;
    } else if (v.op != ALINK_OP_NONE && !snd_op_args_ok(v.op, v.arg, v.value)) {
        return 5;
    }
    *c = v;
    return 0;
}

void snd_ctrl_pack_in(const alink_tx_t *tx, uint64_t sf_frame, uint8_t out[SND_CTRL_IN_LEN]) {
    memset(out, 0, SND_CTRL_IN_LEN);
    // The apply has ended once F reaches its frame (the encoder drops it at
    // the next superframe start).
    alink_tx_t v = *tx;
    if (v.apply_op != ALINK_OP_NONE && v.frame >= v.apply_frame) {
        v.apply_op = ALINK_OP_NONE;
        v.apply_frame = 0;
        v.apply_arg = 0;
        v.apply_value = 0;
    }
    tx = &v;
    out[0] = snd_sf_flags(tx);
    out[1] = tx->rate;
    out[2] = tx->channels;
    out[3] = tx->cls;
    out[4] = tx->hops;
    out[5] = tx->sender;
    for (int i = 0; i < 8; i++) out[8 + i] = (uint8_t)(tx->apply_frame >> (8 * i));
    out[16] = tx->apply_op;
    out[17] = tx->apply_arg;
    put_u16(out + 18, (uint16_t)tx->apply_value);
    uint32_t lo = (uint32_t)(sf_frame & ~(uint64_t)(ALINK_SF_LEN - 1u));
    for (int i = 0; i < 4; i++) out[20 + i] = (uint8_t)(lo >> (8 * i));
}

// ---------------------------------------------------------------------------
// Test tone
// ---------------------------------------------------------------------------

void snd_tone_set_rate(snd_tone_t *t, uint32_t fs) {
    t->step = fs ? (uint32_t)((1000ull << 32) / fs) : 0;
}

int32_t snd_tone_next(snd_tone_t *t) {
    // sin over a quarter-wave folded phase, 5th-order Taylor (error < 0.5 %).
    uint32_t ph = t->phase;
    t->phase += t->step;
    // Quadrants 1 and 3 run backwards; 2 and 3 are negative.
    uint32_t q = ph >> 30;
    uint32_t r = ph & 0x3FFFFFFFu;
    if (q & 1u) r = 0x40000000u - r;
    float x = (float)r * (1.5707963f / 1073741824.0f);
    float x2 = x * x;
    float y = x * (1.0f - x2 * (1.0f / 6.0f - x2 * (1.0f / 120.0f)));
    if (q & 2u) y = -y;
    return (int32_t)(y * 838860.7f);    // 0.1 x 2^23: -20 dBFS
}
