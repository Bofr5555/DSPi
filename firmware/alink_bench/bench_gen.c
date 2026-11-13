/*
 * bench_gen.c - alink bench: real-time frame generator and bit sink
 *
 * See bench_gen.h.  Every symbol comes from LK1's alink_8b10b_encode(), every
 * CRC from alink_crc8()/alink_crc16(), the superframe from alink_sf_pack() and
 * the PRBS from alink_prbs7_fill(), so the only thing this file adds is the
 * order of the work, which the host tests check against alink_tx_frame().
 */

#include "bench_gen.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Sink
// ---------------------------------------------------------------------------

void bench_sink_init(bench_sink_t *s, uint32_t *ring, uint32_t ring_words) {
    s->ring = ring;
    s->mask = ring_words - 1u;
    s->words = 0;
    s->cur = 0;
    s->cur_n = 0;
}

static inline void sink_emit(bench_sink_t *s, uint32_t w) {
    s->ring[(uint32_t)s->words & s->mask] = w;
    s->words++;
}

void bench_sink_put(bench_sink_t *s, uint32_t v, unsigned n) {
    if (n == 0) return;
    if (n < 32) v &= (1u << n) - 1u;
    unsigned room = 32u - s->cur_n;
    if (n < room) {
        s->cur |= v << (room - n);
        s->cur_n += n;
        return;
    }
    unsigned rem = n - room;
    sink_emit(s, s->cur | (v >> rem));
    s->cur = rem ? v << (32u - rem) : 0;
    s->cur_n = rem;
}

void bench_sink_put_bits(bench_sink_t *s, const uint32_t *src, unsigned nbits) {
    // Whole words with the partial word's offset held fixed: one store per
    // source word, the hot path of the transmitter.
    unsigned n = nbits / 32u, o = s->cur_n;
    if (o == 0) {
        for (unsigned i = 0; i < n; i++) sink_emit(s, src[i]);
    } else {
        uint32_t cur = s->cur;
        for (unsigned i = 0; i < n; i++) {
            sink_emit(s, cur | (src[i] >> o));
            cur = src[i] << (32u - o);
        }
        s->cur = cur;
    }
    if (nbits & 31u) bench_sink_put(s, src[n] >> (32u - (nbits & 31u)), nbits & 31u);
}

// ---------------------------------------------------------------------------
// Generator
// ---------------------------------------------------------------------------

int32_t bench_vector_sample(unsigned ch) {
    // 0x8n_sk_kk-style values: the top byte alternates sign, the low bytes
    // carry the channel so a swapped or shifted channel is visible.
    uint32_t c = ch & 0x3Fu;
    uint32_t u = ((c & 1u) ? 0x800000u : 0x000000u) | ((c + 1u) << 16) | (0x5Au << 8) | (0xA0u + c);
    return (int32_t)(u ^ 0x800000u) - 0x800000;
}

// Encode the bench payload (banks and CRC-16s, no head, no idles) once per
// starting disparity.
static void cache_payload(bench_gen_t *g) {
    unsigned c = g->channels;
    uint8_t payload[3u * ALINK_MAX_CHANNELS];
    if (g->pay == BENCH_PAY_PRBS) {
        alink_prbs7_fill(payload, 3u * c);
    } else {
        for (unsigned ch = 0; ch < c; ch++) alink_put_s24(payload + 3u * ch, bench_vector_sample(ch));
    }
    for (unsigned r = 0; r < 2; r++) {
        alink_bitw_t w;
        alink_bitw_init(&w, g->pay_bits[r], BENCH_PAY_WORDS);
        unsigned rd = r;
        for (unsigned b = 0; b < alink_bank_count(c); b++) {
            const uint8_t *p = payload + 3u * ALINK_BANK_CHANNELS * b;
            unsigned n = 3u * alink_bank_channels(c, b);
            uint16_t crc = alink_crc16(p, n);
            for (unsigned i = 0; i < n; i++) alink_bitw_put(&w, alink_8b10b_encode(p[i], false, &rd), 10);
            alink_bitw_put(&w, alink_8b10b_encode((uint8_t)crc, false, &rd), 10);
            alink_bitw_put(&w, alink_8b10b_encode((uint8_t)(crc >> 8), false, &rd), 10);
        }
        g->pay_nbits = (unsigned)w.nbits;
        g->pay_rd_end[r] = (uint8_t)rd;
    }
}

bool bench_gen_init(bench_gen_t *g, unsigned cls, unsigned rate, unsigned channels,
                    unsigned pay, uint64_t first, uint8_t sender) {
    memset(g, 0, sizeof(*g));
    if (!alink_class_baud(cls) || !alink_rate_hz(rate) || channels < 1 ||
        channels > alink_max_channels(cls, rate) || pay > BENCH_PAY_VECTOR)
        return false;
    g->cls = (uint8_t)cls;
    g->rate = (uint8_t)rate;
    g->channels = (uint8_t)channels;
    g->pay = (uint8_t)pay;
    g->sender = sender;
    g->flags = pay == BENCH_PAY_PRBS ? ALINK_SF_FLAG_PRBS : 0;
    g->layout = alink_layout_symbols(channels);
    unsigned lo = alink_frame_symbols_min(cls, rate), hi = alink_frame_symbols_max(cls, rate);
    g->sym_const = lo == hi ? lo : 0;
    cache_payload(g);
    bench_gen_seek(g, first, 0);
    return true;
}

void bench_gen_seek(bench_gen_t *g, uint64_t first, unsigned rd) {
    g->frame = first;
    g->rd = rd & 1u;
    g->started = false;
}

static unsigned frame_symbols(const bench_gen_t *g, uint64_t f) {
    return g->sym_const ? g->sym_const : alink_frame_symbols(g->cls, g->rate, f);
}

unsigned bench_gen_frame_bits(const bench_gen_t *g) {
    return 10u * frame_symbols(g, g->frame);
}

// The superframe as alink_tx_frame() packs it: no apply, so no pending flags.
static void pack_sf(bench_gen_t *g, uint64_t first) {
    alink_sf_t sf = {
        .flags = (uint8_t)(g->flags & (ALINK_SF_FLAGS_DEFINED & ~(ALINK_SF_FLAG_RATE_PEND |
                                                                  ALINK_SF_FLAG_APPLY_PEND))),
        .rate = g->rate, .channels = g->channels, .cls = g->cls,
        .hops = g->hops, .sender = g->sender, .frame = first,
    };
    alink_sf_pack(&sf, g->sf);
}

// D21.5 is 1010101010 in both columns: a run of idles is alternating bits
// starting with 1 and leaves the disparity alone.
static void put_alternating(bench_sink_t *s, unsigned nbits) {
    while (nbits >= 32u) { bench_sink_put(s, 0xAAAAAAAAu, 32); nbits -= 32u; }
    if (nbits) bench_sink_put(s, 0xAAAAAAAAu >> (32u - nbits), nbits);
}

unsigned bench_gen_parts(bench_gen_t *g, bench_parts_t *p) {
    uint64_t f = g->frame;
    if (f % ALINK_SF_LEN == 0 || !g->started) {
        pack_sf(g, f & ~(uint64_t)(ALINK_SF_LEN - 1u));
        g->started = true;
    }
    unsigned total_bits = 10u * frame_symbols(g, f);
    uint8_t head[3];
    head[0] = (uint8_t)f;
    head[1] = g->sf[f % ALINK_SF_LEN];
    head[2] = alink_crc8(head, 2);
    unsigned rd = g->rd;
    uint32_t c = alink_8b10b_encode(ALINK_BYTE_K28_5, true, &rd);
    uint32_t h1 = alink_8b10b_encode(head[0], false, &rd);
    uint32_t h2 = alink_8b10b_encode(head[1], false, &rd);
    uint32_t h3 = alink_8b10b_encode(head[2], false, &rd);
    p->head[0] = (c << 22) | (h1 << 12) | (h2 << 2) | (h3 >> 8);
    p->head[1] = h3 << 24;
    p->comma_rd = g->rd;
    p->pay = g->pay_bits[rd];
    p->pay_nbits = g->pay_nbits;
    p->idle_nbits = total_bits - 40u - g->pay_nbits;
    g->rd = g->pay_rd_end[rd];
    g->frame = f + 1u;
    return total_bits;
}

unsigned bench_gen_frame(bench_gen_t *g, bench_sink_t *s) {
    uint64_t f = g->frame;
    if (f % ALINK_SF_LEN == 0 || !g->started) {
        pack_sf(g, f & ~(uint64_t)(ALINK_SF_LEN - 1u));
        g->started = true;
    }
    unsigned total_bits = 10u * frame_symbols(g, f);

    uint8_t head[3];
    head[0] = (uint8_t)f;
    head[1] = g->sf[f % ALINK_SF_LEN];
    head[2] = alink_crc8(head, 2);
    unsigned rd = g->rd;
    uint16_t sym[4];
    sym[0] = alink_8b10b_encode(ALINK_BYTE_K28_5, true, &rd);
    for (unsigned i = 0; i < 3; i++) sym[i + 1] = alink_8b10b_encode(head[i], false, &rd);
    unsigned pay_rd = rd;
    rd = g->pay_rd_end[pay_rd];

    bool idle = false, comma = false, flip = false;
    int slip = 0;
    if (g->imp_idle) { g->imp_idle--; g->imp.idle++; idle = true; }
    else {
        if (g->imp_comma) { g->imp_comma--; g->imp.comma++; comma = true; }
        if (g->imp_flip) { g->imp_flip--; g->imp.flip++; flip = true; }
    }
    if (g->imp_slip) {
        slip = g->imp_slip;
        g->imp_slip = 0;
        g->imp.slip++;
        g->imp.slip_bits += slip;
    }

    // The encoder state advances exactly as for an unimpaired frame, so the
    // stream after the impairment is the one the receiver expects.
    g->rd = rd;
    g->frame = f + 1u;
    unsigned emitted = (unsigned)((int)total_bits + slip);
    if (!s) return emitted;

    if (idle) {
        put_alternating(s, total_bits);
    } else if (!comma && !flip && slip >= 0) {
        bench_sink_put(s, sym[0], 10);
        bench_sink_put(s, ((uint32_t)sym[1] << 20) | ((uint32_t)sym[2] << 10) | sym[3], 30);
        bench_sink_put_bits(s, g->pay_bits[pay_rd], g->pay_nbits);
        put_alternating(s, total_bits - 40u - g->pay_nbits);
    } else {
        // Impaired frame: build it in a scratch buffer, damage it, copy it
        // out (minus the dropped tail of a negative slip).
        uint32_t tmp[64];
        bench_sink_t t;
        bench_sink_init(&t, tmp, 64u);  // never wraps: a frame is at most 44 words
        bench_sink_put(&t, sym[0], 10);
        bench_sink_put(&t, ((uint32_t)sym[1] << 20) | ((uint32_t)sym[2] << 10) | sym[3], 30);
        bench_sink_put_bits(&t, g->pay_bits[pay_rd], g->pay_nbits);
        put_alternating(&t, total_bits - 40u - g->pay_nbits);
        if (t.cur_n) tmp[t.words] = t.cur;
        if (comma) tmp[0] ^= 0x80000000u >> 5;          // comma bit 'f': no longer a comma
        if (flip) tmp[1] ^= 0x80000000u >> (40u - 32u); // bank 0 symbol 0, bit 'a'
        bench_sink_put_bits(s, tmp, slip < 0 ? total_bits - (unsigned)(-slip) : total_bits);
    }
    if (slip > 0) put_alternating(s, (unsigned)slip);
    return emitted;
}

bool bench_gen_inject(bench_gen_t *g, unsigned kind, int n) {
    switch (kind) {
    case BENCH_IMP_COMMA: if (n < 1 || n > 1000) return false; g->imp_comma = (uint16_t)n; return true;
    case BENCH_IMP_FLIP:  if (n < 1 || n > 1000) return false; g->imp_flip = (uint16_t)n; return true;
    case BENCH_IMP_IDLE:  if (n < 1 || n > 1000) return false; g->imp_idle = (uint16_t)n; return true;
    case BENCH_IMP_SLIP:  if (n < -3 || n > 3 || n == 0) return false; g->imp_slip = (int8_t)n; return true;
    default: return false;
    }
}
