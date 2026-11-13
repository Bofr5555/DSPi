/*
 * link_fast.c - DSPi link protocol v1: production receiver (WP LK5)
 *
 * See link_fast.h.  The state machine is link_decoder.c's, step for step
 * (spec section 8); only the way bits are read and symbols decoded differs.
 * Every step runs only once every bit it reads is below the writer's
 * position, so the output does not depend on how often link_fast_run() is
 * called, exactly as the reference does not depend on chunking.
 *
 * Positions are absolute bit numbers modulo 2^32 and are compared through
 * signed differences; the ring holds a power-of-two number of words, so
 * word (pos / 32) & ring_mask is consistent across the 2^32 wrap.
 */

#include "link_fast.h"

#include <stdatomic.h>
#include <stddef.h>
#include <string.h>

#define BAD     0x80000000u      // the bit of LINK_FAST_DEC_BAD tested

// Cold paths (search, alignment, loss, error counting) stay out of the hot
// functions and out of RAM.
#define COLD    __attribute__((noinline))

// On the RP2350 build (LINK_FAST_M33) the steady state at 48/96 kHz with one
// full bank runs in assembly; the C clean-frame loops (multi-bank, partial
// bank, 44.1 kHz) and the partial-bank decoder then stay in flash, which keeps
// the RAM-resident set near 5 KB.  Elsewhere they are the fast path.
#if LINK_FAST_M33
#define CPATH   COLD
#else
#define CPATH   LINK_FAST_HOT __attribute__((noinline))
#endif

// x <= avail: every bit below x has been written.
static inline bool have(const link_fast_t *f, uint32_t x) {
    return (int32_t)(f->avail - x) >= 0;
}

// The ring without the volatile qualifier: link_fast_run() reads only words
// the writer has finished, after a compiler fence.
static inline const uint32_t *ring_words(const link_fast_t *f) {
    return (const uint32_t *)(uintptr_t)f->ring;
}

// The 32 bits starting at absolute bit pos.
static inline uint32_t bits32(const link_fast_t *f, uint32_t pos) {
    const uint32_t *r = ring_words(f);
    uint32_t wi = pos >> 5, o = pos & 31u;
    uint32_t w0 = r[wi & f->ring_mask];
    if (!o) return w0;
    return (w0 << o) | (r[(wi + 1u) & f->ring_mask] >> (32u - o));
}

// The symbol occupying bits [end - 10, end).
static inline uint32_t sym_end(const link_fast_t *f, uint32_t end) {
    return bits32(f, end - 10u) >> 22;
}

static inline bool is_comma(uint32_t sym) {
    return sym == LINK_SYM_K28_5_RDN || sym == LINK_SYM_K28_5_RDP;
}

static LINK_FAST_HOT void lf_emit(link_fast_t *f, link_rx_event_t *ev) {
    if (f->cb) f->cb(f->ctx, ev);
}

static COLD void lf_emit_simple(link_fast_t *f, link_rx_event_type_t type) {
    link_rx_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = type;
    ev.frame = f->frame;
    lf_emit(f, &ev);
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

// F mod d for d < 2^16 with 32-bit divides only (no 64-bit division call).
static inline uint32_t mod_u64(uint64_t v, uint32_t d) {
    uint32_t hi = (uint32_t)(v >> 32), lo = (uint32_t)v;
    uint32_t r32 = (0xFFFFFFFFu % d + 1u) % d;  // 2^32 mod d
    return ((hi % d) * r32 + lo % d) % d;
}

// Cache everything that depends on (rate, channels) and F mod den, and zero
// the samples of channels the new layout does not have (the reference
// zeroes them on every frame).
static COLD void set_layout(link_fast_t *f) {
    f->num = f->ratio[f->cls][f->rate][0];
    f->den = f->ratio[f->cls][f->rate][1];
    f->fmod = (uint16_t)(f->den == 1u ? 0u : mod_u64(f->frame, f->den));
    unsigned c = f->channels, nb = (c + LINK_BANK_CHANNELS - 1u) / LINK_BANK_CHANNELS;
    f->layout = (uint8_t)(LINK_HEAD_SYMBOLS + 3u * c + 2u * nb);
    f->nbanks = (uint8_t)nb;
    f->decode_banks = (uint8_t)(f->bank_mask & ((1u << nb) - 1u));
    for (unsigned ch = c; ch < LINK_MAX_CHANNELS; ch++) f->samples[ch] = 0;
}

// Symbols in the current frame (link_frame_symbols() from F mod den).
static inline unsigned frame_syms(const link_fast_t *f) {
    if (f->den == 1u) return f->num;
    uint32_t m = f->fmod, n = f->num, d = f->den;
    return ((m + 1u) * n) / d - (m * n) / d;
}

// ---------------------------------------------------------------------------
// Head, superframe
// ---------------------------------------------------------------------------

// HDR, CTL and CRC8 after the comma ending at p (bits p .. p + 29).
static inline bool lf_head_at(link_fast_t *f, uint32_t p, uint8_t *hdr, uint8_t *ctl, bool count) {
    uint32_t x = bits32(f, p);
    uint32_t e0 = f->dec[x >> 22], e1 = f->dec[(x >> 12) & 0x3FFu], e2 = f->dec[(x >> 2) & 0x3FFu];
    if ((e0 | e1 | e2) & BAD) {
        if (count) f->stats.code_errors += (e0 >> 31) + (e1 >> 31) + (e2 >> 31);
        return false;
    }
    if (f->crc8[f->crc8[e0 & 0xFFu] ^ (e1 & 0xFFu)] != (e2 & 0xFFu)) return false;
    *hdr = (uint8_t)e0;
    *ctl = (uint8_t)e1;
    return true;
}

static inline uint16_t crc16_tab(const link_fast_t *f, const uint8_t *p, unsigned n) {
    uint32_t crc = 0xFFFFu;
    for (unsigned i = 0; i < n; i++) crc = (crc << 8) ^ f->crc16[((crc >> 8) ^ p[i]) & 0xFFu];
    return (uint16_t)crc;
}

static inline uint64_t lf_get_u64le(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | p[i];
    return v;
}

// link_sf_unpack() with the table CRC and the cached capacities, then the
// class check: the same checks in the same order.
static LINK_FAST_HOT int lf_sf_check(link_fast_t *f, link_sf_t *sf) {
    const uint8_t *in = f->sf_bytes;
    if (crc16_tab(f, in, 30) != (uint16_t)(in[30] | (in[31] << 8))) return LINK_SF_ERR_CRC;
    if (in[0] != LINK_SF_MAGIC) return LINK_SF_ERR_MAGIC;
    if (in[1] != LINK_PROTOCOL_VERSION) return LINK_SF_ERR_VERSION;
    sf->flags = in[2];
    sf->rate = in[3];
    sf->channels = in[4];
    sf->cls = in[5];
    sf->hops = in[6];
    sf->sender = in[7];
    sf->frame = lf_get_u64le(in + 8);
    sf->apply_frame = lf_get_u64le(in + 16);
    sf->apply_op = in[24];
    sf->apply_arg = in[25];
    sf->apply_value = (int16_t)(uint16_t)(in[26] | (in[27] << 8));
    if (sf->frame % LINK_SF_LEN) return LINK_SF_ERR_FRAME;
    if (sf->cls < 1u || sf->cls > 3u || sf->rate > LINK_RATE_96K || sf->channels < 1u ||
        sf->channels > f->cap[sf->cls][sf->rate])
        return LINK_SF_ERR_LAYOUT;
    if (sf->apply_op > LINK_OP_MAX || (sf->apply_op == LINK_OP_NONE) != (sf->apply_frame == 0))
        return LINK_SF_ERR_APPLY;
    if (sf->cls != f->cls) return LINK_RX_SF_CLASS;
    return LINK_SF_OK;
}

static LINK_FAST_HOT void lf_sf_collect(link_fast_t *f, unsigned idx, uint8_t ctl, uint8_t hdr) {
    bool cont = f->prev_hdr >= 0 && hdr == (uint8_t)(f->prev_hdr + 1);
    if (idx == 0) {
        f->sf_bytes[0] = ctl;
        f->sf_count = 1;
    } else if (cont && f->sf_count == idx) {
        f->sf_bytes[idx] = ctl;
        f->sf_count++;
    } else {
        f->sf_count = 0;
    }
    f->prev_hdr = hdr;
}

static COLD void lf_sf_error(link_fast_t *f, int err) {
    link_rx_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = LINK_RX_EV_SF_ERROR;
    ev.frame = f->frame;
    ev.sf_error = err;
    f->stats.superframe_errors++;
    lf_emit(f, &ev);
}

static LINK_FAST_HOT void lf_adopt_sf(link_fast_t *f, const link_sf_t *sf) {
    f->sf = *sf;
    f->apply_pending = sf->apply_op == LINK_OP_RATE && sf->apply_frame > f->frame;
    if (sf->rate != f->rate || sf->channels != f->channels) {
        f->rate = sf->rate;
        f->channels = sf->channels;
        set_layout(f);
    }
}

// ---------------------------------------------------------------------------
// State changes
// ---------------------------------------------------------------------------

static void lf_clear_frame_state(link_fast_t *f) {
    f->fails = 0;
    f->prev_hdr = -1;
    f->sf_count = 0;
    f->apply_pending = false;
    memset(f->bank_fails, 0, sizeof(f->bank_fails));
    memset(f->samples, 0, sizeof(f->samples));
}

static void lf_enter_search(link_fast_t *f, uint32_t q) {
    f->state = LINK_RX_SEARCH;
    f->q = q;
    lf_clear_frame_state(f);
}

static COLD void lf_loss(link_fast_t *f) {
    if (f->state == LINK_RX_LOCKED) f->stats.loss_count++;
    lf_emit_simple(f, LINK_RX_EV_LOSS);
    lf_enter_search(f, f->p + 1u);
}

// ---------------------------------------------------------------------------
// SEARCH: bit-parallel comma search
// ---------------------------------------------------------------------------

// Bit 31 - j set when a K28.5 (either RD) occupies bits [q + j - 10, q + j).
static inline uint32_t comma_mask(const link_fast_t *f, uint32_t q) {
    uint32_t hi = bits32(f, q - 10u), lo = bits32(f, q + 22u);
#define SH(t) ((hi << (t)) | (lo >> (32 - (t))))
    uint32_t s0 = hi, s1 = SH(1), s2 = SH(2), s3 = SH(3), s4 = SH(4);
    uint32_t s5 = SH(5), s6 = SH(6), s7 = SH(7), s8 = SH(8), s9 = SH(9);
#undef SH
    // K28.5 RD- = 0011111010, RD+ = its complement 1100000101.
    uint32_t ones = s2 & s3 & s4 & s5 & s6 & s8, zeros = s0 | s1 | s7 | s9;
    uint32_t rdn = ones & ~zeros;
    uint32_t ones_p = s0 & s1 & s7 & s9, zeros_p = s2 | s3 | s4 | s5 | s6 | s8;
    uint32_t rdp = ones_p & ~zeros_p;
    return rdn | rdp;
}

static COLD bool lf_step_search(link_fast_t *f) {
    if (!have(f, f->q + 30u)) return false;
    while (have(f, f->q + 30u)) {
        uint32_t n = f->avail - f->q - 30u + 1u;   // positions with q + 30 <= avail
        if (n > 32u) n = 32u;
        uint32_t m = comma_mask(f, f->q);
        if (n < 32u) m &= ~(0xFFFFFFFFu >> n);
        while (m) {
            uint32_t j = (uint32_t)__builtin_clz(m);
            uint32_t q = f->q + j;
            uint8_t hdr, ctl;
            if (lf_head_at(f, q, &hdr, &ctl, false)) {
                f->state = LINK_RX_ALIGNED;
                f->p = q;
                f->body_done = false;
                f->comma_ok = true;
                f->slip = 0;
                lf_clear_frame_state(f);
                link_rx_event_t ev;
                memset(&ev, 0, sizeof(ev));
                ev.type = LINK_RX_EV_ALIGN;
                ev.bitpos = f->avail64 - (uint32_t)(f->avail - q);
                lf_emit(f, &ev);
                return true;
            }
            m &= ~(0x80000000u >> j);
        }
        f->q += n;
    }
    return true;
}

// ---------------------------------------------------------------------------
// ALIGNED
// ---------------------------------------------------------------------------

static COLD bool lf_body_aligned(link_fast_t *f) {
    if (!have(f, f->p + 30u)) return false;
    uint8_t hdr, ctl;
    if (!lf_head_at(f, f->p, &hdr, &ctl, true)) {
        f->stats.crc8_errors++;
        f->prev_hdr = -1;
        f->sf_count = 0;
        if (++f->fails >= LINK_RX_FAIL_LIMIT) { lf_loss(f); return true; }
        f->body_done = true;
        return true;
    }
    f->fails = 0;
    unsigned idx = hdr & (LINK_SF_LEN - 1u);
    lf_sf_collect(f, idx, ctl, hdr);
    if (idx == LINK_SF_LEN - 1u && f->sf_count == LINK_SF_LEN) {
        link_sf_t sf;
        int err = lf_sf_check(f, &sf);
        if (err == LINK_SF_OK && (uint8_t)(sf.frame + LINK_SF_LEN - 1u) != hdr) err = LINK_RX_SF_COUNTER;
        if (err != LINK_SF_OK) {
            lf_sf_error(f, err);
        } else {
            f->state = LINK_RX_LOCKED;
            f->frame = sf.frame + LINK_SF_LEN - 1u;
            memset(f->bank_fails, 0, sizeof(f->bank_fails));
            memset(f->samples, 0, sizeof(f->samples));
            lf_adopt_sf(f, &sf);
            set_layout(f);
            f->stats.lock_count++;
            link_rx_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = LINK_RX_EV_LOCK;
            ev.frame = f->frame;
            ev.sf = &f->sf;
            lf_emit(f, &ev);
        }
    }
    f->body_done = true;
    return true;
}

static COLD bool lf_next_aligned(link_fast_t *f) {
    unsigned num = f->ratio[f->cls][LINK_RATE_44K1][0], den = f->ratio[f->cls][LINK_RATE_44K1][1];
    unsigned maxgap = (num + den - 1u) / den;   // link_frame_symbols_max(cls, 44.1 kHz)
    for (unsigned i = LINK_HEAD_SYMBOLS; i <= maxgap; i++) {
        uint32_t end = f->p + 10u * i;
        if (!have(f, end)) return false;
        if (is_comma(sym_end(f, end))) {
            f->p = end;
            f->body_done = false;
            f->comma_ok = true;
            f->slip = 0;
            return true;
        }
    }
    lf_loss(f);
    return true;
}

// ---------------------------------------------------------------------------
// LOCKED: banks
// ---------------------------------------------------------------------------

// The 32 line bits starting at bit o (0..31) of r[0], from linear words
// r[0], r[1] (no ring masking: the frame code works on a linear view).
static inline uint32_t funnel(const uint32_t *r, unsigned o) {
    return (r[0] << o) | ((r[1] >> 1) >> (31u - o));
}

// Symbol at bit offset `bit` of realigned words a (constant-folded when bit is).
static inline uint32_t ext(const uint32_t *a, unsigned bit) {
    unsigned w = bit >> 5, o = bit & 31u;
    if (o <= 22u) return (a[w] >> (22u - o)) & 0x3FFu;
    return ((a[w] << (o - 22u)) | (a[w + 1] >> (54u - o))) & 0x3FFu;
}

// Three table entries (MSB first) as one word: the sample in bits 0-23,
// bit 31 set when any of the three is not a data code group.
static inline uint32_t join3(uint32_t e0, uint32_t e1, uint32_t e2) {
    return e2 | (e1 << 8) | (e0 << 16);
}

// The sample of a joined word, sign-extended from bit 23.
static inline int32_t sext24(uint32_t u) {
    return (int32_t)(u << 8) >> 8;
}

#if !LINK_FAST_M33 || !LINK_FAST_M33_SW
// A full bank (8 channels, 26 symbols) starting at bit o of linear words
// r[0..9].  True when every symbol is a data code group and the CRC-16
// matches; the samples go to out[0..7] either way.
static CPATH bool bank8(const link_fast_t *f, const uint32_t *r, unsigned o, int32_t *out) {
    uint32_t a[9];
    for (unsigned i = 0; i < 9u; i++) a[i] = funnel(r + i, o);
    const uint32_t *dec = f->dec;
    const uint16_t *t = f->crc16;
    uint32_t err = 0, crc = 0xFFFFu;
#define CRC(e) (crc = (crc << 8) ^ t[((crc >> 8) ^ (e)) & 0xFFu])
#define CH(i) do { \
        uint32_t e0 = dec[ext(a, 30u * (i))], e1 = dec[ext(a, 30u * (i) + 10u)], e2 = dec[ext(a, 30u * (i) + 20u)]; \
        uint32_t u = join3(e0, e1, e2); \
        err |= u; \
        CRC(e0); CRC(e1); CRC(e2); \
        out[i] = sext24(u); \
    } while (0)
    CH(0); CH(1); CH(2); CH(3); CH(4); CH(5); CH(6); CH(7);
    uint32_t c0 = dec[ext(a, 240u)], c1 = dec[ext(a, 250u)];
#undef CH
#undef CRC
    return !((err | c0 | c1) & BAD) && ((crc ^ (c0 & 0xFFu) ^ ((c1 & 0xFFu) << 8)) & 0xFFFFu) == 0;
}

#endif

#if LINK_FAST_CRC_DMA
// CRC-16 of a bank by the RP2350 DMA sniffer (CRC-16-CCITT, polynomial
// 0x1021, seeded with 0xFFFF): a memory-to-memory transfer of the bank's 24
// payload bytes, packed MSB first into 6 words, on channel f->crc_dma, which
// the platform has configured (32-bit, read increment, unpaced, SNIFF_EN,
// SNIFF_CTRL selecting it).  Raw registers, so this file needs no SDK.
#define LF_DMA_BASE         0x50000000u
#define LF_DMA_CH(n)        ((volatile uint32_t *)(uintptr_t)(LF_DMA_BASE + 0x40u * (unsigned)(n)))
#define LF_DMA_SNIFF_DATA   (*(volatile uint32_t *)(uintptr_t)(LF_DMA_BASE + 0x458u))
#define LF_DMA_BUSY         0x04000000u     // CHx_CTRL BUSY (RP2350)

static inline void crc_dma_start(const link_fast_t *f, const uint32_t *w, unsigned n) {
    atomic_thread_fence(memory_order_seq_cst);  // the words are in memory before the DMA reads them
    LF_DMA_SNIFF_DATA = 0xFFFFu;
    volatile uint32_t *ch = LF_DMA_CH(f->crc_dma);
    ch[0] = (uint32_t)(uintptr_t)w;             // READ_ADDR
    ch[7] = n;                                  // AL1_TRANS_COUNT_TRIG
}

static inline uint32_t crc_dma_result(const link_fast_t *f) {
    volatile uint32_t *ch = LF_DMA_CH(f->crc_dma);
    while (ch[4] & LF_DMA_BUSY) {}              // AL1_CTRL
    return LF_DMA_SNIFF_DATA;
}

#if !LINK_FAST_M33
// bank8() with the CRC in the sniffer: the samples are joined first, packed
// into words for the DMA, and sign-extended while it runs.
static LINK_FAST_HOT __attribute__((noinline)) bool bank8_dma(const link_fast_t *f, const uint32_t *r, unsigned o,
                                                              int32_t *out) {
    uint32_t a[9];
    for (unsigned i = 0; i < 9u; i++) a[i] = funnel(r + i, o);
    const uint32_t *dec = f->dec;
    uint32_t u[8], w[6];
#define CH(i) (u[i] = join3(dec[ext(a, 30u * (i))], dec[ext(a, 30u * (i) + 10u)], dec[ext(a, 30u * (i) + 20u)]))
    CH(0); CH(1); CH(2); CH(3); CH(4); CH(5); CH(6); CH(7);
#undef CH
    for (unsigned g = 0; g < 2u; g++) {         // 4 samples = 12 bytes = 3 words
        const uint32_t *v = u + 4u * g;
        w[3u * g] = (v[0] << 8) | ((v[1] >> 16) & 0xFFu);
        w[3u * g + 1u] = (v[1] << 16) | ((v[2] >> 8) & 0xFFFFu);
        w[3u * g + 2u] = (v[2] << 24) | (v[3] & 0xFFFFFFu);
    }
    crc_dma_start(f, w, 6);
    uint32_t err = u[0] | u[1] | u[2] | u[3] | u[4] | u[5] | u[6] | u[7];
    for (unsigned i = 0; i < 8u; i++) out[i] = sext24(u[i]);
    uint32_t c0 = dec[ext(a, 240u)], c1 = dec[ext(a, 250u)];
    uint32_t crc = crc_dma_result(f);
    return !((err | c0 | c1) & BAD) && ((crc ^ (c0 & 0xFFu) ^ ((c1 & 0xFFu) << 8)) & 0xFFFFu) == 0;
}

#endif

void link_fast_set_crc_dma(link_fast_t *f, int ch) {
    f->crc_dma = (int8_t)(ch < 0 ? -1 : ch);
    f->m33.dma_ch = ch < 0 ? 0 : LF_DMA_CH(ch);
}

uint32_t link_fast_crc_dma_probe(link_fast_t *f, const uint32_t words[6]) {
    crc_dma_start(f, words, 6);
    return crc_dma_result(f) & 0xFFFFu;
}
#endif

// A last bank of k < 8 channels at bit o of linear words r: the same
// checks, symbol by symbol (only a partial last bank takes this path).
static CPATH bool bank_k(const link_fast_t *f, const uint32_t *r, unsigned o, unsigned k, int32_t *out) {
    uint32_t err = 0, crc = 0xFFFFu, e[3];
    for (unsigned i = 0; i < k; i++) {
        for (unsigned j = 0; j < 3u; j++, o += 10u) {
            e[j] = f->dec[funnel(r + (o >> 5), o & 31u) >> 22];
            crc = (crc << 8) ^ f->crc16[((crc >> 8) ^ e[j]) & 0xFFu];
        }
        uint32_t u = join3(e[0], e[1], e[2]);
        err |= u;
        out[i] = sext24(u);
    }
    uint32_t c0 = f->dec[funnel(r + (o >> 5), o & 31u) >> 22];
    o += 10u;
    uint32_t c1 = f->dec[funnel(r + (o >> 5), o & 31u) >> 22];
    return !((err | c0 | c1) & BAD) && ((crc ^ (c0 & 0xFFu) ^ ((c1 & 0xFFu) << 8)) & 0xFFFFu) == 0;
}

#if LINK_FAST_M33
bool lf_bank8_sw_m33(const uint32_t *r, unsigned o, int32_t *out, const lf_m33_ctx_t *c);
bool lf_bank8_dma_m33(const uint32_t *r, unsigned o, int32_t *out, const lf_m33_ctx_t *c);
uint32_t lf_clean_m33(lf_clean_ctx_t *c);
_Static_assert(offsetof(lf_clean_ctx_t, dec) == 56 && offsetof(lf_clean_ctx_t, bankfn) == 72 &&
               offsetof(lf_clean_ctx_t, out) == 88 && offsetof(lf_clean_ctx_t, prev_hdr) == 108,
               "link_fast_m33_loop.S context layout");
_Static_assert(offsetof(lf_m33_ctx_t, dec) == 0 && offsetof(lf_m33_ctx_t, crc16) == 4 &&
               offsetof(lf_m33_ctx_t, dma_ch) == 8, "link_fast_m33.S context layout");
#endif

// A full bank at bit o of linear words r, by whichever decoder this build has.
static inline bool full_bank(const link_fast_t *f, const uint32_t *r, unsigned o, int32_t *out) {
#if LINK_FAST_M33
#if LINK_FAST_CRC_DMA
    if (f->crc_dma >= 0) return lf_bank8_dma_m33(r, o, out, &f->m33);
#endif
#if LINK_FAST_M33_SW
    return lf_bank8_sw_m33(r, o, out, &f->m33);
#else
    return bank8(f, r, o, out);
#endif
#else
#if LINK_FAST_CRC_DMA
    if (f->crc_dma >= 0) return bank8_dma(f, r, o, out);
#endif
    return bank8(f, r, o, out);
#endif
}

// Code errors of a failed bank (cold: only when a bank fails).
static COLD void count_code_errors(link_fast_t *f, uint32_t start, unsigned nsym) {
    for (unsigned j = 0; j < nsym; j++)
        if (f->dec[sym_end(f, start + 10u * (j + 1u))] & BAD) f->stats.code_errors++;
}

// Idle padding that is not all D21.5: count the symbols that differ (cold).
static COLD void count_idle_errors(link_fast_t *f, uint32_t r0, uint32_t r1) {
    for (uint32_t end = r0 + 10u; end != r1 + 10u; end += 10u)
        if (sym_end(f, end) != LINK_SYM_IDLE) f->stats.idle_errors++;
}

// Idle padding: the symbols occupying bits [b0, b1) of linear words r
// must all be D21.5, which on the line is 1010... starting with a 1 at b0,
// so whole words are compared with that pattern; the per-symbol count (at
// absolute positions from abs0) runs only when something differs.
static inline void check_idles(link_fast_t *f, const uint32_t *r, uint32_t b0, uint32_t b1, uint32_t abs0) {
    uint32_t pat = (b0 & 1u) ? 0x55555555u : 0xAAAAAAAAu;
    const uint32_t *w = r + (b0 >> 5), *wl = r + ((b1 - 1u) >> 5);
    uint32_t diff = (*w ^ pat) & (0xFFFFFFFFu >> (b0 & 31u));
    uint32_t last_mask = ~(0x7FFFFFFFu >> ((b1 - 1u) & 31u));
    if (w == wl) {
        diff &= last_mask;
    } else {
        for (w++; w != wl; w++) diff |= *w ^ pat;
        diff |= (*wl ^ pat) & last_mask;
    }
    if (diff) count_idle_errors(f, abs0, abs0 + (b1 - b0));
}

// ---------------------------------------------------------------------------
// LOCKED: one frame, the next comma
// ---------------------------------------------------------------------------

// A bank failed its symbols or CRC: counters, HOLD on the first consecutive
// failure, ZERO from the second (cold).
static COLD uint8_t bank_failed(link_fast_t *f, unsigned b, uint32_t start, unsigned k) {
    count_code_errors(f, start, LINK_BANK_SYMBOLS(k));
    f->stats.crc16_errors++;
    if (f->bank_fails[b] < 0xFFu) f->bank_fails[b]++;
    if (f->bank_fails[b] == 1u) return LINK_BANK_HOLD;
    memset(f->samples + LINK_BANK_CHANNELS * b, 0, k * sizeof(int32_t));
    return LINK_BANK_ZERO;
}

// FRAME event (only with link_fast_t.frame_events, i.e. on the host tests).
static COLD void emit_frame(link_fast_t *f, uint8_t flags, const uint8_t *status) {
    link_rx_event_t *ev = &f->ev;
    ev->type = LINK_RX_EV_FRAME;
    ev->frame = f->frame;
    ev->flags = flags;
    ev->slip = f->slip;
    ev->channels = f->channels;
    memcpy(ev->bank_status, status, LINK_MAX_BANKS);
    ev->samples = f->samples;
    lf_emit(f, ev);
}

// Index 31 of the superframe: check it, adopt it, correct F.
static LINK_FAST_HOT __attribute__((noinline)) void sf_complete(link_fast_t *f) {
    link_sf_t sf;
    int err = f->sf_count == LINK_SF_LEN ? lf_sf_check(f, &sf) : LINK_RX_SF_INCOMPLETE;
    if (err != LINK_SF_OK) {
        lf_sf_error(f, err);
        return;
    }
    if (sf.frame + LINK_SF_LEN - 1u != f->frame) {
        f->stats.counter_errors++;
        f->frame = sf.frame + LINK_SF_LEN - 1u;
        set_layout(f);  // F mod den
    }
    lf_adopt_sf(f, &sf);
    link_rx_event_t sev;
    memset(&sev, 0, sizeof(sev));
    sev.type = LINK_RX_EV_SUPERFRAME;
    sev.frame = f->frame;
    sev.sf = &f->sf;
    lf_emit(f, &sev);
}

// Frame F with its comma ending at p and s symbols (spec 8.4).  False when
// it ended in a loss (the receiver is then in SEARCH).
static inline bool lf_frame(link_fast_t *f, uint32_t p, unsigned s, const uint32_t *wp) {
    unsigned o = p & 31u;
    uint8_t flags = f->comma_ok ? 0u : LINK_RXF_COMMA_MISSING;
    if (f->slip) flags |= LINK_RXF_SLIP;

    // Head: HDR, CTL, CRC8 in bits p .. p + 29.
    uint32_t x = funnel(wp, o);
    uint32_t e0 = f->dec[x >> 22], e1 = f->dec[(x >> 12) & 0x3FFu], e2 = f->dec[(x >> 2) & 0x3FFu];
    bool good = false;
    if ((e0 | e1 | e2) & BAD) {
        f->stats.code_errors += (e0 >> 31) + (e1 >> 31) + (e2 >> 31);
    } else {
        good = f->crc8[f->crc8[e0 & 0xFFu] ^ (e1 & 0xFFu)] == (e2 & 0xFFu);
    }
    if (!good) {
        f->stats.crc8_errors++;
        flags |= LINK_RXF_HEAD_BAD;
    } else if ((uint8_t)e0 != (uint8_t)f->frame) {
        f->stats.counter_errors++;
        flags |= LINK_RXF_COUNTER;
        good = false;
    }

    // Banks in the mask; the others are stepped over unread.
    uint8_t status[LINK_MAX_BANKS] = { 0 };
    for (unsigned m = f->decode_banks; m; m &= m - 1u) {
        unsigned b = (unsigned)__builtin_ctz(m);
        uint32_t boff = o + 10u * (LINK_HEAD_SYMBOLS - 1u + b * LINK_BANK_SYMBOLS(LINK_BANK_CHANNELS));
        const uint32_t *br = wp + (boff >> 5);
        unsigned bo = boff & 31u;
        int32_t *out = f->samples + LINK_BANK_CHANNELS * b;
        int32_t tmp[LINK_BANK_CHANNELS];
        unsigned k = f->channels - LINK_BANK_CHANNELS * b;
        bool ok;
        if (k >= LINK_BANK_CHANNELS) {
            k = LINK_BANK_CHANNELS;
            ok = full_bank(f, br, bo, tmp);
            if (ok) memcpy(out, tmp, sizeof(tmp));
        } else {
            ok = bank_k(f, br, bo, k, tmp);
            if (ok)
                for (unsigned i = 0; i < k; i++) out[i] = tmp[i];
        }
        if (ok) {
            status[b] = LINK_BANK_OK;
            f->bank_fails[b] = 0;
        } else {
            status[b] = bank_failed(f, b, p - o + boff, k);
        }
    }
    if (s > f->layout) {
        uint32_t i0 = o + 10u * (f->layout - 1u);
        check_idles(f, wp, i0, o + 10u * (s - 1u), p - o + i0);
    }
    f->stats.frames++;

    // Delivery: the unit's 8 slots into the caller's block.
    if (f->out) {
        int32_t *o = f->out[f->out_n++];
        for (unsigned i = 0; i < 8u; i++) o[i] = *f->slot_src[i];
    }
    if (f->frame_events) emit_frame(f, flags, status);

    // Superframe assembly (8.4 step 5).
    unsigned idx = (unsigned)f->frame & (LINK_SF_LEN - 1u);
    if (good) {
        lf_sf_collect(f, idx, (uint8_t)e1, (uint8_t)e0);
    } else {
        f->sf_count = 0;
        f->prev_hdr = -1;
    }
    if (idx == LINK_SF_LEN - 1u) sf_complete(f);

    if (flags & (LINK_RXF_COMMA_MISSING | LINK_RXF_HEAD_BAD | LINK_RXF_COUNTER)) {
        if (++f->fails >= LINK_RX_FAIL_LIMIT) {
            f->p = p;
            lf_loss(f);
            return false;
        }
    } else {
        f->fails = 0;
    }
    return true;
}

// A RATE apply reached its target frame: switch the layout when it fits (cold).
static COLD void rate_switch(link_fast_t *f) {
    unsigned arg = f->sf.apply_arg;
    unsigned c = f->sf.apply_value ? (unsigned)f->sf.apply_value : f->channels;
    if (arg <= LINK_RATE_96K && f->sf.apply_value >= 0 && c >= 1u && c <= f->cap[f->cls][arg]) {
        f->rate = (uint8_t)arg;
        f->channels = (uint8_t)c;
        set_layout(f);
    }
    f->apply_pending = false;
}

// No comma at e: the slip window (-1, +1, -2, +2), else the flywheel (cold).
// x holds bits e - 12 .. e + 19.  Returns where the next frame starts.
static COLD uint32_t slip_or_flywheel(link_fast_t *f, uint32_t x, uint32_t e) {
    static const int8_t order[4] = { -1, 1, -2, 2 };
    for (unsigned i = 0; i < 4u; i++) {
        int d = order[i];
        if (is_comma((x >> (20 - d)) & 0x3FFu)) {
            f->comma_ok = true;
            f->slip = (int8_t)d;
            f->stats.bit_slips++;
            return e + (uint32_t)d;
        }
    }
    f->comma_ok = false;
    f->slip = 0;
    f->stats.comma_errors++;
    return e;
}

// FRAME event for a clean frame (link_fast_t.frame_events only).
static COLD void emit_clean_frame(link_fast_t *f, uint64_t frame, unsigned banks) {
    uint8_t status[LINK_MAX_BANKS] = { 0 };
    for (unsigned b = 0; b < LINK_MAX_BANKS; b++)
        if ((banks >> b) & 1u) status[b] = LINK_BANK_OK;
    uint64_t keep = f->frame;
    f->frame = frame;
    emit_frame(f, 0, status);
    f->frame = keep;
}

// Non-zero when the bits [b0, b1) of linear words r are not all D21.5
// (the line pattern 1010... starting with a 1 at b0).  Two words a step:
// the padding is a few words long, so the loop must not carry an unrolled
// prologue.
static inline uint32_t idle_diff(const uint32_t *r, uint32_t b0, uint32_t b1) {
    uint32_t pat = (b0 & 1u) ? 0x55555555u : 0xAAAAAAAAu;
    const uint32_t *w = r + (b0 >> 5), *wl = r + ((b1 - 1u) >> 5);
    uint32_t first = (*w ^ pat) & (0xFFFFFFFFu >> (b0 & 31u));
    uint32_t last = (*wl ^ pat) & ~(0x7FFFFFFFu >> ((b1 - 1u) & 31u));
    if (w == wl) return first & last;
    uint32_t diff = first | last;
    w++;
    while (wl - w >= 2) {
        diff |= (w[0] ^ pat) | (w[1] ^ pat);
        w += 2;
    }
    if (w != wl) diff |= *w ^ pat;
    return diff;
}

// The clean-frame fast path.  It decodes LOCKED frames while each one is
// clean: the frame starts at its comma (no slip, no flywheel), its head is
// good with HDR = F & 0xFF, every decoded bank passes, the padding is all
// D21.5, the next comma sits exactly where the layout puts it, the frame is
// not index 31 of the superframe and no RATE switch is due, and the frame
// does not cross the ring's wrap.  For such a frame every effect of 8.4 and
// 8.5 is known in advance (no counter but `frames` moves, the failure count
// is 0, the superframe assembly takes CTL), so they are applied from locals
// here.  It stops, before touching anything, at the first frame that is not
// clean; lf_locked() then takes that frame through lf_frame().
static CPATH void lf_clean(link_fast_t *f) {
    if (!f->comma_ok || f->slip || f->body_done) return;
    const uint32_t avail = f->avail, mask = f->ring_mask;
    const uint32_t *ring = ring_words(f);
    const uint32_t *dec = f->dec;
    const unsigned num = f->num, den = f->den, layout = f->layout, banks = f->decode_banks;
    const unsigned c = f->channels;
    uint32_t p = f->p, fmod = f->fmod, frames = 0;
    uint64_t frame = f->frame;
    unsigned sf_count = f->sf_count;
    int prev_hdr = f->prev_hdr;
    unsigned out_n = f->out_n, out_cap = f->out ? f->out_cap : 0xFFFFu;
    for (;;) {
        unsigned s = den == 1u ? num : ((fmod + 1u) * num) / den - (fmod * num) / den;
        uint32_t e = p + 10u * s;
        if ((int32_t)(avail - (e + LINK_RX_SLIP_MAX)) < 0 || out_n >= out_cap) break;
        unsigned o = p & 31u, idx = (unsigned)frame & (LINK_SF_LEN - 1u);
        uint32_t wb = (p >> 5) & mask;
        if (wb + (o + 10u * s + 2u) / 32u + 2u > mask + 1u || idx == LINK_SF_LEN - 1u) break;
        if (f->apply_pending && frame + 1u == f->sf.apply_frame) break;
        const uint32_t *wp = ring + wb;

        // Head.
        uint32_t x = funnel(wp, o);
        uint32_t e0 = dec[x >> 22], e1 = dec[(x >> 12) & 0x3FFu], e2 = dec[(x >> 2) & 0x3FFu];
        if ((e0 | e1 | e2) & BAD) break;
        if (f->crc8[f->crc8[e0 & 0xFFu] ^ (e1 & 0xFFu)] != (e2 & 0xFFu) || (uint8_t)e0 != (uint8_t)frame) break;

        // Next comma, exactly at e.
        uint32_t xo = o + 10u * s - 12u;
        uint32_t nx = (funnel(wp + (xo >> 5), xo & 31u) >> 20) & 0x3FFu;
        if (!is_comma(nx)) break;

        // Padding.
        if (s > layout && idle_diff(wp, o + 10u * (layout - 1u), o + 10u * (s - 1u))) break;

        // Banks into temporaries; committed only if all pass.
        int32_t tmp[LINK_MAX_BANKS][LINK_BANK_CHANNELS];
        bool ok = true;
        for (unsigned m = banks; m && ok; m &= m - 1u) {
            unsigned b = (unsigned)__builtin_ctz(m);
            uint32_t boff = o + 10u * (LINK_HEAD_SYMBOLS - 1u + b * LINK_BANK_SYMBOLS(LINK_BANK_CHANNELS));
            const uint32_t *br = wp + (boff >> 5);
            unsigned k = c - LINK_BANK_CHANNELS * b;
            if (k >= LINK_BANK_CHANNELS)
                ok = full_bank(f, br, boff & 31u, tmp[b]);
            else
                ok = bank_k(f, br, boff & 31u, k, tmp[b]);
        }
        if (!ok) break;

        // Commit (8.4 steps 2, 4, 5, 6 and 8.5 for a clean frame).
        for (unsigned m = banks; m; m &= m - 1u) {
            unsigned b = (unsigned)__builtin_ctz(m);
            unsigned k = c - LINK_BANK_CHANNELS * b;
            int32_t *dst = f->samples + LINK_BANK_CHANNELS * b;
            if (k >= LINK_BANK_CHANNELS)
                memcpy(dst, tmp[b], sizeof(tmp[b]));
            else
                for (unsigned i = 0; i < k; i++) dst[i] = tmp[b][i];
            f->bank_fails[b] = 0;
        }
        frames++;
        if (f->out) {
            int32_t *d = f->out[out_n];
            for (unsigned i = 0; i < 8u; i++) d[i] = *f->slot_src[i];
        }
        out_n++;
        if (f->frame_events) emit_clean_frame(f, frame, banks);
        uint8_t hdr = (uint8_t)e0;
        if (idx == 0) {
            f->sf_bytes[0] = (uint8_t)e1;
            sf_count = 1;
        } else if (prev_hdr >= 0 && hdr == (uint8_t)(prev_hdr + 1) && sf_count == idx) {
            f->sf_bytes[idx] = (uint8_t)e1;
            sf_count++;
        } else {
            sf_count = 0;
        }
        prev_hdr = hdr;
        frame++;
        if (++fmod == den) fmod = 0;
        p = e;
    }
    if (frames) {
        f->p = p;
        f->frame = frame;
        f->fmod = (uint16_t)fmod;
        f->sf_count = (uint8_t)sf_count;
        f->prev_hdr = (int16_t)prev_hdr;
        f->fails = 0;
        f->stats.frames += frames;
        if (f->out) f->out_n = (uint16_t)out_n;
    }
}

// Eight samples, copied as one object (LDM/STM).
typedef struct { int32_t v[8]; } lf_row_t;

// The clean-frame fast path at 48 and 96 kHz.  There every frame is a whole
// number of words, so the comma's bit offset is the same in every frame and
// all positions inside a frame are constants for the call; how many frames
// may run before a boundary (index 31 of the superframe, a RATE switch, the
// output block, the writer, the ring's wrap) is computed once, so the loop
// itself checks only the frames' content.  Same contract as lf_clean().
// Frames lf_clean_fixed() may take from the current one before any boundary
// (index 31 of the superframe, a RATE switch, the output block, the writer,
// the ring's wrap); 0 when none.  *wb is the ring word holding bit p.
static inline uint32_t clean_budget(const link_fast_t *f, uint32_t *wb) {
    if (!f->comma_ok || f->slip || f->body_done) return 0;
    const uint32_t s10 = 10u * f->num, mask = f->ring_mask, p = f->p;
    uint32_t n = 31u - ((uint32_t)f->frame & (LINK_SF_LEN - 1u));
    if (f->apply_pending) {
        uint64_t d = f->sf.apply_frame - f->frame;      // >= 1 while pending
        if (d - 1u < n) n = (uint32_t)(d - 1u);
    }
    if (f->out && (uint32_t)(f->out_cap - f->out_n) < n) n = f->out_cap - f->out_n;
    int32_t room = (int32_t)(f->avail - p - LINK_RX_SLIP_MAX);
    if (room < (int32_t)s10) return 0;
    if ((uint32_t)room / s10 < n) n = (uint32_t)room / s10;
    const unsigned o = p & 31u;
    const uint32_t nw = (o + s10 + LINK_RX_SLIP_MAX) / 32u + 2u;
    *wb = (p >> 5) & mask;
    if (*wb + nw > mask + 1u) return 0;
    uint32_t wrap = ((mask + 1u - nw - *wb + 1u) * 32u - o + s10 - 1u) / s10;
    return wrap < n ? wrap : n;
}

// Commit what a clean run took: positions, F, the superframe assembly.
static inline void clean_commit(link_fast_t *f, uint32_t done, uint32_t flo, unsigned sf_count, int prev_hdr) {
    if (!done) return;
    f->p += done * 10u * f->num;
    f->frame = (f->frame & ~0xFFFFFFFFull) | flo;   // no carry: index 31 is never taken here
    f->sf_count = (uint8_t)sf_count;
    f->prev_hdr = (int16_t)prev_hdr;
    f->fails = 0;
    f->stats.frames += done;
    if (f->out) f->out_n = (uint16_t)(f->out_n + done);
}

#if LINK_FAST_M33
// The common case in assembly: 48/96 kHz, one full bank, delivered whole or
// not at all, a bank decoder in assembly.  False when the case is not its own
// (lf_clean_fixed() then runs); true otherwise, however many frames it took.
static LINK_FAST_HOT __attribute__((noinline)) bool lf_clean_asm(link_fast_t *f) {
    const unsigned banks = f->decode_banks;
    if (!banks || (banks & (banks - 1u)) || f->frame_events || (f->out && f->slot_run < 0)) return false;
    unsigned b = (unsigned)__builtin_ctz(banks);
    if (f->channels - LINK_BANK_CHANNELS * b < LINK_BANK_CHANNELS) return false;
    bool (*bankfn)(const uint32_t *, unsigned, int32_t *, const lf_m33_ctx_t *) = 0;
#if LINK_FAST_CRC_DMA
    if (f->crc_dma >= 0) bankfn = lf_bank8_dma_m33;
#endif
#if LINK_FAST_M33_SW
    if (!bankfn) bankfn = lf_bank8_sw_m33;
#endif
    if (!bankfn) return false;
    uint32_t wb;
    uint32_t n = clean_budget(f, &wb);
    if (!n) return true;
    const unsigned s = f->num, layout = f->layout, o = f->p & 31u;
    const uint32_t s10 = 10u * s;
    const uint32_t xo = o + s10 - 12u, xw = xo >> 5, xs = xo & 31u;
    const bool pad = s > layout;
    const uint32_t b0 = o + 10u * (layout - 1u), b1 = o + 10u * (s - 1u);
    const uint32_t iw0 = b0 >> 5, iw1 = (b1 - 1u) >> 5;
    const uint32_t fm = 0xFFFFFFFFu >> (b0 & 31u), lm = ~(0x7FFFFFFFu >> ((b1 - 1u) & 31u));
    uint32_t boff = o + 10u * (LINK_HEAD_SYMBOLS - 1u + b * LINK_BANK_SYMBOLS(LINK_BANK_CHANNELS));
    lf_clean_ctx_t k = {
        .wp = ring_words(f) + wb, .n = n, .step = 4u * (s10 / 32u), .o = o, .ro = 32u - o,
        .xw = 4u * xw, .xs = xs, .xro = 32u - xs,
        .iw0 = 4u * iw0, .iw1 = 4u * iw1, .fm = pad ? (iw0 == iw1 ? fm & lm : fm) : 0u,
        .lm = pad && iw0 != iw1 ? lm : 0u, .pat = (b0 & 1u) ? 0x55555555u : 0xAAAAAAAAu,
        .mid = pad && iw1 > iw0 ? iw1 - iw0 - 1u : 0u,
        .dec = f->dec, .crc8 = f->crc8, .bw = 4u * (boff >> 5), .bo = boff & 31u, .bankfn = bankfn,
        .m33 = &f->m33, .samples = f->samples + LINK_BANK_CHANNELS * b, .fails = &f->bank_fails[b],
        .out = f->out ? f->out + f->out_n : 0, .src = f->slot_run >= 0 ? f->samples + f->slot_run : 0,
        .flo = (uint32_t)f->frame, .sf_bytes = f->sf_bytes, .sf_count = f->sf_count, .prev_hdr = f->prev_hdr,
    };
    uint32_t done = lf_clean_m33(&k);
    clean_commit(f, done, k.flo, k.sf_count, k.prev_hdr);
    return true;
}
#endif

static CPATH void lf_clean_fixed(link_fast_t *f) {
    uint32_t wb;
    uint32_t n = clean_budget(f, &wb);
    if (!n) return;
    const unsigned s = f->num, layout = f->layout, banks = f->decode_banks, c = f->channels;
    const uint32_t s10 = 10u * s;
    uint32_t flo = (uint32_t)f->frame;
    const unsigned o = f->p & 31u;

    // Positions inside a frame, from the word holding its comma end.
    const uint32_t *wp = ring_words(f) + wb;
    const uint32_t step = s10 / 32u;
    const uint32_t xo = o + s10 - 12u, xw = xo >> 5, xs = xo & 31u;
    const bool pad = s > layout;
    const uint32_t b0 = o + 10u * (layout - 1u), b1 = o + 10u * (s - 1u);
    const uint32_t iw0 = b0 >> 5, iw1 = (b1 - 1u) >> 5;
    const uint32_t pat = (b0 & 1u) ? 0x55555555u : 0xAAAAAAAAu;
    const uint32_t fm = 0xFFFFFFFFu >> (b0 & 31u), lm = ~(0x7FFFFFFFu >> ((b1 - 1u) & 31u));
    const uint32_t *dec = f->dec;
    const uint8_t *crc8 = f->crc8;
    unsigned sf_count = f->sf_count;
    int prev_hdr = f->prev_hdr;
    int32_t (*out)[8] = f->out ? f->out + f->out_n : 0;
    uint32_t done = 0;

    for (; done < n; done++, wp += step) {
        // Head.
        uint32_t x = funnel(wp, o);
        uint32_t e0 = dec[x >> 22], e1 = dec[(x >> 12) & 0x3FFu], e2 = dec[(x >> 2) & 0x3FFu];
        if ((e0 | e1 | e2) & BAD) break;
        if (crc8[crc8[e0 & 0xFFu] ^ (e1 & 0xFFu)] != (e2 & 0xFFu) || ((e0 ^ flo) & 0xFFu)) break;
        // Next comma, exactly at e.
        if (!is_comma((funnel(wp + xw, xs) >> 20) & 0x3FFu)) break;
        // Padding.
        if (pad) {
            uint32_t diff = (wp[iw0] ^ pat) & fm;
            if (iw0 == iw1) {
                diff &= lm;
            } else {
                diff |= (wp[iw1] ^ pat) & lm;
                // The middle words, two a step, no unrolled prologue: a few words.
                const uint32_t *q = wp + iw0 + 1u, *qe = wp + iw1;
                while (qe - q >= 2) {
                    diff |= (q[0] ^ pat) | (q[1] ^ pat);
                    q += 2;
                }
                if (q != qe) diff |= *q ^ pat;
            }
            if (diff) break;
        }
        // Banks into temporaries, committed only if all pass.
        int32_t tmp[LINK_MAX_BANKS][LINK_BANK_CHANNELS];
        bool ok = true;
        for (unsigned m = banks; m && ok; m &= m - 1u) {
            unsigned b = (unsigned)__builtin_ctz(m);
            uint32_t boff = o + 10u * (LINK_HEAD_SYMBOLS - 1u + b * LINK_BANK_SYMBOLS(LINK_BANK_CHANNELS));
            unsigned k = c - LINK_BANK_CHANNELS * b;
            ok = k >= LINK_BANK_CHANNELS ? full_bank(f, wp + (boff >> 5), boff & 31u, tmp[b])
                                         : bank_k(f, wp + (boff >> 5), boff & 31u, k, tmp[b]);
        }
        if (!ok) break;

        // Commit.
        for (unsigned m = banks; m; m &= m - 1u) {
            unsigned b = (unsigned)__builtin_ctz(m);
            unsigned k = c - LINK_BANK_CHANNELS * b;
            int32_t *dst = f->samples + LINK_BANK_CHANNELS * b;
            if (k >= LINK_BANK_CHANNELS)
                *(lf_row_t *)dst = *(const lf_row_t *)tmp[b];
            else
                for (unsigned i = 0; i < k; i++) dst[i] = tmp[b][i];
            f->bank_fails[b] = 0;
        }
        if (out) {
            if (f->slot_run >= 0) {
                *(lf_row_t *)*out = *(const lf_row_t *)(f->samples + f->slot_run);
            } else {
                for (unsigned i = 0; i < 8u; i++) (*out)[i] = *f->slot_src[i];
            }
            out++;
        }
        if (f->frame_events) emit_clean_frame(f, (f->frame & ~0xFFFFFFFFull) | flo, banks);
        unsigned idx = flo & (LINK_SF_LEN - 1u);
        uint8_t hdr = (uint8_t)e0;
        if (idx == 0) {
            f->sf_bytes[0] = (uint8_t)e1;
            sf_count = 1;
        } else if (prev_hdr >= 0 && hdr == (uint8_t)(prev_hdr + 1) && sf_count == idx) {
            f->sf_bytes[idx] = (uint8_t)e1;
            sf_count++;
        } else {
            sf_count = 0;
        }
        prev_hdr = hdr;
        flo++;
    }
    clean_commit(f, done, flo, sf_count, prev_hdr);
}

#if LINK_FAST_M33
LINK_FAST_HOT unsigned link_fast_run_clean(link_fast_t *f, uint64_t avail_bits) {
    atomic_signal_fence(memory_order_seq_cst);
    f->avail64 = avail_bits;
    f->avail = (uint32_t)avail_bits;
    if (f->state != LINK_RX_LOCKED || f->den != 1u) return 0;
    uint32_t frames0 = f->stats.frames;
    (void)lf_clean_asm(f);
    return f->stats.frames - frames0;
}

LINK_FAST_HOT bool link_fast_needs_general(const link_fast_t *f) {
    if (f->state != LINK_RX_LOCKED || f->den != 1u) return true;
    if (f->out && f->out_n >= f->out_cap) return false;
    return (int32_t)(f->avail - (f->p + 10u * f->num + LINK_RX_SLIP_MAX)) >= 0;
}
#endif

// LOCKED frames while their bits are in the ring and the output block has room.
// Each frame is read through a linear view: the ring itself, or (for the
// one frame per ring pass that crosses the wrap) a copy of its words.
static LINK_FAST_HOT void lf_locked(link_fast_t *f) {
    const uint32_t avail = f->avail, mask = f->ring_mask;
    const uint32_t *ring = ring_words(f);
    for (;;) {
        if (f->den == 1u) {
#if LINK_FAST_M33
            if (!lf_clean_asm(f)) lf_clean_fixed(f);
#else
            lf_clean_fixed(f);
#endif
        } else {
            lf_clean(f);
        }
        uint32_t p = f->p;
        unsigned s = frame_syms(f);
        // Words from the one holding bit p to the one after the next
        // comma's slip window (bit p + 10 s + 2), plus one for funnel().
        uint32_t wb = (p >> 5) & mask, nw = ((p & 31u) + 10u * s + 2u) / 32u + 2u;
        const uint32_t *wp = ring + wb;
        if (wb + nw > mask + 1u) {
            for (uint32_t i = 0; i < nw; i++) f->lin[i] = ring[(wb + i) & mask];
            wp = f->lin;
        }
        if (!f->body_done) {
            if (f->out && f->out_n >= f->out_cap) break;
            if ((int32_t)(avail - (p + 10u * (s - 1u))) < 0) break;
            if (!lf_frame(f, p, s, wp)) return;
            f->body_done = true;
        }
        // The next comma (8.5), needs bits up to e + 2.
        uint32_t e = p + 10u * s;
        if ((int32_t)(avail - (e + LINK_RX_SLIP_MAX)) < 0) break;
        uint32_t xo = (p & 31u) + 10u * s - 12u;
        uint32_t x = funnel(wp + (xo >> 5), xo & 31u);
        f->frame++;
        if (++f->fmod == f->den) f->fmod = 0;
        if (f->apply_pending && f->frame == f->sf.apply_frame) rate_switch(f);
        if (is_comma((x >> 20) & 0x3FFu)) {
            p = e;
            f->comma_ok = true;
            f->slip = 0;
        } else {
            p = slip_or_flywheel(f, x, e);
        }
        f->body_done = false;
        f->p = p;
    }
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void link_fast_init(link_fast_t *f, const volatile uint32_t *ring, uint32_t ring_words, unsigned cls,
                    uint8_t bank_mask, link_rx_cb_t cb, void *ctx, uint64_t start_bit) {
    memset(f, 0, sizeof(*f));
    f->avail64 = start_bit;
    f->avail = (uint32_t)start_bit;
    f->ring = ring;
    f->ring_mask = ring_words - 1u;
    f->cls = (uint8_t)cls;
    f->bank_mask = bank_mask;
    f->cb = cb;
    f->ctx = ctx;
    for (unsigned c = 1; c <= 3u; c++)
        for (unsigned r = 0; r <= LINK_RATE_96K; r++) {
            f->cap[c][r] = (uint8_t)link_max_channels(c, r);
            // Symbols per frame num / den in lowest terms (link_codec.c frame_ratio()).
            uint32_t n = link_class_baud(c), d = 10u * link_rate_hz(r), a = n, b = d;
            while (b) { uint32_t t = a % b; a = b; b = t; }
            f->ratio[c][r][0] = (uint16_t)(n / a);
            f->ratio[c][r][1] = (uint16_t)(d / a);
        }
    for (unsigned i = 0; i < 256u; i++) {
        unsigned c8 = i, c16 = i << 8;
        for (int b = 0; b < 8; b++) {
            c8 = (c8 & 0x80u) ? (c8 << 1) ^ 0x07u : c8 << 1;
            c16 = (c16 & 0x8000u) ? (c16 << 1) ^ 0x1021u : c16 << 1;
        }
        f->crc8[i] = (uint8_t)c8;
        f->crc16[i] = (uint16_t)c16;
    }
    for (unsigned s = 0; s < 1024u; s++) {
        uint8_t byte = 0;
        bool k = false;
        unsigned m = link_8b10b_decode((uint16_t)s, &byte, &k);
        uint32_t e = m ? byte | (k ? LINK_FAST_DEC_K : 0u) | ((m & 1u) ? LINK_FAST_DEC_RDN : 0u) |
                             ((m & 2u) ? LINK_FAST_DEC_RDP : 0u)
                       : 0u;
        if (!m || k) e |= LINK_FAST_DEC_BAD;
        f->dec[s] = e;
    }
    for (unsigned i = 0; i < 8u; i++) f->slot_src[i] = &f->zero;
    f->slot_run = -1;
    f->crc_dma = -1;
    f->m33.dec = f->dec;
    f->m33.crc16 = f->crc16;
    lf_enter_search(f, (uint32_t)start_bit + 10u);
}

void link_fast_restart(link_fast_t *f, unsigned cls, uint64_t start_bit) {
    // Everything before the tables is state or configuration; keep the latter.
    const volatile uint32_t *ring = f->ring;
    uint32_t ring_mask = f->ring_mask;
    uint8_t bank_mask = f->bank_mask;
    link_rx_cb_t cb = f->cb;
    void *ctx = f->ctx;
    int32_t (*out)[8] = f->out;
    uint16_t out_cap = f->out_cap;
    const int32_t *slot_src[8];
    memcpy(slot_src, f->slot_src, sizeof(slot_src));
    int8_t slot_run = f->slot_run, crc_dma = f->crc_dma;
    bool frame_events = f->frame_events;
    lf_m33_ctx_t m33 = f->m33;
    memset(f, 0, offsetof(link_fast_t, cap));
    f->ring = ring;
    f->ring_mask = ring_mask;
    f->cls = (uint8_t)cls;
    f->bank_mask = bank_mask;
    f->cb = cb;
    f->ctx = ctx;
    f->out = out;
    f->out_cap = out_cap;
    memcpy(f->slot_src, slot_src, sizeof(slot_src));    // they point into f->samples or f->zero
    f->slot_run = slot_run;
    f->crc_dma = crc_dma;
    f->frame_events = frame_events;
    f->m33 = m33;
    f->avail64 = start_bit;
    f->avail = (uint32_t)start_bit;
    lf_enter_search(f, (uint32_t)start_bit + 10u);
}

void link_fast_set_slots(link_fast_t *f, const uint8_t slots[8]) {
    f->slot_run = slots[0] % LINK_BANK_CHANNELS == 0 && slots[0] < LINK_MAX_CHANNELS ? (int8_t)slots[0] : -1;
    for (unsigned i = 0; i < 8u; i++) {
        f->slot_src[i] = slots[i] < LINK_MAX_CHANNELS ? &f->samples[slots[i]] : &f->zero;
        if (slots[i] != slots[0] + i) f->slot_run = -1;
    }
}


void link_fast_set_bank_mask(link_fast_t *f, uint8_t bank_mask) {
    for (unsigned b = 0; b < LINK_MAX_BANKS; b++)
        if (((f->bank_mask & ~bank_mask) >> b) & 1u) {
            memset(f->samples + LINK_BANK_CHANNELS * b, 0, LINK_BANK_CHANNELS * sizeof(int32_t));
            f->bank_fails[b] = 0;
        }
    f->bank_mask = bank_mask;
    f->decode_banks = (uint8_t)(bank_mask & ((1u << f->nbanks) - 1u));
}

unsigned link_fast_run(link_fast_t *f, uint64_t avail_bits, uint32_t search_budget_bits) {
    // Ring words below the writer's position are complete; read them only
    // after the caller read that position.
    atomic_signal_fence(memory_order_seq_cst);
    f->avail64 = avail_bits;
    f->avail = (uint32_t)avail_bits;
    uint32_t ring_bits = (f->ring_mask + 1u) * 32u;

    // The oldest bit a step can still read: the comma before p (and the
    // slip window, 2 more), or 10 before the next search position.
    uint32_t oldest = (f->state == LINK_RX_SEARCH ? f->q : f->p) - 12u;
    if ((int32_t)(f->avail - oldest) > (int32_t)(ring_bits - LINK_FAST_GUARD_BITS)) {
        f->overruns++;
        if (f->state != LINK_RX_SEARCH) lf_loss(f);
        f->q = f->avail - ring_bits / 2u;
    } else if (f->state == LINK_RX_SEARCH && search_budget_bits &&
               (int32_t)(f->avail - f->q) > (int32_t)search_budget_bits) {
        f->q = f->avail - search_budget_bits / 2u;
    }

    uint32_t frames0 = f->stats.frames;
    for (;;) {
        if (f->state == LINK_RX_LOCKED) {
            lf_locked(f);
            if (f->state == LINK_RX_LOCKED) break;
            continue;
        }
        bool progressed = f->state == LINK_RX_SEARCH ? lf_step_search(f)
                          : f->body_done             ? lf_next_aligned(f)
                                                     : lf_body_aligned(f);
        if (!progressed) break;
    }
    return f->stats.frames - frames0;
}
