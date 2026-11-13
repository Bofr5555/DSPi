/*
 * bench_check.c - link bench: receiver-side checks
 *
 * See bench_check.h.
 */

#include "bench_check.h"

#include <string.h>

static unsigned popcount32(uint32_t v) {
    v = v - ((v >> 1) & 0x55555555u);
    v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
    return (((v + (v >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
}

// ---------------------------------------------------------------------------
// Acquisition
// ---------------------------------------------------------------------------

uint32_t bench_acquire_bits(unsigned cls) {
    unsigned longest = link_frame_symbols_max(cls, LINK_RATE_44K1);
    return 10u * longest * 68u;
}

typedef struct {
    link_rx_t *rx;
    bool done;
    bench_acq_t *out;
    uint64_t first_bit;
} acq_ctx_t;

static void acq_cb(void *ctx, const link_rx_event_t *ev) {
    acq_ctx_t *a = ctx;
    if (a->done || ev->type != LINK_RX_EV_FRAME || ev->flags) return;
    const link_rx_t *rx = a->rx;
    uint64_t at = rx->p - 10u - rx->base;   // the comma, still in the decoder's buffer
    uint16_t sym = 0;
    for (unsigned i = 0; i < 10u; i++, at++)
        sym = (uint16_t)((sym << 1) | ((rx->buf[at >> 5] >> (31u - (unsigned)(at & 31u))) & 1u));
    if (!link_sym_is_comma(sym)) return;
    a->out->frame = ev->frame;
    a->out->start = a->first_bit + rx->p - 10u;
    a->out->rd = sym == LINK_SYM_K28_5_RDN ? 0u : 1u;
    a->out->sf = rx->sf;
    a->done = true;
}

bool bench_acquire(const uint32_t *words, uint64_t first_bit, size_t nbits, unsigned cls,
                   link_rx_t *rx, bench_acq_t *out) {
    acq_ctx_t a = { .rx = rx, .done = false, .out = out, .first_bit = first_bit };
    link_rx_init(rx, cls, 0x01, acq_cb, &a);
    for (size_t at = 0; at < nbits && !a.done; at += 1024u) {
        size_t n = nbits - at < 1024u ? nbits - at : 1024u;
        link_rx_push(rx, words + at / 32u, n);
    }
    out->stats = rx->stats;
    return a.done;
}

// ---------------------------------------------------------------------------
// Bit error rate tester
// ---------------------------------------------------------------------------

void bench_bert_init(bench_bert_t *b) {
    memset(b, 0, sizeof(*b));
}

bool bench_bert_start(bench_bert_t *b, const bench_acq_t *a) {
    const link_sf_t *sf = &a->sf;
    if (sf->apply_op != LINK_OP_NONE) return false;
    unsigned pay = (sf->flags & LINK_SF_FLAG_PRBS) ? BENCH_PAY_PRBS : BENCH_PAY_VECTOR;
    if (!bench_gen_init(&b->gen, sf->cls, sf->rate, sf->channels, pay, a->frame, sf->sender))
        return false;
    b->gen.flags = sf->flags;
    b->gen.hops = sf->hops;
    bench_gen_seek(&b->gen, a->frame, a->rd);
    b->pos = a->start;
    b->synced = true;
    b->bad_run = 0;
    b->st.syncs++;
    return true;
}

// Bit errors of nbits expected bits (exp, MSB first; exp == NULL: the
// alternating idle pattern 1010...) against the ring from bit pos.  Each
// ring word is loaded once.
static unsigned compare(const bench_ring_t *r, uint64_t pos, const uint32_t *exp, unsigned nbits) {
    unsigned errs = 0, i = 0;
    uint32_t wi = (uint32_t)(pos >> 5), o = (uint32_t)pos & 31u, m = r->mask;
    uint32_t cur = r->ring[wi & m];
    for (; nbits; i++, wi++) {
        uint32_t got = cur;
        if (o) {
            uint32_t nxt = r->ring[(wi + 1u) & m];
            got = (cur << o) | (nxt >> (32u - o));
            cur = nxt;
        } else {
            cur = r->ring[(wi + 1u) & m];
        }
        uint32_t x = got ^ (exp ? exp[i] : 0xAAAAAAAAu);
        if (nbits < 32u) {
            x &= ~(0xFFFFFFFFu >> nbits);
            nbits = 0;
        } else {
            nbits -= 32u;
        }
        if (x) errs += popcount32(x);
    }
    return errs;
}

static unsigned compare_frame(const bench_ring_t *r, uint64_t pos, const bench_parts_t *p) {
    return compare(r, pos, p->head, 40u) + compare(r, pos + 40u, p->pay, p->pay_nbits) +
           compare(r, pos + 40u + p->pay_nbits, NULL, p->idle_nbits);
}

unsigned bench_bert_run(bench_bert_t *b, const bench_ring_t *r, uint64_t oldest, uint64_t newest,
                        unsigned max_frames) {
    static const int8_t tries[2 * BENCH_BERT_SLIP_MAX] = { -1, 1, -2, 2, -3, 3 };
    unsigned done = 0;
    while (b->synced && done < max_frames) {
        unsigned nb = bench_gen_frame_bits(&b->gen);
        // Room for a slip search on either side.
        if (b->pos + nb + 32u + BENCH_BERT_SLIP_MAX > newest) break;
        bench_parts_t p;
        if (b->pos < oldest + BENCH_BERT_SLIP_MAX) {
            bench_gen_parts(&b->gen, &p);
            b->pos += nb;
            b->st.skipped_frames++;
            done++;
            continue;
        }
        bench_gen_parts(&b->gen, &p);
        unsigned errs = compare_frame(r, b->pos, &p);
        if (errs > nb / 8u) {
            for (unsigned i = 0; i < sizeof(tries); i++) {
                unsigned e2 = compare_frame(r, b->pos + (uint64_t)(int64_t)tries[i], &p);
                if (e2 <= 4u) {
                    b->pos += (uint64_t)(int64_t)tries[i];
                    b->st.slips++;
                    b->st.slip_bits += tries[i];
                    errs = e2;
                    break;
                }
            }
        }
        b->st.frames++;
        b->st.bits += nb;
        b->st.errors += errs;
        if (errs) b->st.err_frames++;
        b->pos += nb;
        done++;
        if (errs > nb / 8u) {
            b->st.bad_frames++;
            if (++b->bad_run >= BENCH_BERT_BAD_LIMIT) {
                b->synced = false;
                b->st.losses++;
            }
        } else {
            b->bad_run = 0;
        }
    }
    return done;
}

// ---------------------------------------------------------------------------
// Reference receiver runner
// ---------------------------------------------------------------------------

void bench_stats_add(link_rx_stats_t *acc, const link_rx_stats_t *s) {
    acc->lock_count += s->lock_count;
    acc->loss_count += s->loss_count;
    acc->crc8_errors += s->crc8_errors;
    acc->crc16_errors += s->crc16_errors;
    acc->bit_slips += s->bit_slips;
    acc->superframe_errors += s->superframe_errors;
    acc->comma_errors += s->comma_errors;
    acc->counter_errors += s->counter_errors;
    acc->code_errors += s->code_errors;
    acc->idle_errors += s->idle_errors;
    acc->frames += s->frames;
}

static void ref_cb(void *ctx, const link_rx_event_t *ev) {
    bench_ref_t *r = ctx;
    switch (ev->type) {
    case LINK_RX_EV_LOCK: r->events_lock++; break;
    case LINK_RX_EV_LOSS: r->events_loss++; break;
    case LINK_RX_EV_FRAME: {
        r->last_frame = ev->frame;
        r->comma_offset = (uint8_t)((r->base + r->rx.p - 10u) & 31u);
        // The bench payload is the same in every frame: check decoded banks.
        bool prbs = (r->rx.sf.flags & LINK_SF_FLAG_PRBS) != 0;
        for (unsigned b = 0; b < link_bank_count(ev->channels); b++) {
            if (ev->bank_status[b] != LINK_BANK_OK) continue;
            unsigned k = link_bank_channels(ev->channels, b), c0 = b * LINK_BANK_CHANNELS;
            for (unsigned i = 0; i < k; i++) {
                int32_t want = prbs ? r->expect[c0 + i] : bench_vector_sample(c0 + i);
                if (ev->samples[c0 + i] != want) { r->sample_errors++; break; }
            }
        }
        break;
    }
    default: break;
    }
}

void bench_ref_init(bench_ref_t *r, unsigned cls, unsigned pay) {
    memset(r, 0, sizeof(*r));
    r->cls = (uint8_t)cls;
    r->pay = (uint8_t)pay;
    uint8_t bytes[3u * LINK_MAX_CHANNELS];
    link_prbs7_fill(bytes, sizeof(bytes));
    // PRBS-7 restarts at every frame and runs across the banks (spec 5.8).
    for (unsigned ch = 0; ch < LINK_MAX_CHANNELS; ch++) r->expect[ch] = link_get_s24(bytes + 3u * ch);
    link_rx_init(&r->rx, cls, 0xFF, ref_cb, r);
}

size_t bench_ref_run(bench_ref_t *r, const bench_ring_t *ring, uint64_t oldest, uint64_t newest,
                     size_t max_bits) {
    newest &= ~(uint64_t)31u;
    if (r->pos < oldest || r->restart) {
        r->restart = false;
        // A ring behind: restart a fresh receiver halfway between the oldest
        // and the newest data, so it has half a ring of slack before the
        // writer can overtake it again.
        if (r->bits) r->gaps++;
        bench_stats_add(&r->acc, &r->rx.stats);
        link_rx_init(&r->rx, r->cls, 0xFF, ref_cb, r);
        r->pos = (oldest + (newest - oldest) / 2u) & ~(uint64_t)31u;
        r->base = r->pos;
    }
    size_t pushed = 0;
    while (r->pos < newest && pushed < max_bits) {
        uint32_t wi = (uint32_t)(r->pos >> 5) & ring->mask;
        uint32_t n = ring->mask + 1u - wi;          // words to the ring's end
        uint64_t avail = (newest - r->pos) >> 5;
        if (n > avail) n = (uint32_t)avail;
        if ((size_t)n * 32u > max_bits - pushed) n = (uint32_t)((max_bits - pushed) / 32u);
        if (n > 128u) n = 128u;
        if (!n) break;
        link_rx_push(&r->rx, (const uint32_t *)&ring->ring[wi], (size_t)n * 32u);
        r->pos += (uint64_t)n * 32u;
        r->bits += (uint64_t)n * 32u;
        pushed += (size_t)n * 32u;
    }
    return pushed;
}

void bench_ref_stats(const bench_ref_t *r, link_rx_stats_t *out) {
    *out = r->acc;
    bench_stats_add(out, &r->rx.stats);
}
