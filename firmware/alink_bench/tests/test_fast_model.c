/*
 * test_fast_model.c - LK5's production receiver against LK1's reference on
 * the LK2 PIO model's output (WP LK5)
 *
 * The bench generator's streams (with its TX impairments) go through the
 * model's TX program, a wire with clock offset, pulse-width distortion and
 * jitter, and the RX program, as in test_bench_model.c.  Whatever bits come
 * out, damaged or not, alink_fast (DSPi/alink/alink_fast.c) must log exactly
 * what alink_decoder.c logs: every event, frame, flag, bank status, sample
 * and counter.  Built by run.sh like test_bench_model.c.
 */

#define main lk2_model_main
#include "alink_pio_model.c"
#undef main

#include "bench_cmd.h"
#include "bench_gen.h"

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static unsigned cls_n(unsigned cls) { return cls == 1 ? 20u : cls == 2 ? 10u : 5u; }

typedef struct {
    unsigned cls, rate, channels, frames;
    double ppm, pwd, jit, phase;
    int trim, jmodel;
    unsigned imp_kind; int imp_n; unsigned imp_at;
} McCase;

#include "alink_fast.h"
#include <inttypes.h>

typedef struct { char *s; size_t n, cap; uint64_t origin; } flog_t;

static void flog_add(flog_t *l, const char *t) {
    size_t k = strlen(t);
    if (l->n + k + 2 > l->cap) return;
    if (l->n) l->s[l->n++] = ' ';
    memcpy(l->s + l->n, t, k + 1);
    l->n += k;
}

static void flog_cb(void *ctx, const alink_rx_event_t *ev) {
    flog_t *l = ctx;
    char t[160];
    switch (ev->type) {
    case ALINK_RX_EV_ALIGN: snprintf(t, sizeof(t), "A%" PRIu64, ev->bitpos - l->origin); break;
    case ALINK_RX_EV_LOCK: snprintf(t, sizeof(t), "L%" PRIu64, ev->frame); break;
    case ALINK_RX_EV_SUPERFRAME: snprintf(t, sizeof(t), "S"); break;
    case ALINK_RX_EV_SF_ERROR: snprintf(t, sizeof(t), "E%d", ev->sf_error); break;
    case ALINK_RX_EV_LOSS: snprintf(t, sizeof(t), "X"); break;
    case ALINK_RX_EV_FRAME: {
        uint32_t h = 0x811C9DC5u;
        for (unsigned ch = 0; ch < ev->channels; ch++)
            for (int b = 0; b < 4; b++) h = (h ^ (((uint32_t)ev->samples[ch] >> (8 * b)) & 0xFFu)) * 0x01000193u;
        char banks[9];
        unsigned nb = alink_bank_count(ev->channels);
        for (unsigned b = 0; b < nb; b++) banks[b] = ".ohz"[ev->bank_status[b]];
        banks[nb] = 0;
        snprintf(t, sizeof(t), "F%" PRIu64 ":%x:%d:%s:%08" PRIx32, ev->frame, ev->flags, ev->slip, banks, h);
        break;
    }
    }
    flog_add(l, t);
}

static void flog_stats(flog_t *l, const alink_rx_stats_t *s) {
    char t[200];
    snprintf(t, sizeof(t), "#%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", s->lock_count, s->loss_count, s->crc8_errors,
             s->crc16_errors, s->bit_slips, s->superframe_errors, s->comma_errors, s->counter_errors,
             s->code_errors, s->idle_errors, s->frames);
    flog_add(l, t);
}

static alink_rx_t fm_ref;
static alink_fast_t fm_fast;
static uint32_t fm_ring[256];

// One model run; returns 1 when both receivers agree, and counts frames.
static int fm_case(const McCase *c, uint8_t mask, unsigned *frames_out) {
    bench_gen_t g;
    bench_gen_init(&g, c->cls, c->rate, c->channels, BENCH_PAY_PRBS, 100, 3);
    uint32_t cap = 1u << 17;
    uint32_t *w = calloc(cap, 4);
    bench_sink_t s;
    bench_sink_init(&s, w, cap);
    for (unsigned f = 0; f < c->frames; f++) {
        if (c->imp_kind && f == c->imp_at) bench_gen_inject(&g, c->imp_kind, c->imp_n);
        bench_gen_frame(&g, &s);
    }
    if (s.cur_n) w[s.words] = s.cur;
    Stream st;
    memset(&st, 0, sizeof(st));
    st.n = (size_t)(bench_sink_bits(&s) & ~(uint64_t)31u);
    st.nw = st.n / 32u;
    st.words = w;
    st.bits = malloc(st.n);
    for (size_t i = 0; i < st.n; i++) st.bits[i] = (uint8_t)((w[i >> 5] >> (31u - (i & 31u))) & 1u);
    Edges e;
    tx_run(prog_for(cls_n(c->cls)), &st, c->ppm, &e);
    channel(&e, c->pwd, c->jit, c->jmodel);
    Rx r;
    memset(&r, 0, sizeof(r));
    r.cap = st.n + 4096;
    r.bits = malloc(r.cap);
    rx_run(prog_for(cls_n(c->cls)), prog_for(cls_n(c->cls)), c->trim, &e, c->phase, &r);
    size_t nw = r.n / 32u;
    uint32_t *rw = calloc(nw + 1, 4);
    for (size_t i = 0; i < nw * 32u; i++)
        if (r.bits[i]) rw[i >> 5] |= 0x80000000u >> (i & 31u);

    static char a[1 << 20], b[1 << 20];
    flog_t la = { a, 0, sizeof(a), 0 }, lb = { b, 0, sizeof(b), 0 };
    a[0] = b[0] = 0;
    alink_rx_init(&fm_ref, c->cls, mask, flog_cb, &la);
    alink_rx_push(&fm_ref, rw, nw * 32u);
    flog_stats(&la, &fm_ref.stats);
    // The production receiver reads a 1 KB ring filled 37 words at a time.
    alink_fast_init(&fm_fast, fm_ring, 256, c->cls, mask, flog_cb, &lb, 0);
    fm_fast.frame_events = true;
    for (size_t i = 0; i < nw;) {
        size_t k = nw - i < 37 ? nw - i : 37;
        for (size_t j = 0; j < k; j++) fm_ring[(i + j) & 255u] = rw[i + j];
        i += k;
        alink_fast_run(&fm_fast, (uint64_t)i * 32u, 0);
    }
    flog_stats(&lb, &fm_fast.stats);
    int same = strcmp(a, b) == 0 && fm_fast.overruns == 0;
    if (!same) printf("  differ: cls %u ppm %.0f pwd %.1f jit %.1f imp %u\n   ref  %.300s\n   fast %.300s\n", c->cls,
                      c->ppm, c->pwd, c->jit, c->imp_kind, a, b);
    *frames_out = fm_fast.stats.frames;
    free(w); free(st.bits); free(r.bits); free(rw);
    return same;
}

int main(void) {
    unsigned runs = 0, frames = 0, damaged = 0;
    static const double ppms[] = { -1000, -300, 0, 300, 1000 };
    static const double pwds[] = { 0, 8, -12, 20 };
    static const double jits[] = { 0, 4, 9, 15 };
    for (unsigned cls = 1; cls <= 3; cls++)
        for (unsigned rate = 0; rate <= 2; rate++)
            for (unsigned k = 0; k < 4; k++) {
                // a clean line, then harder ones (the PWD/jitter values in ns are
                // scaled to the class's bit period: L1 x1, L2 x0.5, L3 x0.25)
                double sc = cls == 1 ? 1.0 : cls == 2 ? 0.5 : 0.25;
                McCase c = { .cls = cls, .rate = rate, .channels = bench_default_channels(cls, rate),
                             .frames = cls == 3 ? 120 : 160, .ppm = ppms[(cls + rate + k) % 5],
                             .pwd = pwds[k] * sc, .jit = jits[k] * sc };
                unsigned f;
                CHECK(fm_case(&c, 0xFF, &f), "model cls %u rate %u case %u", cls, rate, k);
                frames += f;
                runs++;
                if (k) damaged++;
                c.trim = (int)(k & 1u);   // L3 allows only -1..+1
                CHECK(fm_case(&c, 0x01, &f), "model cls %u rate %u case %u (bank 0, trim)", cls, rate, k);
                frames += f;
                runs++;
            }
    static const struct { unsigned cls, kind; int n; } imps[] = {
        { 1, BENCH_IMP_COMMA, 1 }, { 2, BENCH_IMP_COMMA, 3 }, { 1, BENCH_IMP_SLIP, 2 }, { 3, BENCH_IMP_SLIP, -1 },
        { 2, BENCH_IMP_FLIP, 4 }, { 1, BENCH_IMP_IDLE, 30 }, { 3, BENCH_IMP_SLIP, 3 }, { 2, BENCH_IMP_IDLE, 2 },
    };
    for (size_t i = 0; i < sizeof(imps) / sizeof(imps[0]); i++) {
        McCase c = { .cls = imps[i].cls, .rate = ALINK_RATE_48K, .channels = 8, .frames = 300, .ppm = 300,
                     .imp_kind = imps[i].kind, .imp_n = imps[i].n, .imp_at = 150 };
        unsigned f;
        CHECK(fm_case(&c, 0xFF, &f), "impairment %zu", i);
        frames += f;
        runs++;
        damaged++;
    }
    printf("test_fast_model: %u model runs (%u impaired or distorted), %u frames, receivers identical: %s\n", runs,
           damaged, frames, failures ? "NO" : "yes");
    return failures ? 1 : 0;
}
