/*
 * test_bench_model.c - the link bench logic driven through LK2's PIO model
 *
 * tools/link_rx_test/link_pio_model.c (LK2, unchanged) runs the assembled
 * link_tx_N / link_rx_N programs cycle by cycle with a clock offset, pulse-
 * width distortion and jitter on the line.  This test feeds it the bench
 * generator's stream and gives what the RX program emits to the bench's
 * acquisition, bit error rate tester and reference runner, i.e. the
 * firmware's data path with the PIO and wire replaced by the model:
 *
 *  - every class locks and checks clean at -1000..+1000 ppm;
 *  - a receiver of the wrong class never locks;
 *  - bench_trim_patch() produces the model's apply_trim() words;
 *  - the trim scan finds a range centred on 0 for a clean line and moves
 *    with the pulse-width distortion (what step (a) and (b) read);
 *  - TX impairments survive the PIO path with the expected counters.
 *
 * Built by run.sh with the pioasm headers on the include path.
 */

#define main lk2_model_main
#include "link_pio_model.c"
#undef main

#include "bench_check.h"
#include "bench_cmd.h"
#include "bench_gen.h"
#include "bench_trim.h"

static int failures, checks;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

static unsigned cls_n(unsigned cls) { return cls == 1 ? 20u : cls == 2 ? 10u : 5u; }

typedef struct {
    unsigned cls, rate, channels, frames;
    double ppm, pwd, jit, phase;
    int trim, jmodel;
    unsigned rx_cls;            // RX program class (0: same as cls)
    unsigned imp_kind; int imp_n; unsigned imp_at;
} McCase;

typedef struct {
    bool acquired;
    bench_bert_stats_t bert;
    bool synced_end;
    link_rx_stats_t ref;
    uint32_t sample_errors;
    uint64_t rx_bits;
} McResult;

static link_rx_t mc_scratch;

static void mc_run(const McCase *c, McResult *res) {
    memset(res, 0, sizeof(*res));
    // Bench stream -> the model's Stream.
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
    st.n = (size_t)(bench_sink_bits(&s) & ~(uint64_t)31u);   // whole words: the TX pulls words
    st.nw = st.n / 32u;
    st.words = w;
    st.bits = malloc(st.n);
    for (size_t i = 0; i < st.n; i++) st.bits[i] = (uint8_t)((w[i >> 5] >> (31u - (i & 31u))) & 1u);

    const Prog *tp = prog_for(cls_n(c->cls));
    const Prog *rp = prog_for(cls_n(c->rx_cls ? c->rx_cls : c->cls));
    Edges e;
    tx_run(tp, &st, c->ppm, &e);
    channel(&e, c->pwd, c->jit, c->jmodel);
    Rx r;
    memset(&r, 0, sizeof(r));
    r.cap = st.n + 4096;
    r.bits = malloc(r.cap);
    rx_run(tp, rp, c->trim, &e, c->phase, &r);
    res->rx_bits = r.n;

    // What the RX program pushed, packed as the DMA ring holds it.
    uint32_t nw = (uint32_t)(r.n / 32u);
    uint32_t ring_words = 1;
    while (ring_words < nw + 1u) ring_words <<= 1;
    uint32_t *ring = calloc(ring_words, 4);
    for (size_t i = 0; i < (size_t)nw * 32u; i++)
        if (r.bits[i]) ring[i >> 5] |= 0x80000000u >> (i & 31u);
    bench_ring_t rv = { .ring = ring, .mask = ring_words - 1u };
    uint64_t newest = (uint64_t)nw * 32u;
    unsigned rcls = c->rx_cls ? c->rx_cls : c->cls;

    bench_bert_t b;
    bench_bert_init(&b);
    bench_ref_t ref;
    bench_ref_init(&ref, rcls, BENCH_PAY_PRBS);
    uint64_t from = 0;
    for (int attempt = 0; attempt < 8 && from + bench_acquire_bits(rcls) <= newest; attempt++) {
        bench_acq_t a;
        uint64_t w0 = from / 32u;
        if (!bench_acquire(ring + w0, w0 * 32u, bench_acquire_bits(rcls), rcls, &mc_scratch, &a)) {
            from += bench_acquire_bits(rcls) / 2u;   // the firmware retries on newer data
            continue;
        }
        res->acquired = true;
        bench_bert_start(&b, &a);
        bench_bert_run(&b, &rv, 0, newest, 1000000u);
        if (b.synced) break;
        from = b.pos;       // lost sync: re-acquire after the damage, as the firmware does
    }
    bench_ref_run(&ref, &rv, 0, newest, (size_t)1 << 30);
    res->bert = b.st;
    res->synced_end = b.synced;
    bench_ref_stats(&ref, &res->ref);
    res->sample_errors = ref.sample_errors;

    free(ring); free(r.bits); free(e.t); free(e.lvl); free(st.bits); free(w);
}

static bool mc_clean(const McResult *r) {
    return r->acquired && r->synced_end && r->bert.errors == 0 && r->bert.slips == 0 && r->bert.losses == 0 &&
           r->bert.frames > 0 && r->ref.crc8_errors == 0 && r->ref.crc16_errors == 0 && r->ref.bit_slips == 0 &&
           r->ref.loss_count == 0 && r->ref.comma_errors == 0;
}

static void test_clean_classes(void) {
    static const double ppms[] = { -1000, -300, 0, 300, 1000 };
    for (unsigned cls = 1; cls <= 3; cls++)
        for (unsigned pi = 0; pi < 5; pi++) {
            McCase c = { .cls = cls, .rate = LINK_RATE_48K, .channels = 8, .frames = 200, .ppm = ppms[pi],
                         .phase = 0.37 * pi };
            McResult r;
            mc_run(&c, &r);
            CHECK(mc_clean(&r), "%s %+.0f ppm: acq %d bert err %llu slips %u losses %u frames %llu ref crc16 %u loss %u",
                  bench_class_name(cls), ppms[pi], r.acquired, (unsigned long long)r.bert.errors, r.bert.slips,
                  r.bert.losses, (unsigned long long)r.bert.frames, r.ref.crc16_errors, r.ref.loss_count);
            CHECK(r.bert.frames + 70u >= 200u, "%s: frames checked %llu", bench_class_name(cls),
                  (unsigned long long)r.bert.frames);
        }
    // 44.1 kHz walks the comma through the words; 96 kHz halves the frame.
    for (unsigned cls = 1; cls <= 3; cls++)
        for (unsigned rate = 0; rate < 3; rate += 2) {
            McCase c = { .cls = cls, .rate = rate, .channels = bench_default_channels(cls, rate), .frames = 200,
                         .ppm = 300 };
            McResult r;
            mc_run(&c, &r);
            CHECK(mc_clean(&r), "%s %s clean", bench_class_name(cls), bench_rate_name(rate));
        }
    printf("clean: 3 classes x 5 clock offsets, and 44.1/96 kHz\n");
}

static void test_wrong_class(void) {
    for (unsigned cls = 1; cls <= 3; cls++)
        for (unsigned rx = 1; rx <= 3; rx++) {
            if (rx == cls) continue;
            McCase c = { .cls = cls, .rate = LINK_RATE_48K, .channels = 8, .frames = 120, .rx_cls = rx };
            McResult r;
            mc_run(&c, &r);
            CHECK(!r.acquired && r.ref.lock_count == 0, "TX %s into RX %s must not lock", bench_class_name(cls),
                  bench_class_name(rx));
        }
}

static void test_trim_patch_matches_model(void) {
    unsigned n = 0;
    for (unsigned cls = 1; cls <= 3; cls++) {
        const Prog *p = prog_for(cls_n(cls));
        int tmax = bench_trim_max(p->rx, p->rx_len, (unsigned)p->rx_hi_entry, (unsigned)p->rx_lo_emit);
        CHECK(tmax == (cls == 1 ? 9 : cls == 2 ? 4 : 1), "%s trim range +-%d", bench_class_name(cls), tmax);
        for (int t = -tmax; t <= tmax; t++) {
            Sm s;
            sm_load(&s, p->rx, p->rx_len, p->rx_wt, p->rx_wr);
            CHECK(apply_trim(&s, p, t), "model trim");
            uint16_t ins[32];
            memcpy(ins, p->rx, p->rx_len * sizeof(uint16_t));
            CHECK(bench_trim_patch(ins, p->rx_len, (unsigned)p->rx_hi_entry, (unsigned)p->rx_lo_emit, t), "bench trim");
            CHECK(!memcmp(ins, s.prog, p->rx_len * sizeof(uint16_t)), "%s trim %d words differ", bench_class_name(cls), t);
            n++;
        }
    }
    printf("trim patch == model apply_trim for %u (class, trim) pairs\n", n);
}

// One scan as the firmware runs it: each trim, a fresh receive, clean or not.
static void scan(unsigned cls, double pwd, double jit, double ppm, bench_scan_t *sc) {
    const Prog *p = prog_for(cls_n(cls));
    bench_scan_begin(sc, bench_trim_max(p->rx, p->rx_len, (unsigned)p->rx_hi_entry, (unsigned)p->rx_lo_emit));
    int t;
    while (bench_scan_next(sc, &t)) {
        McCase c = { .cls = cls, .rate = LINK_RATE_48K, .channels = 8, .frames = 140, .ppm = ppm, .pwd = pwd,
                     .jit = jit, .trim = t, .phase = 0.5 };
        McResult r;
        mc_run(&c, &r);
        bench_scan_record(sc, mc_clean(&r), (uint32_t)r.bert.errors);
    }
    bench_scan_finish(sc);
    printf("scan %s pwd %+5.1f ns jit %4.1f ns: state %u lo %+d hi %+d used %+d mask %05x\n", bench_class_name(cls),
           pwd, jit, sc->state, sc->lo, sc->hi, sc->used, sc->clean_mask);
}

static void test_trim_scan(void) {
    bench_scan_t sc;
    // A clean line (GPIO loopback): a range centred on 0 within one step.
    scan(1, 0, 0, 0, &sc);
    CHECK(sc.state == BENCH_SCAN_DONE && sc.lo <= -8 && sc.hi >= 8 && sc.used >= -1 && sc.used <= 1, "L1 clean scan");
    scan(2, 0, 0, 0, &sc);
    CHECK(sc.state == BENCH_SCAN_DONE && sc.lo <= -3 && sc.hi >= 3 && sc.used >= -1 && sc.used <= 1, "L2 clean scan");
    scan(3, 0, 0, 300, &sc);
    CHECK(sc.state == BENCH_SCAN_DONE && sc.lo <= 0 && sc.hi >= 0 && sc.used >= -1 && sc.used <= 1, "L3 clean scan");
    // A distorted line: trim 0 fails, the range and its centre move with
    // the sign of the PWD (the PLR237 case of LK2's report).
    scan(1, 15, 15, 0, &sc);
    CHECK(sc.state == BENCH_SCAN_DONE && !(sc.clean_mask & (1u << 9)) && sc.lo >= 1 && sc.used >= 3 && sc.used <= 6,
          "L1 +15 ns PWD + 15 ns jitter: positive trims");
    scan(1, -15, 15, 0, &sc);
    CHECK(sc.state == BENCH_SCAN_DONE && !(sc.clean_mask & (1u << 9)) && sc.hi <= -1 && sc.used <= -3 && sc.used >= -6,
          "L1 -15 ns PWD: negative trims");
    scan(2, 9, 6, 0, &sc);
    CHECK(sc.state == BENCH_SCAN_DONE && sc.used >= 1, "L2 +9 ns PWD + 6 ns jitter: trim moves up");
    // Beyond rescue: nothing clean.
    scan(3, 12, 4, 0, &sc);
    CHECK(sc.state == BENCH_SCAN_NOCLEAN, "L3 with 12 ns PWD: nothing clean");
}

static void test_impairments_through_pio(void) {
    struct { unsigned cls, kind; int n; uint32_t comma, slips, loss, lock, crc16; } cases[] = {
        { 1, BENCH_IMP_COMMA, 1, 1, 0, 0, 1, 0 },
        { 2, BENCH_IMP_COMMA, 3, 3, 0, 1, 2, 0 },
        { 1, BENCH_IMP_SLIP, 2, 0, 1, 0, 1, 0 },
        { 3, BENCH_IMP_SLIP, -1, 0, 1, 0, 1, 0 },
        { 2, BENCH_IMP_FLIP, 4, 0, 0, 0, 1, 4 },
        { 1, BENCH_IMP_IDLE, 30, 3, 0, 1, 2, 3 },
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        McCase c = { .cls = cases[i].cls, .rate = LINK_RATE_48K, .channels = 8, .frames = 400, .ppm = 300,
                     .imp_kind = cases[i].kind, .imp_n = cases[i].n, .imp_at = 160 };
        McResult r;
        mc_run(&c, &r);
        CHECK(r.ref.comma_errors == cases[i].comma && r.ref.bit_slips == cases[i].slips &&
              r.ref.loss_count == cases[i].loss && r.ref.lock_count == cases[i].lock &&
              r.ref.crc16_errors == cases[i].crc16,
              "case %u (%s kind %u n %d): ref comma %u slips %u loss %u lock %u crc16 %u", i,
              bench_class_name(cases[i].cls), cases[i].kind, cases[i].n, r.ref.comma_errors, r.ref.bit_slips,
              r.ref.loss_count, r.ref.lock_count, r.ref.crc16_errors);
        CHECK(r.synced_end && r.sample_errors == 0, "case %u: BERT back in sync, no wrong samples", i);
        if (cases[i].kind == BENCH_IMP_SLIP) CHECK(r.bert.slips == 1 && r.bert.slip_bits == cases[i].n, "BERT slip");
        if (cases[i].kind == BENCH_IMP_IDLE) CHECK(r.bert.losses == 1 && r.bert.syncs == 2, "BERT loss and re-sync");
    }
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    test_trim_patch_matches_model();
    test_clean_classes();
    test_wrong_class();
    test_impairments_through_pio();
    test_trim_scan();
    printf("test_bench_model: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
