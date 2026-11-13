/*
 * test_bench_logic.c - host tests for the link bench logic (WP LK3)
 *
 * - bench_gen produces exactly the bits of LK1's link_tx_frame() at every
 *   class, rate and channel count, PRBS and test vector, from any start;
 * - the sink packs like link_bitw_put();
 * - acquisition finds F, the comma position and the disparity from any bit
 *   offset; the bit error rate tester checks every bit of a clean stream
 *   and accounts for every TX impairment exactly; the reference runner's
 *   counters match the spec's rules for the same impairments;
 * - a small ring with a slow reader: skips and gaps are counted, not hidden;
 * - trim patching, the scan's range logic, command parsing and the
 *   EXT_LINK_STATUS / EXT_LINK_DIAG layouts.
 *
 * cc -std=c11 -I firmware/DSPi/link -I firmware/link_bench test_bench_logic.c <sources>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bench_check.h"
#include "bench_cmd.h"
#include "bench_gen.h"
#include "bench_trim.h"

static int failures, checks;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Generate frames into a linear buffer (ring large enough not to wrap).
typedef struct {
    uint32_t *words;
    uint32_t cap;           // power of two
    bench_sink_t s;
} Buf;

static void buf_init(Buf *b, uint32_t cap_words) {
    b->cap = cap_words;
    b->words = calloc(cap_words, 4);
    bench_sink_init(&b->s, b->words, cap_words);
}

static void buf_flush(Buf *b) {
    if (b->s.cur_n) b->words[(uint32_t)b->s.words & b->s.mask] = b->s.cur;
}

// ---------------------------------------------------------------------------
// Generator equals the reference encoder
// ---------------------------------------------------------------------------

static void test_gen_equals_reference(void) {
    static const unsigned classes[] = { LINK_CLASS_L1, LINK_CLASS_L2, LINK_CLASS_L3 };
    static const unsigned rates[] = { LINK_RATE_44K1, LINK_RATE_48K, LINK_RATE_96K };
    unsigned combos = 0;
    for (unsigned ci = 0; ci < 3; ci++)
        for (unsigned ri = 0; ri < 3; ri++) {
            unsigned cls = classes[ci], rate = rates[ri], max = link_max_channels(cls, rate);
            unsigned chans[4] = { 1, bench_default_channels(cls, rate), max, max > 9 ? 9 : max };
            for (unsigned k = 0; k < 4; k++)
                for (unsigned pay = 0; pay < 2; pay++) {
                    unsigned c = chans[k];
                    uint64_t first = 1000u + 13u * k + pay;   // starts inside a superframe
                    link_tx_t tx;
                    link_tx_init(&tx, cls, rate, c, first, 0x5Au);
                    tx.flags = pay == BENCH_PAY_PRBS ? LINK_SF_FLAG_PRBS : 0;
                    bench_gen_t g;
                    CHECK(bench_gen_init(&g, cls, rate, c, pay, first, 0x5Au), "init %u %u %u", cls, rate, c);
                    static uint32_t ref[16384], got[16384];
                    link_bitw_t w;
                    link_bitw_init(&w, ref, 16384);
                    Buf b = { .words = got, .cap = 16384 };
                    bench_sink_init(&b.s, got, 16384);
                    int32_t samples[LINK_MAX_CHANNELS];
                    for (unsigned ch = 0; ch < c; ch++) samples[ch] = bench_vector_sample(ch);
                    uint64_t nbits = 0;
                    for (unsigned f = 0; f < 300; f++) {   // covers the 147-frame 44.1 kHz cycle twice
                        unsigned bits_hint = bench_gen_frame_bits(&g);
                        if (!link_tx_frame(&tx, samples, &w)) break;
                        unsigned n = bench_gen_frame(&g, &b.s);
                        CHECK(n == bits_hint, "frame bits");
                        nbits += n;
                    }
                    buf_flush(&b);
                    CHECK(w.nbits == nbits, "length %u %u %u: ref %zu gen %llu", cls, rate, c, w.nbits,
                          (unsigned long long)nbits);
                    size_t full = (size_t)(nbits / 32u);
                    CHECK(!memcmp(ref, got, full * 4u), "bits differ cls %u rate %u C %u pay %u", cls, rate, c, pay);
                    if (nbits & 31u) {
                        uint32_t m = ~(0xFFFFFFFFu >> (nbits & 31u));
                        CHECK(((ref[full] ^ got[full]) & m) == 0, "tail differs");
                    }
                    combos++;
                }
        }
    printf("gen == link_tx_frame: %u streams of 300 frames\n", combos);
}

static void test_sink(void) {
    uint32_t a[64], ring[16];
    link_bitw_t w;
    link_bitw_init(&w, a, 64);
    bench_sink_t s;
    bench_sink_init(&s, ring, 16);
    uint32_t x = 12345;
    uint64_t total = 0;
    bool ok = true;
    for (int i = 0; i < 200 && total < 64 * 32 - 40; i++) {
        x = x * 1103515245u + 12345u;
        unsigned n = (x >> 8) % 33u;
        uint32_t v = x ^ (x << 7);
        link_bitw_put(&w, n < 32 ? v & ((1u << n) - 1u) : v, n);
        bench_sink_put(&s, v, n);
        total += n;
        // Compare every completed word still in the ring.
        uint64_t first = s.words > 16 ? s.words - 16 : 0;
        for (uint64_t k = first; k < s.words; k++)
            if (ring[k & 15u] != a[k]) ok = false;
    }
    CHECK(ok, "sink words differ from link_bitw_put");
    CHECK(bench_sink_bits(&s) == total && w.nbits == total, "sink bit count");
}

// ---------------------------------------------------------------------------
// Acquisition, BERT, reference runner on generated streams
// ---------------------------------------------------------------------------

typedef struct {
    Buf b;
    bench_gen_t g;
    uint64_t frame_start[4096];   // absolute bit of each generated frame (index = F - first)
    uint64_t first;
    unsigned n;
} Stream;

static void stream_make(Stream *st, unsigned cls, unsigned rate, unsigned c, unsigned pay, uint64_t first,
                        unsigned lead_bits, unsigned frames) {
    buf_init(&st->b, 1u << 18);
    // Lead-in junk of an odd length: the receiver starts mid-symbol.
    uint32_t x = 99;
    for (unsigned i = 0; i < lead_bits; i++) { x = x * 1103515245u + 12345u; bench_sink_put(&st->b.s, x >> 31, 1); }
    bench_gen_init(&st->g, cls, rate, c, pay, first, 7);
    st->first = first;
    st->n = 0;
    for (unsigned f = 0; f < frames; f++) {
        st->frame_start[st->n++] = bench_sink_bits(&st->b.s);
        bench_gen_frame(&st->g, &st->b.s);
    }
    buf_flush(&st->b);
}

static void stream_more(Stream *st, unsigned frames) {
    for (unsigned f = 0; f < frames; f++) {
        st->frame_start[st->n++] = bench_sink_bits(&st->b.s);
        bench_gen_frame(&st->g, &st->b.s);
    }
    buf_flush(&st->b);
}

static link_rx_t scratch_rx;

static void test_acquire(void) {
    for (unsigned cls = 1; cls <= 3; cls++)
        for (unsigned rate = 0; rate < 3; rate++)
            for (unsigned lead = 0; lead < 40; lead += 13) {
                Stream st;
                unsigned c = bench_default_channels(cls, rate);
                stream_make(&st, cls, rate, c, BENCH_PAY_PRBS, 5000u + lead, lead, 120);
                bench_acq_t a;
                uint32_t need = bench_acquire_bits(cls);
                bool ok = bench_acquire(st.b.words, 0, need, cls, &scratch_rx, &a);
                CHECK(ok, "acquire cls %u rate %u lead %u", cls, rate, lead);
                if (ok) {
                    uint64_t idx = a.frame - st.first;
                    CHECK(idx < st.n && st.frame_start[idx] == a.start, "acquired position F %llu",
                          (unsigned long long)a.frame);
                    CHECK(a.sf.cls == cls && a.sf.rate == rate && a.sf.channels == c && a.sf.sender == 7,
                          "acquired superframe");
                    CHECK(a.frame - st.first <= 64u + 1u, "lock within 64 frames (got %llu)",
                          (unsigned long long)(a.frame - st.first));
                }
                // A wrong-class receiver never locks.
                unsigned other = cls == 1 ? 2 : 1;
                CHECK(!bench_acquire(st.b.words, 0, need, other, &scratch_rx, &a), "wrong class locked");
                free(st.b.words);
            }
}

// BERT + reference runner over a stream, in chunks as a DMA would deliver it.
typedef struct {
    bench_bert_t bert;
    bench_ref_t ref;
    bench_acq_t acq;
    bool acquired;
} Checker;

static void run_checkers(Checker *k, Stream *st, uint64_t upto_bits, unsigned cls) {
    bench_ring_t r = { .ring = st->b.words, .mask = st->b.cap - 1u };
    if (!k->acquired) {
        k->acquired = bench_acquire(st->b.words, 0, bench_acquire_bits(cls), cls, &scratch_rx, &k->acq);
        if (k->acquired) bench_bert_start(&k->bert, &k->acq);
    }
    for (uint64_t at = 4096; ; at += 4096) {
        uint64_t newest = at < upto_bits ? at : upto_bits;
        bench_bert_run(&k->bert, &r, 0, newest, 1000000u);
        bench_ref_run(&k->ref, &r, 0, newest, 1u << 30);
        if (newest == upto_bits) break;
    }
}

static void test_bert_clean(void) {
    for (unsigned cls = 1; cls <= 3; cls++)
        for (unsigned rate = 0; rate < 3; rate++)
            for (unsigned pay = 0; pay < 2; pay++) {
                Stream st;
                unsigned c = bench_default_channels(cls, rate);
                stream_make(&st, cls, rate, c, pay, 31u, 5, 600);
                Checker k;
                memset(&k, 0, sizeof(k));
                bench_bert_init(&k.bert);
                bench_ref_init(&k.ref, cls, pay);
                run_checkers(&k, &st, bench_sink_bits(&st.b.s), cls);
                link_rx_stats_t s;
                bench_ref_stats(&k.ref, &s);
                uint64_t gen_bits = bench_sink_bits(&st.b.s) - st.frame_start[0];
                CHECK(k.acquired, "acquired");
                CHECK(k.bert.synced && k.bert.st.errors == 0 && k.bert.st.slips == 0 && k.bert.st.losses == 0,
                      "clean BERT cls %u rate %u pay %u: errors %llu", cls, rate, pay,
                      (unsigned long long)k.bert.st.errors);
                // Every bit after the acquired frame, less the last frame or so, was compared.
                uint64_t after = bench_sink_bits(&st.b.s) - k.acq.start;
                CHECK(k.bert.st.bits + 2u * 1400u + 64u >= after && k.bert.st.bits <= after, "coverage %llu of %llu",
                      (unsigned long long)k.bert.st.bits, (unsigned long long)after);
                CHECK(s.lock_count == 1 && s.loss_count == 0 && s.crc8_errors == 0 && s.crc16_errors == 0 &&
                      s.bit_slips == 0 && s.superframe_errors == 0 && s.comma_errors == 0 &&
                      s.counter_errors == 0 && s.code_errors == 0 && s.idle_errors == 0,
                      "clean reference counters");
                CHECK(k.ref.sample_errors == 0, "samples match the bench payload (pay %u)", pay);
                CHECK(s.frames >= 600u - 70u, "reference frames %u", s.frames);
                (void)gen_bits;
                free(st.b.words);
            }
}

// Each impairment on a locked L1 48 kHz stream: what the BERT and the
// reference receiver must report.
static void test_impairments(void) {
    struct {
        unsigned kind; int n;
        // BERT expectations
        uint32_t slips; int32_t slip_bits; uint32_t losses; uint64_t errors_min, errors_max;
        // reference expectations
        uint32_t comma_errors, crc16_errors, bit_slips, loss_count, lock_count;
    } cases[] = {
        { BENCH_IMP_COMMA, 1, 0, 0, 0, 1, 1,   1, 0, 0, 0, 1 },
        { BENCH_IMP_COMMA, 2, 0, 0, 0, 2, 2,   2, 0, 0, 0, 1 },
        { BENCH_IMP_COMMA, 3, 0, 0, 0, 3, 3,   3, 0, 0, 1, 2 },
        { BENCH_IMP_FLIP, 5, 0, 0, 0, 5, 5,    0, 5, 0, 0, 1 },
        { BENCH_IMP_SLIP, 1, 1, 1, 0, 0, 0,    0, 0, 1, 0, 1 },
        { BENCH_IMP_SLIP, -1, 1, -1, 0, 0, 1,  0, 0, 1, 0, 1 },
        { BENCH_IMP_SLIP, 2, 1, 2, 0, 0, 0,    0, 0, 1, 0, 1 },
        { BENCH_IMP_SLIP, -2, 1, -2, 0, 0, 2,  0, 0, 1, 0, 1 },
        { BENCH_IMP_SLIP, 3, 1, 3, 0, 0, 0,    3, 3, 0, 1, 2 },
        { BENCH_IMP_IDLE, 2, 0, 0, 0, 100, 640, 2, 2, 0, 0, 1 },
        { BENCH_IMP_IDLE, 20, 0, 0, 1, 1000, 100000, 3, 3, 0, 1, 2 },
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        unsigned cls = LINK_CLASS_L1, rate = LINK_RATE_48K;
        Stream st;
        stream_make(&st, cls, rate, 8, BENCH_PAY_PRBS, 64u, 3, 200);
        CHECK(bench_gen_inject(&st.g, cases[i].kind, cases[i].n), "inject");
        stream_more(&st, 300);
        Checker k;
        memset(&k, 0, sizeof(k));
        bench_bert_init(&k.bert);
        bench_ref_init(&k.ref, cls, BENCH_PAY_PRBS);
        run_checkers(&k, &st, bench_sink_bits(&st.b.s), cls);
        // A BERT that lost sync re-acquires from the data after the damage.
        if (!k.bert.synced) {
            uint64_t from = k.bert.pos;
            bench_acq_t a;
            uint64_t w0 = from / 32u;
            bool ok = bench_acquire(st.b.words + w0, w0 * 32u, bench_acquire_bits(cls), cls, &scratch_rx, &a);
            CHECK(ok, "re-acquire after loss");
            if (ok) {
                bench_bert_start(&k.bert, &a);
                bench_ring_t r = { .ring = st.b.words, .mask = st.b.cap - 1u };
                bench_bert_run(&k.bert, &r, 0, bench_sink_bits(&st.b.s), 1000000u);
            }
        }
        link_rx_stats_t s;
        bench_ref_stats(&k.ref, &s);
        const char *name = cases[i].kind == BENCH_IMP_COMMA ? "comma" : cases[i].kind == BENCH_IMP_FLIP ? "flip"
                         : cases[i].kind == BENCH_IMP_SLIP ? "slip" : "idle";
        CHECK(k.bert.st.slips == cases[i].slips && k.bert.st.slip_bits == cases[i].slip_bits &&
              k.bert.st.losses == cases[i].losses,
              "%s %d: BERT slips %u (%d bits) losses %u", name, cases[i].n, k.bert.st.slips, k.bert.st.slip_bits,
              k.bert.st.losses);
        CHECK(k.bert.st.errors >= cases[i].errors_min && k.bert.st.errors <= cases[i].errors_max,
              "%s %d: BERT errors %llu", name, cases[i].n, (unsigned long long)k.bert.st.errors);
        CHECK(k.bert.synced, "%s %d: BERT synced at the end", name, cases[i].n);
        CHECK(s.comma_errors == cases[i].comma_errors && s.crc16_errors == cases[i].crc16_errors &&
              s.bit_slips == cases[i].bit_slips && s.loss_count == cases[i].loss_count &&
              s.lock_count == cases[i].lock_count,
              "%s %d: reference comma %u crc16 %u slips %u loss %u lock %u", name, cases[i].n, s.comma_errors,
              s.crc16_errors, s.bit_slips, s.loss_count, s.lock_count);
        CHECK(k.ref.sample_errors == 0, "%s: no wrong samples delivered", name);
        free(st.b.words);
    }
}

// A ring of 4096 words with readers that cannot keep up: frames that left
// the ring are skipped and counted, the reference runner restarts and
// counts gaps, and what was checked is still clean.
static void test_small_ring(void) {
    unsigned cls = LINK_CLASS_L2, rate = LINK_RATE_48K;
    Stream st;
    stream_make(&st, cls, rate, 16, BENCH_PAY_PRBS, 0, 0, 3000);
    const uint32_t ring_words = 4096;
    uint32_t *ring = calloc(ring_words, 4);
    bench_ring_t r = { .ring = ring, .mask = ring_words - 1u };
    uint64_t total_words = bench_sink_bits(&st.b.s) / 32u;
    Checker k;
    memset(&k, 0, sizeof(k));
    bench_bert_init(&k.bert);
    bench_ref_init(&k.ref, cls, BENCH_PAY_PRBS);
    uint64_t written = 0;
    unsigned round = 0;
    while (written < total_words) {
        // The "DMA" writes 200 words per round.
        for (unsigned i = 0; i < 200 && written < total_words; i++, written++)
            ring[written & (ring_words - 1u)] = st.b.words[written];
        uint64_t newest = written * 32u, oldest = written > ring_words ? (written - ring_words + 8u) * 32u : 0;
        if (!k.bert.synced && newest >= bench_acquire_bits(cls) + oldest) {
            // Snapshot the newest data, as the firmware does.
            uint32_t need = bench_acquire_bits(cls);
            uint64_t w0 = (newest - need) / 32u;
            static uint32_t snap[8192];
            uint32_t nw = (uint32_t)(newest / 32u - w0);
            for (uint32_t i = 0; i < nw; i++) snap[i] = ring[(w0 + i) & (ring_words - 1u)];
            bench_acq_t a;
            if (bench_acquire(snap, w0 * 32u, (size_t)nw * 32u, cls, &scratch_rx, &a)) bench_bert_start(&k.bert, &a);
        }
        // The checkers get a slice of CPU each round: the BERT stalls for
        // 30 rounds (more than a ring) and catches up, the reference gets
        // less than the line rate throughout.
        bench_bert_run(&k.bert, &r, oldest, newest, round >= 50 && round < 80 ? 0u : 40u);
        bench_ref_run(&k.ref, &r, oldest, newest, 3000);
        round++;
    }
    link_rx_stats_t s;
    bench_ref_stats(&k.ref, &s);
    CHECK(k.bert.st.syncs >= 1 && k.bert.st.errors == 0 && k.bert.st.losses == 0, "small ring BERT clean");
    CHECK(k.bert.st.skipped_frames > 0, "small ring: skipped frames counted (%llu)",
          (unsigned long long)k.bert.st.skipped_frames);
    CHECK(k.ref.gaps > 0, "small ring: reference gaps counted (%u)", k.ref.gaps);
    CHECK(s.crc8_errors == 0 && s.crc16_errors == 0 && s.loss_count == 0 && s.bit_slips == 0,
          "small ring: reference counters clean across gaps");
    CHECK(s.lock_count >= 1 && s.lock_count <= k.ref.gaps + 1u, "small ring: one lock per instance");
    printf("small ring: BERT %llu bits, %llu frames skipped; reference %u gaps, %llu bits\n",
           (unsigned long long)k.bert.st.bits, (unsigned long long)k.bert.st.skipped_frames, k.ref.gaps,
           (unsigned long long)k.ref.bits);
    free(ring);
    free(st.b.words);
}

// ---------------------------------------------------------------------------
// Trim, scan, parsing, layouts
// ---------------------------------------------------------------------------

static void test_trim_patch(void) {
    // A link_rx_20-shaped program: hi_entry at 6 with delay 9, three
    // `jmp lo_emit [9]` (lo_emit at 3), other jumps untouched.
    uint16_t prog[16] = {0};
    for (int i = 0; i < 16; i++) prog[i] = (uint16_t)(0xA042);   // nop
    prog[6] = (uint16_t)(0x0000 | (9u << 8) | 13u);   // jmp hi_emit [9]
    prog[10] = (uint16_t)(0x0000 | (9u << 8) | 3u);   // jmp lo_emit [9]
    prog[14] = (uint16_t)(0x0000 | (9u << 8) | 3u);
    prog[15] = (uint16_t)(0x0000 | (9u << 8) | 3u);
    prog[2] = (uint16_t)(0x00C0 | 6u);                // jmp pin hi_entry (condition: not patched)
    uint16_t p[16];
    memcpy(p, prog, sizeof(p));
    CHECK(bench_trim_patch(p, 16, 6, 3, 4), "trim +4");
    CHECK(((p[6] >> 8) & 31) == 13 && ((p[10] >> 8) & 31) == 5 && ((p[14] >> 8) & 31) == 5 &&
          ((p[15] >> 8) & 31) == 5 && p[2] == prog[2], "trim +4 fields");
    memcpy(p, prog, sizeof(p));
    CHECK(bench_trim_patch(p, 16, 6, 3, -9), "trim -9");
    CHECK(((p[6] >> 8) & 31) == 0 && ((p[10] >> 8) & 31) == 18, "trim -9 fields");
    memcpy(p, prog, sizeof(p));
    CHECK(!bench_trim_patch(p, 16, 6, 3, 10) && !memcmp(p, prog, sizeof(p)), "trim 10 refused, unchanged");
    CHECK(bench_trim_max(prog, 16, 6, 3) == 9, "trim max 9");
}

static void test_scan(void) {
    bench_scan_t s;
    bench_scan_begin(&s, 9);
    int t, n = 0;
    while (bench_scan_next(&s, &t)) {
        bench_scan_record(&s, t >= -7 && t <= 8, t >= -7 && t <= 8 ? 0u : 100u);
        n++;
    }
    CHECK(n == 19, "19 steps");
    bench_scan_finish(&s);
    CHECK(s.state == BENCH_SCAN_DONE && s.lo == -7 && s.hi == 8 && s.used == 0, "range -7..8 -> used 0 (%d)", s.used);
    CHECK(s.clean_mask == (((1u << 16) - 1u) << 2), "mask %08x", s.clean_mask);

    bench_scan_begin(&s, 4);
    while (bench_scan_next(&s, &t)) bench_scan_record(&s, t >= 1 && t <= 4, 0);
    bench_scan_finish(&s);
    CHECK(s.lo == 1 && s.hi == 4 && s.used == 2, "range 1..4 -> used 2 (%d)", s.used);

    bench_scan_begin(&s, 9);
    while (bench_scan_next(&s, &t)) bench_scan_record(&s, t == -5 || t == -4 || t == 2 || t == 3, 0);
    bench_scan_finish(&s);
    CHECK(s.lo == 2 && s.hi == 3 && s.used == 2, "tie goes to the run nearer 0");

    bench_scan_begin(&s, 1);
    while (bench_scan_next(&s, &t)) bench_scan_record(&s, false, 7);
    bench_scan_finish(&s);
    CHECK(s.state == BENCH_SCAN_NOCLEAN && s.used == 0, "nothing clean");
}

static void test_parse(void) {
    char line[] = "tx L2 48k ch=12 pay=vec pins=2,3,9 f0=77 sender=9";
    bench_args_t a;
    CHECK(bench_split(line, &a) == 8, "split");
    bench_txcfg_t c;
    const char *err = NULL;
    CHECK(bench_parse_tx(&a, 1, "pins", &c, &err), "parse tx");
    CHECK(c.cls == 2 && c.rate == 1 && c.channels == 12 && c.pay == BENCH_PAY_VECTOR && c.npins == 3 &&
          c.pins[2] == 9 && c.f0 == 77 && c.sender == 9, "tx fields");
    char l2[] = "tx L1 96k";
    bench_split(l2, &a);
    CHECK(bench_parse_tx(&a, 1, "pins", &c, &err) && c.channels == 3 && c.pins[0] == 2 && c.npins == 1,
          "defaults: L1 96k -> 3 channels, pin 2");
    char l3[] = "tx L1 48k ch=9";
    bench_split(l3, &a);
    CHECK(!bench_parse_tx(&a, 1, "pins", &c, &err) && !strcmp(err, "ch"), "C above capacity refused");
    char l4[] = "tx L3 44k1 pins=2,2";
    bench_split(l4, &a);
    CHECK(!bench_parse_tx(&a, 1, "pins", &c, &err), "duplicate pin refused");
    char l5[] = "tx L3 44k1 pins=24";
    bench_split(l5, &a);
    CHECK(!bench_parse_tx(&a, 1, "pins", &c, &err), "internal GPIO refused");
    CHECK(bench_default_channels(3, 1) == 32 && bench_default_channels(3, 2) == 18 &&
          bench_default_channels(2, 0) == 16 && bench_default_channels(1, 1) == 8, "default channels");
    long v;
    CHECK(bench_parse_int("-3", -3, 3, &v) && v == -3 && !bench_parse_int("4", -3, 3, &v) &&
          !bench_parse_int("1x", 0, 9, &v), "ints");
}

static void test_layouts(void) {
    bench_status_t s = { .state = 2, .cls = 1, .rate = 1, .channels = 8, .sender = 7, .sf_flags = 0x10,
                         .frame = 0x0102030405060708ull };
    s.st.lock_count = 1; s.st.loss_count = 2; s.st.crc8_errors = 3; s.st.crc16_errors = 4;
    s.st.bit_slips = 5; s.st.superframe_errors = 6;
    uint8_t o[BENCH_STATUS_LEN];
    bench_pack_status(o, &s);
    CHECK(o[0] == 2 && o[1] == 1 && o[2] == 1 && o[3] == 8 && o[5] == 0xFF && o[6] == 7 && o[7] == 0x10, "status head");
    CHECK(o[8] == 0x08 && o[15] == 0x01 && o[16] == 1 && o[20] == 2 && o[24] == 3 && o[28] == 4 && o[32] == 5 &&
          o[36] == 6 && o[40] == 0 && o[51] == 0, "status fields at 2.8.2 offsets");
    bench_scan_t sc;
    bench_scan_begin(&sc, 9);
    int t;
    while (bench_scan_next(&sc, &t)) bench_scan_record(&sc, t >= -8 && t <= 8, 0);
    bench_scan_finish(&sc);
    bench_diag_t d = { .comma_offset = 37, .ring_wr_bytes = 0x100, .scan = &sc };
    d.st.comma_errors = 0x11; d.st.counter_errors = 0x22; d.st.code_errors = 0x33; d.st.idle_errors = 0x44;
    uint8_t g[BENCH_DIAG_LEN];
    bench_pack_diag(g, &d);
    CHECK(g[0] == 1 && g[1] == 5 && g[17] == 1, "diag head");
    CHECK((int8_t)g[32] == -8 && (int8_t)g[33] == 8 && (int8_t)g[34] == 0 && g[35] == 2, "diag trim fields");
    CHECK(g[36] == 0xFE && g[37] == 0xFF && g[38] == 0x03 && g[39] == 0, "diag clean mask");
    CHECK(g[40] == 0x11 && g[44] == 0x22 && g[48] == 0x33 && g[52] == 0x44 && g[56] == 0 && g[63] == 0,
          "diag bench counters at 40-55");
    char hex[2 * BENCH_DIAG_LEN + 1];
    bench_hex(hex, g, 4);
    CHECK(!strcmp(hex, "01050000"), "hex");
}

int main(void) {
    test_gen_equals_reference();
    test_sink();
    test_acquire();
    test_bert_clean();
    test_impairments();
    test_small_ring();
    test_trim_patch();
    test_scan();
    test_parse();
    test_layouts();
    printf("test_bench_logic: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
