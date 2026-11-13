// Host test for the production alink receiver (WP LK5): firmware/DSPi/alink/
// alink_fast.c must give exactly the reference receiver's log (alink_decoder.c,
// spec 9.2: events, frames, flags, bank status, sample hashes, counters) on
//  - every stream vector of alink_vectors.json, with the line written into a
//    DMA-style ring in steps of 1, 7, 32, 320, 4096 and random bits, into the
//    production 32 KB ring and a 1 KB ring that wraps every few frames, and
//    with absolute bit positions that cross 2^32;
//  - a differential fuzz: random streams (every class and rate, 1..capacity
//    channels, random F, flags, RATE applies, bank masks) with random damage
//    (flips, bursts, deletions, insertions, fills) and raw noise, decoded by
//    both receivers;
//  - optionally more vector files given on the command line (review lk's
//    fuzz1.jsonl, for instance).
// It also checks what only the production receiver has: overrun recovery and
// the search budget.
// Built by run.sh: cc -std=c11 -Wall -Wextra -Werror -I ../DSPi test_alink_fast.c
#define main test_alink_codec_main
#include "test_alink_codec.c"
#undef main
#include "alink/alink_fast.c"

// ---------------------------------------------------------------------------
// The production receiver over a ring
// ---------------------------------------------------------------------------

typedef struct {
    logbuf_t *lb;
    uint64_t origin;    // absolute bit of stream bit 0
} fast_log_t;

static void fast_log_cb(void *ctx, const alink_rx_event_t *ev) {
    fast_log_t *fl = ctx;
    if (ev->type == ALINK_RX_EV_ALIGN) {
        alink_rx_event_t e = *ev;
        e.bitpos -= fl->origin;
        log_cb(fl->lb, &e);
    } else {
        log_cb(fl->lb, ev);
    }
}

static alink_fast_t fx;
static uint32_t fring[8192];
static uint32_t lcg_state = 12345;
static uint32_t rand_chunk_max = 3000;  // random chunks: 1..this many bits
static bool use_restart;                // decode_fast: alink_fast_restart() after a wrong-class init
static uint32_t lcg(void) {
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return lcg_state >> 8;
}

// Write bits[0..n) into a ring of ring_words words at absolute bits
// origin.., calling alink_fast_run() after every chunk (0 = random chunks).
static void decode_fast(const uint8_t *bits, size_t n, unsigned cls, uint8_t mask, size_t chunk,
                        uint32_t ring_words, uint64_t origin, logbuf_t *lb) {
    lb->len = 0;
    lb->log[0] = 0;
    fast_log_t fl = { lb, origin };
    memset(fring, 0xA5, sizeof(fring));     // stale ring contents must never matter
    if (use_restart) {
        // A decoder of another class that has seen other bits, then restarted:
        // must be the fresh decoder exactly (alink_input.c's alink-up steps).
        alink_fast_init(&fx, fring, ring_words, cls % 3u + 1u, mask, fast_log_cb, &fl, 0);
        alink_fast_run(&fx, 4096, 0);
        fx.stats.crc8_errors = 99;
        alink_fast_restart(&fx, cls, origin);
    } else {
        alink_fast_init(&fx, fring, ring_words, cls, mask, fast_log_cb, &fl, origin);
    }
    fx.frame_events = true;
    uint32_t wmask = ring_words - 1u;
    for (size_t at = 0; at < n;) {
        size_t k = chunk ? chunk : 1u + lcg() % rand_chunk_max;
        if (k > n - at) k = n - at;
        for (size_t i = at; i < at + k; i++) {
            uint64_t b = origin + i;
            uint32_t *w = &fring[(uint32_t)(b >> 5) & wmask];
            uint32_t bit = 0x80000000u >> (b & 31u);
            if (bits[i]) *w |= bit;
            else *w &= ~bit;
        }
        at += k;
        alink_fast_run(&fx, origin + at, 0);
    }
    log_stats(lb, &fx.stats);
}

// ---------------------------------------------------------------------------
// Vectors
// ---------------------------------------------------------------------------

static unsigned streams_checked, runs_checked;

static void check_fast_stream(const char *l, const char *file) {
    static char want_log[1 << 18], got_log[1 << 18], ops[1 << 14];
    get_str(l, "name", name, sizeof(name));
    size_t nbits = (size_t)get_u64(l, "bits");
    size_t nbytes = get_hex(l, "stream", hexbuf, sizeof(hexbuf));
    CHECK(nbytes == (nbits + 7) / 8, "%s %s: length", file, name);
    for (size_t i = 0; i < nbits; i++) bits_a[i] = (uint8_t)((hexbuf[i >> 3] >> (7 - (i & 7))) & 1u);
    get_str(l, "corrupt", ops, sizeof(ops));
    size_t n = apply_ops(bits_a, nbits, ops, bits_b);
    get_str(l, "log", want_log, sizeof(want_log));
    unsigned cls = (unsigned)get_u64(l, "cls");
    uint8_t mask = (uint8_t)get_u64(l, "bank_mask");
    static const struct { size_t chunk; uint32_t ring; uint64_t origin; } runs[] = {
        { 32, 8192, 0 }, { 1, 8192, 0 }, { 7, 8192, 0 }, { 320, 8192, 0 }, { 4096, 8192, 0 },
        { 0, 8192, (1ull << 32) - 32u * 777u },     // random chunks, positions cross 2^32
        { 0, 256, 0 },                              // 1 KB ring: wraps every few frames
        { 640, 256, (1ull << 32) - 32u * 3001u },
        { 320, 8192, 32u * 4096u },                 // after alink_fast_restart() (below)
    };
    for (size_t i = 0; i < sizeof(runs) / sizeof(runs[0]); i++) {
        logbuf_t lb = { got_log, 0, sizeof(got_log) };
        rand_chunk_max = 3000;
        use_restart = i == sizeof(runs) / sizeof(runs[0]) - 1u;
        decode_fast(bits_b, n, cls, mask, runs[i].chunk, runs[i].ring, runs[i].origin, &lb);
        use_restart = false;
        CHECK(strcmp(got_log, want_log) == 0,
              "%s %s (chunk %zu ring %u origin %" PRIu64 "): log differs\n got  %.800s\n want %.800s", file, name,
              runs[i].chunk, runs[i].ring, runs[i].origin, got_log, want_log);
        CHECK(fx.overruns == 0, "%s %s: overrun", file, name);
        runs_checked++;
    }
    streams_checked++;
}

static void fast_vectors(const char *path) {
    FILE *f = fopen(path, "r");
    CHECK(f != NULL, "cannot open %s", path);
    if (!f) return;
    unsigned before = streams_checked;
    while (fgets(line, sizeof(line), f)) {
        char type[32];
        get_str(line, "type", type, sizeof(type));
        if (strcmp(type, "stream") == 0) check_fast_stream(line, path);
    }
    fclose(f);
    printf("  %s: %u streams, all runs identical to the vector logs\n", path, streams_checked - before);
}

// ---------------------------------------------------------------------------
// Differential fuzz
// ---------------------------------------------------------------------------

static uint64_t xs = 0x9E3779B97F4A7C15ull;
static uint64_t rnd(void) { xs ^= xs << 13; xs ^= xs >> 7; xs ^= xs << 17; return xs; }
static unsigned rndn(unsigned n) { return (unsigned)(rnd() % n); }

// A random stream into bits_a; returns its length.
static size_t fuzz_encode(unsigned cls, unsigned rate, unsigned c, uint64_t first, unsigned frames,
                          uint32_t seed, uint8_t flags, int sched_index, uint64_t sched_at, uint8_t op,
                          uint8_t arg, int16_t value) {
    static uint32_t words[MAX_BITS / 32];
    alink_tx_t tx;
    if (!alink_tx_init(&tx, cls, rate, c, first, (uint8_t)seed)) return 0;
    tx.flags = flags;
    tx.hops = (uint8_t)rndn(3);
    alink_bitw_t w;
    alink_bitw_init(&w, words, MAX_BITS / 32);
    for (unsigned i = 0; i < frames; i++) {
        if ((int)i == sched_index) (void)alink_tx_schedule(&tx, sched_at, op, arg, value);
        int32_t s[ALINK_MAX_CHANNELS];
        for (unsigned ch = 0; ch < tx.channels; ch++) s[ch] = test_sample(tx.frame, ch, seed);
        if (!alink_tx_frame(&tx, s, &w)) break;
    }
    for (size_t i = 0; i < w.nbits; i++) bits_a[i] = (uint8_t)((words[i >> 5] >> (31u - (i & 31u))) & 1u);
    return w.nbits;
}

static size_t fuzz_ops(size_t nbits, char *ops, size_t cap) {
    size_t len = 0;
    ops[0] = 0;
    static const unsigned nops[] = { 0, 1, 1, 2, 2, 3, 4, 6, 12 };
    unsigned count = nops[rndn(sizeof(nops) / sizeof(nops[0]))];
    for (unsigned i = 0; i < count && len + 64 < cap; i++) {
        unsigned kind = rndn(100);
        size_t pos = rndn((unsigned)nbits);
        if (kind < 35) {
            len += (size_t)snprintf(ops + len, cap - len, "%sflip:%zu", len ? ";" : "", pos);
        } else if (kind < 47) {
            len += (size_t)snprintf(ops + len, cap - len, "%sdel:%zu", len ? ";" : "", pos);
        } else if (kind < 59) {
            len += (size_t)snprintf(ops + len, cap - len, "%sins:%zu:%u", len ? ";" : "", pos, rndn(2));
        } else if (kind < 75) {
            static const unsigned fl[] = { 1, 2, 5, 10, 37, 300, 2000, 5000 };
            len += (size_t)snprintf(ops + len, cap - len, "%sfill:%zu:%u:%u", len ? ";" : "", pos,
                                    fl[rndn(8)], rndn(2));
        } else if (kind < 90) {
            unsigned burst = 2 + rndn(7);
            for (unsigned j = 0; j < burst && len + 32 < cap; j++) {
                size_t p = pos + rndn(40);
                if (p >= nbits) p = nbits - 1;
                len += (size_t)snprintf(ops + len, cap - len, "%sflip:%zu", len ? ";" : "", p);
            }
        } else {
            // a slip of 1..3 bits: deletions or insertions close together
            unsigned s = 1 + rndn(3);
            bool del = rndn(2);
            for (unsigned j = 0; j < s && len + 32 < cap; j++) {
                size_t p = pos + 3 * j;
                if (p >= nbits) p = nbits - 1;
                if (del) len += (size_t)snprintf(ops + len, cap - len, "%sdel:%zu", len ? ";" : "", p);
                else len += (size_t)snprintf(ops + len, cap - len, "%sins:%zu:%u", len ? ";" : "", p, rndn(2));
            }
        }
    }
    return len;
}

static void fuzz(unsigned rounds) {
    static char ref_log[1 << 18], fast_log[1 << 18], ops[4096];
    static const uint64_t firsts[] = { 0, 1, 31, 32, 33, 146, 147, 255, 256, (1ull << 32) - 40, (1ull << 32) + 5,
                                       1ull << 40, (1ull << 63) + 12345, 0xFFFFFFFFFFFFE000ull, 147u * 1000u + 5 };
    unsigned locked_runs = 0, damaged = 0, noise = 0;
    for (unsigned r = 0; r < rounds; r++) {
        size_t n;
        unsigned cls = 1 + rndn(3), rate = rndn(3);
        uint8_t mask = (uint8_t)(rndn(4) == 0 ? rnd() : rndn(3) == 0 ? 1u << rndn(8) : 0xFF);
        if (rndn(40) == 0) {
            // raw noise, or noise with sparse commas
            n = 2000 + rndn(60000);
            for (size_t i = 0; i < n; i++) bits_a[i] = (uint8_t)(rnd() & 1u);
            if (rndn(2))
                for (unsigned k = 0; k < 40; k++) {
                    size_t at = rndn((unsigned)(n - 10));
                    uint16_t s = rndn(2) ? ALINK_SYM_K28_5_RDN : ALINK_SYM_K28_5_RDP;
                    for (unsigned b = 0; b < 10; b++) bits_a[at + b] = (uint8_t)((s >> (9 - b)) & 1u);
                }
            noise++;
        } else {
            unsigned mx = alink_max_channels(cls, rate);
            unsigned choice[] = { 1, 2, mx, 1 + rndn(mx), mx < 8 ? mx : 8, mx < 9 ? mx : 9, mx < 16 ? mx : 16 };
            unsigned c = choice[rndn(7)];
            uint64_t first = rndn(4) ? firsts[rndn(sizeof(firsts) / sizeof(firsts[0]))] : rnd() >> 20;
            unsigned frames = 40 + rndn(cls == 3 ? 90 : 160);
            static const uint8_t fls[] = { 0, 0, ALINK_SF_FLAG_USB_LIVE, ALINK_SF_FLAG_PRBS, 0x09, 0x3F };
            int sched = -1;
            uint64_t at = 0;
            uint8_t op = 0, arg = 0;
            int16_t value = 0;
            if (rndn(3) == 0) {
                sched = (int)rndn(frames / 2);
                op = (uint8_t)(1 + rndn(8));
                uint64_t base = first + (uint64_t)sched;
                at = ((base + 64u + rndn(96)) & ~31ull);
                if (op == ALINK_OP_RATE) {
                    arg = (uint8_t)rndn(3);
                    unsigned m2 = alink_max_channels(cls, arg);
                    value = (int16_t)(rndn(3) == 0 ? 0 : 1 + rndn(m2));
                } else {
                    arg = (uint8_t)rndn(4);
                    value = (int16_t)rnd();
                    at += rndn(31);
                }
            }
            n = fuzz_encode(cls, rate, c, first, frames, (uint32_t)rndn(1000), fls[rndn(6)], sched, at, op, arg,
                            value);
            if (!n) continue;
        }
        size_t olen = fuzz_ops(n, ops, sizeof(ops));
        if (olen) damaged++;
        size_t m = apply_ops(bits_a, n, ops, bits_b);
        // Sometimes the receiver runs at the wrong class.
        unsigned rx_cls = rndn(20) == 0 ? 1 + rndn(3) : cls;
        logbuf_t a = { ref_log, 0, sizeof(ref_log) }, b = { fast_log, 0, sizeof(fast_log) };
        decode_bits(bits_b, m, rx_cls, mask, 4096, &a);
        // Chunks stay below ring - guard - longest frame, as the firmware's
        // service interval does.
        static const uint32_t rings[] = { 8192, 256, 128 }, limits[] = { 100000, 4000, 600 };
        unsigned ri = rndn(3);
        uint32_t ring = rings[ri];
        rand_chunk_max = limits[ri] < 3000u ? limits[ri] : 3000u;
        size_t chunk = rndn(3) == 0 ? 0 : 1u + rndn(limits[ri]);
        uint64_t origin = rndn(4) == 0 ? (1ull << 32) - 32u * rndn(4000) : 32u * (uint64_t)rndn(1000);
        decode_fast(bits_b, m, rx_cls, mask, chunk, ring, origin, &b);
        if (rx.stats.lock_count) locked_runs++;
        CHECK(strcmp(ref_log, fast_log) == 0 && fx.overruns == 0,
              "fuzz round %u (cls %u, mask %02x, chunk %zu, ring %u, ops %s): logs differ\n fast %.900s\n ref  %.900s",
              r, rx_cls, mask, chunk, ring, ops, fast_log, ref_log);
        if (failures > 20) break;
    }
    printf("  fuzz: %u rounds (%u damaged, %u noise, %u locked at least once), identical logs\n", rounds, damaged,
           noise, locked_runs);
}

// ---------------------------------------------------------------------------
// Production-only behaviour
// ---------------------------------------------------------------------------

typedef struct { unsigned frames, locks, losses; } count_ctx_t;
static void count_cb(void *ctx, const alink_rx_event_t *ev) {
    count_ctx_t *c = ctx;
    if (ev->type == ALINK_RX_EV_FRAME) c->frames++;
    if (ev->type == ALINK_RX_EV_LOCK) c->locks++;
    if (ev->type == ALINK_RX_EV_LOSS) c->losses++;
}

// Write words of a clean L1 stream into the ring as a DMA would.
static void test_overrun_and_budget(void) {
    static uint32_t words[1 << 15];
    alink_tx_t tx;
    alink_bitw_t w;
    alink_bitw_init(&w, words, sizeof(words) / 4);
    CHECK(alink_tx_init(&tx, 1, 1, 8, 5000, 1), "init");
    int32_t s[8] = { 0 };
    while (alink_tx_frame(&tx, s, &w)) {}
    size_t nw = w.nbits / 32u;

    // Locked receiver that is not called for longer than the ring holds:
    // one overrun, one loss, then it locks again on newer bits.
    count_ctx_t c = { 0, 0, 0 };
    alink_fast_init(&fx, fring, 256, 1, 0xFF, count_cb, &c, 0);
    fx.frame_events = true;
    size_t wr = 0;
    for (; wr < 3000; wr++) {
        fring[wr & 255u] = words[wr];
        if ((wr & 63u) == 63u) alink_fast_run(&fx, (uint64_t)(wr + 1u) * 32u, 0);
    }
    CHECK(fx.state == ALINK_RX_LOCKED && c.locks == 1, "locked before the gap");
    for (; wr < 3000 + 600; wr++) fring[wr & 255u] = words[wr];     // > 256 words unread
    alink_fast_run(&fx, (uint64_t)wr * 32u, 0);
    CHECK(fx.overruns == 1 && c.losses == 1 && fx.stats.loss_count == 1, "overrun: %u, losses %u", fx.overruns,
          c.losses);
    for (; wr < nw; wr += 64) {
        size_t end = wr + 64 < nw ? wr + 64 : nw;
        for (size_t i = wr; i < end; i++) fring[i & 255u] = words[i];
        alink_fast_run(&fx, (uint64_t)end * 32u, 0);
    }
    CHECK(fx.state == ALINK_RX_LOCKED && c.locks == 2 && fx.overruns == 1, "re-locked after the overrun (locks %u)",
          c.locks);

    // Fixup LK45F (review lk45 finding 5): a service gap at the stall rule's
    // limit (alink_ring_clock.h, 15/16 of the production ring unread) is no
    // overrun: no loss, no frame missed.
    count_ctx_t c3 = { 0, 0, 0 };
    alink_fast_init(&fx, fring, 8192, 1, 0xFF, count_cb, &c3, 0);
    fx.frame_events = true;
    for (wr = 0; wr < 3000; wr++) {
        fring[wr & 8191u] = words[wr];
        if ((wr & 63u) == 63u) alink_fast_run(&fx, (uint64_t)(wr + 1u) * 32u, 0);
    }
    CHECK(fx.state == ALINK_RX_LOCKED && c3.locks == 1, "locked before the stall");
    unsigned frames_before = c3.frames;
    size_t stall_words = 8192u - 8192u / 16u + 31u;     // 15/16 of a pass, plus words since the last call
    for (; wr < 3000 + stall_words; wr++) fring[wr & 8191u] = words[wr];
    alink_fast_run(&fx, (uint64_t)wr * 32u, 0);
    CHECK(fx.overruns == 0 && c3.losses == 0 && fx.state == ALINK_RX_LOCKED, "stall: overruns %u, losses %u",
          fx.overruns, c3.losses);
    CHECK(c3.frames - frames_before >= stall_words * 32u / 320u - 1u, "stall: %u frames decoded after it",
          c3.frames - frames_before);

    // Search budget: a receiver started 20000 bits late skips to the newest
    // bits and still locks; it decodes no frame older than the skip.
    count_ctx_t c2 = { 0, 0, 0 };
    alink_fast_init(&fx, fring, 8192, 1, 0xFF, count_cb, &c2, 0);
    fx.frame_events = true;
    for (size_t i = 0; i < nw && i < 8192; i++) fring[i] = words[i];
    alink_fast_run(&fx, 1000u * 32u, 4096);
    CHECK(fx.state != ALINK_RX_SEARCH || fx.q >= 1000u * 32u - 4096u, "budget skip");
    alink_fast_run(&fx, 4000u * 32u, 4096);
    CHECK(fx.state == ALINK_RX_LOCKED && fx.overruns == 0, "locks with a search budget");
    printf("  overrun recovery and search budget: OK\n");
}

int main(int argc, char **argv) {
    char path[1024];
    snprintf(path, sizeof(path), "%s", __FILE__);
    char *slash = strrchr(path, '/');
    if (slash) strcpy(slash + 1, "alink_vectors.json");
    else strcpy(path, "alink_vectors.json");
    printf("  sizeof(alink_fast_t) %zu\n", sizeof(alink_fast_t));
    fast_vectors(path);
    unsigned rounds = 3000;
    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--fuzz=", 7) == 0) rounds = (unsigned)strtoul(argv[i] + 7, NULL, 10);
        else fast_vectors(argv[i]);
    }
    fuzz(rounds);
    test_overrun_and_budget();
    printf("  %u streams x %u runs\n", streams_checked, streams_checked ? runs_checked / streams_checked : 0);
    if (failures) {
        fprintf(stderr, "test_alink_fast: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_alink_fast: OK\n");
    return 0;
}
