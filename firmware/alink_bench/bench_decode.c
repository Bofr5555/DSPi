/*
 * bench_decode.c - alink bench: cycles per frame of LK5's receiver (WP LK5)
 *
 *   decode all [crc=dma|sw]
 *   decode canned <L1|L2|L3> <44k1|48k|96k> [ch=C] [mask=M] [slot=S] [crc=dma|sw] [runs=N] [batch=F]
 *   decode live [mask=M] [slot=S] [crc=dma|sw] [ms=T] [batch=F]
 *
 * mask is the decimal bank mask (1 = bank 0, 3 = banks 0-1, 15 = banks 0-3);
 * every form ends with OK (or ERR), so `tools/alink/bench.sh cmd` returns.
 *
 * Runs alink_fast (firmware/DSPi/alink/alink_fast.c, built here exactly as in
 * DSPi: ALINK_FAST_M33 and ALINK_FAST_CRC_DMA) over a stream and times every
 * alink_fast_run() with the Cortex-M33's DWT cycle counter.
 *
 *  - canned: a bench stream (PRBS-7 payload, sender's layout) generated into
 *    RAM, written into the decoder's ring F frames at a time (default 48, one
 *    DSPi block at 48 kHz) as the RX DMA would; repeated N times.
 *  - all: canned, for the configuration matrix of the LK5 report.
 *  - live: the RX ring of the `rx` role (start it first: `rx` or `loop`),
 *    with the real DMA writing it, for T ms.
 *
 * Only calls that start locked and deliver frames are counted.  The glue
 * DSPi adds per frame (8 delivered samples to float with a preamp, as
 * adat_input_poll() does) is timed separately.  The delivered samples are
 * checked against the PRBS-7 payload after timing; any mismatch, error
 * counter or loss is reported.
 *
 * Output (one line):
 *   DECODE cls=3 rate=1 ch=32 mask=0x01 crc=dma frames=… cyc_frame=… cyc_min=… cyc_max=…
 *          glue_frame=… pct_core=… bad=0 errors=… lock=1 loss=0
 * cyc_frame is the mean over counted calls (decoder only), cyc_min/max the
 * per-call extremes divided by that call's frames, pct_core = cyc_frame /
 * (sys_clk / Fs) * 100.
 */

#include "bench_decode.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#if PICO_RP2350
#include "hardware/dma.h"
#include "hardware/structs/m33.h"

#include "bench_gen.h"
#include "alink_fast.h"

#define DEC_RING_WORDS  8192u
#define DEC_BLOCK       64u

static uint32_t dec_ring[DEC_RING_WORDS] __attribute__((aligned(4)));
static uint32_t canned[DEC_RING_WORDS] __attribute__((aligned(4)));
static alink_fast_t lf;
static int32_t out_rows[DEC_BLOCK][8];
static float in_buf[8][DEC_BLOCK];
static float preamp[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
static uint32_t crc_sink;
static int crc_ch = -1;

typedef struct {
    uint64_t cyc, glue;
    uint32_t frames, calls, min_q16, max_q16, bad;
} dec_acc_t;

static inline uint32_t cyccnt(void) { return m33_hw->dwt_cyccnt; }

static void cyc_enable(void) {
    m33_hw->demcr |= M33_DEMCR_TRCENA_BITS;
    m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;
}

static void ev_cb(void *ctx, const alink_rx_event_t *ev) { (void)ctx; (void)ev; }

// The sniffer channel: configured once, self-tested against alink_crc16().
// Returns the state for the report.
static const char *crc_setup(bool want) {
    if (!want) {
        alink_fast_set_crc_dma(&lf, -1);
        return "sw";
    }
    if (crc_ch < 0) {
        crc_ch = dma_claim_unused_channel(false);
        if (crc_ch < 0) return "sw(no channel)";
        dma_channel_config c = dma_channel_get_default_config((uint)crc_ch);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, DREQ_FORCE);
        channel_config_set_sniff_enable(&c, true);
        dma_channel_configure((uint)crc_ch, &c, &crc_sink, canned, 0, false);
    }
    alink_fast_set_crc_dma(&lf, crc_ch);
    static const uint8_t bytes[24] = "123456789abcdefghijklmno";
    uint32_t w[6];
    for (unsigned i = 0; i < 6u; i++)
        w[i] = (uint32_t)bytes[4 * i] << 24 | (uint32_t)bytes[4 * i + 1] << 16 | (uint32_t)bytes[4 * i + 2] << 8 |
               bytes[4 * i + 3];
    uint32_t want_crc = alink_crc16(bytes, 24);
    for (int swap = 0; swap < 2; swap++) {
        dma_sniffer_enable((uint)crc_ch, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, false);
        dma_sniffer_set_byte_swap_enabled(swap != 0);
        if (alink_fast_crc_dma_probe(&lf, w) == want_crc) {
            return swap ? "dma(bswap)" : "dma";
        }
    }
    alink_fast_set_crc_dma(&lf, -1);
    return "sw(sniffer mismatch)";
}

// Give the sniffer channel back (every command ends with this: `stop` and the
// roles claim and release channels freely).
static void crc_release(void) {
    if (crc_ch >= 0) {
        dma_sniffer_disable();
        dma_channel_unclaim((uint)crc_ch);
        crc_ch = -1;
    }
}

// The glue DSPi runs per delivered block: float input buffers with preamp.
static uint32_t glue(unsigned n) {
    uint32_t t0 = cyccnt();
    const float k = 1.0f / 8388608.0f;
    for (unsigned i = 0; i < n; i++)
        for (unsigned ch = 0; ch < 8u; ch++) in_buf[ch][i] = (float)out_rows[i][ch] * k * preamp[ch];
    return cyccnt() - t0;
}

// Delivered rows against the PRBS-7 payload (every frame carries the same one).
static uint32_t check_rows(unsigned n, const int32_t *expect) {
    uint32_t bad = 0;
    for (unsigned i = 0; i < n; i++)
        if (memcmp(out_rows[i], expect, sizeof(out_rows[i]))) bad++;
    return bad;
}

static void one_call(dec_acc_t *a, uint64_t avail, const int32_t *expect) {
    bool locked = lf.state == ALINK_RX_LOCKED;
    uint32_t t0 = cyccnt();
    unsigned n = alink_fast_run(&lf, avail, 4096);
    uint32_t dt = cyccnt() - t0;
    unsigned got = lf.out_n;
    if (locked && n && got == n) {
        a->cyc += dt;
        a->frames += n;
        a->calls++;
        uint32_t q = (uint32_t)(((uint64_t)dt << 16) / n);
        if (q < a->min_q16) a->min_q16 = q;
        if (q > a->max_q16) a->max_q16 = q;
        a->glue += glue(got);
        a->bad += check_rows(got, expect);
    }
    lf.out_n = 0;
}

static void report(const char *mode, unsigned cls, unsigned rate, unsigned ch, unsigned mask, const char *crc,
                   const dec_acc_t *a, uint32_t sys_hz) {
    uint32_t fs = alink_rate_hz(rate);
    uint32_t mean = a->frames ? (uint32_t)(a->cyc / a->frames) : 0;
    uint32_t glue_f = a->frames ? (uint32_t)(a->glue / a->frames) : 0;
    uint32_t pct10 = fs ? (uint32_t)((uint64_t)mean * fs * 1000u / sys_hz) : 0;     // 0.1 %
    const alink_rx_stats_t *s = &lf.stats;
    uint32_t errors = s->crc8_errors + s->crc16_errors + s->bit_slips + s->superframe_errors + s->comma_errors +
                      s->counter_errors + s->code_errors + s->idle_errors;
    printf("DECODE %s cls=%u rate=%u ch=%u mask=0x%02x crc=%s frames=%lu calls=%lu cyc_frame=%lu cyc_min=%lu "
           "cyc_max=%lu glue_frame=%lu pct_core=%lu.%lu bad=%lu errors=%lu lock=%lu loss=%lu overruns=%lu\n",
           mode, cls, rate, ch, mask, crc, (unsigned long)a->frames, (unsigned long)a->calls, (unsigned long)mean,
           (unsigned long)(a->frames ? a->min_q16 >> 16 : 0), (unsigned long)(a->frames ? a->max_q16 >> 16 : 0),
           (unsigned long)glue_f, (unsigned long)(pct10 / 10u), (unsigned long)(pct10 % 10u),
           (unsigned long)a->bad, (unsigned long)errors, (unsigned long)s->lock_count,
           (unsigned long)s->loss_count, (unsigned long)lf.overruns);
}

static bool parse_common(const bench_args_t *a, long *mask, long *slot, bool *dma, long *batch) {
    const char *s;
    *mask = 0x01;
    *slot = -1;
    *dma = true;
    *batch = 48;
    if ((s = bench_kv(a, "mask")) && !bench_parse_int(s, 1, 255, mask)) return false;
    if ((s = bench_kv(a, "slot")) && !bench_parse_int(s, 0, 63, slot)) return false;
    if ((s = bench_kv(a, "crc"))) {
        if (!strcmp(s, "sw")) *dma = false;
        else if (strcmp(s, "dma")) return false;
    }
    if ((s = bench_kv(a, "batch")) && !bench_parse_int(s, 1, DEC_BLOCK, batch)) return false;
    return true;
}

// The 8 outputs: channels slot .. slot + 7 (default: the first channel of
// the lowest bank in the mask), and what they carry in a PRBS-7 stream.
static void outputs(long slot, long mask, unsigned ch, int32_t expect[8]) {
    unsigned s0 = slot >= 0 ? (unsigned)slot : 8u * (unsigned)__builtin_ctz((unsigned)mask);
    uint8_t slots[8];
    uint8_t prbs[3 * ALINK_MAX_CHANNELS];
    alink_prbs7_fill(prbs, 3u * ch);
    for (unsigned i = 0; i < 8u; i++) {
        unsigned c = s0 + i;
        slots[i] = (uint8_t)c;
        bool decoded = c < ch && (((unsigned)mask >> (c / 8u)) & 1u);
        expect[i] = decoded ? alink_get_s24(prbs + 3u * c) : 0;
    }
    alink_fast_set_slots(&lf, slots);
}

// One canned measurement; false for a layout that does not exist.
static bool canned_run(unsigned cls, unsigned rate, unsigned ch, long mask, long slot, bool dma, long batch,
                       long runs, uint32_t sys_hz) {
    if (!ch) ch = cls == 1 ? (rate == ALINK_RATE_96K ? 3u : 8u) : cls == 2 ? (rate == ALINK_RATE_96K ? 8u : 16u)
                           : (rate == ALINK_RATE_96K ? 16u : 32u);
    static bench_gen_t g;
    if (!bench_gen_init(&g, cls, rate, ch, BENCH_PAY_PRBS, 1000, 7)) return false;
    bench_sink_t sink;
    bench_sink_init(&sink, canned, DEC_RING_WORDS);
    memset(canned, 0, sizeof(canned));
    while (bench_sink_bits(&sink) + bench_gen_frame_bits(&g) + 64u <= DEC_RING_WORDS * 32ull)
        bench_gen_frame(&g, &sink);
    uint32_t words = (uint32_t)sink.words;
    uint32_t step = (uint32_t)batch * 10u * alink_frame_symbols_max(cls, rate) / 32u + 1u;
    dec_acc_t acc = { 0, 0, 0, 0, 0xFFFFFFFFu, 0, 0 };
    int32_t expect[8];
    const char *crc = "";
    for (long r = 0; r < runs; r++) {
        alink_fast_init(&lf, dec_ring, DEC_RING_WORDS, cls, (uint8_t)mask, ev_cb, 0, 0);
        alink_fast_set_output(&lf, out_rows, (uint16_t)batch);
        outputs(slot, mask, ch, expect);
        crc = crc_setup(dma);
        memset(dec_ring, 0, sizeof(dec_ring));
        for (uint32_t w = 0; w < words;) {
            uint32_t k = words - w < step ? words - w : step;
            memcpy(dec_ring + w, canned + w, 4u * k);       // the "DMA"
            w += k;
            // Decode until this block's frames are all out (the output holds batch frames).
            for (int i = 0; i < 4; i++) one_call(&acc, (uint64_t)w * 32u, expect);
        }
    }
    crc_release();
    report("canned", cls, rate, ch, (unsigned)mask, crc, &acc, sys_hz);
    return true;
}

void bench_decode_cmd(const bench_args_t *a, const bench_decode_live_t *live, uint32_t sys_hz) {
    cyc_enable();
    long mask, slot, batch;
    bool dma;
    if (a->argc < 2 || !parse_common(a, &mask, &slot, &dma, &batch)) {
        printf("ERR decode canned <cls> <rate> [ch=] [mask=] [slot=] [crc=dma|sw] [runs=] [batch=] | "
               "decode live [mask=] [slot=] [crc=] [ms=] [batch=]\n");
        return;
    }
    if (!strcmp(a->argv[1], "canned")) {
        unsigned cls, rate;
        long ch = 0, runs = 4;
        const char *s;
        if (a->argc < 4 || !bench_parse_class(a->argv[2], &cls) || !bench_parse_rate(a->argv[3], &rate)) {
            printf("ERR decode canned <L1|L2|L3> <44k1|48k|96k> ...\n");
            return;
        }
        if ((s = bench_kv(a, "ch")) && !bench_parse_int(s, 1, ALINK_MAX_CHANNELS, &ch)) { printf("ERR ch\n"); return; }
        if ((s = bench_kv(a, "runs")) && !bench_parse_int(s, 1, 100, &runs)) { printf("ERR runs\n"); return; }
        if (!canned_run(cls, rate, (unsigned)ch, mask, slot, dma, batch, runs, sys_hz)) printf("ERR layout\n");
        else printf("OK\n");
        return;
    }
    if (!strcmp(a->argv[1], "all")) {
        // The matrix of the LK5 report: class x rate x banks decoded.
        static const struct { uint8_t cls, rate, ch, mask; } m[] = {
            { 1, 1, 8, 1 }, { 1, 2, 3, 1 }, { 1, 0, 8, 1 }, { 2, 1, 16, 1 }, { 2, 1, 16, 3 }, { 2, 2, 8, 1 },
            { 2, 0, 16, 1 }, { 3, 1, 32, 1 }, { 3, 1, 32, 3 }, { 3, 1, 32, 15 }, { 3, 1, 38, 31 }, { 3, 1, 8, 1 },
            { 3, 2, 16, 1 }, { 3, 2, 16, 3 }, { 3, 2, 18, 7 }, { 3, 0, 32, 1 },
        };
        for (size_t i = 0; i < sizeof(m) / sizeof(m[0]); i++)
            canned_run(m[i].cls, m[i].rate, m[i].ch, m[i].mask, -1, dma, batch, 2, sys_hz);
        printf("OK\n");
        return;
    }
    if (!strcmp(a->argv[1], "live")) {
        if (!live || !live->ring) { printf("ERR start the rx role first (rx or loop)\n"); return; }
        long ms = 1000;
        const char *s;
        if ((s = bench_kv(a, "ms")) && !bench_parse_int(s, 10, 60000, &ms)) { printf("ERR ms\n"); return; }
        dec_acc_t acc = { 0, 0, 0, 0, 0xFFFFFFFFu, 0, 0 };
        int32_t expect[8];
        uint64_t start = live->writer_bits();
        alink_fast_init(&lf, live->ring, live->ring_words, live->cls, (uint8_t)mask, ev_cb, 0, start & ~31ull);
        alink_fast_set_output(&lf, out_rows, (uint16_t)batch);
        // A live stream's C comes from its superframe: lock first, then set the outputs.
        uint64_t t_end = time_us_64() + 500000u;
        while (lf.state != ALINK_RX_LOCKED && time_us_64() < t_end) {
            alink_fast_run(&lf, live->writer_bits(), 4096);
            lf.out_n = 0;
        }
        if (lf.state != ALINK_RX_LOCKED) { printf("ERR no lock within 500 ms\n"); return; }
        outputs(slot, mask, lf.channels, expect);
        const char *crc = crc_setup(dma);
        memset(&lf.stats, 0, sizeof(lf.stats));
        lf.stats.lock_count = 1;
        t_end = time_us_64() + (uint64_t)ms * 1000u;
        while (time_us_64() < t_end) {
            one_call(&acc, live->writer_bits(), expect);
            busy_wait_us_32(200);       // the rest of DSPi's main loop pass
        }
        crc_release();
        report("live", live->cls, lf.rate, lf.channels, (unsigned)mask, crc, &acc, sys_hz);
        printf("OK\n");
        return;
    }
    printf("ERR decode canned|live\n");
}

#else   // RP2040: no DWT cycle counter, and the alink is RP2350-only

void bench_decode_cmd(const bench_args_t *a, const bench_decode_live_t *live, uint32_t sys_hz) {
    (void)a; (void)live; (void)sys_hz;
    printf("ERR decode needs an RP2350 (DWT cycle counter, LK5's Cortex-M33 build)\n");
}

#endif
