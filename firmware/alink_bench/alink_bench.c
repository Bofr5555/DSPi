/*
 * link_bench.c - DSPi audio link bench image (WP LK3)
 *
 * A standalone image for spare Pico (RP2040) and Pico 2 (RP2350) boards
 * wired GPIO to GPIO; it is not DSPi and shares no code with its audio
 * pipeline.  Roles, chosen at run time over the USB serial console:
 *
 *   tx      send a bench stream (PRBS-7 or a test vector) on 1..N pins with
 *           link_tx_N, all SMs started in sync, every link DMA-fed from one
 *           word ring
 *   rx      receive with link_rx_N into a 32 KB raw ring (two DMA channels:
 *           data + address reload, the LK5 design), acquire with LK1's
 *           reference receiver, check every bit against the regenerated
 *           stream, run the reference receiver for the spec 8.9 counters,
 *           and scan the sampling trim
 *   loop    tx and rx on one board, wired pin to pin
 *   impair  re-time a passing stream through a PIO pass-through and corrupt
 *           it in timed bursts (bit flips, dropouts)
 *
 * Clocking: 307.2 MHz, the clock the class dividers assume (20 / 10 / 5
 * cycles per bit), on both chips (DSPi runs the RP2040 there too), core at
 * 1.20 V.  "clock 153" halves it: link_tx_10/rx_10 then carry L1 and
 * link_tx_5/rx_5 carry L2, and L3 is not available; "clock 76" (76.8 MHz,
 * below the RP2040's stock clock) leaves only L1, on link_tx_5/rx_5.  The whole image runs from RAM (copy_to_ram), so the flash
 * clock never matters after boot.
 *
 * Work split: core 1 runs the real-time parts (TX ring refill, bit error
 * checker); core 0 runs the console, acquisition, the reference receiver
 * (best effort, with gaps counted), the trim scan and IMPAIR bursts.
 *
 * Nothing here writes flash.  Documentation/Bench/link_gpio_bench.md has
 * the wiring, the commands and the expected results.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/structs/busctrl.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"

#include "bench_check.h"
#include "bench_cmd.h"
#include "bench_decode.h"
#include "bench_gen.h"
#include "bench_trim.h"

#include "bench_retime.pio.h"
#include "link_rx_10.pio.h"
#include "link_rx_20.pio.h"
#include "link_rx_5.pio.h"
#include "link_tx_10.pio.h"
#include "link_tx_20.pio.h"
#include "link_tx_5.pio.h"

#ifndef BENCH_GIT
#define BENCH_GIT "unknown"
#endif

#if PICO_RP2350
#define CHIP_NAME   "rp2350"
#else
#define CHIP_NAME   "rp2040"
#endif

#define LED_PIN         25
#define RX_RING_WORDS   8192u           // 32 KB
#define TX_RING_WORDS   4096u           // 16 KB
#define RX_GUARD_WORDS  512u            // never read this close to the writer's lap
#define TX_MARGIN_WORDS 64u
#define MAX_LINKS       ((NUM_DMA_CHANNELS - 2u) / 2u < BENCH_MAX_TX_PINS ? (NUM_DMA_CHANNELS - 2u) / 2u : BENCH_MAX_TX_PINS)

// ---------------------------------------------------------------------------
// Programs per bit period
// ---------------------------------------------------------------------------

typedef struct {
    unsigned n;
    const pio_program_t *tx, *rx;
    pio_sm_config (*tx_cfg)(uint);
    pio_sm_config (*rx_cfg)(uint);
    unsigned hi_entry, lo_emit, rx_wrap_target;
} prog_set_t;

static const prog_set_t progs[3] = {
    { 20, &link_tx_20_program, &link_rx_20_program, link_tx_20_program_get_default_config,
      link_rx_20_program_get_default_config, link_rx_20_offset_hi_entry, link_rx_20_offset_lo_emit,
      link_rx_20_wrap_target },
    { 10, &link_tx_10_program, &link_rx_10_program, link_tx_10_program_get_default_config,
      link_rx_10_program_get_default_config, link_rx_10_offset_hi_entry, link_rx_10_offset_lo_emit,
      link_rx_10_wrap_target },
    { 5, &link_tx_5_program, &link_rx_5_program, link_tx_5_program_get_default_config,
      link_rx_5_program_get_default_config, link_rx_5_offset_hi_entry, link_rx_5_offset_lo_emit,
      link_rx_5_wrap_target },
};

static uint32_t sys_hz;

// The program set for a class at the current clock, NULL if the clock has
// no integer N for it.
static const prog_set_t *progs_for(unsigned cls) {
    uint32_t baud = link_class_baud(cls);
    if (!baud || sys_hz % baud) return NULL;
    unsigned n = sys_hz / baud;
    for (unsigned i = 0; i < 3; i++) if (progs[i].n == n) return &progs[i];
    return NULL;
}

static PIO pio_n(unsigned i) {
#if NUM_PIOS > 2
    if (i == 2) return pio2;
#endif
    return i == 1 ? pio1 : pio0;
}

// ---------------------------------------------------------------------------
// Shared state (core 0 writes configuration, core 1 the real-time state)
// ---------------------------------------------------------------------------

#define JOB_IDLE    0u
#define JOB_TX      1u
#define JOB_RX      2u

static volatile uint32_t g_job, g_job_ack;

static uint32_t rx_ring[RX_RING_WORDS] __attribute__((aligned(4)));
static uint32_t tx_ring[TX_RING_WORDS] __attribute__((aligned(4)));
static const uint32_t *volatile rx_ring_addr = rx_ring;
static const uint32_t *volatile tx_ring_addr = tx_ring;

// TX.
static struct {
    bool on;
    bench_txcfg_t cfg;
    const prog_set_t *ps;
    PIO pio[BENCH_MAX_TX_PINS];
    uint sm[BENCH_MAX_TX_PINS], off[NUM_PIOS];
    bool loaded[NUM_PIOS];
    int dma_data[BENCH_MAX_TX_PINS], dma_ctrl[BENCH_MAX_TX_PINS];
    uint8_t drive;
} tx;
static bench_gen_t tx_gen;              // core 1 once running
static bench_sink_t tx_sink;
static uint32_t tx_last_idx, tx_passes;
static volatile uint64_t tx_read_words; // published by core 1
static volatile uint32_t tx_late, tx_stall;
static volatile int32_t imp_req_kind, imp_req_n;   // core 0 -> core 1 mailbox
static volatile uint64_t tx_frame_wire;
static volatile bench_imp_stats_t tx_imp;

// RX.
static struct {
    bool on;
    uint8_t pin;
    uint8_t cls;            // current program class
    bool auto_cls;
    int8_t trim;
    const prog_set_t *ps;
    PIO pio;
    uint sm, off;
    uint16_t ins[32];       // the loaded (patched) program, unrelocated
    pio_program_t prog;
    int dma_data, dma_ctrl;
} rx;
static uint32_t rx_last_idx, rx_passes;
static volatile uint32_t rx_seq;
static volatile uint64_t rx_words_pub;  // published by core 1 under rx_seq
static volatile uint32_t rx_stall;

static bench_bert_t bert;               // core 1
static uint32_t bert_gen;               // core 1: the reset generation applied
static volatile uint32_t reset_gen;     // core 0 bumps it to ask for a checker reset
static volatile uint64_t seam_words;    // core 0: ring position of the last program (re)load
static volatile bool stats_clear_req;
static volatile bool acq_req, acq_avail;
static bench_acq_t acq_result;
static volatile uint32_t acq_gen;       // reset generation the acquisition belongs to
static volatile uint32_t bert_seq, bert_gen_pub;
static bench_bert_stats_t bert_pub;     // copies under bert_seq
static volatile bool bert_synced_pub;
static volatile uint64_t bert_frame_pub, bert_pos_pub;
static link_sf_t bert_sf_pub;

static bench_ref_t ref;                 // core 0
static link_rx_t acq_rx;                // core 0 scratch
static uint32_t snap[(10u * LINK_MAX_FRAME_SYMBOLS * 68u + 31u) / 32u + 2u];
static bench_scan_t scan;
static bool scan_valid;
static uint32_t acq_tries;

// IMPAIR.
static struct {
    bool on;
    PIO pio;
    uint sm, off;
    uint8_t in, out;
    uint32_t div;
    uint32_t flip_cycles, flip_period_ms;
    uint32_t drop_us, drop_period_ms;
    uint64_t next_flip, next_drop;
    uint32_t flips, drops;
} imp;

// ---------------------------------------------------------------------------
// Hardware setup and teardown
// ---------------------------------------------------------------------------

// 307.2, 153.6 or 76.8 MHz from a 1536 MHz VCO (12 MHz x 128).
static void set_clock(unsigned mhz) {
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(2);
    set_sys_clock_pll(1536000000u, 5, mhz == 76 ? 4u : mhz == 153 ? 2u : 1u);
    sys_hz = clock_get_hz(clk_sys);
}

static void all_stop(void) {
    g_job = JOB_IDLE;
    while (g_job_ack != JOB_IDLE) tight_loop_contents();
    for (unsigned p = 0; p < NUM_PIOS; p++) {
        PIO pio = pio_n(p);
        for (uint sm = 0; sm < 4; sm++) {
            if (pio_sm_is_claimed(pio, sm)) {
                pio_sm_set_enabled(pio, sm, false);
                pio_sm_unclaim(pio, sm);
            }
        }
        pio_clear_instruction_memory(pio);
    }
    for (unsigned c = 0; c < NUM_DMA_CHANNELS; c++) {
        if (dma_channel_is_claimed(c)) {
            dma_channel_abort(c);
            dma_channel_unclaim(c);
        }
    }
    if (tx.on) for (unsigned i = 0; i < tx.cfg.npins; i++) gpio_init(tx.cfg.pins[i]);
    if (imp.on) gpio_init(imp.out);
    tx.on = rx.on = imp.on = false;
    memset(tx.loaded, 0, sizeof(tx.loaded));
    scan_valid = false;
}

static void input_pin(uint pin) {
    gpio_init(pin);                     // SIO function also clears the RP2350 pad ISO bit
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_input_enabled(pin, true);
    gpio_set_input_hysteresis_enabled(pin, true);
#if PICO_RP2350
    gpio_disable_pulls(pin);            // erratum RP2350-E9: a pad pull-down can latch the input high
#else
    gpio_pull_down(pin);                // an unwired input reads a steady low, not noise
#endif
}

static enum gpio_drive_strength drive_of(uint8_t ma) {
    switch (ma) {
    case 2: return GPIO_DRIVE_STRENGTH_2MA;
    case 8: return GPIO_DRIVE_STRENGTH_8MA;
    case 12: return GPIO_DRIVE_STRENGTH_12MA;
    default: return GPIO_DRIVE_STRENGTH_4MA;
    }
}

static void output_pin(PIO pio, uint sm, uint pin, uint8_t drive_ma) {
    pio_gpio_init(pio, pin);
    gpio_set_drive_strength(pin, drive_of(drive_ma));
    gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
}

// Two DMA channels: `data` moves count words between ring and FIFO and
// chains to `ctrl`, which rewrites data's ring address (with trigger), so the
// ring repeats forever without a ring-wrap alignment or a transfer-count
// limit.  `to_ring` selects RX (FIFO -> ring) or TX (ring -> FIFO).
static void dma_ring_pair(int data, int ctrl, bool to_ring, const uint32_t *const volatile *ring_addr,
                          uint32_t ring_words, volatile void *fifo, uint dreq) {
    dma_channel_config c = dma_channel_get_default_config(data);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, !to_ring);
    channel_config_set_write_increment(&c, to_ring);
    channel_config_set_dreq(&c, dreq);
    channel_config_set_chain_to(&c, ctrl);
    dma_channel_configure(data, &c, to_ring ? (void *)*ring_addr : (void *)fifo,
                          to_ring ? (const void *)fifo : (const void *)*ring_addr, ring_words, false);

    dma_channel_config k = dma_channel_get_default_config(ctrl);
    channel_config_set_transfer_data_size(&k, DMA_SIZE_32);
    channel_config_set_read_increment(&k, false);
    channel_config_set_write_increment(&k, false);
    dma_channel_configure(ctrl, &k,
                          to_ring ? (void *)&dma_hw->ch[data].al2_write_addr_trig
                                  : (void *)&dma_hw->ch[data].al3_read_addr_trig,
                          (const void *)ring_addr, 1, false);
}

// ---------------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------------

// Absolute words the first link's DMA has read (core 1).
static uint64_t tx_read_abs(void) {
    uint32_t idx = (dma_hw->ch[tx.dma_data[0]].read_addr - (uint32_t)tx_ring) >> 2;
    if (idx < tx_last_idx) tx_passes++;
    tx_last_idx = idx;
    return (uint64_t)tx_passes * TX_RING_WORDS + idx;
}

static void tx_fill(uint64_t read_abs) {
    while (tx_sink.words + 48u <= read_abs + TX_RING_WORDS - TX_MARGIN_WORDS) {
        int32_t k = imp_req_kind;
        if (k) {
            bench_gen_inject(&tx_gen, (unsigned)k, imp_req_n);
            imp_req_kind = 0;
        }
        bench_gen_frame(&tx_gen, &tx_sink);
    }
}

static void tx_service(void) {
    uint64_t r = tx_read_abs();
    if (r + 8u > tx_sink.words) tx_late = tx_late + 1u;   // the DMA caught up: stale words sent
    tx_fill(r);
    tx_read_words = r;
    unsigned fb = bench_gen_frame_bits(&tx_gen);
    uint64_t ahead_bits = (tx_sink.words - r) * 32u;
    tx_frame_wire = tx_gen.frame - ahead_bits / fb;
    tx_imp = tx_gen.imp;
    if (stats_clear_req && !(g_job & JOB_RX)) {
        memset((void *)&tx_gen.imp, 0, sizeof(tx_gen.imp));
        stats_clear_req = false;
    }
    for (unsigned i = 0; i < tx.cfg.npins; i++) {
        uint32_t bit = 1u << (PIO_FDEBUG_TXSTALL_LSB + tx.sm[i]);
        if (tx.pio[i]->fdebug & bit) {
            tx.pio[i]->fdebug = bit;
            tx_stall = tx_stall + 1u;
        }
    }
}

static const char *tx_start(const bench_txcfg_t *c, uint8_t drive, unsigned skip_pio) {
    tx.ps = progs_for(c->cls);
    if (!tx.ps) return "class not available at this clock";
    if (c->npins > MAX_LINKS) return "too many links";
    tx.cfg = *c;
    tx.drive = drive;
    // SMs: fill PIO blocks in order, skipping the RX block in loopback.
    unsigned p = 0;
    for (unsigned i = 0; i < c->npins; i++) {
        int sm = -1;
        while (p < NUM_PIOS) {
            if (p != skip_pio) {
                sm = pio_claim_unused_sm(pio_n(p), false);
                if (sm >= 0) break;
            }
            p++;
        }
        if (sm < 0) return "no free state machine";
        PIO pio = pio_n(p);
        if (!tx.loaded[p]) {
            if (!pio_can_add_program(pio, tx.ps->tx)) return "no PIO instruction space";
            tx.off[p] = pio_add_program(pio, tx.ps->tx);
            tx.loaded[p] = true;
        }
        tx.pio[i] = pio;
        tx.sm[i] = (uint)sm;
        pio_sm_config cfg = tx.ps->tx_cfg(tx.off[p]);
        sm_config_set_out_pins(&cfg, c->pins[i], 1);
        sm_config_set_out_shift(&cfg, false, true, 32);
        sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv_int_frac8(&cfg, 1, 0);
        pio_sm_init(pio, (uint)sm, tx.off[p], &cfg);
        output_pin(pio, (uint)sm, c->pins[i], drive);
    }
    // The stream, prefilled.
    if (!bench_gen_init(&tx_gen, c->cls, c->rate, c->channels, c->pay, c->f0, c->sender)) return "stream";
    bench_sink_init(&tx_sink, tx_ring, TX_RING_WORDS);
    tx_last_idx = 0;
    tx_passes = 0;
    tx_late = tx_stall = 0;
    imp_req_kind = 0;
    while (tx_sink.words + 48u <= TX_RING_WORDS - TX_MARGIN_WORDS) bench_gen_frame(&tx_gen, &tx_sink);
    // DMA, then all SMs at once.
    uint32_t dma_mask = 0;
    for (unsigned i = 0; i < c->npins; i++) {
        tx.dma_data[i] = dma_claim_unused_channel(true);
        tx.dma_ctrl[i] = dma_claim_unused_channel(true);
        dma_ring_pair(tx.dma_data[i], tx.dma_ctrl[i], false, &tx_ring_addr, TX_RING_WORDS,
                      &tx.pio[i]->txf[tx.sm[i]], pio_get_dreq(tx.pio[i], tx.sm[i], true));
        dma_mask |= 1u << tx.dma_data[i];
    }
    dma_start_channel_mask(dma_mask);
    uint32_t sm_mask[NUM_PIOS] = {0};
    for (unsigned i = 0; i < c->npins; i++) {
        PIO pio = tx.pio[i];
        for (unsigned q = 0; q < NUM_PIOS; q++) if (pio_n(q) == pio) sm_mask[q] |= 1u << tx.sm[i];
        while (pio_sm_get_tx_fifo_level(pio, tx.sm[i]) < 8u) tight_loop_contents();
    }
    for (unsigned i = 0; i < c->npins; i++) tx.pio[i]->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + tx.sm[i]);
#if NUM_PIOS > 2
    pio_enable_sm_multi_mask_in_sync(pio1, sm_mask[0], sm_mask[1], sm_mask[2]);
#else
    // The RP2040 cannot start two blocks on the same cycle; links on pio1
    // start a few cycles after those on pio0 (a constant skew).
    if (sm_mask[0]) pio_enable_sm_mask_in_sync(pio0, sm_mask[0]);
    if (sm_mask[1]) pio_enable_sm_mask_in_sync(pio1, sm_mask[1]);
#endif
    tx.on = true;
    return NULL;
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------

// Absolute words the RX DMA has written (core 1 only).
static uint64_t rx_write_abs(void) {
    uint32_t idx = (dma_hw->ch[rx.dma_data].write_addr - (uint32_t)rx_ring) >> 2;
    if (idx < rx_last_idx) rx_passes++;
    rx_last_idx = idx;
    return (uint64_t)rx_passes * RX_RING_WORDS + idx;
}

// The last value core 1 published (core 0).
static uint64_t rx_words_now(void) {
    uint32_t s0, s1;
    uint64_t v;
    do {
        s0 = rx_seq;
        __dmb();
        v = rx_words_pub;
        __dmb();
        s1 = rx_seq;
    } while (s0 != s1 || (s0 & 1u));
    return v;
}

static void ring_window(uint64_t words, uint64_t *oldest, uint64_t *newest) {
    *newest = words * 32u;
    *oldest = words > RX_RING_WORDS - RX_GUARD_WORDS ? (words - RX_RING_WORDS + RX_GUARD_WORDS) * 32u : 0;
}

static void bert_publish(void) {
    bert_seq = bert_seq + 1u;
    __dmb();
    bert_pub = bert.st;
    bert_synced_pub = bert.synced;
    bert_frame_pub = bert.gen.frame;
    bert_pos_pub = bert.pos;
    bert_sf_pub.flags = bert.gen.flags;
    bert_sf_pub.rate = bert.gen.rate;
    bert_sf_pub.channels = bert.gen.channels;
    bert_sf_pub.cls = bert.gen.cls;
    bert_sf_pub.hops = bert.gen.hops;
    bert_sf_pub.sender = bert.gen.sender;
    bert_gen_pub = bert_gen;
    __dmb();
    bert_seq = bert_seq + 1u;
}

static void rx_service(void) {
    uint64_t w = rx_write_abs();
    rx_seq = rx_seq + 1u;
    __dmb();
    rx_words_pub = w;
    __dmb();
    rx_seq = rx_seq + 1u;
    uint32_t bit = 1u << (PIO_FDEBUG_RXSTALL_LSB + rx.sm);
    if (rx.pio->fdebug & bit) {
        rx.pio->fdebug = bit;
        rx_stall = rx_stall + 1u;
    }
    uint32_t g = reset_gen;
    if (g != bert_gen) {
        bench_bert_init(&bert);
        acq_avail = false;
        bert_gen = g;
    }
    if (stats_clear_req) {
        bool s = bert.synced;
        memset(&bert.st, 0, sizeof(bert.st));
        bert.synced = s;
        memset((void *)&tx_gen.imp, 0, sizeof(tx_gen.imp));
        stats_clear_req = false;
    }
    if (!bert.synced) {
        if (acq_avail) {
            if (acq_gen == bert_gen) bench_bert_start(&bert, &acq_result);
            acq_avail = false;
            acq_req = false;
        } else {
            acq_req = true;
        }
    } else {
        uint64_t oldest, newest;
        ring_window(w, &oldest, &newest);
        bench_bert_run(&bert, &(bench_ring_t){ .ring = rx_ring, .mask = RX_RING_WORDS - 1u }, oldest, newest, 24);
    }
    bert_publish();
}

// *synced is true only once core 1 has applied the latest reset.
static void rx_bert_snapshot(bench_bert_stats_t *st, bool *synced, uint64_t *frame, link_sf_t *sf) {
    uint32_t s0, s1;
    do {
        s0 = bert_seq;
        __dmb();
        *st = bert_pub;
        *synced = bert_synced_pub && bert_gen_pub == reset_gen;
        *frame = bert_frame_pub;
        if (sf) *sf = bert_sf_pub;
        __dmb();
        s1 = bert_seq;
    } while (s0 != s1 || (s0 & 1u));
}

// Load the RX program for rx.cls with trim t into rx.pio at rx.off and
// (re)start the SM.  The DMA keeps running; the bit stream simply has a
// seam, which the checker re-acquires across.
static bool rx_load(unsigned cls, int t) {
    const prog_set_t *ps = progs_for(cls);
    if (!ps) return false;
    uint16_t ins[32];
    memcpy(ins, ps->rx->instructions, ps->rx->length * sizeof(uint16_t));
    if (!bench_trim_patch(ins, ps->rx->length, ps->hi_entry, ps->lo_emit, t)) return false;
    pio_sm_set_enabled(rx.pio, rx.sm, false);
    if (rx.ps) pio_remove_program(rx.pio, &rx.prog, rx.off);
    memcpy(rx.ins, ins, sizeof(ins));
    rx.prog = *ps->rx;
    rx.prog.instructions = rx.ins;
    rx.prog.origin = -1;
    if (!pio_can_add_program(rx.pio, &rx.prog)) return false;
    rx.off = pio_add_program(rx.pio, &rx.prog);
    rx.ps = ps;
    rx.cls = (uint8_t)cls;
    rx.trim = (int8_t)t;
    pio_sm_config cfg = ps->rx_cfg(rx.off);
    sm_config_set_jmp_pin(&cfg, rx.pin);
    sm_config_set_in_shift(&cfg, false, true, 32);
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac8(&cfg, 1, 0);
    pio_sm_init(rx.pio, rx.sm, rx.off + ps->rx_wrap_target, &cfg);
    pio_sm_exec(rx.pio, rx.sm, pio_encode_mov_not(pio_osr, pio_null));
    pio_sm_set_enabled(rx.pio, rx.sm, true);
    // Data before this point came from the previous program: acquisitions
    // must start after it, and the checker starts over.
    seam_words = rx_words_now() + 8u;
    reset_gen = reset_gen + 1u;
    return true;
}

static const char *rx_start(unsigned pin, unsigned cls, bool auto_cls, unsigned pio_idx) {
    if (!progs_for(cls)) return "class not available at this clock";
    rx.pin = (uint8_t)pin;
    rx.auto_cls = auto_cls;
    rx.pio = pio_n(pio_idx);
    int sm = pio_claim_unused_sm(rx.pio, false);
    if (sm < 0) return "no free state machine";
    rx.sm = (uint)sm;
    rx.ps = NULL;
    input_pin(pin);
    memset(rx_ring, 0, sizeof(rx_ring));
    rx_last_idx = 0;
    rx_passes = 0;
    rx_stall = 0;
    rx_words_pub = 0;
    rx.dma_data = dma_claim_unused_channel(true);
    rx.dma_ctrl = dma_claim_unused_channel(true);
    dma_ring_pair(rx.dma_data, rx.dma_ctrl, true, &rx_ring_addr, RX_RING_WORDS, (volatile void *)&rx.pio->rxf[rx.sm],
                  pio_get_dreq(rx.pio, rx.sm, false));
    dma_channel_start(rx.dma_data);
    bench_bert_init(&bert);
    bert_gen = reset_gen;
    acq_req = acq_avail = false;
    acq_tries = 0;
    scan_valid = false;
    if (!rx_load(cls, 0)) return "program";
    bench_ref_init(&ref, cls, BENCH_PAY_PRBS);
    rx.on = true;
    return NULL;
}

// Core 0: run an acquisition when core 1 asks for one, on data received
// after the last program load.  In auto-class mode a failed attempt moves
// on to the next class the clock allows.
static void acquisition_service(void) {
    if (!rx.on || !acq_req || acq_avail || bert_gen_pub != reset_gen) return;
    uint32_t gen = reset_gen;
    uint32_t need = bench_acquire_bits(rx.cls);
    uint64_t words = rx_words_now();
    uint64_t w0 = words > need / 32u + 1u ? words - need / 32u - 1u : 0;
    if (w0 < seam_words) return;
    uint32_t nw = (uint32_t)(words - w0);
    for (uint32_t i = 0; i < nw; i++) snap[i] = rx_ring[(uint32_t)(w0 + i) & (RX_RING_WORDS - 1u)];
    // The writer must not have lapped the copy.
    if (rx_words_now() - w0 > RX_RING_WORDS - RX_GUARD_WORDS) return;
    acq_tries++;
    bench_acq_t a;
    if (bench_acquire(snap, w0 * 32u, (size_t)nw * 32u, rx.cls, &acq_rx, &a) && a.sf.apply_op == LINK_OP_NONE) {
        if (gen != reset_gen) return;
        acq_result = a;
        acq_gen = gen;
        __dmb();
        acq_avail = true;
        if (ref.cls != rx.cls) bench_ref_init(&ref, rx.cls, BENCH_PAY_PRBS);
        return;
    }
    if (rx.auto_cls) {
        unsigned next = rx.cls;
        for (unsigned k = 0; k < 3; k++) {
            next = next % 3u + 1u;
            if (progs_for(next)) break;
        }
        if (next != rx.cls) rx_load(next, 0);
    }
}

// Core 0: feed the reference receiver for about `budget_us`.
static void ref_service(uint32_t budget_us) {
    if (!rx.on || ref.cls != rx.cls) return;
    bench_ring_t ring = { .ring = rx_ring, .mask = RX_RING_WORDS - 1u };
    uint64_t t0 = time_us_64();
    while (time_us_64() - t0 < budget_us) {
        uint64_t oldest, newest;
        ring_window(rx_words_now(), &oldest, &newest);
        size_t n = bench_ref_run(&ref, &ring, oldest, newest, 2048);
        if (!n) break;
        ring_window(rx_words_now(), &oldest, &newest);
        if (ref.pos - n < oldest) bench_ref_restart(&ref);   // overtaken while decoding: data suspect
    }
}

// ---------------------------------------------------------------------------
// IMPAIR
// ---------------------------------------------------------------------------

static const char *impair_start(unsigned in, unsigned out, unsigned div, uint8_t drive) {
    imp.pio = pio0;
    int sm = pio_claim_unused_sm(imp.pio, false);
    if (sm < 0) return "no free state machine";
    imp.sm = (uint)sm;
    imp.off = pio_add_program(imp.pio, &bench_retime_program);
    imp.in = (uint8_t)in;
    imp.out = (uint8_t)out;
    imp.div = div;
    imp.flip_cycles = imp.flip_period_ms = imp.drop_us = imp.drop_period_ms = 0;
    imp.flips = imp.drops = 0;
    input_pin(in);
    pio_sm_config cfg = bench_retime_program_get_default_config(imp.off);
    sm_config_set_in_pins(&cfg, in);
    sm_config_set_out_pins(&cfg, out, 1);
    sm_config_set_clkdiv_int_frac8(&cfg, div, 0);
    pio_sm_init(imp.pio, imp.sm, imp.off + bench_retime_offset_pass, &cfg);
    output_pin(imp.pio, imp.sm, out, drive);
    pio_sm_set_enabled(imp.pio, imp.sm, true);
    imp.on = true;
    return NULL;
}

static void impair_burst(unsigned label, uint32_t cycles) {
    uint32_t save = save_and_disable_interrupts();
    pio_sm_exec(imp.pio, imp.sm, pio_encode_jmp(imp.off + label));
    busy_wait_at_least_cycles(cycles);
    pio_sm_exec(imp.pio, imp.sm, pio_encode_jmp(imp.off + bench_retime_offset_pass));
    restore_interrupts(save);
}

static void impair_service(void) {
    if (!imp.on) return;
    uint64_t now = time_us_64();
    if (imp.flip_period_ms && now >= imp.next_flip) {
        impair_burst(bench_retime_offset_inv, imp.flip_cycles);
        imp.flips++;
        imp.next_flip = now + (uint64_t)imp.flip_period_ms * 1000u;
    }
    if (imp.drop_period_ms && now >= imp.next_drop) {
        impair_burst(bench_retime_offset_low, (uint32_t)((uint64_t)imp.drop_us * (sys_hz / 1000000u)));
        imp.drops++;
        imp.next_drop = now + (uint64_t)imp.drop_period_ms * 1000u;
    }
}

// ---------------------------------------------------------------------------
// Core 1
// ---------------------------------------------------------------------------

static void core1_main(void) {
    for (;;) {
        uint32_t job = g_job;
        g_job_ack = job;
        if (job == JOB_IDLE) {
            tight_loop_contents();
            continue;
        }
        while (g_job == job) {
            if (job & JOB_TX) tx_service();
            if (job & JOB_RX) rx_service();
        }
    }
}

static void start_job(uint32_t job) {
    g_job = job;
    while (g_job_ack != job) tight_loop_contents();
}

// ---------------------------------------------------------------------------
// Reports
// ---------------------------------------------------------------------------

static const char *role_name(void) {
    if (imp.on) return "impair";
    if (tx.on && rx.on) return "loop";
    if (tx.on) return "tx";
    if (rx.on) return "rx";
    return "idle";
}

static void print_stat(void) {
    printf("STAT role=%s clock=%lu up_ms=%llu", role_name(), (unsigned long)sys_hz,
           (unsigned long long)(time_us_64() / 1000u));
    if (tx.on) {
        bench_imp_stats_t im;
        memcpy(&im, (const void *)&tx_imp, sizeof(im));
        printf(" tx_cls=%s tx_rate=%s tx_ch=%u tx_pay=%s tx_links=%u tx_frame=%llu tx_words=%llu tx_late=%lu"
               " tx_stall=%lu imp_comma=%lu imp_flip=%lu imp_slip=%lu imp_slip_bits=%ld imp_idle=%lu",
               bench_class_name(tx.cfg.cls), bench_rate_name(tx.cfg.rate), tx.cfg.channels,
               tx.cfg.pay == BENCH_PAY_PRBS ? "prbs" : "vec", tx.cfg.npins, (unsigned long long)tx_frame_wire,
               (unsigned long long)tx_read_words, (unsigned long)tx_late, (unsigned long)tx_stall,
               (unsigned long)im.comma, (unsigned long)im.flip, (unsigned long)im.slip, (long)im.slip_bits,
               (unsigned long)im.idle);
    }
    if (rx.on) {
        bench_bert_stats_t b;
        bool synced;
        uint64_t frame;
        link_sf_t sf;
        rx_bert_snapshot(&b, &synced, &frame, &sf);
        link_rx_stats_t s;
        bench_ref_stats(&ref, &s);
        printf(" rx_pin=%u rx_cls=%s trim=%d sync=%d frame=%llu rate=%s ch=%u sf_flags=0x%02x sender=%u"
               " rx_words=%llu rx_stall=%lu acq_tries=%lu",
               rx.pin, bench_class_name(rx.cls), rx.trim, synced, (unsigned long long)frame,
               synced ? bench_rate_name(sf.rate) : "none", synced ? sf.channels : 0u, sf.flags, sf.sender,
               (unsigned long long)rx_words_now(), (unsigned long)rx_stall, (unsigned long)acq_tries);
        printf(" bert_bits=%llu bert_errors=%llu bert_frames=%llu bert_err_frames=%llu bert_bad_frames=%llu"
               " bert_skipped=%llu bert_slips=%lu bert_slip_bits=%ld bert_syncs=%lu bert_losses=%lu",
               (unsigned long long)b.bits, (unsigned long long)b.errors, (unsigned long long)b.frames,
               (unsigned long long)b.err_frames, (unsigned long long)b.bad_frames,
               (unsigned long long)b.skipped_frames, (unsigned long)b.slips, (long)b.slip_bits,
               (unsigned long)b.syncs, (unsigned long)b.losses);
        printf(" ref_bits=%llu ref_gaps=%lu ref_frames=%lu lock=%lu loss=%lu crc8=%lu crc16=%lu slips=%lu"
               " sf_err=%lu comma=%lu counter=%lu code=%lu idle=%lu samples_bad=%lu",
               (unsigned long long)ref.bits, (unsigned long)ref.gaps, (unsigned long)s.frames,
               (unsigned long)s.lock_count, (unsigned long)s.loss_count, (unsigned long)s.crc8_errors,
               (unsigned long)s.crc16_errors, (unsigned long)s.bit_slips, (unsigned long)s.superframe_errors,
               (unsigned long)s.comma_errors, (unsigned long)s.counter_errors, (unsigned long)s.code_errors,
               (unsigned long)s.idle_errors, (unsigned long)ref.sample_errors);
        if (tx.on) printf(" fdelta=%lld", (long long)(tx_frame_wire - frame));
    }
    if (imp.on) {
        printf(" imp_in=%u imp_out=%u imp_div=%lu imp_flips=%lu imp_drops=%lu", imp.in, imp.out,
               (unsigned long)imp.div, (unsigned long)imp.flips, (unsigned long)imp.drops);
    }
    printf("\n");
}

static void fill_status(bench_status_t *st) {
    memset(st, 0, sizeof(*st));
    bench_bert_stats_t b;
    bool synced;
    uint64_t frame;
    link_sf_t sf;
    rx_bert_snapshot(&b, &synced, &frame, &sf);
    st->state = rx.on ? (synced ? 2u : 1u) : 0u;
    st->cls = synced ? sf.cls : 0u;
    st->rate = sf.rate;
    st->channels = sf.channels;
    st->hops = sf.hops;
    st->sender = sf.sender;
    st->sf_flags = sf.flags;
    st->frame = frame;
    bench_ref_stats(&ref, &st->st);
}

static void print_layouts(bool diag) {
    char hex[2 * BENCH_DIAG_LEN + 1];
    bench_status_t st;
    fill_status(&st);
    if (!diag) {
        uint8_t o[BENCH_STATUS_LEN];
        bench_pack_status(o, &st);
        bench_hex(hex, o, sizeof(o));
        printf("XSTATUS %s\n", hex);
        return;
    }
    bench_scan_t forced;
    bench_diag_t d = { .comma_offset = (uint8_t)(bert_pos_pub & 31u), .st = st.st,
                       .ring_wr_bytes = (uint32_t)((rx_words_now() % RX_RING_WORDS) * 4u),
                       .parser_rd_bytes = (uint32_t)(((ref.pos / 32u) % RX_RING_WORDS) * 4u),
                       .tx_words = tx.on ? (uint32_t)tx_read_words : 0u,
                       .rx_xfers = (uint32_t)rx_words_now() };
    if (scan_valid) {
        d.scan = &scan;
    } else {
        memset(&forced, 0, sizeof(forced));
        forced.used = rx.trim;
        d.scan = &forced;
    }
    uint8_t o[BENCH_DIAG_LEN];
    bench_pack_diag(o, &d);
    bench_hex(hex, o, sizeof(o));
    printf("DIAG %s\n", hex);
    printf("DIAGF comma_offset=%u trim_lo=%d trim_hi=%d trim_used=%d scan_state=%u clean_mask=0x%05lx"
           " comma=%lu counter=%lu code=%lu idle=%lu\n",
           o[1], (int8_t)o[32], (int8_t)o[33], (int8_t)o[34], o[35], (unsigned long)d.scan->clean_mask,
           (unsigned long)st.st.comma_errors, (unsigned long)st.st.counter_errors,
           (unsigned long)st.st.code_errors, (unsigned long)st.st.idle_errors);
}

// ---------------------------------------------------------------------------
// Trim scan (blocking; core 1 keeps checking, core 0 acquires)
// ---------------------------------------------------------------------------

static bool wait_sync(uint32_t timeout_ms) {
    uint64_t t_end = time_us_64() + (uint64_t)timeout_ms * 1000u;
    while (time_us_64() < t_end) {
        acquisition_service();
        bench_bert_stats_t b;
        bool synced;
        uint64_t frame;
        rx_bert_snapshot(&b, &synced, &frame, NULL);
        if (synced) return true;
    }
    return false;
}

static void run_scan(uint32_t dwell_ms) {
    int tmax = bench_trim_max(rx.ps->rx->instructions, rx.ps->rx->length, rx.ps->hi_entry, rx.ps->lo_emit);
    bench_scan_begin(&scan, tmax);
    scan_valid = true;
    int t;
    while (bench_scan_next(&scan, &t)) {
        rx_load(rx.cls, t);
        bool ok = wait_sync(300);
        bench_bert_stats_t b0, b1;
        bool synced;
        uint64_t frame;
        uint32_t errs = 0xFFFFFFFFu;
        bool clean = false;
        if (ok) {
            rx_bert_snapshot(&b0, &synced, &frame, NULL);
            uint64_t t_end = time_us_64() + (uint64_t)dwell_ms * 1000u;
            while (time_us_64() < t_end) tight_loop_contents();
            rx_bert_snapshot(&b1, &synced, &frame, NULL);
            errs = (uint32_t)(b1.errors - b0.errors);
            clean = synced && b1.losses == b0.losses && b1.slips == b0.slips && errs == 0 && b1.bits > b0.bits;
        }
        printf("SCANSTEP trim=%d sync=%d clean=%d errors=%ld bits=%llu\n", t, ok, clean,
               ok ? (long)errs : -1L, ok ? (unsigned long long)(b1.bits - b0.bits) : 0ull);
        bench_scan_record(&scan, clean, errs);
    }
    bench_scan_finish(&scan);
    rx_load(rx.cls, scan.used);
    wait_sync(300);
    printf("SCAN state=%u lo=%d hi=%d used=%d mask=0x%05lx cls=%s\n", scan.state, scan.lo, scan.hi, scan.used,
           (unsigned long)scan.clean_mask, bench_class_name(rx.cls));
}

// ---------------------------------------------------------------------------
// Console
// ---------------------------------------------------------------------------

// WP LK5: the RX ring as the decode benchmark sees it.
static uint64_t live_writer_bits(void) { return rx_words_now() * 32u; }

static void print_id(void) {
    char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(serial, sizeof(serial));
    printf("ID fw=link_bench ver=%s chip=%s clock=%lu serial=%s max_links=%u classes=%s%s%s\n", BENCH_GIT, CHIP_NAME,
           (unsigned long)sys_hz, serial, (unsigned)MAX_LINKS, progs_for(1) ? "L1" : "", progs_for(2) ? ",L2" : "",
           progs_for(3) ? ",L3" : "");
}

static void print_help(void) {
    printf("OK commands: id | help | clock 307|153|76 | stop | stat | reset | diag | xstatus\n"
           "OK  tx <L1|L2|L3> <44k1|48k|96k> [ch=C] [pay=prbs|vec] [pins=2,3] [f0=N] [sender=N] [drive=2|4|8|12]\n"
           "OK  rx [pin=0] [cls=auto|L1|L2|L3]\n"
           "OK  loop <cls> <rate> [ch=] [pay=] [txpins=2] [rxpin=0] [drive=]\n"
           "OK  inject comma|flip|idle <n> | inject slip <-3..3>\n"
           "OK  trim <t> | scan [dwell_ms]\n"
           "OK  impair [in=0] [out=2] [div=1] [drive=4] | impair flip <cycles> <period_ms> | impair drop <us> <period_ms> | impair off\n"
           "OK  decode all [crc=dma|sw] | decode canned <cls> <rate> [ch=C] [mask=M] [slot=S] [crc=dma|sw] [runs=N] [batch=F]\n"
           "OK  decode live [mask=M] [slot=S] [crc=dma|sw] [ms=T] [batch=F]   (LK5 receiver cycles per frame)\n");
}

static void reply(const char *err) {
    if (err) printf("ERR %s\n", err);
    else printf("OK\n");
}

static uint8_t parse_drive(const bench_args_t *a) {
    long v;
    const char *s = bench_kv(a, "drive");
    if (s && bench_parse_int(s, 2, 12, &v) && (v == 2 || v == 4 || v == 8 || v == 12)) return (uint8_t)v;
    return 4;
}

static void cmd(char *line) {
    bench_args_t a;
    if (!bench_split(line, &a)) return;
    const char *c = a.argv[0], *err = NULL, *s;
    long v;
    if (!strcmp(c, "id")) { print_id(); return; }
    if (!strcmp(c, "help")) { print_help(); return; }
    if (!strcmp(c, "stat")) { print_stat(); return; }
    if (!strcmp(c, "diag")) { print_layouts(true); return; }
    if (!strcmp(c, "xstatus")) { print_layouts(false); return; }
    if (!strcmp(c, "stop")) { all_stop(); reply(NULL); return; }
    if (!strcmp(c, "reset")) {
        stats_clear_req = true;
        uint64_t t_end = time_us_64() + 100000u;
        while (stats_clear_req && g_job != JOB_IDLE && time_us_64() < t_end) tight_loop_contents();
        stats_clear_req = false;
        uint8_t cls = ref.cls;
        bench_ref_init(&ref, cls, BENCH_PAY_PRBS);
        reply(NULL);
        return;
    }
    if (!strcmp(c, "clock")) {
        long mhz;
        if (a.argc < 2 || !bench_parse_int(a.argv[1], 76, 307, &mhz) || (mhz != 307 && mhz != 153 && mhz != 76)) {
            reply("clock 307|153|76");
            return;
        }
        all_stop();
        set_clock((unsigned)mhz);
        print_id();
        return;
    }
    if (!strcmp(c, "tx") || !strcmp(c, "loop")) {
        bool loop = !strcmp(c, "loop");
        bench_txcfg_t cfg;
        if (!bench_parse_tx(&a, 1, loop ? "txpins" : "pins", &cfg, &err)) { printf("ERR bad %s\n", err); return; }
        long rxpin = 0;
        if (loop && (s = bench_kv(&a, "rxpin")) && (!bench_parse_int(s, 0, 28, &rxpin) || !bench_pin_ok((unsigned)rxpin))) {
            reply("bad rxpin");
            return;
        }
        if (loop) for (unsigned i = 0; i < cfg.npins; i++) if (cfg.pins[i] == rxpin) { reply("rx pin is a tx pin"); return; }
        all_stop();
        unsigned rx_pio = NUM_PIOS - 1u;
        err = tx_start(&cfg, parse_drive(&a), loop ? rx_pio : NUM_PIOS);
        if (!err && loop) err = rx_start((unsigned)rxpin, cfg.cls, false, rx_pio);
        if (err) { all_stop(); reply(err); return; }
        start_job(loop ? (JOB_TX | JOB_RX) : JOB_TX);
        reply(NULL);
        return;
    }
    if (!strcmp(c, "rx")) {
        long pin = 0;
        unsigned cls = LINK_CLASS_L1;
        bool aut = true;
        if ((s = bench_kv(&a, "pin")) && (!bench_parse_int(s, 0, 28, &pin) || !bench_pin_ok((unsigned)pin))) {
            reply("bad pin");
            return;
        }
        if ((s = bench_kv(&a, "cls")) && strcmp(s, "auto")) {
            if (!bench_parse_class(s, &cls)) { reply("bad cls"); return; }
            aut = false;
        }
        all_stop();
        err = rx_start((unsigned)pin, cls, aut, NUM_PIOS - 1u);
        if (err) { all_stop(); reply(err); return; }
        start_job(JOB_RX);
        reply(NULL);
        return;
    }
    if (!strcmp(c, "decode")) {
        bench_decode_live_t live = { rx.on ? rx_ring : NULL, RX_RING_WORDS, rx.cls, live_writer_bits };
        bench_decode_cmd(&a, &live, sys_hz);
        return;
    }
    if (!strcmp(c, "inject")) {
        if (!tx.on) { reply("not transmitting"); return; }
        unsigned kind = 0;
        if (a.argc >= 3) {
            if (!strcmp(a.argv[1], "comma")) kind = BENCH_IMP_COMMA;
            else if (!strcmp(a.argv[1], "flip")) kind = BENCH_IMP_FLIP;
            else if (!strcmp(a.argv[1], "slip")) kind = BENCH_IMP_SLIP;
            else if (!strcmp(a.argv[1], "idle")) kind = BENCH_IMP_IDLE;
        }
        if (!kind || !bench_parse_int(a.argv[2], -1000, 1000, &v)) { reply("inject comma|flip|idle <n> | slip <-3..3>"); return; }
        bench_gen_t probe;
        memset(&probe, 0, sizeof(probe));
        if (!bench_gen_inject(&probe, kind, (int)v)) { reply("bad count"); return; }
        while (imp_req_kind) tight_loop_contents();
        imp_req_n = (int32_t)v;
        __dmb();
        imp_req_kind = (int32_t)kind;
        reply(NULL);
        return;
    }
    if (!strcmp(c, "trim")) {
        if (!rx.on) { reply("not receiving"); return; }
        int tmax = bench_trim_max(rx.ps->rx->instructions, rx.ps->rx->length, rx.ps->hi_entry, rx.ps->lo_emit);
        if (a.argc < 2 || !bench_parse_int(a.argv[1], -tmax, tmax, &v)) { printf("ERR trim -%d..%d\n", tmax, tmax); return; }
        scan_valid = false;
        reply(rx_load(rx.cls, (int)v) ? NULL : "trim");
        return;
    }
    if (!strcmp(c, "scan")) {
        if (!rx.on) { reply("not receiving"); return; }
        long dwell = 50;
        if (a.argc >= 2 && !bench_parse_int(a.argv[1], 5, 10000, &dwell)) { reply("scan [dwell_ms 5..10000]"); return; }
        if (!wait_sync(1000)) { reply("no sync"); return; }
        run_scan((uint32_t)dwell);
        return;
    }
    if (!strcmp(c, "impair")) {
        if (a.argc >= 2 && !strcmp(a.argv[1], "off")) { all_stop(); reply(NULL); return; }
        if (a.argc >= 4 && (!strcmp(a.argv[1], "flip") || !strcmp(a.argv[1], "drop"))) {
            long n, period;
            if (!imp.on) { reply("impair not running"); return; }
            if (!bench_parse_int(a.argv[2], 0, 1000000, &n) || !bench_parse_int(a.argv[3], 0, 100000, &period)) {
                reply("impair flip <cycles> <period_ms> | drop <us> <period_ms>");
                return;
            }
            if (a.argv[1][0] == 'f') { imp.flip_cycles = (uint32_t)n; imp.flip_period_ms = (uint32_t)period; }
            else { imp.drop_us = (uint32_t)n; imp.drop_period_ms = (uint32_t)period; }
            imp.next_flip = imp.next_drop = time_us_64();
            reply(NULL);
            return;
        }
        long in = 0, out = 2, div = 1;
        if (((s = bench_kv(&a, "in")) && (!bench_parse_int(s, 0, 28, &in) || !bench_pin_ok((unsigned)in))) ||
            ((s = bench_kv(&a, "out")) && (!bench_parse_int(s, 0, 28, &out) || !bench_pin_ok((unsigned)out))) ||
            ((s = bench_kv(&a, "div")) && !bench_parse_int(s, 1, 255, &div)) || in == out) {
            reply("impair [in=0] [out=2] [div=1..255] [drive=]");
            return;
        }
        all_stop();
        err = impair_start((unsigned)in, (unsigned)out, (unsigned)div, parse_drive(&a));
        if (err) { all_stop(); reply(err); return; }
        reply(NULL);
        return;
    }
    reply("unknown command (help)");
}

static void events_poll(void) {
    static bool was_synced;
    static uint32_t last_losses;
    if (!rx.on) { was_synced = false; return; }
    bench_bert_stats_t b;
    bool synced;
    uint64_t frame;
    link_sf_t sf;
    rx_bert_snapshot(&b, &synced, &frame, &sf);
    if (synced && !was_synced)
        printf("EVT sync cls=%s rate=%s ch=%u frame=%llu\n", bench_class_name(sf.cls), bench_rate_name(sf.rate),
               sf.channels, (unsigned long long)frame);
    if (b.losses != last_losses && !synced) printf("EVT loss frame=%llu\n", (unsigned long long)frame);
    was_synced = synced;
    last_losses = b.losses;
}

static void led_service(void) {
    bool on;
    uint32_t ms = (uint32_t)(time_us_64() / 1000u);
    if (rx.on) on = bert_synced_pub ? true : (ms / 100u) & 1u;
    else if (tx.on || imp.on) on = (ms / 500u) & 1u;
    else on = (ms % 2000u) < 50u;
    gpio_put(LED_PIN, on);
}

int main(void) {
    set_clock(307);
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    busctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
    multicore_launch_core1(core1_main);
    char line[160];
    unsigned n = 0;
    for (;;) {
        int ch = getchar_timeout_us(0);
        while (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\n' || ch == '\r') {
                line[n] = 0;
                if (n) cmd(line);
                n = 0;
            } else if (n + 1u < sizeof(line)) {
                line[n++] = (char)ch;
            }
            ch = getchar_timeout_us(0);
        }
        acquisition_service();
        ref_service(2000);
        events_poll();
        impair_service();
        led_service();
    }
}
