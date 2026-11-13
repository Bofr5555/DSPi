/*
 * link_input.c - the multi-unit audio link receiver: INPUT_SOURCE_LINK (WP LK5)
 *
 * See link_input.h.  Hardware: PIO2 SM0 (link_rx_20/_10/_5 with the LK2 trim
 * patched in), DMA 5 (FIFO to ring) chained to DMA 6 (rewrites 5's write
 * address, so the 32 KB ring repeats without a ring-wrap alignment), DMA 15
 * for the CRC-16 sniffer.  All of them belong to receivers that are stopped
 * while LINK is the source (S/PDIF and I2S RX: PIO2, DMA 5/6; ADAT RX: 15).
 *
 * Only the decode service and the delivery are hot (RAM); link-up, the
 * control plane and the status packing run from flash.
 */

#include "link_input.h"

volatile bool link_input_restart_pending = false;

#if DSPI_LINK

#include "audio_pipeline.h"
#include "config.h"
#include "input_capture_arena.h"
#include "input_servo.h"
#include "notify.h"
#include "soft_vcxo.h"
#include "usb_audio.h"

#include "link/bench_trim.h"
#include "link/link_fast.h"
#include "link/link_ring_clock.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "link_rx_10.pio.h"
#include "link_rx_20.pio.h"
#include "link_rx_5.pio.h"

#if DSPI_LINK_DECODE_CORE1
#include "pdm_generator.h"      // core1_mode
#endif

#include <stdio.h>
#include <string.h>

// ============================================================================
// RESOURCES AND TUNING
// ============================================================================

#define LINK_PIO            pio2
#define LINK_SM             0u
#define LINK_DMA_DATA       PICO_SPDIF_RX_DMA_CH0   // 5 on RP2350
#define LINK_DMA_CTRL       PICO_SPDIF_RX_DMA_CH1   // 6
#define LINK_DMA_CRC        15u                     // ADAT RX's channel, free under LINK
#define LINK_RING_WORDS     INPUT_ARENA_LINK_WORDS
#define LINK_RING_BITS      (LINK_RING_WORDS * 32u)
#define LINK_RING           (input_capture_arena.link_ring)
#define LINK_FIFO_ROWS      256u                    // decoded frames between decoder and pipeline

_Static_assert((LINK_FIFO_ROWS & (LINK_FIFO_ROWS - 1u)) == 0, "FIFO rows must be a power of two");
_Static_assert(sizeof(LINK_RING) == LINK_RING_WORDS * 4u, "arena link ring size");
// The stall rule (link_ring_clock.h) gives up 1/16 of the ring; the decoder's
// overrun limit (guard, comma window, one frame not yet consumed) must fit in it.
_Static_assert(LINK_RING_BITS / 16u > LINK_FAST_GUARD_BITS + 12u + 400u, "stall rule vs overrun limit");

#define DETECT_DWELL_US     12000u      // per class until a comma with a good head
#define DETECT_LOCK_US      15000u      // after that, until the superframe locks
#define SCAN_FRAMES         256u        // clean frames that make a trim step clean
#define SCAN_STEP_US        25000u      // longest a trim step may take
#define RELOCK_US           300000u     // a loss longer than this re-detects the class
#define SEARCH_BUDGET       (LINK_RING_BITS / 4u)
#define EVENT_MIN_MS        250u        // NOTIFY_EVT_LINK_STATE rate limit

// Link-up phases.
enum { PH_DETECT, PH_SCAN, PH_SETTLE, PH_LOCKED, PH_RELOCK };

// ============================================================================
// STATE
// ============================================================================

static link_fast_t lf;                      // the decoder (tables + state, .bss)
static bool running;
static bool tables_built;
static uint8_t phase;
static uint8_t cls;                         // class of the loaded program, 0 none
static bool cls_known;                      // class detection finished
static int8_t trim;
static uint8_t rx_pin = 0xFF, los_pin = 0xFF;
static uint64_t deadline_us;
static bool aligned_seen;

// The loaded program (patched copy).
static uint16_t prog_ins[32];
static pio_program_t prog;
static uint prog_off;
static bool prog_loaded;

// Trim scan.
static bench_scan_t scan;
static bool scan_valid;
static bool scan_locked;
static uint32_t scan_frames0, scan_errs0;

// Ring position: absolute bits the DMA has written, tracked by whoever runs
// the decode service and published with the time it was read under a
// sequence count.
static const uint32_t *volatile ring_addr;
static uint32_t ring_last_idx;
static uint64_t ring_passes;
static volatile uint32_t ring_seq;
static uint64_t ring_bits_pub, ring_us_pub;
static uint64_t service_last_us;
static uint32_t pass_us = 4000u;            // one ring pass at the loaded class

// Counters of decoder instances that have finished (link-up steps excluded).
static link_rx_stats_t stats_acc;
static bool count_this;                     // the current instance counts into stats_acc

// Decoded rows: the decode service writes, the core-0 poll reads.
static int32_t fifo[LINK_FIFO_ROWS][8];
static volatile uint32_t fifo_head, fifo_tail;
static volatile bool fifo_drop;
static volatile bool slots_dirty;

// Stream.
static uint32_t detected_rate;              // Hz while LOCKED
static volatile bool loss_latch;
static link_sf_t sf_last;
static bool sf_valid;
static volatile bool stream_mute;           // superframe: system mute or test pattern (PRBS) payload

// Wire-rate measurement (as adat_input.c's slave machine).
static link_meas_t meas;
static uint32_t servo_last_us;

// Head-error counters at the last poll that saw the decoder locked: the
// loss reason looks only at what failed since (EXT_LINK_STATUS keeps totals).
static uint32_t lock_comma0, lock_crc80;

// NOTIFY_EVT_LINK_STATE, held until the rate limit allows.
static bool ev_pending, ev_once;
static uint8_t ev_state, ev_cls, ev_reason, pub_state;
static uint32_t ev_f, ev_last_ms;

#if DSPI_LINK_DECODE_CORE1
static volatile bool c1_active;             // Core 1 owns the decode service (LOCKED only)
static volatile bool c1_busy;               // Core 1 is inside it
#define C1_OWNS() (c1_active)
#else
#define C1_OWNS() (false)
#endif

typedef struct {
    const pio_program_t *p;
    pio_sm_config (*cfg)(uint);
    uint hi_entry, lo_emit, wrap_target;
} link_prog_t;

static const link_prog_t progs[3] = {
    { &link_rx_20_program, link_rx_20_program_get_default_config, link_rx_20_offset_hi_entry,
      link_rx_20_offset_lo_emit, link_rx_20_wrap_target },
    { &link_rx_10_program, link_rx_10_program_get_default_config, link_rx_10_offset_hi_entry,
      link_rx_10_offset_lo_emit, link_rx_10_wrap_target },
    { &link_rx_5_program, link_rx_5_program_get_default_config, link_rx_5_offset_hi_entry,
      link_rx_5_offset_lo_emit, link_rx_5_wrap_target },
};

// ============================================================================
// HELPERS
// ============================================================================

static inline bool link_pin_ok(uint8_t p) {
    return p == 0xFF || p <= 22u || (p >= 26u && p <= 28u);
}

static uint32_t rate_hz(uint8_t code) {
    return code == LINK_RATE_44K1 ? 44100u : code == LINK_RATE_48K ? 48000u : code == LINK_RATE_96K ? 96000u : 0u;
}

// Banks that hold the assigned slots.
static uint8_t slot_banks(void) {
    uint8_t m = 0;
    for (unsigned i = 0; i < LINK_SLOTS_LEN; i++)
        if (link_slots[i] < LINK_MAX_CHANNELS) m |= (uint8_t)(1u << (link_slots[i] / LINK_BANK_CHANNELS));
    return m;
}

static void stats_add(link_rx_stats_t *a, const link_rx_stats_t *s) {
    a->lock_count += s->lock_count;
    a->loss_count += s->loss_count;
    a->crc8_errors += s->crc8_errors;
    a->crc16_errors += s->crc16_errors;
    a->bit_slips += s->bit_slips;
    a->superframe_errors += s->superframe_errors;
    a->comma_errors += s->comma_errors;
    a->counter_errors += s->counter_errors;
    a->code_errors += s->code_errors;
    a->idle_errors += s->idle_errors;
    a->frames += s->frames;
}

static uint32_t error_sum(const link_rx_stats_t *s) {
    return s->loss_count + s->crc8_errors + s->crc16_errors + s->bit_slips + s->superframe_errors +
           s->comma_errors + s->counter_errors + s->code_errors + s->idle_errors;
}

// The last published position and, in *us, the time it was read.
static uint64_t ring_bits_read(uint64_t *us) {
    uint32_t s0, s1;
    uint64_t v, t;
    do {
        s0 = ring_seq;
        __dmb();
        v = ring_bits_pub;
        t = ring_us_pub;
        __dmb();
        s1 = ring_seq;
    } while (s0 != s1 || (s0 & 1u));
    if (us) *us = t;
    return v;
}

static void event(uint8_t state, uint8_t reason) {
    if (state == pub_state && reason != LINK_REASON_REANCHOR) return;
    pub_state = state;
    ev_state = state;
    ev_cls = cls_known ? cls : 0;
    ev_reason = reason;
    ev_f = (uint32_t)lf.frame;
    ev_pending = true;
}

void link_input_event_service(uint32_t now_ms) {
    if (!ev_pending) return;
    if (ev_once && (uint32_t)(now_ms - ev_last_ms) < EVENT_MIN_MS) return;
    ev_pending = false;
    ev_once = true;
    ev_last_ms = now_ms;
    notify_push_link_state(ev_state, ev_cls, ev_reason, ev_f);
}

// ============================================================================
// HARDWARE
// ============================================================================

// Load link_rx_N for class c with trim t and restart the SM.  The DMA keeps
// running; the bit stream has a seam that the decoder re-acquires across.
static bool load_program(unsigned c, int t) {
    const link_prog_t *lp = &progs[c - 1u];
    uint16_t ins[32];
    memcpy(ins, lp->p->instructions, lp->p->length * sizeof(uint16_t));
    if (!bench_trim_patch(ins, lp->p->length, lp->hi_entry, lp->lo_emit, t)) return false;
    pio_sm_set_enabled(LINK_PIO, LINK_SM, false);
    if (prog_loaded) pio_remove_program(LINK_PIO, &prog, prog_off);
    prog_loaded = false;
    memcpy(prog_ins, ins, sizeof(ins));
    prog = *lp->p;
    prog.instructions = prog_ins;
    prog.origin = -1;
    if (!pio_can_add_program(LINK_PIO, &prog)) return false;
    prog_off = pio_add_program(LINK_PIO, &prog);
    prog_loaded = true;
    cls = (uint8_t)c;
    trim = (int8_t)t;
    pass_us = (uint32_t)((uint64_t)LINK_RING_BITS * 1000000u / link_class_baud(c));
    pio_sm_config cfg = lp->cfg(prog_off);
    sm_config_set_jmp_pin(&cfg, rx_pin);
    sm_config_set_in_shift(&cfg, false, true, 32);     // MSB first, autopush 32
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac8(&cfg, 1, 0);         // the class divider is the program's N
    pio_sm_init(LINK_PIO, LINK_SM, prog_off + lp->wrap_target, &cfg);
    pio_sm_exec(LINK_PIO, LINK_SM, pio_encode_mov_not(pio_osr, pio_null));   // `in osr, 1` emits a 1
    pio_sm_set_enabled(LINK_PIO, LINK_SM, true);
    return true;
}

// Absolute bits written; must run at least once per ring pass (the decode
// service does, and restarts the decoder after a longer gap: link_ring_gap_ok()).
static uint64_t ring_writer_bits(void) {
    uint32_t idx = (dma_hw->ch[LINK_DMA_DATA].write_addr - (uint32_t)(uintptr_t)LINK_RING) >> 2;
    if (idx >= LINK_RING_WORDS) idx = 0;    // the reload is in flight
    return link_ring_count(idx, LINK_RING_WORDS, &ring_last_idx, &ring_passes) * 32u;
}

// us: when the DMA position behind bits was read (the rate measurement's clock).
static void ring_publish(uint64_t bits, uint64_t us) {
    ring_seq++;
    __dmb();
    ring_bits_pub = bits;
    ring_us_pub = us;
    __dmb();
    ring_seq++;
}

// The CRC-16 sniffer on DMA 15, self-tested against link_crc16(); the table
// CRC (C, flash) stays the decoder's fallback.
static void crc_dma_setup(void) {
    link_fast_set_crc_dma(&lf, -1);
    if (dma_channel_is_claimed(LINK_DMA_CRC)) return;
    dma_channel_claim(LINK_DMA_CRC);
    static uint32_t sink;
    dma_channel_config c = dma_channel_get_default_config(LINK_DMA_CRC);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_FORCE);
    channel_config_set_sniff_enable(&c, true);
    dma_channel_configure(LINK_DMA_CRC, &c, &sink, LINK_RING, 0, false);
    link_fast_set_crc_dma(&lf, LINK_DMA_CRC);
    static const uint8_t bytes[24] = "123456789abcdefghijklmno";
    uint32_t w[6];
    for (unsigned i = 0; i < 6u; i++)
        w[i] = (uint32_t)bytes[4 * i] << 24 | (uint32_t)bytes[4 * i + 1] << 16 | (uint32_t)bytes[4 * i + 2] << 8 |
               bytes[4 * i + 3];
    uint32_t want = link_crc16(bytes, 24);
    for (int swap = 0; swap < 2; swap++) {
        dma_sniffer_enable(LINK_DMA_CRC, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, false);
        dma_sniffer_set_byte_swap_enabled(swap != 0);
        if (link_fast_crc_dma_probe(&lf, w) == want) {
            printf("LINK RX: CRC-16 by the DMA sniffer%s\n", swap ? " (byte swap)" : "");
            return;
        }
    }
    printf("LINK RX: DMA sniffer CRC mismatch, table CRC\n");
    link_fast_set_crc_dma(&lf, -1);
    dma_sniffer_disable();
    dma_channel_unclaim(LINK_DMA_CRC);
}

// ============================================================================
// DECODER INSTANCES AND LINK-UP
// ============================================================================

// A fresh decoder for class c at the writer's position.  count: whether its
// counters go into EXT_LINK_STATUS (not for detection and scan steps).
static void decoder_restart(unsigned c, bool count) {
    if (count_this) stats_add(&stats_acc, &lf.stats);
    count_this = count;
    uint64_t at = ring_writer_bits() & ~31ull;
    link_fast_restart(&lf, c, at);
    uint8_t m = slot_banks();
    link_fast_set_bank_mask(&lf, (phase == PH_SCAN && !m) ? 1u : m);
    link_fast_set_output(&lf, NULL, 0);
    service_last_us = time_us_64();
    ring_publish(at, service_last_us);
}

static void enter_detect(uint64_t now) {
    phase = PH_DETECT;
    cls_known = false;
    scan_valid = false;
    unsigned c = link_config[LINK_CFG_CLASS] ? link_config[LINK_CFG_CLASS] : 1u;
    if (!load_program(c, 0)) printf("LINK RX: no room in PIO2\n");
    decoder_restart(c, false);
    deadline_us = now + DETECT_DWELL_US;
    aligned_seen = false;
}

static void scan_step_begin(uint64_t now, int t) {
    load_program(cls, t);
    decoder_restart(cls, false);
    scan_locked = false;
    deadline_us = now + SCAN_STEP_US;
}

static void enter_scan(uint64_t now) {
    phase = PH_SCAN;
    cls_known = true;
    const link_prog_t *lp = &progs[cls - 1u];
    bench_scan_begin(&scan, bench_trim_max(lp->p->instructions, lp->p->length, lp->hi_entry, lp->lo_emit));
    scan_valid = true;
    int t;
    bench_scan_next(&scan, &t);
    scan_step_begin(now, t);
}

static void enter_locked(void) {
    phase = PH_LOCKED;
    detected_rate = rate_hz(lf.rate);
    fifo_tail = fifo_head;
    link_meas_reset(&meas);
    lock_comma0 = lf.stats.comma_errors;
    lock_crc80 = lf.stats.crc8_errors;
    event(LINK_INPUT_LOCKED, LINK_REASON_LOCKED);

}

static void enter_relock(uint64_t now, uint8_t reason) {
    phase = PH_RELOCK;
    deadline_us = now + RELOCK_US;
    loss_latch = true;
    detected_rate = 0;
    event(LINK_INPUT_SEARCHING, reason);
}

// One link-up step (core 0, flash).
static void link_up_step(uint64_t now) {
    link_rx_state_t st = lf.state;
    switch (phase) {
    case PH_DETECT:
        if (st == LINK_RX_LOCKED) {
            enter_scan(now);
        } else if (st == LINK_RX_ALIGNED && !aligned_seen) {
            aligned_seen = true;
            deadline_us = now + DETECT_LOCK_US;
        } else if ((int64_t)(now - deadline_us) > 0) {
            unsigned c = link_config[LINK_CFG_CLASS] ? link_config[LINK_CFG_CLASS] : (cls % 3u) + 1u;
            load_program(c, 0);
            decoder_restart(c, false);
            deadline_us = now + DETECT_DWELL_US;
            aligned_seen = false;
        }
        break;
    case PH_SCAN: {
        bool done = false, clean = false;
        uint32_t errs = 0;
        if (!scan_locked && st == LINK_RX_LOCKED) {
            scan_locked = true;
            scan_frames0 = lf.stats.frames;
            scan_errs0 = error_sum(&lf.stats);
        }
        if (scan_locked) {
            errs = error_sum(&lf.stats) - scan_errs0;
            if (errs || st != LINK_RX_LOCKED) done = true;
            else if (lf.stats.frames - scan_frames0 >= SCAN_FRAMES) done = clean = true;
        }
        if (!done && (int64_t)(now - deadline_us) > 0) done = true;
        if (!done) break;
        bench_scan_record(&scan, clean, errs);
        int t;
        if (bench_scan_next(&scan, &t)) {
            scan_step_begin(now, t);
        } else {
            bench_scan_finish(&scan);
            printf("LINK RX: L%u trim scan %d..%d, using %d (mask 0x%05lx)\n", cls, scan.lo, scan.hi, scan.used,
                   (unsigned long)scan.clean_mask);
            load_program(cls, scan.used);
            decoder_restart(cls, true);
            phase = PH_SETTLE;
            deadline_us = now + RELOCK_US;
        }
        break;
    }
    case PH_SETTLE:
    case PH_RELOCK:
        if (st == LINK_RX_LOCKED) enter_locked();
        else if ((int64_t)(now - deadline_us) > 0) enter_detect(now);
        break;
    case PH_LOCKED:
        if (C1_OWNS()) break;
        if (st != LINK_RX_LOCKED) {
            // All three failed frames were flywheel frames: the comma is gone.
            uint32_t commas = lf.stats.comma_errors - lock_comma0, crc8s = lf.stats.crc8_errors - lock_crc80;
            enter_relock(now, commas >= 3u && !crc8s ? LINK_REASON_COMMA_LOST : LINK_REASON_FRAME_ERRORS);
        } else if (rate_hz(lf.rate) != detected_rate) {
            // A RATE switch in the stream: the pipeline follows (mute, rate change, prefill).
            detected_rate = rate_hz(lf.rate);
            loss_latch = true;
        }
        break;
    }
}

// ============================================================================
// DECODE SERVICE (hot) AND DELIVERY (hot)
// ============================================================================

// Decoded frames into the FIFO, in at most two contiguous pieces; clean_only:
// the assembly path alone (Core 1).  Returns false when a frame needs the
// general decoder (clean_only) or the FIFO is full.
// Inlined into each caller with clean_only constant, so Core 1's copy holds
// no call to link_fast_run() (check_ram_placement.py B2).
static inline __attribute__((always_inline)) bool decode_into_fifo(uint64_t bits, bool clean_only) {
    uint32_t head = fifo_head;
    bool ok = true;
    for (int piece = 0; piece < 2; piece++) {
        uint32_t space = LINK_FIFO_ROWS - (head - fifo_tail);
        if (!space) {
            // The pipeline is not taking audio: keep the lock, drop the backlog.
            if ((int32_t)((uint32_t)bits - lf.p) > (int32_t)(LINK_RING_BITS / 2u)) fifo_drop = true;
            ok = false;
            break;
        }
        uint32_t idx = head & (LINK_FIFO_ROWS - 1u), n = LINK_FIFO_ROWS - idx;
        if (n > space) n = space;
        link_fast_set_output(&lf, &fifo[idx], (uint16_t)n);
#if DSPI_LINK_DECODE_CORE1
        if (clean_only) {
            link_fast_run_clean(&lf, bits);
            head += lf.out_n;
            if (link_fast_needs_general(&lf)) { ok = false; break; }
            if (lf.out_n < n) break;
            continue;
        }
#else
        (void)clean_only;
#endif
        link_fast_run(&lf, bits, SEARCH_BUDGET);
        head += lf.out_n;
        if (lf.out_n < n) break;
    }
    link_fast_set_output(&lf, NULL, 0);
    __dmb();
    fifo_head = head;
    return ok;
}

DSP_TIME_CRITICAL
void link_input_decode_service(void) {
    if (!running) return;
    uint64_t now = time_us_64();
    uint64_t bits = ring_writer_bits();
    // A gap of nearly a ring pass may have hidden a whole pass from the
    // position count, and the bits are being overwritten: start over at the
    // writer (a loss if locked).
    if (!link_ring_gap_ok(now - service_last_us, pass_us)) {
        link_fast_restart(&lf, lf.cls, bits & ~31ull);
        ring_publish(bits & ~31ull, now);
        service_last_us = now;
        return;
    }
    service_last_us = now;
    if (slots_dirty) {
        slots_dirty = false;
        link_fast_set_slots(&lf, link_slots);
        link_fast_set_bank_mask(&lf, slot_banks());
    }
    if (phase != PH_LOCKED) link_fast_run(&lf, bits, SEARCH_BUDGET);
    else (void)decode_into_fifo(bits, false);
    ring_publish(bits, now);
}

#if DSPI_LINK_DECODE_CORE1
// Core 1 (EQ-worker wait loop): clean frames only, all in RAM.  Anything else
// (a gap, a superframe frame, damage, a slot change) hands the decoder back
// to core 0 for one service pass; core 0 re-enables Core 1 afterwards.
DSP_TIME_CRITICAL
void link_input_core1_service(void) {
    c1_busy = true;
    __dmb();
    if (c1_active) {
        uint64_t now = time_us_64();
        uint64_t bits = ring_writer_bits();
        // A long gap is core 0's to handle (it restarts the decoder): leave
        // service_last_us so that its service pass sees the gap too.
        bool keep = link_ring_gap_ok(now - service_last_us, pass_us);
        if (keep) {
            service_last_us = now;
            keep = !slots_dirty && decode_into_fifo(bits, true);
            ring_publish(bits, now);
        }
        if (!keep) c1_active = false;
    }
    __dmb();
    c1_busy = false;
}

// Core 0 takes the decoder back: no Core 1 service may be in progress after this.
static void c1_release(void) {
    c1_active = false;
    __dmb();
    while (c1_busy) tight_loop_contents();
}
#endif

DSP_TIME_CRITICAL
static uint32_t deliver(void) {
    if (fifo_drop) {
        fifo_tail = fifo_head;
        fifo_drop = false;
    }
    uint32_t head = fifo_head;
    __dmb();
    uint32_t tail = fifo_tail, avail = head - tail;
    if (phase != PH_LOCKED || detected_rate != audio_state.freq) {
        fifo_tail = head;       // keep decoding (and the lock); nothing plays
        return 0;
    }
    uint32_t min_frames, max_frames = producer_block_bounds(&min_frames);
    if (max_frames == 0 || avail < min_frames || avail == 0) return 0;
    uint32_t n = avail < max_frames ? avail : max_frames;
    float *dst[8] = { buf_l, buf_r, buf_in_ext[0], buf_in_ext[1], buf_in_ext[2],
                      buf_in_ext[3], buf_in_ext[4], buf_in_ext[5] };
    // A test-pattern (PRBS-7) or system-muted stream plays silence: its
    // payload is not audio (superframe flags b4, b0).
    float k[8];
    float scale = stream_mute ? 0.0f : 1.0f / 8388608.0f;
    for (int ch = 0; ch < 8; ch++) k[ch] = global_preamp_linear[ch] * scale;
    for (uint32_t i = 0; i < n; i++) {
        const int32_t *row = fifo[(tail + i) & (LINK_FIFO_ROWS - 1u)];
        for (int ch = 0; ch < 8; ch++) dst[ch][i] = (float)row[ch] * k[ch];
    }
    __dmb();
    fifo_tail = tail + n;
    process_input_block(n);
    link_input_on_delivered(n, avail - n);
    return n;
}

__attribute__((weak)) void link_input_on_delivered(uint32_t frames, uint32_t queued) {
    (void)frames;
    (void)queued;
}

// Wire rate from the RX DMA position (flash; once per poll, cheap).  The
// pair is the one the decode service published (core 0 or core 1), never
// this poll's own time: core 1 publishes at its own moments.
static void measure(void) {
    if (phase != PH_LOCKED || !detected_rate) return;
    uint64_t us, bits = ring_bits_read(&us);
    link_meas_sample(&meas, us, bits, 1e6f * (float)detected_rate / (float)link_class_baud(cls));
}

uint32_t link_input_poll(void) {
    if (!running) return 0;
    uint64_t now = time_us_64();
#if DSPI_LINK_DECODE_CORE1
    if (c1_active && core1_mode != CORE1_MODE_EQ_WORKER) c1_release();
    if (!c1_active) while (c1_busy) tight_loop_contents();
#endif
    if (!C1_OWNS()) link_input_decode_service();
    if (phase != PH_LOCKED || !C1_OWNS()) link_up_step(now);
#if DSPI_LINK_DECODE_CORE1
    // Hand the steady state (back) to Core 1 after this core-0 pass.
    if (!c1_active && phase == PH_LOCKED && lf.state == LINK_RX_LOCKED &&
        core1_mode == CORE1_MODE_EQ_WORKER) {
        __dmb();
        c1_active = true;
    }
#endif
    if (lf.state == LINK_RX_LOCKED) {
        sf_last = lf.sf;
        sf_valid = true;
        stream_mute = (sf_last.flags & (LINK_SF_FLAG_MUTE | LINK_SF_FLAG_PRBS)) != 0;
        if (phase == PH_LOCKED) {
            lock_comma0 = lf.stats.comma_errors;
            lock_crc80 = lf.stats.crc8_errors;
        }
    }
    measure();
    return deliver();
}

// ============================================================================
// LIFECYCLE
// ============================================================================

void link_input_init(void) {
    running = false;
    pub_state = LINK_INPUT_OFF;
}

void link_input_start(void) {
    if (running || !link_config[LINK_CFG_ENABLE]) return;
    rx_pin = link_config[LINK_CFG_RX_PIN] == 0xFF ? LINK_RX_PIN_DEFAULT : link_config[LINK_CFG_RX_PIN];
    los_pin = link_config[LINK_CFG_LOS_PIN];

    input_arena_claim(INPUT_ARENA_LINK);
    pio_sm_claim(LINK_PIO, LINK_SM);
    dma_channel_claim(LINK_DMA_DATA);
    dma_channel_claim(LINK_DMA_CTRL);

    // RX pin: input with the Schmitt trigger, no pull (RP2350 erratum E9),
    // pad isolation cleared (gpio_set_input_enabled() does not; see adat_input.c).
    gpio_set_pulls(rx_pin, false, false);
    gpio_set_input_hysteresis_enabled(rx_pin, true);
    gpio_set_input_enabled(rx_pin, true);
#if HAS_PADS_BANK0_ISOLATION
    hw_clear_bits(&pads_bank0_hw->io[rx_pin], PADS_BANK0_GPIO0_ISO_BITS);
#endif
    if (los_pin != 0xFF) {
        gpio_init(los_pin);
        gpio_set_dir(los_pin, GPIO_IN);
        gpio_pull_up(los_pin);
    }

    memset(LINK_RING, 0, sizeof(LINK_RING));
    ring_addr = LINK_RING;
    ring_last_idx = 0;
    ring_passes = 0;
    ring_publish(0, time_us_64());

    // DMA 5 moves FIFO words into the ring and chains to 6, which rewrites
    // 5's write address with trigger (LK3's two-channel ring).
    dma_channel_config c = dma_channel_get_default_config(LINK_DMA_DATA);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(LINK_PIO, LINK_SM, false));
    channel_config_set_chain_to(&c, LINK_DMA_CTRL);
    dma_channel_configure(LINK_DMA_DATA, &c, LINK_RING, &LINK_PIO->rxf[LINK_SM], LINK_RING_WORDS, false);
    dma_channel_config k = dma_channel_get_default_config(LINK_DMA_CTRL);
    channel_config_set_transfer_data_size(&k, DMA_SIZE_32);
    channel_config_set_read_increment(&k, false);
    channel_config_set_write_increment(&k, false);
    dma_channel_configure(LINK_DMA_CTRL, &k, &dma_hw->ch[LINK_DMA_DATA].al2_write_addr_trig,
                          (const void *)&ring_addr, 1, false);
    dma_channel_start(LINK_DMA_DATA);

    if (!tables_built) {
        link_fast_init(&lf, LINK_RING, LINK_RING_WORDS, 1, slot_banks(), NULL, NULL, 0);
        tables_built = true;
    }
    link_fast_set_slots(&lf, link_slots);
    crc_dma_setup();

    fifo_head = fifo_tail = 0;
    fifo_drop = slots_dirty = false;
    loss_latch = false;
    detected_rate = 0;
    sf_valid = false;
    count_this = false;
    servo_last_us = time_us_32();
    input_servo_reset();
    running = true;
    enter_detect(time_us_64());
    event(LINK_INPUT_SEARCHING, LINK_REASON_NONE);
    printf("LINK RX: started on GPIO %u (class %s)\n", rx_pin,
           link_config[LINK_CFG_CLASS] ? "forced" : "auto");
}

void link_input_stop(void) {
    if (!running) return;
#if DSPI_LINK_DECODE_CORE1
    c1_release();
#endif
    running = false;
    pio_sm_set_enabled(LINK_PIO, LINK_SM, false);
    if (prog_loaded) pio_remove_program(LINK_PIO, &prog, prog_off);
    prog_loaded = false;
    pio_sm_unclaim(LINK_PIO, LINK_SM);
    // The reload channel re-triggers the data channel: stop it first.
    dma_channel_abort(LINK_DMA_CTRL);
    dma_channel_abort(LINK_DMA_DATA);
    dma_channel_abort(LINK_DMA_CTRL);
    dma_channel_unclaim(LINK_DMA_DATA);
    dma_channel_unclaim(LINK_DMA_CTRL);
    if (lf.crc_dma >= 0) {
        dma_sniffer_disable();
        dma_channel_unclaim(LINK_DMA_CRC);
        link_fast_set_crc_dma(&lf, -1);
    }
    gpio_set_input_enabled(rx_pin, false);
    if (los_pin != 0xFF) gpio_deinit(los_pin);
    if (count_this) stats_add(&stats_acc, &lf.stats);
    count_this = false;
    memset(&lf.stats, 0, sizeof(lf.stats));
    input_arena_release(INPUT_ARENA_LINK);
    input_servo_reset();
    detected_rate = 0;
    cls_known = false;
    event(LINK_INPUT_OFF, LINK_REASON_DISABLED);
    printf("LINK RX: stopped\n");
}

bool link_input_is_running(void) { return running; }

LinkInputState link_input_get_state(void) {
    if (!running) return LINK_INPUT_OFF;
    return phase == PH_LOCKED ? LINK_INPUT_LOCKED : LINK_INPUT_SEARCHING;
}

bool link_input_take_loss(void) {
    if (!loss_latch) return false;
    loss_latch = false;
    return true;
}

void link_input_discard_queued(void) {
    fifo_tail = fifo_head;
}

DSP_TIME_CRITICAL
void link_input_update_clock_servo(void) {
    if (!running || phase != PH_LOCKED || !detected_rate || detected_rate != audio_state.freq) return;
    float hz = link_meas_hz(&meas);
    if (hz == 0.0f) return;
    if (!input_servo_tick_due(&servo_last_us)) return;
    input_servo_apply(hz, 0);
}

bool link_input_check_rate_change(void) {
    if (!running || phase != PH_LOCKED) return false;
    uint32_t r = detected_rate;
    if (!r || r == audio_state.freq) return false;
    pending_rate = r;
    __dmb();
    rate_change_pending = true;
    return true;
}

uint32_t link_input_get_detected_rate(void) {
    return running && phase == PH_LOCKED ? detected_rate : 0;
}

bool link_input_owns_pin(uint8_t pin) {
    if (active_input_source != INPUT_SOURCE_LINK || !link_config[LINK_CFG_ENABLE]) return false;
    uint8_t rx = link_config[LINK_CFG_RX_PIN] == 0xFF ? LINK_RX_PIN_DEFAULT : link_config[LINK_CFG_RX_PIN];
    return pin == rx || (link_config[LINK_CFG_LOS_PIN] != 0xFF && pin == link_config[LINK_CFG_LOS_PIN]);
}

uint64_t link_input_get_frame(bool *valid) {
    bool v = running && phase == PH_LOCKED;
    if (valid) *valid = v;
    return v ? lf.frame : 0;
}

// ============================================================================
// CONTROL PLANE (extension_channel_spec.md 2.8)
// ============================================================================

static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
static void putf(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); put32(p, u); }

bool link_input_set_config(const uint8_t c[LINK_CFG_LEN]) {
    if (c[LINK_CFG_ENABLE] > 1u || c[LINK_CFG_CLASS] > 3u || (c[LINK_CFG_FLAGS] & ~0x03u) || c[7]) return false;
    // No repeater in this build: no TX or TX_DISABLE pin.
    if (c[LINK_CFG_TX_PIN] != 0xFF || c[LINK_CFG_TXDIS_PIN] != 0xFF) return false;
    if (!link_pin_ok(c[LINK_CFG_RX_PIN]) || !link_pin_ok(c[LINK_CFG_LOS_PIN])) return false;
    uint8_t rx = c[LINK_CFG_RX_PIN] == 0xFF ? LINK_RX_PIN_DEFAULT : c[LINK_CFG_RX_PIN];
    if (c[LINK_CFG_LOS_PIN] != 0xFF && c[LINK_CFG_LOS_PIN] == rx) return false;
    bool restart = c[LINK_CFG_ENABLE] != link_config[LINK_CFG_ENABLE] ||
                   c[LINK_CFG_RX_PIN] != link_config[LINK_CFG_RX_PIN] ||
                   c[LINK_CFG_CLASS] != link_config[LINK_CFG_CLASS] ||
                   c[LINK_CFG_LOS_PIN] != link_config[LINK_CFG_LOS_PIN];
    memcpy(link_config, c, LINK_CFG_LEN);
    if (restart && active_input_source == INPUT_SOURCE_LINK) {
        __dmb();
        link_input_restart_pending = true;
    }
    return true;
}

bool link_input_set_slots(const uint8_t s[LINK_SLOTS_LEN]) {
    for (unsigned i = 0; i < LINK_SLOTS_LEN; i++)
        if (s[i] >= LINK_MAX_CHANNELS && s[i] != 0xFF) return false;
    memcpy(link_slots, s, LINK_SLOTS_LEN);
    __dmb();
    slots_dirty = true;
    return true;
}

bool link_input_stats_reset(uint8_t v) {
    if (v != 1u) return false;
    memset(&stats_acc, 0, sizeof(stats_acc));
    memset(&lf.stats, 0, sizeof(lf.stats));
    lock_comma0 = lock_crc80 = 0;
    return true;
}

void link_input_get_info(uint8_t o[LINK_INFO_LEN]) {
    memset(o, 0, LINK_INFO_LEN);
    o[0] = 1;                               // receiver
    o[1] = LINK_PROTOCOL_VERSION;
    o[2] = 0x07;                            // L1, L2, L3
    o[3] = 1;                               // one link input
    o[4] = LINK_MAX_CHANNELS;
    o[5] = LINK_SLOTS_LEN;
    o[8] = link_config[LINK_CFG_RX_PIN] == 0xFF ? LINK_RX_PIN_DEFAULT : link_config[LINK_CFG_RX_PIN];
    o[9] = 0xFF;                            // no repeater
    put32(o + 12, 1u << 5);                 // LOS_PIN; PHASE_LOCK / APPLY_AT_FRAME come with LK6
    // d_local_min / d_local_max (16-19): LK6.
}

static void current_stats(link_rx_stats_t *s) {
    *s = stats_acc;
    if (count_this) stats_add(s, &lf.stats);
}

void link_input_get_status(uint8_t o[LINK_STATUS_LEN]) {
    memset(o, 0, LINK_STATUS_LEN);
    bool locked = running && phase == PH_LOCKED;
    o[0] = (uint8_t)link_input_get_state();
    o[1] = running && cls_known ? cls : 0;
    o[2] = locked ? lf.rate : 0xFF;
    o[3] = locked ? lf.channels : 0;
    o[4] = sf_valid ? sf_last.hops : 0;
    o[5] = (running && los_pin != 0xFF) ? (gpio_get(los_pin) ? 1 : 0) : 0xFF;
    o[6] = sf_valid ? sf_last.sender : 0;
    o[7] = sf_valid ? sf_last.flags : 0;
    uint64_t f = locked ? lf.frame : 0;
    put32(o + 8, (uint32_t)f);
    put32(o + 12, (uint32_t)(f >> 32));
    link_rx_stats_t s;
    current_stats(&s);
    put32(o + 16, s.lock_count);
    put32(o + 20, s.loss_count);
    put32(o + 24, s.crc8_errors);
    put32(o + 28, s.crc16_errors);
    put32(o + 32, s.bit_slips);
    put32(o + 36, s.superframe_errors);
    putf(o + 40, soft_vcxo_current_ppm());
    putf(o + 44, 0.0f);                     // phase error: LK6
    put16(o + 48, slot_ring_fill_frames(0));
}

void link_input_get_ctrl_msg(uint8_t o[LINK_CTRL_MSG_LEN]) {
    memset(o, 0, LINK_CTRL_MSG_LEN);
    if (!sf_valid) return;
    o[0] = sf_last.flags;
    o[1] = sf_last.rate;
    o[2] = sf_last.channels;
    o[3] = sf_last.cls;
    o[4] = sf_last.hops;
    o[5] = sf_last.sender;
    put32(o + 8, (uint32_t)sf_last.apply_frame);
    put32(o + 12, (uint32_t)(sf_last.apply_frame >> 32));
    o[16] = sf_last.apply_op;
    o[17] = sf_last.apply_arg;
    put16(o + 18, (uint16_t)sf_last.apply_value);
    put32(o + 20, (uint32_t)sf_last.frame);
}

void link_input_get_diag(uint8_t o[LINK_DIAG_LEN]) {
    memset(o, 0, LINK_DIAG_LEN);
    o[0] = 1;                               // layout version
    o[1] = (uint8_t)(lf.p & 31u);
    float ff = 0.0f, actual = link_meas_hz(&meas);
    if (running && detected_rate && actual > 0.0f) ff = (actual / (float)detected_rate - 1.0f) * 1e6f;
    putf(o + 4, ff);                        // ppm_ff; ppm_phase and integral (8, 12): LK6
    uint64_t bits = running ? ring_bits_read(NULL) : 0;
    put32(o + 16, (uint32_t)((bits / 8u) % (LINK_RING_WORDS * 4u)));
    put32(o + 20, (uint32_t)((uint64_t)lf.p / 8u % (LINK_RING_WORDS * 4u)));
    put32(o + 24, 0);                       // output DMA words consumed: LK6
    put32(o + 28, running ? dma_hw->ch[LINK_DMA_DATA].transfer_count : 0);
    if (scan_valid) {
        o[32] = (uint8_t)scan.lo;
        o[33] = (uint8_t)scan.hi;
        o[34] = (uint8_t)(running ? trim : scan.used);
        o[35] = scan.state;
        put32(o + 36, scan.clean_mask);
    }
    link_rx_stats_t s;
    current_stats(&s);
    put32(o + 40, s.comma_errors);
    put32(o + 44, s.counter_errors);
    put32(o + 48, s.code_errors);
    put32(o + 52, s.idle_errors);
}

#endif // DSPI_LINK
