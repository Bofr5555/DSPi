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
#include "bulk_params.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "input_capture_arena.h"
#include "input_servo.h"
#include "notify.h"
#include "output_latency.h"
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

#include <stddef.h>
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
#define ANCHOR_SPIN_US      1100u       // the anchor waits in place for the last ms or so
#define MUTE_HOLD_MS        5u          // soft-mute refresh while anchoring or holding
#define LINK_RATE_READY_PPM 1.0f        // VCXO this close to the feed-forward: pulled in

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

// Phase lock and apply at frame (WP LK6, link/link_phase.h).  The decoder's
// reference (frame pub_f, comma ending at bit pub_p) and the FIFO's frame
// numbering (rows below pub_head end at frame pub_head_f) are published with
// the ring position, under ring_seq.
static link_phase_t ph;
static link_apply_q_t aq;
static uint64_t pub_f, pub_head_f;
static uint32_t pub_p, pub_head;
static bool pub_locked;
static uint32_t drop_rows;                  // whole frames still to drop (anchor correction)
static bool pull_in;                        // the drain waits for the VCXO: rows are dropped meanwhile
static uint64_t hold_f, hold_us;            // holdover: F extrapolated from here
static const link_apply_limits_t apply_lim = { PRESET_SLOTS, NUM_OUTPUT_CHANNELS, MAX_DELAY_SAMPLES };
extern volatile bool preset_loading;        // main.c: the prefill handshake's flag

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

// The FIFO's frame numbering as last published: rows below *head end at
// frame *head_f.
static inline __attribute__((always_inline)) void pub_rows(uint32_t *head, uint64_t *head_f) {
    uint32_t s0, s1;
    do {
        s0 = ring_seq;
        __dmb();
        *head = pub_head;
        *head_f = pub_head_f;
        __dmb();
        s1 = ring_seq;
    } while (s0 != s1 || (s0 & 1u));
}

// The decoder's reference as last published with its ring position.
static bool pub_ref(uint64_t *bits, uint64_t *f, uint32_t *p) {
    uint32_t s0, s1;
    bool ok;
    do {
        s0 = ring_seq;
        __dmb();
        *bits = ring_bits_pub;
        *f = pub_f;
        *p = pub_p;
        ok = pub_locked;
        __dmb();
        s1 = ring_seq;
    } while (s0 != s1 || (s0 & 1u));
    return ok;
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
// After a decoder restart: the position alone, no reference.
static inline __attribute__((always_inline)) void ring_publish_lost(uint64_t bits, uint64_t us) {
    ring_seq++;
    __dmb();
    ring_bits_pub = bits;
    ring_us_pub = us;
    pub_locked = false;
    __dmb();
    ring_seq++;
}

// The decoder's reference and the FIFO numbering go with it (WP LK6): the
// next row written is frame lf.frame, or lf.frame + 1 once its body is done.
static inline __attribute__((always_inline)) void ring_publish(uint64_t bits, uint64_t us) {
    ring_seq++;
    __dmb();
    ring_bits_pub = bits;
    ring_us_pub = us;
    pub_locked = lf.state == LINK_RX_LOCKED;
    if (pub_locked) {
        pub_f = lf.frame;
        pub_p = lf.p;
        pub_head = fifo_head;
        pub_head_f = lf.frame + (lf.body_done ? 1u : 0u);
    }
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
    ring_publish_lost(at, service_last_us);
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
    uint32_t was = detected_rate;
    detected_rate = rate_hz(lf.rate);
    fifo_tail = fifo_head;
    link_meas_reset(&meas);
    lock_comma0 = lf.stats.comma_errors;
    lock_crc80 = lf.stats.crc8_errors;
    if (ph.hold) {
        // Back from a holdover: at the same rate the new frames go onto the
        // held timeline by F and the phase decides (link_phase_relock()); a
        // new rate needs the full reset.
        if (detected_rate == was && detected_rate == audio_state.freq) {
            link_phase_relock(&ph);
        } else {
            ph.hold = false;
            loss_latch = true;
        }
    }
    event(LINK_INPUT_LOCKED, LINK_REASON_LOCKED);
}

static void enter_relock(uint64_t now, uint8_t reason) {
    pull_in = false;
    phase = PH_RELOCK;
    deadline_us = now + RELOCK_US;
    loss_latch = true;
    detected_rate = 0;
    event(LINK_INPUT_SEARCHING, reason);
}

// Frames arriving now from the decoder's published reference and the live
// RX DMA position (bits per frame: the class's baud over the stream's rate),
// relative to frame fx.  False without a locked reference.
static bool arrival_since(uint64_t fx, float *since) {
    uint64_t bits, f;
    uint32_t p;
    if (!pub_ref(&bits, &f, &p) || !detected_rate) return false;
    uint32_t idx = (dma_hw->ch[LINK_DMA_DATA].write_addr - (uint32_t)(uintptr_t)LINK_RING) >> 2;
    if (idx >= LINK_RING_WORDS) idx = 0;
    uint32_t words = (idx - (uint32_t)(bits >> 5)) & (LINK_RING_WORDS - 1u);
    uint32_t now32 = (uint32_t)bits + words * 32u;
    *since = link_phase_since(f, fx, (int32_t)(now32 - p),
                              (float)link_class_baud(cls) / (float)detected_rate);
    return true;
}

// A phase-locked loss: keep the timeline on silence (holdover), F
// extrapolated from the last arrival; false when there is no timeline.
static bool enter_holdover(uint64_t now, uint8_t reason) {
    uint32_t fill = slot_ring_fill_frames(0);
    if (!link_phase_active(&ph) || !link_phase_loss(&ph, fill)) return false;
    // Phase-locked, the frame arriving now is the one playing plus the delay.
    hold_f = ph.next_w - fill + (uint64_t)(int64_t)link_phase_delay(&ph);
    hold_us = now;
    phase = PH_RELOCK;
    deadline_us = now + RELOCK_US;
    event(LINK_INPUT_HOLDOVER, reason);
    return true;
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
            uint8_t reason = commas >= 3u && !crc8s ? LINK_REASON_COMMA_LOST : LINK_REASON_FRAME_ERRORS;
            if (!enter_holdover(now, reason)) enter_relock(now, reason);
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
        ring_publish_lost(bits & ~31ull, now);
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

// FIFO rows [tail, tail + n), frames f.., into the pipeline.  Inlined into
// each caller: deliver() itself is inlined into link_input_poll(), as it was
// before LK6, so the steady-state copy costs no RAM.
static inline __attribute__((always_inline)) void deliver_rows(uint32_t tail, uint32_t n, uint64_t f) {
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
    pipeline_click_output = -1;
    link_phase_written(&ph, f, n);
}

static uint32_t deliver_slow(uint32_t tail, uint32_t avail, uint64_t f, uint32_t n);
static uint32_t deliver_hold(uint32_t head);
static bool deliver_place(uint32_t *tail, uint32_t *avail, uint64_t *f);

DSP_TIME_CRITICAL
static uint32_t deliver(void) {
    uint32_t head;
    uint64_t head_f;
    pub_rows(&head, &head_f);
    if (fifo_drop) {
        fifo_tail = head;
        fifo_drop = false;
    }
    if (ph.hold) return deliver_hold(head);
    uint32_t tail = fifo_tail, avail = head - tail;
    if (phase != PH_LOCKED || detected_rate != audio_state.freq || (pull_in && preset_loading)) {
        fifo_tail = head;       // keep decoding (and the lock); nothing plays
        return 0;
    }
    uint64_t f = head_f - avail;    // frame of row `tail`
    // Rows placed by F after a holdover, or an anchor correction (cold).
    if ((ph.resync || drop_rows) && !deliver_place(&tail, &avail, &f)) return 0;
    uint32_t min_frames, max_frames = producer_block_bounds(&min_frames);
    if (max_frames == 0 || avail < min_frames || avail == 0) return 0;
    uint32_t n = avail < max_frames ? avail : max_frames;
    // An apply op due inside this block splits it (cold).
    if (link_apply_next(&aq) < f + n) return deliver_slow(tail, avail, f, n);
    deliver_rows(tail, n, f);
    return n;
}

// ----------------------------------------------------------------------------
// Off the steady state (flash): apply ops, silence, holdover, placement
// ----------------------------------------------------------------------------

// n frames of silence onto the timeline (holdover, a gap before placed rows,
// an anchor correction).
static void silence_block(uint32_t n) {
    float *dst[8] = { buf_l, buf_r, buf_in_ext[0], buf_in_ext[1], buf_in_ext[2],
                      buf_in_ext[3], buf_in_ext[4], buf_in_ext[5] };
    for (int ch = 0; ch < 8; ch++) memset(dst[ch], 0, n * sizeof(float));
    process_input_block(n);
    pipeline_click_output = -1;
    link_phase_written(&ph, ph.next_w, n);
}

// What an op does (extension_channel_spec.md 2.8.7).  Mute, volume and click
// act on the next sample the pipeline processes, which the caller has made
// the op's frame; a preset load and a delay change start there.
static uint8_t apply_exec(const link_apply_op_t *op) {
    switch (op->op) {
    case LINK_OP_MUTE:
    case LINK_OP_UNMUTE: {
        user_mute = op->op == LINK_OP_MUTE;
        uint8_t v = user_mute ? 1 : 0;
        notify_param_write(offsetof(WireBulkParams, user_volume.user_mute), sizeof(uint8_t), &v);
        return LINK_APPLY_APPLIED;
    }
    case LINK_OP_PRESET_LOAD:
        pending_preset_load_slot = op->arg;
        __dmb();
        preset_load_pending = true;
        return LINK_APPLY_APPLIED;
    case LINK_OP_MASTER_VOLUME:
        update_master_volume((float)op->value * 0.01f);
        return LINK_APPLY_APPLIED;
    case LINK_OP_OUT_DELAY: {
        float fs = (float)(audio_state.freq ? audio_state.freq : 48000u);
        // + 0.5: dsp_update_delay_samples() truncates the product back to samples.
        float ms = ((float)op->value + 0.5f) * 1000.0f / fs;
        channel_delays_ms[CH_OUT_1 + op->arg] = ms;
        dsp_update_delay_samples(fs);
        notify_param_write((uint16_t)(offsetof(WireBulkParams, delays.delay_ms) +
                                      (CH_OUT_1 + op->arg) * sizeof(float)), sizeof(float), &ms);
        return LINK_APPLY_APPLIED;
    }
    case LINK_OP_CLICK:
        // The PDM sub (the last output) is not finalized as PCM: refused.
        if (op->arg >= NUM_SPDIF_INSTANCES * 2) return LINK_APPLY_REFUSED;
        pipeline_click_output = (int8_t)op->arg;
        return LINK_APPLY_APPLIED;
    default:
        // SYNC_PULSE: a receiver has no sync GPIO (EXT_LINK_CONFIG has none).
        return LINK_APPLY_REFUSED;
    }
}

static void apply_report(uint8_t op, uint8_t result, uint64_t frame) {
    link_apply_done(&aq, op, result, frame);
    notify_push_apply_result(op, result, (uint32_t)frame);
}

static void apply_run(bool late) {
    link_apply_op_t op;
    link_apply_pop(&aq, &op);
    uint8_t r = apply_exec(&op);
    if (late && r == LINK_APPLY_APPLIED) r = LINK_APPLY_LATE;
    apply_report(op.op, r, op.frame);
}

// n frames from frame f (FIFO rows from tail, or silence), split where an
// apply op is due so that it acts on exactly its frame; an op whose frame has
// passed runs at once and reports late.
static uint32_t deliver_span(bool rows, uint32_t tail, uint64_t f, uint32_t n) {
    uint32_t done = 0;
    while (done < n) {
        uint64_t at = link_apply_next(&aq), here = f + done;
        if (at <= here) {
            apply_run(at < here);
            continue;
        }
        uint32_t k = n - done;
        if (at - here < k) k = (uint32_t)(at - here);
        if (rows) deliver_rows(tail + done, k, here);
        else silence_block(k);
        done += k;
    }
    return done;
}

static uint32_t deliver_slow(uint32_t tail, uint32_t avail, uint64_t f, uint32_t n) {
    (void)avail;
    return deliver_span(true, tail, f, n);
}

// Holdover: rows from the search are dropped; silence keeps the rings at
// their fill, so the timeline runs on at the held VCXO rate.
static uint32_t deliver_hold(uint32_t head) {
    fifo_tail = head;
    uint32_t min_frames, max_frames = producer_block_bounds(&min_frames);
    uint32_t n = link_phase_hold_frames(&ph, slot_ring_fill_frames(0), max_frames);
    return n ? deliver_span(false, 0, ph.next_w, n) : 0;
}

static void reanchor(void) {
    drop_rows = 0;
    ph.resync = false;
    ph.hold = false;
    loss_latch = true;      // the main loop mutes, drains, prefills, starts in sync
    event(LINK_INPUT_LOCKED, LINK_REASON_REANCHOR);
}

// Rows against the timeline before they play: an anchor correction drops
// whole frames; after a holdover the first rows are placed by F (late rows
// dropped, a gap filled with silence).  False: nothing to deliver this pass.
static bool deliver_place(uint32_t *tail, uint32_t *avail, uint64_t *f) {
    uint32_t k;
    if (drop_rows) {
        k = drop_rows < *avail ? drop_rows : *avail;
        fifo_tail = *tail + k;
        link_phase_written(&ph, *f, k);     // skipped: the timeline moves on
        *tail += k;
        *avail -= k;
        *f += k;
        drop_rows -= k;
        if (drop_rows) return false;
    }
    if (!ph.resync) return true;
    int64_t gap = link_phase_gap(&ph, *f);
    if (gap < 0) {
        k = (uint64_t)-gap < *avail ? (uint32_t)-gap : *avail;
        fifo_tail = *tail + k;
        *tail += k;
        *avail -= k;
        *f += k;
        if ((int64_t)k < -gap) return false;
    } else if (gap > 0) {
        if (gap > (int64_t)output_latency_ring_span()) {
            reanchor();         // another stream, or a long gap: start over
            return false;
        }
        uint32_t min_frames, max_frames = producer_block_bounds(&min_frames);
        k = (uint64_t)gap < max_frames ? (uint32_t)gap : max_frames;
        if (k) deliver_span(false, 0, ph.next_w, k);
        return false;
    }
    ph.resync = false;          // in place: the next check verifies the phase
    return true;
}

// Wire rate from the RX DMA position (flash; once per poll, cheap).  The
// pair is the one the decode service published (core 0 or core 1), never
// this poll's own time: core 1 publishes at its own moments.
static void measure(void) {
    if (phase != PH_LOCKED || !detected_rate) return;
    uint64_t us, bits = ring_bits_read(&us);
    link_meas_sample(&meas, us, bits, 1e6f * (float)detected_rate / (float)link_class_baud(cls));
}

// ----------------------------------------------------------------------------
// Phase measurement, verification and servo (flash)
// ----------------------------------------------------------------------------

// The phase error now (frames): the arrival against the timeline and the live
// ring fill, read together.  False without a timeline or a locked reference.
static bool phase_error(float *e) {
    if (!ph.timeline || ph.hold || ph.resync) return false;
    float fill, since;
    uint32_t played;
    uint32_t save = save_and_disable_interrupts();
    bool ok = slot_ring_play_position(0, &fill, &played) && arrival_since(ph.next_w, &since);
    restore_interrupts(save);
    if (!ok) return false;
    *e = link_phase_error(&ph, since, fill);
    return true;
}

static void phase_act(int act, int32_t frames) {
    if (act == LINK_PHASE_ACT_REANCHOR) {
        reanchor();
    } else if (act == LINK_PHASE_ACT_CORRECT) {
        if (frames > 0) {
            drop_rows += (uint32_t)frames;      // the next deliveries skip them
        } else {
            uint32_t k = (uint32_t)-frames, cap = output_ring_block_cap;
            while (k) {
                uint32_t n = k < cap ? k : cap;
                deliver_span(false, 0, ph.next_w, n);
                k -= n;
            }
        }
    }
    // ACT_RESUME: the soft mute is no longer refreshed and fades out.
}

// Anchor or resync verification, every poll until it passes (muted).
static void phase_verify(void) {
    float e;
    if (!phase_error(&e)) return;
    int32_t k = 0;
    int act = link_phase_check(&ph, e, (link_config[LINK_CFG_FLAGS] & 0x02u) != 0, &k);    // sets k: not in the same call's arguments
    phase_act(act, k);
}

// One servo tick in phase lock: the rate feed-forward plus the phase trim.
// False when the fill servo runs instead (mode 0, delay out of range).
static __attribute__((noinline)) bool phase_servo_tick(float hz) {
    if (!link_phase_active(&ph)) return false;
    float e, trim = 0.0f;
    if (ph.started && phase_error(&e)) {
        int32_t k = 0;
        int act = LINK_PHASE_ACT_NONE;
        if (ph.state != LINK_PHASE_ANCHORING) {
            act = link_phase_check(&ph, e, (link_config[LINK_CFG_FLAGS] & 0x02u) != 0, &k);
            phase_act(act, k);
        }
        // A re-anchor's error is not the phase's: feed-forward only, fresh PI.
        if (act == LINK_PHASE_ACT_REANCHOR) link_phase_pi_reset(&ph.pi);
        else if (ph.state == LINK_PHASE_LOCKED || ph.state == LINK_PHASE_PENDING) trim = link_phase_servo(&ph, e);
    }
    input_servo_apply_trim(hz, trim);
    return true;
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
            // The superframe's apply (the broadcast path): same queue.
            if (link_apply_take_sf(&aq, &apply_lim, &sf_last) == 2)
                apply_report(sf_last.apply_op, LINK_APPLY_REFUSED, sf_last.apply_frame);
        }
    }
    measure();
    uint32_t n = deliver();
    if (link_phase_active(&ph)) {
        if (ph.started && ph.state == LINK_PHASE_ANCHORING && !ph.hold && !ph.resync && !drop_rows &&
            phase == PH_LOCKED)
            phase_verify();
        if (ph.hold || ph.state == LINK_PHASE_ANCHORING) {
            uint32_t fs = audio_state.freq ? audio_state.freq : 48000u;
            pipeline_request_soft_mute(fs / 1000u * MUTE_HOLD_MS);
        }
    }
    return n;
}

// ============================================================================
// LIFECYCLE
// ============================================================================

void link_input_init(void) {
    running = false;
    pub_state = LINK_INPUT_OFF;
    link_phase_init(&ph);
    link_apply_init(&aq);
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
    ring_publish_lost(0, time_us_64());

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
    // No frames come any more: pending ops are cancelled, the timeline ends.
    while (aq.n) {
        link_apply_op_t op;
        link_apply_pop(&aq, &op);
        apply_report(op.op, LINK_APPLY_CANCELLED, op.frame);
    }
    link_phase_drain(&ph);
    ph.state = LINK_PHASE_FREE;
    drop_rows = 0;
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
    if (ph.hold) return LINK_INPUT_HOLDOVER;
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
    if (phase_servo_tick(hz)) return;
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

// ----------------------------------------------------------------------------
// Phase lock and apply at frame: control plane (2.8.5-2.8.7)
// ----------------------------------------------------------------------------

// d_local_min / d_local_max under the requested latency option at the
// current rate, as the LINK source would run it.  While LINK runs, the limits
// in force stay (output_latency_apply() set them with the prefill target), so
// a read between a latency change and its reset cannot switch the lock off.
static void refresh_limits(void) {
    if (running && active_input_source == INPUT_SOURCE_LINK) return;
    uint32_t fs = audio_state.freq ? audio_state.freq : 48000u;
    const OutputLatencyOption *opt = output_latency_resolve(output_latency_requested(), INPUT_SOURCE_LINK);
    link_phase_limits(&ph, output_latency_target_frames(opt, fs), output_latency_block_cap(opt, fs),
                      output_latency_ring_span());
}

uint32_t link_input_ring_target(uint32_t target, uint32_t block_cap) {
    uint32_t span = output_latency_ring_span();
    link_phase_limits(&ph, target, block_cap, span);
    return link_phase_prefill(&ph, target, block_cap, span);
}

void link_input_load_phase(void) {
    ph.mode = link_phase_store[0];
    ph.d_local = (uint16_t)(link_phase_store[1] | link_phase_store[2] << 8);
    ph.hops_offset = (int16_t)(link_phase_store[3] | link_phase_store[4] << 8);
}

bool link_input_set_phase(const uint8_t p[LINK_PHASE_LEN]) {
    refresh_limits();
    bool was = link_phase_active(&ph);
    int r = link_phase_set(&ph, p);
    if (r < 0) return false;
    memcpy(link_phase_store, p, 1);
    memcpy(link_phase_store + 1, p + 2, 4);
    // A new mode or delay while the outputs play phase-locked: re-anchor
    // (2-6 ms of silence, event REANCHOR).
    if (r > 0 && running && phase == PH_LOCKED && !ph.hold && (was || link_phase_active(&ph))) reanchor();
    return true;
}

void link_input_get_phase(uint8_t o[LINK_PHASE_LEN]) {
    refresh_limits();
    link_phase_get(&ph, o);
    if (!running || !link_phase_active(&ph)) o[7] = LINK_PHASE_FREE;
}

bool link_input_rate_ready(void) {
    pull_in = false;
    if (!link_phase_active(&ph)) return true;
    float d = soft_vcxo_current_ppm() - input_servo_feedforward_ppm();
    // Not yet: the decoder keeps up (deliver() drops the rows), so the
    // prefill starts from current frames once the VCXO is there.
    pull_in = link_meas_hz(&meas) == 0.0f || input_servo_feedforward_ppm() == 0.0f ||
              d <= -LINK_RATE_READY_PPM || d >= LINK_RATE_READY_PPM;
    return !pull_in;
}

void link_input_on_drain(void) {
    link_phase_drain(&ph);
    drop_rows = 0;
    pull_in = false;
    // The prefill target follows the delay (output_latency_apply() asks
    // link_input_ring_target()); the outputs are drained, so this is inside
    // the reset bracket that call requires.
    output_latency_apply(audio_state.freq ? audio_state.freq : 48000u, INPUT_SOURCE_LINK);
    if (link_phase_active(&ph)) ph.state = LINK_PHASE_ANCHORING;
}

bool link_input_anchor_ready(void) {
    if (!link_phase_active(&ph) || !ph.timeline) return true;
    float since;
    if (!arrival_since(ph.first, &since)) return false;
    float wait = link_phase_anchor_wait(&ph, since);
    // The last millisecond in place (main-loop passes are shorter), so the
    // start lands on the delay to a fraction of a frame; the verification
    // corrects a late start, the servo trims the rest.
    if (wait > (float)(detected_rate / 1000u)) return false;
    uint32_t t0 = time_us_32();
    while (wait > 0.0f && time_us_32() - t0 < ANCHOR_SPIN_US) {
        if (!arrival_since(ph.first, &since)) return false;
        wait = link_phase_anchor_wait(&ph, since);
    }
    return true;
}

void link_input_on_enabled(void) {
    link_phase_started(&ph);
}

// F now: phase-locked from the published reference and the live DMA
// position; in holdover extrapolated at the stream's rate.
static uint64_t frame_now(bool *valid) {
    *valid = false;
    if (!running) return 0;
    if (ph.hold) {
        *valid = true;
        uint32_t fs = detected_rate ? detected_rate : audio_state.freq;
        return hold_f + (time_us_64() - hold_us) * fs / 1000000u;
    }
    if (phase != PH_LOCKED) return 0;
    *valid = true;
    float since;
    if (arrival_since(pub_f, &since) && since >= 0.0f) return pub_f + (uint64_t)since;
    return lf.frame;
}

void link_input_get_frame_counter(uint8_t o[LINK_FRAME_COUNTER_LEN]) {
    memset(o, 0, LINK_FRAME_COUNTER_LEN);
    uint32_t save = save_and_disable_interrupts();
    bool valid;
    uint64_t f = frame_now(&valid);
    uint32_t t = time_us_32();
    float fill;
    uint32_t played = 0;
    if (!slot_ring_play_position(0, &fill, &played)) played = 0;
    restore_interrupts(save);
    put32(o, (uint32_t)f);
    put32(o + 4, (uint32_t)(f >> 32));
    put32(o + 8, t);
    put32(o + 12, played);
}

bool link_input_apply(const uint8_t p[LINK_APPLY_LEN]) {
    link_apply_op_t op;
    if (!link_apply_parse(&apply_lim, p, &op)) return false;
    if (op.op == LINK_APPLY_CANCEL_ALL) {
        while (aq.n) {
            link_apply_pop(&aq, &op);
            apply_report(op.op, LINK_APPLY_CANCELLED, op.frame);
        }
        return true;
    }
    if (link_input_get_state() != LINK_INPUT_LOCKED) return false;   // no lock, no F
    return link_apply_push(&aq, &op);
}

void link_input_get_apply(uint8_t o[LINK_APPLY_LEN]) {
    link_apply_in(&aq, o);
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
    put32(o + 12, 1u << 0 | 1u << 1 | 1u << 5);  // PHASE_LOCK, APPLY_AT_FRAME, LOS_PIN
    refresh_limits();
    put16(o + 16, ph.d_min);
    put16(o + 18, ph.d_max);
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
    bool fv;
    uint64_t f = frame_now(&fv);
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
    putf(o + 44, ph.state == LINK_PHASE_LOCKED && ph.e_valid ? ph.e : 0.0f);
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
    putf(o + 4, ff);                        // ppm_ff
    putf(o + 8, ph.pi.trim - ph.pi.integ);  // ppm_phase (KP x smoothed e)
    putf(o + 12, ph.pi.integ);
    uint64_t bits = running ? ring_bits_read(NULL) : 0;
    put32(o + 16, (uint32_t)((bits / 8u) % (LINK_RING_WORDS * 4u)));
    put32(o + 20, (uint32_t)((uint64_t)lf.p / 8u % (LINK_RING_WORDS * 4u)));
    float fill;
    uint32_t played = 0;
    if (!slot_ring_play_position(0, &fill, &played)) played = 0;
    put32(o + 24, played);                  // output frames played (slot 0)
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
