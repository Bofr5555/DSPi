/*
 * scope_marker.c - WP SCOPE1: GPIO marker on the shared time base (test
 * builds only, -DALINK_SCOPE_MARKER=ON / -DALINK_SCOPE_MARKER_GPIO=<pin>).  See scope_marker.h and
 * Documentation/Bench/scope_marker.md.
 *
 * How the edge is placed.
 *  1. Main loop (flash, every pass): with interrupts off, read the time base's
 *     next_w (the S or F of the next sample written to the output rings),
 *     slot 0's ring counters (wr_frames, consumed frames + word remainder,
 *     updated from the DMA read address) and the time.  The sample at the
 *     DMA is S_dma = next_w - (wr - consumed): the same relation the phase
 *     servo regulates (e = F_arr - (next_w - fill) - D).
 *  2. The next multiple M of N ahead of S_dma; the DMA read address after it
 *     has fetched M's first word, and the time until then.  Within 2 ms: arm a
 *     dedicated hardware alarm a little more than two DMA words early.
 *  3. Alarm IRQ (RAM, lowest priority, so USB and DMA interrupts preempt it):
 *     spin on the DMA read address; the moment it equals the target, set the
 *     pin.  If the spin was preempted around that moment (two time reads more
 *     than 1 us apart), the address went past, or the spin timed out, the edge
 *     is skipped rather than placed late: a missing edge, never a wrong one.
 *  4. The main loop lowers the pin half a period later (only the rising edge
 *     carries timing).
 */

#include "scope_marker.h"

#if defined(ALINK_SCOPE_MARKER_GPIO)

#include <stddef.h>

#include "audio_input.h"
#include "audio_pipeline.h"
#include "config.h"
#include "usb_audio.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#if DSPI_ALINK_USB
#include "alink_usb.h"
#endif

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

#define MARKER_MASK     (1u << (ALINK_SCOPE_MARKER_GPIO))
#define ARM_WINDOW_US   2000u      // arm only for a target this close
#define ARM_STALE_US    6000u      // an armed alarm that never ran is dropped
#define MARKER_N        ((uint64_t)(ALINK_SCOPE_MARKER_N))

volatile scope_marker_stats_t scope_marker_stats;

static int alarm_num = -1;
static uint alarm_irq;

// Shared with the IRQ: written by the main loop before arming, read after.
static volatile bool armed;
static volatile uint8_t result;            // 0 pending, 1 edge placed, 2 skipped
static volatile uint32_t tgt_addr;         // DMA read address once M's first word is fetched
static volatile uint32_t tgt_mask;         // ring bytes - 1 (power of two)
static volatile uint32_t tgt_dma;          // slot 0's DMA channel
static volatile uint32_t tgt_spin_us;      // spin budget

// Main loop only.
static uint32_t armed_at;
static uint64_t pending_m, last_m;
static bool have_last, high;

// ---------------------------------------------------------------------------
// Alarm IRQ (RAM)
// ---------------------------------------------------------------------------

static void __not_in_flash_func(marker_irq)(void) {
    timer_hw_t *t = PICO_DEFAULT_TIMER_INSTANCE();
    t->intr = 1u << (uint)alarm_num;
    const volatile uint32_t *ra = &dma_hw->ch[tgt_dma].read_addr;
    const uint32_t target = tgt_addr, mask = tgt_mask, half = (tgt_mask + 1u) / 2u, budget = tgt_spin_us;
    uint32_t t0 = t->timerawl, prev = t0;
    uint8_t r = 2u;
    for (;;) {
        uint32_t a = *ra;
        uint32_t now = t->timerawl;
        uint32_t d = (a - target) & mask;
        if (d == 0u) {
            // Placed only if nothing held this loop off around the change:
            // the previous read (not yet there) was at most 1 us before.
            uint32_t s = save_and_disable_interrupts();
            uint32_t n2 = t->timerawl;
            if (*ra == target && n2 - prev <= 1u) {
                sio_hw->gpio_set = MARKER_MASK;
                r = 1u;
            }
            restore_interrupts(s);
            break;
        }
        if (d < half) break;                // went past while we were away
        if (now - t0 > budget) break;       // the estimate was off: give up
        prev = now;
    }
    result = r;
    armed = false;
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

static bool timeline(uint64_t *next_w, bool *locked) {
#if DSPI_ALINK_USB
    if (active_input_source == INPUT_SOURCE_USB) {
        if (!alink_usb.started || !alink_usb.ph.timeline) return false;
        *next_w = alink_usb.ph.next_w;
        *locked = alink_usb.state == ALINK_USB_ST_PHASE_LOCK && alink_usb.ph.state == ALINK_PHASE_LOCKED;
        return true;
    }
#endif
#if DSPI_ALINK
    if (active_input_source == INPUT_SOURCE_ALINK) return alink_input_scope_timeline(next_w, locked);
#endif
    (void)next_w;
    (void)locked;
    return false;
}

typedef struct {
    uint32_t base, mask, wpf, dma, fs;
    uint32_t wr, consumed, rem;
} slot0_t;

// Interrupts off.  Slot 0's ring and DMA position (the phase servo's slot).
static bool slot0(slot0_t *g) {
    if (output_type_switch_in_progress) return false;
    if (output_types[0] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[0];
        if (!inst || !inst->ring || !inst->enabled || !inst->freq) return false;
        (void)audio_i2s_ring_consumed_words(inst);
        g->base = (uint32_t)(uintptr_t)inst->ring;
        g->mask = PICO_AUDIO_I2S_RING_BYTES - 1u;
        g->wpf = 2u;
        g->dma = inst->dma_channel;
        g->fs = inst->freq;
        g->wr = inst->wr_frames;
        g->consumed = inst->consumed_frames;
        g->rem = inst->consumed_word_rem;
    } else {
        audio_spdif_instance_t *inst = spdif_instance_ptrs[0];
        if (!inst || !inst->ring || !inst->enabled || !inst->freq) return false;
        (void)audio_spdif_ring_consumed_words(inst);
        g->base = (uint32_t)(uintptr_t)inst->ring;
        g->mask = PICO_AUDIO_SPDIF_RING_BYTES - 1u;
        g->wpf = 4u;
        g->dma = inst->dma_channel;
        g->fs = inst->freq;
        g->wr = inst->wr_frames;
        g->consumed = inst->consumed_frames;
        g->rem = inst->consumed_word_rem;
    }
    return true;
}

static void lock_pin(bool on) {
#if defined(ALINK_SCOPE_MARKER_LOCK_GPIO)
    gpio_put(ALINK_SCOPE_MARKER_LOCK_GPIO, on);
#else
    (void)on;
#endif
}

void scope_marker_init(void) {
    gpio_init(ALINK_SCOPE_MARKER_GPIO);
    gpio_set_dir(ALINK_SCOPE_MARKER_GPIO, GPIO_OUT);
    gpio_put(ALINK_SCOPE_MARKER_GPIO, 0);
#if defined(ALINK_SCOPE_MARKER_LOCK_GPIO)
    gpio_init(ALINK_SCOPE_MARKER_LOCK_GPIO);
    gpio_set_dir(ALINK_SCOPE_MARKER_LOCK_GPIO, GPIO_OUT);
    gpio_put(ALINK_SCOPE_MARKER_LOCK_GPIO, 0);
#endif
    alarm_num = hardware_alarm_claim_unused(false);
    if (alarm_num < 0) return;              // no alarm free: the marker stays low
    alarm_irq = hardware_alarm_get_irq_num((uint)alarm_num);
    irq_set_exclusive_handler(alarm_irq, marker_irq);
    irq_set_priority(alarm_irq, PICO_LOWEST_IRQ_PRIORITY);
    hw_set_bits(&PICO_DEFAULT_TIMER_INSTANCE()->inte, 1u << (uint)alarm_num);
    irq_set_enabled(alarm_irq, true);
}

static void disarm(void) {
    timer_hw_t *t = PICO_DEFAULT_TIMER_INSTANCE();
    uint32_t s = save_and_disable_interrupts();
    if (armed) {
        t->armed = 1u << (uint)alarm_num;   // write 1 to disarm
        t->intr = 1u << (uint)alarm_num;
        armed = false;
        result = 2u;
    }
    restore_interrupts(s);
}

void scope_marker_service(void) {
    if (alarm_num < 0) return;

    // Collect the IRQ's result.
    if (!armed && result != 0u) {
        if (result == 1u) {
            scope_marker_stats.edges++;
            high = true;
        } else {
            scope_marker_stats.skipped++;
        }
        last_m = pending_m;
        have_last = true;
        result = 0u;
    }
    if (armed) {
        if (time_us_32() - armed_at > ARM_STALE_US) disarm();
        return;
    }

    uint64_t next_w = 0;
    bool locked = false;
    slot0_t g;
    uint32_t s = save_and_disable_interrupts();
    bool ok = timeline(&next_w, &locked) && slot0(&g);
    uint32_t now = time_us_32();
    restore_interrupts(s);
    lock_pin(ok && locked);
    if (!ok) {
        if (high) gpio_put(ALINK_SCOPE_MARKER_GPIO, 0);
        high = false;
        have_last = false;
        return;
    }

    // The sample at the DMA (being fetched now, or next).
    uint64_t s_dma = next_w - (uint64_t)(int64_t)(int32_t)(g.wr - g.consumed);
    if (high && have_last && (int64_t)(s_dma - last_m) >= (int64_t)(MARKER_N / 2u)) {
        gpio_put(ALINK_SCOPE_MARKER_GPIO, 0);
        high = false;
    }

    // Next multiple of N at least two samples ahead (one being fetched).
    uint64_t m = (s_dma + 2u + MARKER_N - 1u) & ~(MARKER_N - 1u);
    if (have_last && m <= last_m) m = last_m + MARKER_N;
    uint32_t frames_ahead = (uint32_t)(m - s_dma);
    uint32_t words_until = frames_ahead * g.wpf + 1u - g.rem;
    uint32_t wps = g.fs * g.wpf;                     // DMA words per second
    uint32_t us_until = (uint32_t)(((uint64_t)words_until * 1000000u) / wps);
    if (us_until > ARM_WINDOW_US) return;

    if (have_last && m > last_m + MARKER_N) scope_marker_stats.missed += (uint32_t)((m - last_m) / MARKER_N - 1u);

    // Ring position of M: counter f_M = consumed + frames_ahead; the read
    // address points one word past the first word of f_M once it is fetched.
    uint32_t f_m = g.consumed + frames_ahead;
    uint32_t words = g.mask / 4u + 1u;               // ring words (power of two)
    uint32_t idx = (f_m * g.wpf + 1u) & (words - 1u);
    uint32_t word_us = (1000000u + wps - 1u) / wps;  // one DMA word, rounded up
    uint32_t early = 2u * word_us + 5u;
    uint32_t lead = us_until > early ? us_until - early : 0u;

    tgt_addr = g.base + idx * 4u;
    tgt_mask = g.mask;
    tgt_dma = g.dma;
    tgt_spin_us = (us_until - lead) + 3u * word_us + 10u;
    pending_m = m;
    result = 0u;

    timer_hw_t *t = PICO_DEFAULT_TIMER_INSTANCE();
    s = save_and_disable_interrupts();
    uint32_t at = now + lead;
    uint32_t tn = t->timerawl;
    // The alarm matches on equality: never set it in the past.
    if ((int32_t)(at - tn) < 3) at = tn + 3u;
    armed = true;
    armed_at = tn;
    t->intr = 1u << (uint)alarm_num;
    t->alarm[alarm_num] = at;
    restore_interrupts(s);
    scope_marker_stats.armed++;
}

#endif // ALINK_SCOPE_MARKER_GPIO
