/*
 * sender_tx.c - DSPi alink sender (WP LK4): the alink engine on core 1
 *
 * Owns every PIO state machine and DMA channel of the alinks:
 *
 *  - alinks 0-3 on PIO0 SM0-3, alinks 4-7 on PIO1 SM0-3, each running LK2's
 *    one-instruction alink_tx_N for the class; the SYNC pin on PIO2 SM1 with
 *    the same program, fed from a parallel ring, so its pulse edge sits on a
 *    comma's first bit; all of them started on one cycle with
 *    pio_enable_sm_multi_mask_in_sync() across the three blocks;
 *  - one DMA channel per state machine reading the same 16 KB word ring
 *    (ring-wrapped read address, endless transfer count: RP2350 only);
 *  - the encoder (sender_alink.c) keeps the ring about 0.5 ms ahead of the
 *    DMA; a disabled alink keeps its state machine running (so it stays in
 *    step) and has its GPIO switched to a low SIO output;
 *  - fan-out mode: alink_rx_N on PIO2 SM0 into a 32 KB ring, the fan-out core
 *    (sender_fanout.c), the alignment that puts each re-sent frame a fixed
 *    number of frames after it arrived, and the soft-VCXO servo that keeps
 *    that delay.
 *
 * Core 0 never touches this hardware; it posts requests (sender.h).
 */

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/structs/m33.h"

#include "sender.h"
#include "sender_fanout.h"
#include "soft_vcxo.h"

#include "alink_rx_10.pio.h"
#include "alink_rx_20.pio.h"
#include "alink_rx_5.pio.h"
#include "alink_tx_10.pio.h"
#include "alink_tx_20.pio.h"
#include "alink_tx_5.pio.h"

const uint8_t snd_alink_pins[SND_MAX_ALINKS] = SND_ALINK_PIN_TABLE;
static const uint8_t alink_drive[SND_MAX_ALINKS] = SND_ALINK_DRIVE_TABLE;

#define TX_RING_WORDS   4096u           // 16 KB: 8.5 ms at L1, 2.1 ms at L3
#define TX_RING_BITS    14u             // log2 of the ring in bytes (DMA ring wrap)
#define RX_RING_WORDS   8192u           // 32 KB: 17 ms at L1, 4.3 ms at L3
#define RX_RING_BITS    15u             // log2 bytes
#define SYNC_SM         1u              // PIO2
#define RX_SM           0u              // PIO2
#define IDLE_WORD       0xAAAAAAAAu

_Static_assert(4u * TX_RING_WORDS == 1u << TX_RING_BITS, "TX ring wrap bits");
_Static_assert(4u * RX_RING_WORDS == 1u << RX_RING_BITS, "RX ring wrap bits");
// The frame log (SND_LOG_LEN) holds the frames between the wire and the
// encoder: 0.5 ms of lead is at most 48 frames (96 kHz), a fan-out's a few.

static uint32_t tx_ring[TX_RING_WORDS] __attribute__((aligned(4u * TX_RING_WORDS)));
static uint32_t sync_ring[TX_RING_WORDS] __attribute__((aligned(4u * TX_RING_WORDS)));
static uint32_t rx_ring[RX_RING_WORDS] __attribute__((aligned(4u * RX_RING_WORDS)));

// ---------------------------------------------------------------------------
// Programs
// ---------------------------------------------------------------------------

typedef struct {
    unsigned n;                         // cycles per bit
    const pio_program_t *tx, *rx;
    pio_sm_config (*tx_cfg)(uint);
    pio_sm_config (*rx_cfg)(uint);
    unsigned rx_wrap_target;
} prog_set_t;

static const prog_set_t progs[4] = {
    { 0 },
    { 20, &alink_tx_20_program, &alink_rx_20_program, alink_tx_20_program_get_default_config,
      alink_rx_20_program_get_default_config, alink_rx_20_wrap_target },
    { 10, &alink_tx_10_program, &alink_rx_10_program, alink_tx_10_program_get_default_config,
      alink_rx_10_program_get_default_config, alink_rx_10_wrap_target },
    { 5, &alink_tx_5_program, &alink_rx_5_program, alink_tx_5_program_get_default_config,
      alink_rx_5_program_get_default_config, alink_rx_5_wrap_target },
};

static PIO const pios[3] = { pio0, pio1, pio2 };

// ---------------------------------------------------------------------------
// Engine state (core 1 only)
// ---------------------------------------------------------------------------

static struct {
    bool     inited;
    int      dma_alink[SND_MAX_ALINKS];
    int      dma_sync, dma_rx;
    // Loaded TX program per block (class) and offset.
    uint8_t  tx_loaded[3];
    uint     tx_off[3];
    uint8_t  rx_loaded;                 // class of the RX program on PIO2, 0 none
    uint     rx_off;
    // Running configuration.
    uint8_t  cls;
    uint8_t  sync_pin;                  // pin the SYNC SM drives, 0xFF none
    uint16_t enable;                    // GPIOs switched to PIO
    bool     fanout;
    // Ring tracking.
    uint32_t tx_last, tx_passes;
    uint32_t rx_last, rx_passes;
    uint32_t lead_target;               // words
    uint32_t pulse_bits;                // SYNC pulse width
    uint64_t pps_next;                  // next frame with F mod Fs = 0
    uint32_t pps_fs;
    snd_tone_t tone;
    uint32_t tone_fs;
    // Encoder cost.
    uint32_t cyc_sum, cyc_n;
} eng;

static snd_sink_t sink;
static snd_log_t flog;

uint32_t sender_sys_hz(void) {
    return clock_get_hz(clk_sys);
}

bool sender_pin_used(uint8_t pin, uint8_t ignore_sync) {
    if (pin == 0xFF) return false;
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++) {
        if (snd_alink_pins[n] == pin || SND_TXDIS_PIN(n) == pin || SND_LOS_PIN(n) == pin) return true;
    }
    if (!ignore_sync && snd.cfg.sync_pin == pin) return true;
    if (snd.config[0]) {
        uint8_t rx = snd.config[1] == 0xFF ? SND_FANOUT_RX_PIN : snd.config[1];
        if (pin == rx || pin == snd.config[4]) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Cycle counter (this core's DWT)
// ---------------------------------------------------------------------------

static inline uint32_t cyccnt(void) {
    return m33_hw->dwt_cyccnt;
}

static void cyccnt_enable(void) {
    m33_hw->demcr |= M33_DEMCR_TRCENA_BITS;
    m33_hw->dwt_cyccnt = 0;
    m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;
}

// ---------------------------------------------------------------------------
// Ring positions
// ---------------------------------------------------------------------------

// Words the alink DMA has read (absolute).  Alink 0's channel stands for all:
// every alink SM runs on the same cycle from the same ring.
static uint64_t tx_read_abs(void) {
    uint32_t idx = (dma_hw->ch[eng.dma_alink[0]].read_addr - (uint32_t)(uintptr_t)tx_ring) >> 2;
    if (idx < eng.tx_last) eng.tx_passes++;
    eng.tx_last = idx;
    return (uint64_t)eng.tx_passes * TX_RING_WORDS + idx;
}

// The bit on the wire now, within about a word: words read less those
// still in alink 0's TX FIFO, less half the word in its shift register.
static uint64_t tx_wire_bit(uint64_t read_words) {
    uint32_t lvl = pio_sm_get_tx_fifo_level(pio0, 0);
    uint64_t w = read_words > lvl + 1u ? read_words - lvl - 1u : 0;
    return w * 32u + 16u;
}

static uint64_t rx_write_abs(void) {
    uint32_t idx = (dma_hw->ch[eng.dma_rx].write_addr - (uint32_t)(uintptr_t)rx_ring) >> 2;
    if (idx < eng.rx_last) eng.rx_passes++;
    eng.rx_last = idx;
    return (uint64_t)eng.rx_passes * RX_RING_WORDS + idx;
}

// ---------------------------------------------------------------------------
// Hardware setup
// ---------------------------------------------------------------------------

static void engine_init_once(void) {
    if (eng.inited) return;
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++) {
        pio_sm_claim(pios[n / 4u], n % 4u);
        eng.dma_alink[n] = dma_claim_unused_channel(true);
    }
    pio_sm_claim(pio2, SYNC_SM);
    pio_sm_claim(pio2, RX_SM);
    eng.dma_sync = dma_claim_unused_channel(true);
    eng.dma_rx = dma_claim_unused_channel(true);
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++) {
        uint pin = snd_alink_pins[n];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
        uint8_t td = SND_TXDIS_PIN(n), los = SND_LOS_PIN(n);
        if (td != 0xFF) {
            gpio_init(td);
            gpio_set_dir(td, GPIO_OUT);
            gpio_put(td, 1);                // off until the host enables it
        }
        if (los != 0xFF) {
            gpio_init(los);
            gpio_set_dir(los, GPIO_IN);
            gpio_pull_up(los);
        }
    }
    eng.sync_pin = 0xFF;
    eng.inited = true;
}

static void stop_all(void) {
    pio_set_sm_mask_enabled(pio0, 0xFu, false);
    pio_set_sm_mask_enabled(pio1, 0xFu, false);
    pio_set_sm_mask_enabled(pio2, (1u << SYNC_SM) | (1u << RX_SM), false);
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++) dma_channel_abort((uint)eng.dma_alink[n]);
    dma_channel_abort((uint)eng.dma_sync);
    dma_channel_abort((uint)eng.dma_rx);
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++) {
        pio_sm_clear_fifos(pios[n / 4u], n % 4u);
        pio_sm_restart(pios[n / 4u], n % 4u);
    }
    pio_sm_clear_fifos(pio2, SYNC_SM);
    pio_sm_clear_fifos(pio2, RX_SM);
    snd.engine = SND_ENG_OFF;
    snd.run_cls = 0;
}

static void load_tx_program(unsigned blk, unsigned cls) {
    PIO pio = pios[blk];
    if (eng.tx_loaded[blk] == cls) return;
    if (eng.tx_loaded[blk]) pio_remove_program(pio, progs[eng.tx_loaded[blk]].tx, eng.tx_off[blk]);
    eng.tx_off[blk] = pio_add_program(pio, progs[cls].tx);
    eng.tx_loaded[blk] = (uint8_t)cls;
}

static void config_tx_sm(PIO pio, uint sm, uint off, unsigned cls, uint pin) {
    pio_sm_config c = progs[cls].tx_cfg(off);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_out_shift(&c, false, true, 32);   // MSB first, autopull 32
    sm_config_set_clkdiv_int_frac8(&c, 1, 0);
    // FIFO not joined: 4 words is enough at L3 and keeps the fan-out's
    // input-to-output delay short.
    pio_sm_init(pio, sm, off, &c);
    pio_sm_set_pins_with_mask(pio, sm, 0, 1u << pin);
    pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin, 1u << pin);
}

static void config_tx_dma(int ch, const uint32_t *ring, PIO pio, uint sm) {
    dma_channel_config c = dma_channel_get_default_config((uint)ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, TX_RING_BITS);          // wrap the read address at 16 KB
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    dma_channel_configure((uint)ch, &c, &pio->txf[sm], ring, dma_encode_endless_transfer_count(), false);
}

// GPIO functions, TX_DISABLE levels and the SYNC pin, from snd.cfg.
static void apply_pins(void) {
    uint32_t s = snd_lock_take();
    uint16_t en = snd.cfg.alink_enable, txdis = snd.cfg.txdis_mask;
    uint8_t sync = snd.cfg.sync_pin;
    snd_lock_give(s);
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++) {
        uint pin = snd_alink_pins[n];
        bool on = (en >> n) & 1u;
        if (on && !((eng.enable >> n) & 1u)) {
            pio_gpio_init(pios[n / 4u], pin);
            gpio_set_drive_strength(pin, alink_drive[n] >= 12 ? GPIO_DRIVE_STRENGTH_12MA :
                                         alink_drive[n] >= 8 ? GPIO_DRIVE_STRENGTH_8MA : GPIO_DRIVE_STRENGTH_4MA);
            gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        } else if (!on && ((eng.enable >> n) & 1u)) {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            gpio_put(pin, 0);
        }
        uint8_t td = SND_TXDIS_PIN(n);
        if (td != 0xFF) gpio_put(td, (txdis >> n) & 1u);
    }
    eng.enable = en;
    if (sync != eng.sync_pin) {
        // The SYNC SM never stops (it would fall out of step with the alinks):
        // only its OUT base moves, and the pad's output enable is forced on,
        // since its pin direction was set for another pin before it started.
        if (eng.sync_pin != 0xFF) {
            gpio_set_oeover(eng.sync_pin, GPIO_OVERRIDE_NORMAL);
            gpio_init(eng.sync_pin);
        }
        if (sync != 0xFF) {
            pio_sm_set_out_pins(pio2, SYNC_SM, sync, 1);
            pio_gpio_init(pio2, sync);
            gpio_set_oeover(sync, GPIO_OVERRIDE_HIGH);
            gpio_set_slew_rate(sync, GPIO_SLEW_RATE_FAST);
        }
        eng.sync_pin = sync;
    }
}

// ---------------------------------------------------------------------------
// Fan-out state
// ---------------------------------------------------------------------------

#define FO_ACQ          0u      // acquiring (class detection, lock)
#define FO_SYNC         1u      // locked, waiting for the rate estimate and a superframe start
#define FO_RUN          2u      // re-sending

static struct {
    unsigned phase;
    snd_fo_t fo;
    snd_dectab_t dec;
    uint8_t  in_cls;            // class of the RX program
    bool     auto_cls;
    uint64_t prog_start;        // RX bit where the current program's data starts
    uint32_t tries;
    // Rate estimate (feed-forward): RX bits against the crystal timer.
    uint64_t anchor_bits[2];
    uint32_t anchor_us[2];
    bool     anchored;
    float    ppm_ff;
    bool     ff_valid;          // ppm_ff holds a good estimate (kept through a loss)
    bool     ff_fresh;          // ... measured since the last acquisition
    // Delay servo.
    unsigned delay_frames;
    uint64_t delay_bits;
    int64_t  err_sum;
    uint32_t err_n;
    float    integ;
    float    err_frames;        // last tick's mean
    uint32_t tick_us;
    uint32_t realigns;
    uint32_t overruns;          // parser fell half a ring behind (too slow for this class and C)
    uint32_t start_us;
} fo;

static alink_rx_t fo_scratch;
static uint32_t fo_snap[(10u * 140u * 68u + 31u) / 32u + 2u];

static void rx_program(unsigned cls) {
    pio_sm_set_enabled(pio2, RX_SM, false);
    if (eng.rx_loaded) pio_remove_program(pio2, progs[eng.rx_loaded].rx, eng.rx_off);
    eng.rx_off = pio_add_program(pio2, progs[cls].rx);
    eng.rx_loaded = (uint8_t)cls;
    uint8_t pin = snd.config[1] == 0xFF ? SND_FANOUT_RX_PIN : snd.config[1];
    pio_sm_config c = progs[cls].rx_cfg(eng.rx_off);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv_int_frac8(&c, 1, 0);
    pio_sm_init(pio2, RX_SM, eng.rx_off + progs[cls].rx_wrap_target, &c);
    pio_sm_exec(pio2, RX_SM, pio_encode_mov_not(pio_osr, pio_null));   // OSR all ones
    pio_sm_set_enabled(pio2, RX_SM, true);
    fo.in_cls = (uint8_t)cls;
    fo.prog_start = rx_write_abs() * 32u + 64u;
}

static void rx_start(void) {
    uint8_t pin = snd.config[1] == 0xFF ? SND_FANOUT_RX_PIN : snd.config[1];
    gpio_init(pin);                     // SIO function clears the RP2350 pad ISO bit
    gpio_set_dir(pin, GPIO_IN);
    gpio_set_input_enabled(pin, true);
    gpio_set_input_hysteresis_enabled(pin, true);
    gpio_disable_pulls(pin);            // RP2350-E9: a pull-down can latch the input high
    dma_channel_config c = dma_channel_get_default_config((uint)eng.dma_rx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RX_RING_BITS);           // wrap the write address at 32 KB
    channel_config_set_dreq(&c, pio_get_dreq(pio2, RX_SM, false));
    dma_channel_configure((uint)eng.dma_rx, &c, rx_ring, &pio2->rxf[RX_SM],
                          dma_encode_endless_transfer_count(), true);
    eng.rx_last = 0;
    eng.rx_passes = 0;
    memset(&fo, 0, sizeof(fo));
    snd_dectab_init(&fo.dec);
    fo.auto_cls = snd.config[2] == 0;
    rx_program(fo.auto_cls ? ALINK_CLASS_L1 : snd.config[2]);
    fo.tick_us = time_us_32();
}

// ---------------------------------------------------------------------------
// Start, restart
// ---------------------------------------------------------------------------

static void encode_one(void);

static void fill_idles(void) {
    for (unsigned i = 0; i < TX_RING_WORDS; i++) {
        tx_ring[i] = IDLE_WORD;
        sync_ring[i] = 0;
    }
}

static void engine_restart(void) {
    engine_init_once();
    stop_all();
    soft_vcxo_set_ppm(0.0f);

    uint32_t s = snd_lock_take();
    unsigned cls = snd.cfg.cls;
    bool fanout = snd.config[0] != 0;
    // The stream continues its frame counter across a restart.
    alink_tx_t *tx = &snd.tx;
    uint64_t first = tx->started ? tx->frame : 0;
    uint8_t flags = tx->flags;
    uint64_t af = tx->apply_frame;
    uint8_t aop = tx->apply_op, aarg = tx->apply_arg;
    int16_t aval = tx->apply_value;
    alink_tx_init(tx, cls, snd.cfg.rate, snd.cfg.channels, first, snd.sender_id);
    tx->flags = flags;
    // A pending RATE is already in cfg (the layout above); another op keeps
    // its frame if a superframe can still carry it.
    if (aop != ALINK_OP_NONE && aop != ALINK_OP_RATE) (void)alink_tx_schedule(tx, af, aop, aarg, aval);
    snd_lock_give(s);

    for (unsigned b = 0; b < 3; b++) load_tx_program(b, cls);
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++)
        config_tx_sm(pios[n / 4u], n % 4u, eng.tx_off[n / 4u], cls, snd_alink_pins[n]);
    // The SYNC SM reaches a pad only once apply_pins() gives it one.
    config_tx_sm(pio2, SYNC_SM, eng.tx_off[2], cls, eng.sync_pin != 0xFF ? eng.sync_pin : 0u);

    fill_idles();
    snd_sink_init(&sink, tx_ring, TX_RING_WORDS, sync_ring);
    snd_log_init(&flog, first);
    uint32_t baud = alink_class_baud(cls);
    eng.cls = (uint8_t)cls;
    eng.fanout = fanout;
    eng.pulse_bits = baud / 100000u;                         // 10 us
    eng.lead_target = fanout ? 2u : (baud / 32u / 2000u > 64u ? baud / 32u / 2000u : 64u);   // 0.5 ms
    eng.pps_fs = 0;
    eng.tx_last = 0;
    eng.tx_passes = 0;

    for (unsigned n = 0; n < SND_MAX_ALINKS; n++)
        config_tx_dma(eng.dma_alink[n], tx_ring, pios[n / 4u], n % 4u);
    config_tx_dma(eng.dma_sync, sync_ring, pio2, SYNC_SM);

    snd.start_frame = first;
    snd.wire_frame = first;
    if (fanout) {
        rx_start();
        snd.engine = SND_ENG_FANOUT_IDLE;
        sink.words = eng.lead_target;               // the ring holds idles
        sink.sync = NULL;                           // no SYNC pulses in fan-out: the ring stays zero
    } else {
        snd.engine = SND_ENG_MASTER;
        while (sink.words < eng.lead_target) encode_one();
    }

    // DMA first (it fills the FIFOs), then every SM on the same cycle.
    uint32_t dmask = 1u << eng.dma_sync;
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++) dmask |= 1u << eng.dma_alink[n];
    dma_start_channel_mask(dmask);
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++)
        while (!pio_sm_is_tx_fifo_full(pios[n / 4u], n % 4u)) tight_loop_contents();
    while (!pio_sm_is_tx_fifo_full(pio2, SYNC_SM)) tight_loop_contents();
    pio0->fdebug = 0xFu << PIO_FDEBUG_TXSTALL_LSB;
    pio1->fdebug = 0xFu << PIO_FDEBUG_TXSTALL_LSB;
    pio_enable_sm_multi_mask_in_sync(pio1, 0xFu, 0xFu, 1u << SYNC_SM);
    snd.run_cls = (uint8_t)cls;
    eng.enable = 0;
    apply_pins();
}

// ---------------------------------------------------------------------------
// Master mode: encode ahead of the DMA
// ---------------------------------------------------------------------------

static bool usb_ready(uint32_t fs) {
    return snd.usb_alt && snd.usb_rate == fs && snd.cfg.test_pattern == SND_TP_AUDIO;
}

// Samples of the next frame (lock held; tx describes the frame).
static void fill_samples(const alink_tx_t *tx, int32_t *smp) {
    unsigned c = tx->channels;
    uint32_t fs = alink_rate_hz(tx->rate);
    uint8_t tp = snd.cfg.test_pattern;
    memset(smp, 0, sizeof(int32_t) * c);
    if (tp == SND_TP_TONE) {
        if (eng.tone_fs != fs) {
            snd_tone_set_rate(&eng.tone, fs);
            eng.tone_fs = fs;
        }
        int32_t v = snd_tone_next(&eng.tone);
        for (unsigned ch = 0; ch < c; ch++) smp[ch] = v;
        snd.usb_playing = false;
        return;
    }
    uint32_t wr = snd_fifo.wr, rd = snd_fifo.rd, fill = wr - rd, target = snd_fifo_target(fs);
    if (!usb_ready(fs)) {
        // Hold the FIFO at its target so playing starts with the right fill.
        if (fill > target) snd_fifo.rd = wr - target;
        snd.usb_playing = false;
        return;
    }
    if (!snd.usb_playing) {
        if (fill < target) return;
        snd.usb_playing = true;
    }
    if (fill == 0) {
        snd.usb_underruns = snd.usb_underruns + 1u;
        snd.usb_playing = false;
        return;
    }
    __dmb();
    const int32_t *in = snd_fifo.s[rd & (SND_FIFO_FRAMES - 1u)];
    unsigned n = c < SND_USB_CHANNELS ? c : SND_USB_CHANNELS;
    for (unsigned ch = 0; ch < n; ch++) smp[ch] = in[ch];
    __dmb();
    snd_fifo.rd = rd + 1u;
}

static void encode_one(void) {
    int32_t smp[ALINK_MAX_CHANNELS];
    uint32_t c0 = cyccnt();
    uint32_t s = snd_lock_take();
    alink_tx_t *tx = &snd.tx;
    uint64_t f = tx->frame;
    uint32_t fs = alink_rate_hz(tx->rate);
    fill_samples(tx, smp);
    uint8_t tp = snd.cfg.test_pattern;
    tx->flags = (uint8_t)((tx->flags & ~(ALINK_SF_FLAG_PRBS | ALINK_SF_FLAG_USB_LIVE | ALINK_SF_FLAG_MUTE)) |
                          (tp == SND_TP_PRBS ? ALINK_SF_FLAG_PRBS : 0) |
                          (snd.usb_playing ? ALINK_SF_FLAG_USB_LIVE : 0) |
                          (snd.cfg.flags & 1u ? ALINK_SF_FLAG_MUTE : 0));
    // SYNC: once a second (F mod Fs = 0) and at a SYNC_PULSE apply's frame.
    if (eng.pps_fs != fs) {
        eng.pps_fs = fs;
        eng.pps_next = (f + fs - 1u) / fs * fs;
    }
    bool pulse = f == eng.pps_next;
    if (pulse) eng.pps_next += fs;
    if (snd.sync_pulse_frame && f >= snd.sync_pulse_frame) {
        pulse = pulse || f == snd.sync_pulse_frame;
        snd.sync_pulse_frame = 0;
    }
    if (pulse) {
        sink.pulse_from = snd_sink_bits(&sink);
        sink.pulse_to = sink.pulse_from + eng.pulse_bits;
    }
    if (tp == SND_TP_IDLE) snd_idle_frame(tx, &sink, &flog);
    else snd_frame(tx, &snd_tables, smp, &sink, &flog);
    snd_lock_give(s);
    uint32_t dc = cyccnt() - c0;
    eng.cyc_sum += dc;
    if (dc > snd.enc_cycles_max) snd.enc_cycles_max = dc;
    if (++eng.cyc_n == 1024u) {
        snd.enc_cycles_avg = eng.cyc_sum / 1024u;
        eng.cyc_sum = 0;
        eng.cyc_n = 0;
    }
}

static void master_service(void) {
    uint64_t r = tx_read_abs();
    snd.wire_frame = snd_log_wire(&flog, tx_wire_bit(r));
    if (r + 1u > sink.words) snd.tx_late = snd.tx_late + 1u;  // the DMA read words not yet written
    // Worst frame: 140 symbols = 44 words.
    while (sink.words < r + eng.lead_target && sink.words + 48u < r + TX_RING_WORDS)
        encode_one();
    snd.lead_words = (uint32_t)(sink.words > r ? sink.words - r : 0);
}

// ---------------------------------------------------------------------------
// Fan-out mode
// ---------------------------------------------------------------------------

// The delay in frames: the whole input frame must have arrived before its
// re-sent copy starts, plus the RX word, the TX FIFO and shift register (5
// words) and 6 us of processing.  2 frames at 44.1/48 kHz, 2-3 at 96 kHz.
static unsigned fanout_delay_frames(unsigned in_cls, unsigned out_cls, unsigned rate) {
    uint32_t fs = alink_rate_hz(rate);
    uint64_t lat_ns = 5ull * 32u * 1000000000ull / alink_class_baud(out_cls) +
                      32ull * 1000000000ull / alink_class_baud(in_cls) + 6000u;
    uint64_t frame_ns = 1000000000ull / fs;
    unsigned k = (unsigned)((frame_ns + lat_ns + frame_ns - 1u) / frame_ns);
    return k < 2u ? 2u : k;
}

static void fo_idle_fill(uint64_t r) {
    // Keep a couple of idle words ahead of the DMA; if it overtook us (a
    // long parse), the words it re-read were idles: skip ahead.
    if (sink.words + 1u < r) {
        sink.words = r + 1u;
        sink.n = 0;
        sink.cur = 0;
    }
    while (sink.words < r + eng.lead_target) snd_put(&sink, IDLE_WORD, 32);
}

static void fo_go_idle(void) {
    fo.fo.out_on = false;
    fill_idles();
    sink.n = 0;
    sink.cur = 0;
    snd.engine = SND_ENG_FANOUT_IDLE;
}

static void fo_rate_tick(uint64_t rx_bits) {
    uint32_t now = time_us_32();
    if (!fo.anchored) {
        fo.anchor_bits[0] = fo.anchor_bits[1] = rx_bits;
        fo.anchor_us[0] = fo.anchor_us[1] = now;
        fo.anchored = true;
        return;
    }
    // Two anchors 8 s apart: the estimate spans 8-16 s once warm, 1 s at first.
    if (now - fo.anchor_us[1] >= 8000000u) {
        fo.anchor_bits[0] = fo.anchor_bits[1];
        fo.anchor_us[0] = fo.anchor_us[1];
        fo.anchor_bits[1] = rx_bits;
        fo.anchor_us[1] = now;
    }
    uint32_t dt = now - fo.anchor_us[0];
    if (dt < 1000000u) return;
    double rate = (double)(rx_bits - fo.anchor_bits[0]) * 1e6 / (double)dt;
    fo.ppm_ff = (float)((rate / (double)alink_class_baud(fo.in_cls) - 1.0) * 1e6);
    fo.ff_valid = fo.ppm_ff > -300.0f && fo.ppm_ff < 300.0f;
    fo.ff_fresh = fo.ff_valid;
}

static void fo_servo_tick(void) {
    float ppm = fo.ff_valid ? fo.ppm_ff : 0.0f;
    if (fo.phase == FO_RUN && fo.err_n) {
        // Delay error in frames: positive = the copy leaves late: run faster.
        float e = (float)((double)fo.err_sum / (double)fo.err_n) /
                  (float)(10u * alink_frame_symbols_min(eng.cls, fo.fo.rate));
        fo.err_frames = e;
        fo.integ += 0.1f * e;
        if (fo.integ > 50.0f) fo.integ = 50.0f;
        if (fo.integ < -50.0f) fo.integ = -50.0f;
        ppm += 4.0f * e + fo.integ;
        if (e > 0.5f || e < -0.5f) {
            fo.realigns++;
            fo_go_idle();
            fo.phase = FO_SYNC;
        }
    }
    fo.err_sum = 0;
    fo.err_n = 0;
    soft_vcxo_set_ppm(ppm);
}

static void fanout_service(void) {
    uint64_t r = tx_read_abs();
    uint64_t wire = tx_wire_bit(r);
    uint64_t rx_bits = rx_write_abs() * 32u;
    snd.wire_frame = snd_log_wire(&flog, wire);
    snd_ring_t ring = { .ring = rx_ring, .mask = RX_RING_WORDS - 1u };

    if (fo.phase == FO_ACQ) {
        fo_idle_fill(r);
        uint32_t need = snd_fo_acquire_bits(fo.in_cls);
        if (rx_bits < fo.prog_start + need) return;
        // Snapshot the newest `need` bits (word aligned) and acquire.
        uint64_t w1 = rx_bits / 32u, w0 = w1 - (need + 31u) / 32u;
        for (uint64_t w = w0; w < w1; w++) fo_snap[w - w0] = rx_ring[w & (RX_RING_WORDS - 1u)];
        alink_rx_stats_t keep = fo.fo.st;          // counters run across acquisitions
        snd_fo_init(&fo.fo, fo.in_cls, eng.cls, &snd_tables, &fo.dec);
        fo.fo.st = keep;
        bool ok = snd_fo_acquire(&fo.fo, fo_snap, w0 * 32u, (uint32_t)((w1 - w0) * 32u), &fo_scratch);
        if (ok) {
            // The acquisition may have taken longer than the ring holds: move
            // to a frame the ring still has (half a ring back).
            uint64_t now = rx_write_abs() * 32u, half = (RX_RING_WORDS * 32u) / 2u;
            ok = snd_fo_project(&fo.fo, now > half ? now - half : 0);
        }
        if (ok) {
            fo.phase = FO_SYNC;
            fo.delay_frames = fanout_delay_frames(fo.in_cls, eng.cls, fo.fo.rate);
            // A fresh rate estimate before re-sending; the VCXO keeps its
            // last trim meanwhile (holdover).
            fo.anchored = false;
            fo.ff_fresh = false;
            fo.start_us = time_us_32();
            return;
        }
        // Next class (auto) or another try at the same one.
        fo.tries++;
        if (fo.auto_cls) rx_program(fo.in_cls % 3u + 1u);
        else fo.prog_start = rx_bits;
        return;
    }

    // Locked: parse every complete frame.  A parser more than half a ring
    // behind cannot keep up at this class and C (the ring would overwrite
    // what it has not read): count a loss and acquire again.
    if (fo.phase == FO_SYNC) fo_idle_fill(r);
    if (fo.fo.state == SND_FO_LOCKED && rx_bits > fo.fo.pos + (RX_RING_WORDS * 32u) / 2u) {
        fo.fo.state = SND_FO_SEARCH;
        fo.fo.out_on = false;
        fo.fo.st.loss_count++;
        fo.overruns++;
    }
    while (fo.fo.state == SND_FO_LOCKED && snd_fo_need(&fo.fo) <= rx_bits) {
        if (fo.phase == FO_SYNC && fo.ff_fresh && fo.fo.sf_valid && fo.fo.frame % ALINK_SF_LEN == 0 &&
            time_us_32() - fo.start_us > 1500000u) {
            // Put this frame where it belongs: delay_frames after its comma
            // arrived.  The idles before it fill the gap.
            uint32_t fs = alink_rate_hz(fo.fo.rate);
            fo.delay_bits = (uint64_t)fo.delay_frames * alink_class_baud(eng.cls) / fs;
            int64_t t = snd_fo_target_bit(fo.fo.pos, rx_write_abs() * 32u, tx_wire_bit(tx_read_abs()),
                                          progs[fo.in_cls].n, progs[eng.cls].n, fo.delay_bits);
            int64_t now_bits = (int64_t)snd_sink_bits(&sink);
            if (t >= now_bits + 8) {
                snd_put_idles(&sink, (unsigned)(t - now_bits));
                snd_fo_start_output(&fo.fo);
                fo.phase = FO_RUN;
                snd.engine = SND_ENG_FANOUT;
                fo.integ = 0;
            } else {
                fo.delay_frames++;          // too late for this delay: one more frame
            }
        }
        uint64_t comma = fo.fo.pos;
        uint64_t at = snd_sink_bits(&sink);
        unsigned res = snd_fo_frame(&fo.fo, &ring, &sink, &flog);
        if (fo.phase == FO_RUN && fo.fo.out_on) {
            int64_t t = snd_fo_target_bit(comma, rx_write_abs() * 32u, tx_wire_bit(tx_read_abs()),
                                          progs[fo.in_cls].n, progs[eng.cls].n, fo.delay_bits);
            fo.err_sum += (int64_t)at - t;
            fo.err_n++;
        }
        if (res & SND_FO_R_LOSS) {
            fo_go_idle();
            fo.anchored = false;            // the input bits are noise until the next lock
            fo.phase = FO_ACQ;
            fo.prog_start = rx_write_abs() * 32u;
            break;
        }
        if (fo.phase == FO_RUN && sink.words < tx_read_abs() + 2u) {
            // The DMA caught up with the re-sent stream: realign.
            snd.tx_late = snd.tx_late + 1u;
            fo.realigns++;
            fo_go_idle();
            fo.phase = FO_SYNC;
        }
    }
    if (fo.fo.state != SND_FO_LOCKED && fo.phase != FO_ACQ) {
        fo_go_idle();
        fo.anchored = false;
        fo.phase = FO_ACQ;
        fo.prog_start = rx_bits;
    }
    if (time_us_32() - fo.tick_us >= 75000u) {
        fo.tick_us = time_us_32();
        if (fo.phase != FO_ACQ) fo_rate_tick(rx_write_abs() * 32u);
        fo_servo_tick();
    }
    snd.lead_words = (uint32_t)(sink.words > r ? sink.words - r : 0);
}

// ---------------------------------------------------------------------------
// Status for core 0
// ---------------------------------------------------------------------------

// EXT_ALINK_STATUS fields of the fan-out input (offsets 0-7, 16-51 less F).
void sender_fanout_status(uint8_t *state, uint8_t *cls, uint8_t *rate, uint8_t *chans, uint8_t *hops,
                          uint8_t *sender, uint8_t *sflags, alink_rx_stats_t *st, float *phase_err,
                          uint32_t *delay_frames, uint32_t *realigns, float *ppm_ff) {
    if (!eng.fanout) {
        *state = 0;
        *cls = *chans = *hops = *sender = *sflags = 0;
        *rate = 0xFF;
        memset(st, 0, sizeof(*st));
        *phase_err = 0;
        *delay_frames = 0;
        *realigns = 0;
        *ppm_ff = 0;
        return;
    }
    bool locked = fo.phase != FO_ACQ && fo.fo.state == SND_FO_LOCKED;
    *state = locked ? 2u : 1u;
    *cls = locked ? fo.in_cls : 0;
    *rate = locked ? fo.fo.rate : 0xFF;
    *chans = locked ? fo.fo.channels : 0;
    *hops = locked ? fo.fo.sf.hops : 0;
    *sender = locked ? fo.fo.sf.sender : 0;
    *sflags = locked ? fo.fo.sf.flags : 0;
    *st = fo.fo.st;
    *phase_err = fo.phase == FO_RUN ? fo.err_frames : 0.0f;
    *delay_frames = fo.delay_frames;
    *realigns = fo.realigns + fo.overruns;
    *ppm_ff = fo.ppm_ff;
}

// ---------------------------------------------------------------------------
// Core 1 entry
// ---------------------------------------------------------------------------

void sender_engine_core1(void) {
    cyccnt_enable();
    for (;;) {
        uint32_t t0 = time_us_32();
        if (snd.restart_req != snd.restart_done) {
            uint32_t g = snd.restart_req;
            engine_restart();
            snd.restart_done = g;
            snd.pins_done = snd.pins_req;
        }
        if (snd.pins_req != snd.pins_done) {
            uint32_t g = snd.pins_req;
            apply_pins();
            snd.pins_done = g;
        }
        if (snd.stats_req != snd.stats_done) {
            uint32_t g = snd.stats_req;
            memset(&fo.fo.st, 0, sizeof(fo.fo.st));
            snd.tx_late = 0;
            snd.tx_stall = 0;
            snd.usb_underruns = 0;
            snd.stats_done = g;
        }
        if (!snd.run_cls) continue;
        if (eng.fanout) fanout_service();
        else master_service();
        // FIFO stalls on the running alink SMs (a stretched bit on the line).
        uint32_t st0 = pio0->fdebug & (0xFu << PIO_FDEBUG_TXSTALL_LSB);
        uint32_t st1 = pio1->fdebug & (0xFu << PIO_FDEBUG_TXSTALL_LSB);
        if (st0 | st1) {
            pio0->fdebug = st0;
            pio1->fdebug = st1;
            snd.tx_stall = snd.tx_stall + 1u;
        }
        uint16_t los = 0;
        for (unsigned n = 0; n < SND_MAX_ALINKS; n++)
            if (SND_LOS_PIN(n) != 0xFF && gpio_get(SND_LOS_PIN(n))) los |= (uint16_t)(1u << n);
        snd.los_mask = los;
        uint32_t dt = time_us_32() - t0;
        if (dt > snd.loop_us_max) snd.loop_us_max = dt;
    }
}
