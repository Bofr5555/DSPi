/*
 * sender.h - DSPi alink sender (WP LK4): state shared by its modules
 *
 * The sender is a Pico 2 that appears to the PC as a USB sound card (UAC1,
 * DSPi's descriptors with two more alts) and sends what it plays on up to 8
 * alink outputs (alink_protocol_spec.md), all from one encoder and one word
 * ring, every alink's state machine started on the same cycle. Its crystal is
 * the system's master clock: the host adapts through the feedback endpoint.
 * In fan-out mode (EXT_ALINK_CONFIG.enable) it is instead slaved to an alink
 * input and re-sends that stream.  SYS2 section 4 is the design.
 *
 * Threads:
 *  - core 0: TinyUSB (UAC1 driver, vendor requests), the main loop (USB rate
 *    follow, persistence, LED), and every EXT_ALINK_* request;
 *  - core 1: the alink engine (sender_tx.c): encoder, DMA ring, SYNC pin,
 *    fan-out, all PIO and DMA of the alinks.
 * The encoder state `snd.tx` is shared under `snd_lock`: core 1 holds it for
 * one frame at a time, core 0 for a validation and a schedule.  Hardware
 * changes core 0 wants (class restart, pins) go to core 1 as requests.
 */

#ifndef SENDER_H
#define SENDER_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/sync.h"

#include "alink_codec.h"
#include "alink_decoder.h"
#include "sender_alink.h"

#if !PICO_RP2350
#error "The alink sender is RP2350-only (PIO2, SYS2 1.2)"
#endif

// ---------------------------------------------------------------------------
// Pins of the v1 carrier (SYS2 6.3)
// ---------------------------------------------------------------------------

// Alink n's data GPIO: TOSLINK 1-2, Broadcom 1-2, SFP 1-2 TD+ (TD- through an
// inverter), spare 7-8.  Alinks 0-3 run on PIO0 SM0-3, alinks 4-7 on PIO1.
#define SND_ALINK_PIN_TABLE     { 2, 3, 4, 5, 6, 8, 14, 15 }
// GPIO drive per alink: the Broadcom transmitters want fast edges (12 mA);
// the others 4 mA (SYS2 10.1 EMI row; the LK3 bench uses 4 mA too).
#define SND_ALINK_DRIVE_TABLE   { 4, 4, 12, 12, 4, 4, 4, 4 }
// SFP control pins: only alinks 4 and 5 (SFP 1, 2) have them.
#define SND_TXDIS_PIN(n)        ((n) == 4 ? 10u : (n) == 5 ? 11u : 0xFFu)
#define SND_LOS_PIN(n)          ((n) == 4 ? 12u : (n) == 5 ? 13u : 0xFFu)
#define SND_FANOUT_RX_PIN       20u     // EXT_ALINK_CONFIG.rx_pin 0xFF on a sender
#define SND_LED_PIN             25u

extern const uint8_t snd_alink_pins[SND_MAX_ALINKS];

// ---------------------------------------------------------------------------
// USB -> encoder sample FIFO (single producer core 0, single consumer core 1)
// ---------------------------------------------------------------------------

#define SND_USB_CHANNELS        8u
#define SND_FIFO_FRAMES         1024u   // a power of two; 21 ms at 48 kHz

typedef struct {
    int32_t s[SND_FIFO_FRAMES][SND_USB_CHANNELS];   // 24-bit samples, slot order
    volatile uint32_t wr;       // frames written (core 0)
    volatile uint32_t rd;       // frames read (core 1)
} snd_fifo_t;

extern snd_fifo_t snd_fifo;

// The fill the encoder waits for before it plays USB audio, and that the
// feedback servo holds: two USB packets.
static inline uint32_t snd_fifo_target(uint32_t fs) { return fs / 500u; }

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

// Engine states (EXT_ALINK_DIAG byte 1).
#define SND_ENG_OFF             0u      // alinks stopped (boot, restart)
#define SND_ENG_MASTER          1u      // sending, sys_clk is the master clock
#define SND_ENG_FANOUT_IDLE     2u      // fan-out: no valid input, sending idles
#define SND_ENG_FANOUT          3u      // fan-out: re-sending the input stream

typedef struct {
    // --- Configuration, written by core 0 under snd_lock ------------------
    snd_txcfg_t cfg;            // EXT_ALINK_TX (cfg.channels = the C in use)
    uint8_t     channels_req;   // C the host asked for; cfg.channels is capped by the rate
    uint8_t     config[8];      // EXT_ALINK_CONFIG
    uint8_t     sender_id;      // superframe byte 7: low byte of the serial

    // --- Requests from core 0 to core 1 (generation counters) --------------
    volatile uint32_t restart_req, restart_done;    // restart the alinks (class, fan-out)
    volatile uint32_t pins_req, pins_done;          // enables, TX_DISABLE, SYNC pin
    volatile uint32_t stats_req, stats_done;        // EXT_ALINK_STATS_RESET

    // --- Encoder, core 1 under snd_lock (core 0 for short critical sections)
    alink_tx_t  tx;
    volatile uint64_t sync_pulse_frame;     // a SYNC_PULSE apply's frame (0 none)

    // --- Published by core 1 ----------------------------------------------
    volatile uint8_t  engine;               // SND_ENG_*
    volatile uint8_t  run_cls;              // class the alinks run (0 stopped)
    volatile uint64_t wire_frame;           // F on the wire now
    volatile uint64_t start_frame;          // F the alinks started at
    volatile uint32_t lead_words;           // ring words ahead of the DMA
    volatile uint32_t enc_cycles_avg;       // encoder cycles per frame, mean of the last 1024
    volatile uint32_t enc_cycles_max;       // worst frame since the last DIAG read
    volatile uint32_t loop_us_max;          // longest core-1 pass since the last DIAG read
    volatile uint32_t tx_late;              // refills that came after the DMA had read the words
    volatile uint32_t tx_stall;             // PIO TX FIFO stalls (a stretched bit on the line)
    volatile uint32_t usb_underruns;        // FIFO empty while playing USB audio
    volatile bool     usb_playing;          // encoder takes samples from the FIFO
    volatile uint16_t los_mask;             // LOS pins of the SFP alinks

    // --- USB, core 0 -------------------------------------------------------
    volatile uint8_t  usb_alt;              // active AS alt, 0 = none
    volatile uint8_t  usb_channels;         // channels of the active alt
    volatile uint32_t usb_rate;             // Hz the host streams at
    volatile bool     usb_rate_follow;      // a USB rate change still to schedule
    volatile uint32_t usb_overruns;         // frames dropped: FIFO full
    volatile uint32_t usb_packets;
    volatile uint32_t usb_gaps;             // packet gaps of 2-50 ms
    volatile uint32_t feedback_10_14;
    volatile int32_t  usb_gain_q15;         // host volume, 0 = muted

    // --- Persistence ---------------------------------------------------------
    volatile bool     save_req;             // REQ_SAVE_OUTPUT_CONFIG (main loop)
} sender_t;

extern sender_t snd;
extern spin_lock_t *snd_lock;
extern snd_tables_t snd_tables;

static inline uint32_t snd_lock_take(void) { return spin_lock_blocking(snd_lock); }
static inline void snd_lock_give(uint32_t s) { spin_unlock(snd_lock, s); }

// ---------------------------------------------------------------------------
// Module entry points
// ---------------------------------------------------------------------------

// sender_tx.c (core 1)
void sender_engine_core1(void);
uint32_t sender_sys_hz(void);
bool sender_pin_used(uint8_t pin, uint8_t ignore_sync);  // an alink, SFP or fan-out pin in use
// The fan-out input for EXT_ALINK_STATUS / DIAG (zeros unless in fan-out mode).
void sender_fanout_status(uint8_t *state, uint8_t *cls, uint8_t *rate, uint8_t *chans, uint8_t *hops,
                          uint8_t *sender, uint8_t *sflags, alink_rx_stats_t *st, float *phase_err,
                          uint32_t *delay_frames, uint32_t *realigns, float *ppm_ff);

// sender_usb.c (core 0)
void sender_usb_init(void);
void sender_usb_task(void);             // main loop: notification keep-alive

// sender_vendor.c (core 0)
// Apply an EXT_ALINK_TX change (validated) that the USB rate requires.
bool sender_follow_usb_rate(void);

// sender_store.c (core 0)
void sender_store_load(void);           // at boot, before the engine starts
bool sender_store_save(void);           // REQ_SAVE_OUTPUT_CONFIG

// sender_main.c
void sender_defaults(void);

#endif // SENDER_H
