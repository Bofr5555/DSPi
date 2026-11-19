/*
 * alink_usb.h - USB-direct sync (WP USB1): the firmware side of
 * alink_usb/alink_usb_core.[ch].
 *
 * A unit on the USB input locks its soft VCXO to the host's Start of Frame
 * (EXT_ALINK_USB mode 1) and, in mode 2, plays USB sample S exactly D samples
 * after it arrived, so units on one host play sample-aligned.  Opt-in,
 * persisted with REQ_SAVE_OUTPUT_CONFIG, off by default: with mode 0 the
 * stock asynchronous behaviour is unchanged.  RP2040 and RP2350; CLICK needs
 * the RP2350's float pipeline.  Spec: extension_channel_spec.md 2.9.
 */

#ifndef ALINK_USB_H
#define ALINK_USB_H

#include <stdbool.h>
#include <stdint.h>

#include "alink_usb/alink_usb_core.h"

extern alink_usb_t alink_usb;
// Read by the SOF ISR: report exactly nominal feedback (the host follows the
// SOF-locked clock).
extern volatile bool alink_usb_fb_nominal;

void alink_usb_fw_init(void);

// USB IRQ (RAM): every SOF; the audio OUT packet completed.
void alink_usb_isr_sof(uint32_t frame11);
#include "hardware/timer.h"
static inline void alink_usb_isr_out_done(void) {
    alink_usb_tb_stamp(&alink_usb_tb, time_us_32() * 48u);
}

// Drain loop (main loop): one packet while alink_usb_engaged(&alink_usb).
void alink_usb_fw_deliver(uint32_t frame, const uint8_t *data, uint16_t len);

// Main loop, every pass while USB is the input source.
void alink_usb_fw_service(void);
// The input source leaves USB (release_dac false: its reset bracket owns the
// DAC mute), or mode 0 was written (true).
void alink_usb_fw_stop(bool release_dac);
// complete_pipeline_reset(): true = keep the DAC's hardware mute asserted, the
// USB phase anchor follows and releases it (WP USB3).
bool alink_usb_fw_keep_dac_mute(void);
// The anchor released the DAC mute after the start (main.c).
void alink_usb_fw_dac_released(void);
// WP USB3b: how long the out-of-step soft mute has held the output at zero (us, 0 not).
uint32_t alink_usb_fw_silent_us(void);
// The lock-error action (EXT_ALINK_USB byte 4), persisted in the V26 alink block's spare byte.
uint8_t alink_usb_fw_policy_byte(void);
void alink_usb_fw_policy_load(uint8_t b);
// The stored host rate (WP USB3): main loop, flash bracket like core_vreg's
// persist (the write re-anchors a phase timeline; only due while idle).
bool alink_usb_fw_seed_persist_due(void);
void alink_usb_fw_seed_persist(void);
void alink_usb_fw_seed_load(const uint8_t s[ALINK_USB_SEED_LEN]);
void alink_usb_fw_seed_bytes(uint8_t s[ALINK_USB_SEED_LEN]);
// The anchor's start: true when the outputs should start now (waits the
// last millisecond in place).
bool alink_usb_fw_anchor_ready(void);
// output_latency_apply() for the USB source.
uint32_t alink_usb_fw_ring_target(uint32_t fs, uint32_t target, uint32_t cap);

// Control plane (extension_channel_spec.md 2.9).
bool alink_usb_fw_set_config(const uint8_t p[ALINK_USB_CFG_LEN]);
void alink_usb_fw_get_config(uint8_t o[ALINK_USB_CFG_LEN]);
void alink_usb_fw_get_status(uint8_t o[ALINK_USB_STATUS_LEN]);
// The USB time base answers EXT_FRAME_COUNTER / EXT_APPLY_AT_FRAME IN.
bool alink_usb_fw_time_base_active(void);
void alink_usb_fw_get_frame_counter(uint8_t o[20]);
// EXT_APPLY_AT_FRAME OUT on the USB time base: 0 accepted, else the refusal
// (timebase.h TB_ERR_*, the tunnel's VC_ERR_* code).
uint8_t alink_usb_fw_apply(const uint8_t p[ALINK_APPLY_LEN]);
#if DSPI_SIGGEN_SYNC
void alink_usb_fw_tb_register(void);       // WP FWTB1: the time-base provider
#endif
void alink_usb_fw_get_apply(uint8_t o[ALINK_APPLY_LEN]);

// EXT_ALINK_USB_DIAG (WP USB2).
void alink_usb_fw_get_diag(uint8_t o[ALINK_USB_DIAG_LEN]);
bool alink_usb_fw_clear_diag(uint8_t v);

// Persistence (flash_storage.c, V27 directory block).
void alink_usb_fw_store(uint8_t s[ALINK_USB_STORE_LEN]);
void alink_usb_fw_load(const uint8_t s[ALINK_USB_STORE_LEN]);

#endif // ALINK_USB_H
