/*
 * alink_spdif_port.h - the firmware glue of the S/PDIF time-base roles (WP
 * FWSP2): what main.c, spdif_input.c, audio_pipeline.c, vendor_commands.c,
 * output_latency.c and flash_storage.c call.  See alink_spdif_port.c and
 * alink_spdif.h.  DSPI_ALINK_SPDIF builds only (RP2350).
 */
#ifndef ALINK_SPDIF_PORT_H
#define ALINK_SPDIF_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "alink_spdif.h"

extern alink_spdif_t alink_spdif;
extern volatile bool alink_spdif_split_armed;

void     alink_spdif_fw_init(void);
void     alink_spdif_fw_tb_register(void);
// Main loop, every pass.
void     alink_spdif_fw_service(void);
// Mode 1 with S/PDIF the input source; engaged: the phase lock runs (not
// unaligned by on_lock_error); owns: its timeline feeds the rings.
bool     alink_spdif_fw_following(void);
bool     alink_spdif_fw_engaged(void);
bool     alink_spdif_fw_owns_rings(void);
// spdif_input.c: the follower's delivery pass and servo tick.
uint32_t alink_spdif_fw_poll(void);
bool     alink_spdif_fw_servo(float hz, bool long_ok);
// main.c: the anchor's last millisecond (spins), output stops and starts.
bool     alink_spdif_fw_anchor_ready(void);
void     alink_spdif_fw_outputs_stopping(void);
void     alink_spdif_fw_outputs_started(void);
void     alink_spdif_fw_outputs_reset(void);
// audio_pipeline.c: process_input_block()'s split hook (alink_spdif_split_armed).
bool     alink_spdif_fw_src_block(uint32_t n);
// output_latency.c: the follower prefills to its delay.
uint32_t alink_spdif_fw_ring_target(uint32_t fs, uint32_t target, uint32_t cap);
// vendor_commands.c (refusals: timebase.h TB_ERR_* = VC_ERR_*).
uint8_t  alink_spdif_fw_set_config(const uint8_t p[ALINK_SPDIF_CFG_LEN]);
void     alink_spdif_fw_get_config(uint8_t o[ALINK_SPDIF_CFG_LEN]);
void     alink_spdif_fw_get_status(uint8_t o[ALINK_SPDIF_STATUS_LEN]);
uint8_t  alink_spdif_fw_ctrl_write(const uint8_t p[ALINK_SPDIF_CTRL_LEN]);
void     alink_spdif_fw_ctrl_read(uint8_t o[ALINK_SPDIF_CTRL_LEN]);
uint8_t  alink_spdif_fw_apply(const uint8_t p[ALINK_APPLY_LEN]);
void     alink_spdif_fw_get_apply(uint8_t o[ALINK_APPLY_LEN]);
bool     alink_spdif_fw_first_entry(void);
// flash_storage.c (REQ_SAVE_OUTPUT_CONFIG, boot).
void     alink_spdif_fw_store(uint8_t s[ALINK_SPDIF_STORE_LEN]);
void     alink_spdif_fw_load(const uint8_t s[ALINK_SPDIF_STORE_LEN]);

#endif // ALINK_SPDIF_PORT_H
