/*
 * sender_vc.h - DSPi alink sender (WP FWPG4): the vendor control front end
 *
 * The sender is dual-stack (extension_channel_spec.md 1.3.5, PLAN-PG 1.9):
 *
 *  - page 0 (wIndex high byte 0): the 1.1.6 requests it always had, unchanged:
 *    REQ_EXTENSION (0x00) with its EXT_LIST, REQ_GET_PLATFORM, REQ_GET_SERIAL,
 *    REQ_GET_BUILD_INFO, REQ_SAVE_OUTPUT_CONFIG and REQ_ENTER_BOOTLOADER; a
 *    refusal is a STALL (1.1);
 *  - the paged 1.1.7 envelope (upstream release/v1.1.7 103ba37, the migration
 *    guide): SYS 0x01 (discovery, identity, bootloader), STORE 0x02 (only
 *    OUTPUT_CONFIG_SAVE: no ALL_PARAMS, the sender has no DSP) and the fork's
 *    page 0xE0 with the tunnel 0xE010 (1.3.2).  Refusals are upstream's
 *    VC_ERR_* codes, recorded in SYS.LAST_RESULT, at upstream's stages.
 *
 * Every other page and command is VC_ERR_UNKNOWN: fail closed.  This replaces
 * the PAGEGUARD refusal of every nonzero page (1.3.3).
 *
 * This file and sender_vc.c are pure (no SDK, no TinyUSB) so both envelopes
 * are host-tested byte for byte (tests/test_vc.c).  sender_vendor.c is the
 * TinyUSB glue and implements the hooks at the end of this header.
 */

#ifndef SENDER_VC_H
#define SENDER_VC_H

#include <stdbool.h>
#include <stdint.h>

// Results (upstream vc_ctrl.h VcResult; 0x02 and 0x03 are never answered).
#define SND_VC_OK               0x00u
#define SND_VC_ERR_UNKNOWN      0x10u
#define SND_VC_ERR_DIRECTION    0x11u
#define SND_VC_ERR_LENGTH       0x12u
#define SND_VC_ERR_ADDRESS      0x13u
#define SND_VC_ERR_VALUE        0x14u
#define SND_VC_ERR_UNSUPPORTED  0x15u
#define SND_VC_ERR_BLOCKED      0x16u
#define SND_VC_ERR_BUSY         0x17u
#define SND_VC_ERR_STATE        0x18u
#define SND_VC_ERR_PIN_INVALID  0x19u
#define SND_VC_ERR_PIN_IN_USE   0x1Au

// Pages and command ids ((page << 8) | bRequest).
#define SND_PAGE_SYS            0x01u
#define SND_PAGE_STORE          0x02u
#define SND_PAGE_EXT            0xE0u
#define SND_PAGE_EXT_REVISION   1u
#define SND_SYS_PROTOCOL_INFO   0x0110u
#define SND_SYS_PAGE_LIST       0x0111u
#define SND_SYS_PAGE_COMMANDS   0x0112u
#define SND_SYS_COMMAND_INFO    0x0113u
#define SND_SYS_LAST_RESULT     0x0114u
#define SND_SYS_PLATFORM        0x0120u
#define SND_SYS_SERIAL          0x0121u
#define SND_SYS_BUILD_INFO      0x0122u
#define SND_SYS_ENTER_BOOTLOADER 0x0130u
#define SND_STORE_OUTPUT_CONFIG_SAVE 0x0221u
#define SND_EXT_TUNNEL          0xE010u

// The sender's buffers: no reply or SET payload is longer (SYS.PROTOCOL_INFO
// bytes 8-11 say so).
#define SND_VC_MAX              64u

// Extension sub-commands the sender lists (2.8; 0x0000 is EXT_LIST itself).
#define SND_EXT_LIST            0x0000u
#define SND_EXT_NOTIFY_FEATURES 0x0003u
#define SND_EXT_ALINK_INFO      0x8000u
#define SND_EXT_ALINK_STATUS    0x8001u
#define SND_EXT_ALINK_CONFIG    0x8002u
#define SND_EXT_FRAME_COUNTER   0x8005u
#define SND_EXT_ALINK_TX        0x8007u
#define SND_EXT_ALINK_CTRL_MSG  0x8008u
#define SND_EXT_ALINK_STATS_RESET 0x8009u
#define SND_EXT_ALINK_DIAG      0x800Au

// A control request as it arrives (the SETUP packet's fields).
typedef struct {
    uint8_t  bmRequestType;     // bit 7 set: IN (GET)
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} snd_vc_req_t;

typedef enum {
    SND_VC_STALL,               // refuse (STALL this stage)
    SND_VC_SEND,                // IN: send *out, *n bytes (already capped to wLength)
    SND_VC_SEND_REBOOT,         // IN: send *out, then enter the bootloader (1.1.6 0xF0)
    SND_VC_RECV,                // OUT: receive wLength bytes, then snd_vc_data()
    SND_VC_STATUS,              // OUT with no data: accepted, ACK the status stage
} snd_vc_action_t;

// SETUP stage of a vendor request (the MS OS 2.0 descriptor request is the
// caller's).  *out points into a static buffer valid until the next call.
snd_vc_action_t snd_vc_setup(const snd_vc_req_t *r, const uint8_t **out, uint16_t *n);

// DATA stage of an OUT request whose SETUP answered SND_VC_RECV.  False
// STALLs the status stage; a refused write changed nothing.
bool snd_vc_data(const snd_vc_req_t *r, const uint8_t *data);

// ACK stage: true once, when a paged SYS.ENTER_BOOTLOADER has completed its
// status stage and the device must reboot now.
bool snd_vc_ack(const snd_vc_req_t *r);

// The extension table (both envelopes): listed, has an IN form, OUT length
// (0 = no OUT form).
bool     snd_ext_listed(uint16_t id);
bool     snd_ext_has_get(uint16_t id);
uint16_t snd_ext_set_len(uint16_t id);

// ---------------------------------------------------------------------------
// Hooks, implemented by sender_vendor.c (and by the host tests)
// ---------------------------------------------------------------------------

// The IN form of an alink sub-command (never EXT_LIST or EXT_NOTIFY_FEATURES):
// *n bytes into o (64 bytes of room).  SND_VC_OK or a refusal code.
uint8_t snd_hook_ext_get(uint16_t id, uint8_t *o, uint16_t *n);
// The OUT form, payload already exactly snd_ext_set_len(id) bytes.
uint8_t snd_hook_ext_set(uint16_t id, const uint8_t *p);

typedef enum { SND_IDENT_PLATFORM, SND_IDENT_SERIAL, SND_IDENT_BUILD_INFO } snd_ident_t;
// The identity replies, the same bytes on both stacks: platform 7 bytes,
// serial 16, build info 64.  Returns the length.
uint16_t snd_hook_ident(snd_ident_t which, uint8_t *o);
// Persist the alink settings (main loop does the flash write).
void snd_hook_save(void);

#endif
