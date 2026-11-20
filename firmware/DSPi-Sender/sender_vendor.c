/*
 * sender_vendor.c - DSPi alink sender (WP LK4): vendor requests
 *
 * The control surface of a sender build on the vendor interface (EP0,
 * interface 2), the same transport and conventions as DSPi's
 * vendor_commands.c: the MS OS 2.0 descriptor request, REQ_EXTENSION (0x00)
 * with the sender's alink block (extension_channel_spec.md 2.8: 0x8000,
 * 0x8001, 0x8002, 0x8005, 0x8007, 0x8008, 0x8009, 0x800A), and the few plain
 * requests a host uses to identify a DSPi (platform, serial, build info),
 * REQ_SAVE_OUTPUT_CONFIG (0x52) to persist the alink settings, and
 * REQ_ENTER_BOOTLOADER (0xF0).  Everything else STALLs: a sender has no DSP.
 *
 * An OUT extension request STALLs at SETUP for an unknown ID or a wrong
 * length, and at the status stage when the payload is refused; a refused
 * write changes nothing.
 *
 * WP FWPG4: the sender is dual-stack.  The same requests are also reached
 * through upstream 1.1.7's paged envelope (SYS, STORE.OUTPUT_CONFIG_SAVE and
 * the tunnel 0xE010); sender_vc.c is the front end of both envelopes, this
 * file the TinyUSB glue and the handlers.  Handlers answer upstream's result
 * codes; page 0 turns any refusal into a STALL as before.
 */

#include <string.h>

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "config.h"
#include "sender.h"
#include "soft_vcxo.h"
#include "usb_descriptors.h"
#include "build_info.h"
#include "sender_vc.h"

// sender_vc.c keeps its own copies of the 1.1.6 opcodes and extension ids.
_Static_assert(REQ_EXTENSION == 0x00 && REQ_SAVE_OUTPUT_CONFIG == 0x52 && REQ_GET_SERIAL == 0x7E &&
               REQ_GET_PLATFORM == 0x7F && REQ_GET_BUILD_INFO == 0x80 && REQ_ENTER_BOOTLOADER == 0xF0 &&
               PRESET_OK == 0x00, "sender_vc.c's legacy opcodes");
_Static_assert(EXT_LIST == SND_EXT_LIST && EXT_NOTIFY_FEATURES == SND_EXT_NOTIFY_FEATURES, "extension ids");
_Static_assert(SND_TX_LEN == 16 && SND_CTRL_OUT_LEN == 16, "sender_vc.c's OUT lengths");

#define ALINK_ROLE_SENDER       2u
#define ALINK_ROLE_FANOUT       4u
#define ALINK_FEAT_FAN_OUT      0x08u
#define ALINK_FEAT_TEST_PATTERNS 0x10u
#define ALINK_FEAT_LOS_PIN      0x20u
#define ALINK_FEAT_SIGGEN_SYNC  0x40u   // EXT_ALINK_CTRL_MSG carries ops 9 and 10 (WP FWTB1)

// Sender layout of EXT_ALINK_DIAG (byte 0; a receiver's layout is 1).
#define SENDER_DIAG_VERSION     0x81u

static uint8_t rx_buf[SND_VC_MAX];

static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void putf(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); put32(p, u); }

static uint8_t rate_code_of(uint32_t hz) {
    return hz == 44100u ? ALINK_RATE_44K1 : hz == 96000u ? ALINK_RATE_96K : ALINK_RATE_48K;
}

// A pin some alink function already uses (another one than the config being
// written): the alink outputs, their SFP pins and the SYNC pin.
static bool pin_taken(uint8_t pin) {
    if (pin == 0xFF) return false;
    for (unsigned n = 0; n < SND_MAX_ALINKS; n++)
        if (snd_alink_pins[n] == pin || SND_TXDIS_PIN(n) == pin || SND_LOS_PIN(n) == pin) return true;
    return snd.cfg.sync_pin == pin;
}

// ---------------------------------------------------------------------------
// OUT
// ---------------------------------------------------------------------------

// The OUT handlers answer upstream's result codes (extension_channel_spec.md
// 1.3.4); page 0 STALLs on any of them.
static uint8_t set_config(const uint8_t *p) {
    uint8_t enable = p[0], rx = p[1], cls = p[2], tx = p[3], los = p[4], txdis = p[5], flags = p[6];
    if (enable > 1 || cls > 3 || (flags & ~3u) || p[7]) return SND_VC_ERR_VALUE;
    if (!snd_pin_ok(rx) || !snd_pin_ok(tx) || !snd_pin_ok(los) || !snd_pin_ok(txdis)) return SND_VC_ERR_PIN_INVALID;
    if (tx != 0xFF) return SND_VC_ERR_UNSUPPORTED;      // no repeater output on a sender
    uint8_t pins[3] = { rx, los, txdis };
    for (int i = 0; i < 3; i++) {
        if (pins[i] == 0xFF) continue;
        for (int j = i + 1; j < 3; j++) if (pins[i] == pins[j]) return SND_VC_ERR_PIN_INVALID;
    }
    for (int i = 0; i < 3; i++)
        if (pins[i] != 0xFF && pin_taken(pins[i])) return SND_VC_ERR_PIN_IN_USE;
    uint8_t eff_rx = rx == 0xFF ? SND_FANOUT_RX_PIN : rx;
    if (enable && pin_taken(eff_rx)) return SND_VC_ERR_PIN_IN_USE;
    uint32_t s = snd_lock_take();
    bool restart = (enable != snd.config[0]) ||
                   (enable && (rx != snd.config[1] || cls != snd.config[2]));
    memcpy(snd.config, p, 8);
    snd_lock_give(s);
    if (restart) snd.restart_req = snd.restart_req + 1u;
    return SND_VC_OK;
}

static uint8_t set_tx(const uint8_t *p) {
    snd_txcfg_t c;
    int bad = snd_txcfg_parse(p, SND_MAX_ALINKS, &c);
    if (bad) return bad == 7 ? SND_VC_ERR_PIN_INVALID : SND_VC_ERR_VALUE;  // 7: sync_pin
    if (c.sync_pin != 0xFF) {
        for (unsigned n = 0; n < SND_MAX_ALINKS; n++)
            if (snd_alink_pins[n] == c.sync_pin || SND_TXDIS_PIN(n) == c.sync_pin || SND_LOS_PIN(n) == c.sync_pin)
                return SND_VC_ERR_PIN_IN_USE;
        if (snd.config[0] && c.sync_pin == (snd.config[1] == 0xFF ? SND_FANOUT_RX_PIN : snd.config[1]))
            return SND_VC_ERR_PIN_IN_USE;
        if (c.sync_pin == snd.config[4]) return SND_VC_ERR_PIN_IN_USE;
    }
    bool restart = false, pins;
    uint32_t s = snd_lock_take();
    snd_txcfg_t old = snd.cfg;
    bool running = old.alink_enable != 0 || c.alink_enable != 0;
    if (c.cls != old.cls) {
        // Not announced in-band: the alinks restart at the new line rate and
        // the receivers re-detect (2.8.8).  A pending RATE's layout is in
        // cfg, the restart takes it at once.
        restart = true;
    } else if (c.rate != old.rate || c.channels != old.channels) {
        if (!running) {
            restart = true;                             // no alink sends: switch now
        } else {
            uint64_t at;
            if (snd.tx.apply_op != ALINK_OP_NONE || !snd_schedule_layout(&snd.tx, c.rate, c.channels, &at)) {
                snd_lock_give(s);
                return SND_VC_ERR_STATE;                // one apply at a time
            }
        }
    }
    pins = c.alink_enable != old.alink_enable || c.txdis_mask != old.txdis_mask || c.sync_pin != old.sync_pin;
    snd.cfg = c;
    snd.channels_req = c.channels;
    snd.usb_rate_follow = false;                        // the host's rate wins over a pending follow
    snd_lock_give(s);
    if (restart) snd.restart_req = snd.restart_req + 1u;
    else if (pins) snd.pins_req = snd.pins_req + 1u;
    return SND_VC_OK;
}

static uint8_t set_ctrl(const uint8_t *p) {
    snd_ctrl_t m;
    if (snd_ctrl_parse(p, &m)) return SND_VC_ERR_VALUE;
    uint32_t s = snd_lock_take();
    if (m.op != ALINK_OP_NONE && !alink_tx_schedule(&snd.tx, m.frame, m.op, m.arg, m.value)) {
        snd_lock_give(s);
        return SND_VC_ERR_STATE;            // lead, one at a time, RATE fit (alink_tx_schedule)
    }
    if (m.op == ALINK_OP_RATE) snd.cfg.rate = m.arg;
    if (m.op == ALINK_OP_SYNC_PULSE) snd.sync_pulse_frame = m.frame;
    snd.cfg.flags = (uint8_t)((snd.cfg.flags & ~1u) | (m.system_flags & 1u));
    snd_lock_give(s);
    return SND_VC_OK;
}

uint8_t snd_hook_ext_set(uint16_t id, const uint8_t *p) {
    switch (id) {
    case SND_EXT_ALINK_CONFIG:    return set_config(p);
    case SND_EXT_ALINK_TX:        return set_tx(p);
    case SND_EXT_ALINK_CTRL_MSG:  return set_ctrl(p);
    case SND_EXT_ALINK_STATS_RESET:
        if (p[0] != 1) return SND_VC_ERR_VALUE;
        snd.stats_req = snd.stats_req + 1u;
        return SND_VC_OK;
    }
    return SND_VC_ERR_ADDRESS;
}

// ---------------------------------------------------------------------------
// IN
// ---------------------------------------------------------------------------

static uint16_t get_info(uint8_t *o) {
    memset(o, 0, 32);
    o[0] = snd.config[0] ? ALINK_ROLE_FANOUT : ALINK_ROLE_SENDER;
    o[1] = ALINK_PROTOCOL_VERSION;
    o[2] = 0x07;                                    // L1, L2, L3
    o[3] = SND_MAX_ALINKS;
    o[4] = ALINK_MAX_CHANNELS;
    o[5] = 0;                                       // slots: a sender has none
    o[8] = snd.config[1] == 0xFF ? SND_FANOUT_RX_PIN : snd.config[1];
    o[9] = 0xFF;
    put32(o + 12, ALINK_FEAT_FAN_OUT | ALINK_FEAT_TEST_PATTERNS | ALINK_FEAT_LOS_PIN | ALINK_FEAT_SIGGEN_SYNC);
    return 32;
}

static uint16_t get_status(uint8_t *o) {
    uint8_t state, cls, rate, chans, hops, sender, sflags;
    alink_rx_stats_t st;
    float phase, ppm_ff;
    uint32_t delay, realigns;
    sender_fanout_status(&state, &cls, &rate, &chans, &hops, &sender, &sflags, &st, &phase, &delay,
                         &realigns, &ppm_ff);
    memset(o, 0, 52);
    o[0] = state;
    o[1] = cls;
    o[2] = rate;
    o[3] = chans;
    o[4] = hops;
    o[5] = snd.config[0] && snd.config[4] != 0xFF ? (uint8_t)gpio_get(snd.config[4]) : 0xFF;
    o[6] = sender;
    o[7] = sflags;
    put64(o + 8, snd.wire_frame);                   // the counter the sender sends
    put32(o + 16, st.lock_count);
    put32(o + 20, st.loss_count);
    put32(o + 24, st.crc8_errors);
    put32(o + 28, st.crc16_errors);
    put32(o + 32, st.bit_slips);
    put32(o + 36, st.superframe_errors);
    putf(o + 40, soft_vcxo_current_ppm());
    putf(o + 44, phase);
    uint32_t fill = snd_fifo.wr - snd_fifo.rd;
    o[48] = (uint8_t)fill;
    o[49] = (uint8_t)(fill >> 8);
    if (snd.engine == SND_ENG_FANOUT)                // hop_delay_frames: 0 unless re-sending
        o[50] = (uint8_t)(delay > 255u ? 255u : delay);
    return 52;
}

static uint16_t get_diag(uint8_t *o) {
    uint8_t state, cls, rate, chans, hops, sender, sflags;
    alink_rx_stats_t st;
    float phase, ppm_ff;
    uint32_t delay, realigns;
    sender_fanout_status(&state, &cls, &rate, &chans, &hops, &sender, &sflags, &st, &phase, &delay,
                         &realigns, &ppm_ff);
    uint32_t s = snd_lock_take();
    uint32_t fs = alink_rate_hz(snd.tx.rate);
    snd_lock_give(s);
    memset(o, 0, 64);
    o[0] = SENDER_DIAG_VERSION;
    o[1] = snd.engine;
    o[2] = snd.run_cls;
    o[3] = (uint8_t)delay;
    put32(o + 4, snd.enc_cycles_avg);
    put32(o + 8, snd.enc_cycles_max);
    snd.enc_cycles_max = 0;
    put32(o + 12, fs ? sender_sys_hz() / fs : 0);   // cycles in one frame
    put32(o + 16, snd.lead_words);
    put32(o + 20, snd.tx_late);
    put32(o + 24, snd.tx_stall);
    put32(o + 28, snd_fifo.wr - snd_fifo.rd);
    put32(o + 32, snd.usb_underruns);
    put32(o + 36, snd.usb_overruns);
    put32(o + 40, snd.feedback_10_14);
    put32(o + 44, snd.usb_packets);
    put32(o + 48, snd.usb_gaps);
    putf(o + 52, ppm_ff);
    put32(o + 56, snd.loop_us_max);
    snd.loop_us_max = 0;
    o[60] = (uint8_t)realigns;
    o[61] = (uint8_t)(realigns >> 8);
    o[62] = snd.usb_playing;
    return 64;
}

uint8_t snd_hook_ext_get(uint16_t id, uint8_t *o, uint16_t *n) {
    switch (id) {
    case EXT_ALINK_INFO:    *n = get_info(o); break;
    case EXT_ALINK_STATUS:  *n = get_status(o); break;
    case EXT_ALINK_CONFIG:  memcpy(o, snd.config, 8); *n = 8; break;
    case EXT_FRAME_COUNTER: {
        uint64_t f = snd.wire_frame;
        put64(o, f);
        put32(o + 8, time_us_32());
        put32(o + 12, (uint32_t)(f - snd.start_frame));
        *n = 16;
        break;
    }
    case SND_EXT_ALINK_TX: {
        uint32_t s = snd_lock_take();
        snd_txcfg_t c = snd.cfg;
        snd_lock_give(s);
        snd_txcfg_pack(&c, snd.los_mask, snd.usb_alt, o);
        *n = SND_TX_LEN;
        break;
    }
    case EXT_ALINK_CTRL_MSG: {
        uint32_t s = snd_lock_take();
        snd_ctrl_pack_in(&snd.tx, snd.wire_frame, o);
        snd_lock_give(s);
        *n = SND_CTRL_IN_LEN;
        break;
    }
    case EXT_ALINK_DIAG:    *n = get_diag(o); break;
    default:
        return SND_VC_ERR_ADDRESS;
    }
    return SND_VC_OK;
}

// ---------------------------------------------------------------------------
// The main loop's part
// ---------------------------------------------------------------------------

bool sender_follow_usb_rate(void) {
    if (!snd.usb_rate_follow) return true;
    uint8_t code = rate_code_of(snd.usb_rate);
    bool done = true, restart = false;
    uint32_t s = snd_lock_take();
    if (snd.cfg.rate != code) {
        unsigned cap = alink_max_channels(snd.cfg.cls, code);
        unsigned c = snd.channels_req < cap ? snd.channels_req : cap;
        if (snd.cfg.alink_enable == 0) {
            restart = true;
        } else {
            uint64_t at;
            done = snd.tx.apply_op == ALINK_OP_NONE && snd_schedule_layout(&snd.tx, code, c, &at);
        }
        if (done) {
            snd.cfg.rate = code;
            snd.cfg.channels = (uint8_t)c;
        }
    }
    if (done) snd.usb_rate_follow = false;
    snd_lock_give(s);
    if (restart) snd.restart_req = snd.restart_req + 1u;
    return done;
}

// ---------------------------------------------------------------------------
// TinyUSB entry
// ---------------------------------------------------------------------------

uint16_t snd_hook_ident(snd_ident_t which, uint8_t *o) {
    switch (which) {
    case SND_IDENT_PLATFORM:
        o[0] = PLATFORM_RP2350;
        o[1] = (uint8_t)(FW_VERSION_PACKED >> 8);
        o[2] = (uint8_t)(FW_VERSION_PACKED & 0xFF);
        o[3] = 0;                                   // no output channels: a sender
        o[4] = FW_VERSION_MINOR;
        o[5] = FW_VERSION_PATCH;
        o[6] = FW_VERSION_BETA;
        return 7;
    case SND_IDENT_SERIAL:
        memcpy(o, usb_descriptor_str_serial, 16);
        return 16;
    case SND_IDENT_BUILD_INFO:
        memset(o, 0, 64);
        strncpy((char *)o, BUILD_GIT_DESCRIBE, 47);
        strncpy((char *)o + 48, BUILD_DATE, 11);
        return 64;
    }
    return 0;
}

void snd_hook_save(void) {
    snd.save_req = true;                            // flash write in the main loop
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    snd_vc_req_t r = {
        .bmRequestType = req->bmRequestType, .bRequest = req->bRequest,
        .wValue = req->wValue, .wIndex = req->wIndex, .wLength = req->wLength,
    };
    if (stage == CONTROL_STAGE_SETUP) {
        // Page 0 and the paged envelope (WP FWPG4, sender_vc.c); this
        // replaces the PAGEGUARD refusal of every nonzero page.
        if ((req->wIndex >> 8) == 0 && req->bRequest == MS_VENDOR_CODE) {
            if (req->bmRequestType_bit.direction == TUSB_DIR_IN && req->wIndex == 7)
                return tud_control_xfer(rhport, req, (void *)(uintptr_t)desc_ms_os_20, (uint16_t)desc_ms_os_20_len);
            return false;
        }
        const uint8_t *out;
        uint16_t n;
        switch (snd_vc_setup(&r, &out, &n)) {
        case SND_VC_SEND:
            return tud_control_xfer(rhport, req, (void *)(uintptr_t)out, n);
        case SND_VC_SEND_REBOOT:
            tud_control_xfer(rhport, req, (void *)(uintptr_t)out, n);
            busy_wait_ms(100);
            reset_usb_boot(0, 0);
            return true;
        case SND_VC_RECV:
            return tud_control_xfer(rhport, req, rx_buf, req->wLength);
        case SND_VC_STATUS:
            return tud_control_status(rhport, req);
        case SND_VC_STALL:
        default:
            return false;
        }
    }
    if (stage == CONTROL_STAGE_DATA && req->bmRequestType_bit.direction == TUSB_DIR_OUT)
        return snd_vc_data(&r, rx_buf);                         // false: STALL the status stage
    if (stage == CONTROL_STAGE_ACK && snd_vc_ack(&r))
        reset_usb_boot(0, 0);                                   // paged SYS.ENTER_BOOTLOADER
    return true;
}
