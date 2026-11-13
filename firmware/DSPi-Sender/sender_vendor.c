/*
 * sender_vendor.c - DSPi link sender (WP LK4): vendor requests
 *
 * The control surface of a sender build on the vendor interface (EP0,
 * interface 2), the same transport and conventions as DSPi's
 * vendor_commands.c: the MS OS 2.0 descriptor request, REQ_EXTENSION (0x00)
 * with the sender's link block (extension_channel_spec.md 2.8: 0x8000,
 * 0x8001, 0x8002, 0x8005, 0x8007, 0x8008, 0x8009, 0x800A), and the few plain
 * requests a host uses to identify a DSPi (platform, serial, build info),
 * REQ_SAVE_OUTPUT_CONFIG (0x52) to persist the link settings, and
 * REQ_ENTER_BOOTLOADER (0xF0).  Everything else STALLs: a sender has no DSP.
 *
 * An OUT extension request STALLs at SETUP for an unknown ID or a wrong
 * length, and at the status stage when the payload is refused; a refused
 * write changes nothing.
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

// Extension IDs of the link block (extension_channel_spec.md 2.8).
#define EXT_LINK_INFO           0x8000
#define EXT_LINK_STATUS         0x8001
#define EXT_LINK_CONFIG         0x8002
#define EXT_FRAME_COUNTER       0x8005
#define EXT_LINK_TX             0x8007
#define EXT_LINK_CTRL_MSG       0x8008
#define EXT_LINK_STATS_RESET    0x8009
#define EXT_LINK_DIAG           0x800A

#define LINK_ROLE_SENDER        2u
#define LINK_ROLE_FANOUT        4u
#define LINK_FEAT_FAN_OUT       0x08u
#define LINK_FEAT_TEST_PATTERNS 0x10u
#define LINK_FEAT_LOS_PIN       0x20u

// Sender layout of EXT_LINK_DIAG (byte 0; a receiver's layout is 1).
#define SENDER_DIAG_VERSION     0x81u

static const uint16_t ext_ids[] = {
    EXT_NOTIFY_FEATURES,
    EXT_LINK_INFO, EXT_LINK_STATUS, EXT_LINK_CONFIG, EXT_FRAME_COUNTER,
    EXT_LINK_TX, EXT_LINK_CTRL_MSG, EXT_LINK_STATS_RESET, EXT_LINK_DIAG,
};

static uint8_t resp[64];
static uint8_t rx_buf[64];

static uint16_t ext_set_len(uint16_t id) {
    switch (id) {
    case EXT_LINK_CONFIG:       return 8;
    case EXT_LINK_TX:           return SND_TX_LEN;
    case EXT_LINK_CTRL_MSG:     return SND_CTRL_OUT_LEN;
    case EXT_LINK_STATS_RESET:  return 1;
    default:                    return 0;
    }
}

static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void putf(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); put32(p, u); }

static uint8_t rate_code_of(uint32_t hz) {
    return hz == 44100u ? LINK_RATE_44K1 : hz == 96000u ? LINK_RATE_96K : LINK_RATE_48K;
}

// A pin some link function already uses (another one than the config being
// written): the link outputs, their SFP pins and the SYNC pin.
static bool pin_taken(uint8_t pin) {
    if (pin == 0xFF) return false;
    for (unsigned n = 0; n < SND_MAX_LINKS; n++)
        if (snd_link_pins[n] == pin || SND_TXDIS_PIN(n) == pin || SND_LOS_PIN(n) == pin) return true;
    return snd.cfg.sync_pin == pin;
}

// ---------------------------------------------------------------------------
// OUT
// ---------------------------------------------------------------------------

static bool set_config(const uint8_t *p) {
    uint8_t enable = p[0], rx = p[1], cls = p[2], tx = p[3], los = p[4], txdis = p[5], flags = p[6];
    if (enable > 1 || cls > 3 || (flags & ~3u) || p[7]) return false;
    if (!snd_pin_ok(rx) || !snd_pin_ok(tx) || !snd_pin_ok(los) || !snd_pin_ok(txdis)) return false;
    if (tx != 0xFF) return false;                       // no repeater output on a sender
    uint8_t pins[3] = { rx, los, txdis };
    for (int i = 0; i < 3; i++) {
        if (pins[i] == 0xFF) continue;
        if (pin_taken(pins[i])) return false;
        for (int j = i + 1; j < 3; j++) if (pins[i] == pins[j]) return false;
    }
    uint8_t eff_rx = rx == 0xFF ? SND_FANOUT_RX_PIN : rx;
    if (enable && pin_taken(eff_rx)) return false;
    uint32_t s = snd_lock_take();
    bool restart = (enable != snd.config[0]) ||
                   (enable && (rx != snd.config[1] || cls != snd.config[2]));
    memcpy(snd.config, p, 8);
    snd_lock_give(s);
    if (restart) snd.restart_req = snd.restart_req + 1u;
    return true;
}

static bool set_tx(const uint8_t *p) {
    snd_txcfg_t c;
    if (snd_txcfg_parse(p, SND_MAX_LINKS, &c)) return false;
    if (c.sync_pin != 0xFF) {
        for (unsigned n = 0; n < SND_MAX_LINKS; n++)
            if (snd_link_pins[n] == c.sync_pin || SND_TXDIS_PIN(n) == c.sync_pin || SND_LOS_PIN(n) == c.sync_pin)
                return false;
        if (snd.config[0] && c.sync_pin == (snd.config[1] == 0xFF ? SND_FANOUT_RX_PIN : snd.config[1]))
            return false;
        if (c.sync_pin == snd.config[4]) return false;
    }
    bool restart = false, pins;
    uint32_t s = snd_lock_take();
    snd_txcfg_t old = snd.cfg;
    bool running = old.link_enable != 0 || c.link_enable != 0;
    if (c.cls != old.cls) {
        // Not announced in-band: the links restart at the new line rate and
        // the receivers re-detect (2.8.8).  A pending RATE's layout is in
        // cfg, the restart takes it at once.
        restart = true;
    } else if (c.rate != old.rate || c.channels != old.channels) {
        if (!running) {
            restart = true;                             // no link sends: switch now
        } else {
            uint64_t at;
            if (snd.tx.apply_op != LINK_OP_NONE || !snd_schedule_layout(&snd.tx, c.rate, c.channels, &at)) {
                snd_lock_give(s);
                return false;                           // one apply at a time
            }
        }
    }
    pins = c.link_enable != old.link_enable || c.txdis_mask != old.txdis_mask || c.sync_pin != old.sync_pin;
    snd.cfg = c;
    snd.channels_req = c.channels;
    snd.usb_rate_follow = false;                        // the host's rate wins over a pending follow
    snd_lock_give(s);
    if (restart) snd.restart_req = snd.restart_req + 1u;
    else if (pins) snd.pins_req = snd.pins_req + 1u;
    return true;
}

static bool set_ctrl(const uint8_t *p) {
    snd_ctrl_t m;
    if (snd_ctrl_parse(p, &m)) return false;
    uint32_t s = snd_lock_take();
    if (m.op != LINK_OP_NONE && !link_tx_schedule(&snd.tx, m.frame, m.op, m.arg, m.value)) {
        snd_lock_give(s);
        return false;                       // lead, one at a time, RATE fit (link_tx_schedule)
    }
    if (m.op == LINK_OP_RATE) snd.cfg.rate = m.arg;
    if (m.op == LINK_OP_SYNC_PULSE) snd.sync_pulse_frame = m.frame;
    snd.cfg.flags = (uint8_t)((snd.cfg.flags & ~1u) | (m.system_flags & 1u));
    snd_lock_give(s);
    return true;
}

static bool ext_set(uint16_t id, const uint8_t *p, uint16_t len) {
    if (ext_set_len(id) == 0 || len != ext_set_len(id)) return false;
    switch (id) {
    case EXT_LINK_CONFIG:   return set_config(p);
    case EXT_LINK_TX:       return set_tx(p);
    case EXT_LINK_CTRL_MSG: return set_ctrl(p);
    case EXT_LINK_STATS_RESET:
        if (p[0] != 1) return false;
        snd.stats_req = snd.stats_req + 1u;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// IN
// ---------------------------------------------------------------------------

static uint16_t get_info(uint8_t *o) {
    memset(o, 0, 32);
    o[0] = snd.config[0] ? LINK_ROLE_FANOUT : LINK_ROLE_SENDER;
    o[1] = LINK_PROTOCOL_VERSION;
    o[2] = 0x07;                                    // L1, L2, L3
    o[3] = SND_MAX_LINKS;
    o[4] = LINK_MAX_CHANNELS;
    o[5] = 0;                                       // slots: a sender has none
    o[8] = snd.config[1] == 0xFF ? SND_FANOUT_RX_PIN : snd.config[1];
    o[9] = 0xFF;
    put32(o + 12, LINK_FEAT_FAN_OUT | LINK_FEAT_TEST_PATTERNS | LINK_FEAT_LOS_PIN);
    return 32;
}

static uint16_t get_status(uint8_t *o) {
    uint8_t state, cls, rate, chans, hops, sender, sflags;
    link_rx_stats_t st;
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
    return 52;
}

static uint16_t get_diag(uint8_t *o) {
    uint8_t state, cls, rate, chans, hops, sender, sflags;
    link_rx_stats_t st;
    float phase, ppm_ff;
    uint32_t delay, realigns;
    sender_fanout_status(&state, &cls, &rate, &chans, &hops, &sender, &sflags, &st, &phase, &delay,
                         &realigns, &ppm_ff);
    uint32_t s = snd_lock_take();
    uint32_t fs = link_rate_hz(snd.tx.rate);
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

static bool ext_get(uint16_t id, uint16_t wlen, uint8_t *o, uint16_t *n) {
    switch (id) {
    case EXT_LIST:
        *n = 0;
        for (size_t i = 0; i < sizeof(ext_ids) / sizeof(ext_ids[0]); i++) {
            o[(*n)++] = (uint8_t)ext_ids[i];
            o[(*n)++] = (uint8_t)(ext_ids[i] >> 8);
        }
        break;
    case EXT_NOTIFY_FEATURES:
        put32(o, 0);                                // the sender sends no notifications
        *n = 4;
        break;
    case EXT_LINK_INFO:     *n = get_info(o); break;
    case EXT_LINK_STATUS:   *n = get_status(o); break;
    case EXT_LINK_CONFIG:   memcpy(o, snd.config, 8); *n = 8; break;
    case EXT_FRAME_COUNTER: {
        uint64_t f = snd.wire_frame;
        put64(o, f);
        put32(o + 8, time_us_32());
        put32(o + 12, (uint32_t)(f - snd.start_frame));
        *n = 16;
        break;
    }
    case EXT_LINK_TX: {
        uint32_t s = snd_lock_take();
        snd_txcfg_t c = snd.cfg;
        snd_lock_give(s);
        snd_txcfg_pack(&c, snd.los_mask, snd.usb_alt, o);
        *n = SND_TX_LEN;
        break;
    }
    case EXT_LINK_CTRL_MSG: {
        uint32_t s = snd_lock_take();
        snd_ctrl_pack_in(&snd.tx, snd.wire_frame, o);
        snd_lock_give(s);
        *n = SND_CTRL_IN_LEN;
        break;
    }
    case EXT_LINK_DIAG:     *n = get_diag(o); break;
    default:
        return false;
    }
    if (wlen && wlen < *n) *n = wlen;
    return true;
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
        unsigned cap = link_max_channels(snd.cfg.cls, code);
        unsigned c = snd.channels_req < cap ? snd.channels_req : cap;
        if (snd.cfg.link_enable == 0) {
            restart = true;
        } else {
            uint64_t at;
            done = snd.tx.apply_op == LINK_OP_NONE && snd_schedule_layout(&snd.tx, code, c, &at);
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

static bool handle_get(uint8_t rhport, tusb_control_request_t const *req) {
    uint16_t n = 0;
    switch (req->bRequest) {
    case REQ_EXTENSION:
        if (!ext_get(req->wValue, req->wLength, resp, &n)) return false;
        break;
    case REQ_GET_PLATFORM:
        resp[0] = PLATFORM_RP2350;
        resp[1] = (uint8_t)(FW_VERSION_PACKED >> 8);
        resp[2] = (uint8_t)(FW_VERSION_PACKED & 0xFF);
        resp[3] = 0;                                // no output channels: a sender
        resp[4] = FW_VERSION_MINOR;
        resp[5] = FW_VERSION_PATCH;
        resp[6] = FW_VERSION_BETA;
        n = 7;
        break;
    case REQ_GET_SERIAL:
        memcpy(resp, usb_descriptor_str_serial, 16);
        n = 16;
        break;
    case REQ_GET_BUILD_INFO:
        memset(resp, 0, 64);
        strncpy((char *)resp, BUILD_GIT_DESCRIBE, 47);
        strncpy((char *)resp + 48, BUILD_DATE, 11);
        n = 64;
        break;
    case REQ_SAVE_OUTPUT_CONFIG:
        snd.save_req = true;                        // flash write in the main loop
        resp[0] = PRESET_OK;
        n = 1;
        break;
    case REQ_ENTER_BOOTLOADER:
        resp[0] = 1;
        tud_control_xfer(rhport, req, resp, 1);
        busy_wait_ms(100);
        reset_usb_boot(0, 0);
        return true;
    default:
        return false;
    }
    if (n > req->wLength) n = req->wLength;
    return tud_control_xfer(rhport, req, resp, n);
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage == CONTROL_STAGE_SETUP) {
        if (req->bRequest == MS_VENDOR_CODE) {
            if (req->bmRequestType_bit.direction == TUSB_DIR_IN && req->wIndex == 7)
                return tud_control_xfer(rhport, req, (void *)(uintptr_t)desc_ms_os_20, (uint16_t)desc_ms_os_20_len);
            return false;
        }
        if (req->bmRequestType_bit.direction == TUSB_DIR_IN) return handle_get(rhport, req);
        if (req->bRequest != REQ_EXTENSION) return false;
        if (ext_set_len(req->wValue) == 0 || req->wLength != ext_set_len(req->wValue)) return false;
        return tud_control_xfer(rhport, req, rx_buf, req->wLength);
    }
    if (stage == CONTROL_STAGE_DATA && req->bmRequestType_bit.direction == TUSB_DIR_OUT &&
        req->bRequest == REQ_EXTENSION) {
        return ext_set(req->wValue, rx_buf, req->wLength);      // false: STALL the status stage
    }
    return true;
}
