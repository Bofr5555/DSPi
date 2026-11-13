/*
 * sender_usb.c - DSPi link sender (WP LK4): the USB sound card
 *
 * A slim copy of DSPi's UAC1 class driver (usb_audio.c), with DSPi's
 * descriptors (usb_descriptors.c built with DSPI_ROLE_SENDER):
 *
 *   alt 1 / 2   2 ch 16 / 24-bit, 44.1 / 48 / 96 kHz (SET_CUR picks)
 *   alt 3 4 5   4 / 6 / 8 ch 16-bit 48 kHz
 *   alt 6       4 ch 24-bit 48 kHz
 *   alt 7       6 ch 24-bit 48 kHz           (sender only, SYS2 4.1)
 *   alt 8       4 ch 16-bit 96 kHz           (sender only)
 *
 * Packets become 24-bit frames in the sample FIFO (sender.h), USB channel i
 * to link slot i, with the host's volume and mute applied.  The feedback
 * endpoint uses DSPi's controller (usb_feedback_controller.c) unchanged
 * except for its source: the "output words consumed" are the link frames
 * on the wire (rate_shift 14: one word per sample), and the fill is the
 * sample FIFO's.  The sender's crystal is therefore the master clock.
 */

#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "tusb.h"
#include "device/usbd_pvt.h"

#include "sender.h"
#include "usb_descriptors.h"
#include "usb_feedback_controller.h"

snd_fifo_t snd_fifo;

#define NUM_ALTS        8u
#define VOLUME_MIN      (-96 * 256)     // 1/256 dB
#define VOLUME_MAX      0
#define VOLUME_RES      256

typedef struct {
    uint8_t  channels, bytes;
    uint32_t rate;                      // 0 = SET_CUR selects (stereo alts)
} alt_fmt_t;

static const alt_fmt_t alt_fmt[NUM_ALTS + 1] = {
    { 0, 0, 0 },
    { 2, 2, 0 }, { 2, 3, 0 },
    { 4, 2, 48000 }, { 6, 2, 48000 }, { 8, 2, 48000 }, { 4, 3, 48000 },
    { 6, 3, 48000 }, { 4, 2, 96000 },
};

static uint8_t __attribute__((aligned(4))) ep_out_buf[AUDIO_EP_MAX_PKT];
static uint8_t __attribute__((aligned(4))) ep_fb_buf[4];
static uint8_t ctrl_buf[8];

static struct {
    uint8_t ac_itf, as_itf, vendor_itf, cur_alt;
    bool    ep_data_open, ep_fb_open, notify_ep_open;
    uint8_t pending_cs, pending_recipient;
    uint32_t selected_rate;             // stereo alts' SET_CUR rate
    bool    mute;
    int16_t volume;
    uint32_t last_packet_us;
} uac1 = { .vendor_itf = 0xFF, .selected_rate = 48000 };

static usb_feedback_ctrl_t fb;
static uint32_t fb_rate;

// ---------------------------------------------------------------------------
// Rate and gain
// ---------------------------------------------------------------------------

static void set_stream_rate(uint32_t rate) {
    if (rate != snd.usb_rate) {
        snd.usb_rate = rate;
        __dmb();
        snd.usb_rate_follow = true;     // the main loop schedules RATE on the links
    }
}

static void update_gain(void) {
    int32_t g = 0;
    if (!uac1.mute) {
        float db = (float)uac1.volume / 256.0f;
        g = (int32_t)(32768.0f * powf(10.0f, db / 20.0f) + 0.5f);
        if (g > 32768) g = 32768;
    }
    snd.usb_gain_q15 = g;
}

// ---------------------------------------------------------------------------
// Data path: packet -> FIFO
// ---------------------------------------------------------------------------

static void push_packet(const uint8_t *p, uint32_t len) {
    const alt_fmt_t *a = &alt_fmt[uac1.cur_alt];
    if (!a->channels) return;
    unsigned fbytes = (unsigned)a->channels * a->bytes;
    unsigned frames = len / fbytes;
    int32_t g = snd.usb_gain_q15;
    uint32_t wr = snd_fifo.wr;
    for (unsigned i = 0; i < frames; i++) {
        if (wr - snd_fifo.rd >= SND_FIFO_FRAMES) {
            snd.usb_overruns = snd.usb_overruns + (frames - i);
            break;
        }
        int32_t *out = snd_fifo.s[wr & (SND_FIFO_FRAMES - 1u)];
        for (unsigned ch = 0; ch < a->channels; ch++) {
            int32_t v;
            if (a->bytes == 2) {
                v = (int32_t)(int16_t)(p[0] | (p[1] << 8)) * 256;
                p += 2;
            } else {
                uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
                v = (int32_t)(u ^ 0x800000u) - 0x800000;
                p += 3;
            }
            if (g != 32768) v = (int32_t)(((int64_t)v * g) >> 15);
            out[ch] = v;
        }
        for (unsigned ch = a->channels; ch < SND_USB_CHANNELS; ch++) out[ch] = 0;
        wr++;
    }
    __dmb();
    snd_fifo.wr = wr;
}

// ---------------------------------------------------------------------------
// Class driver
// ---------------------------------------------------------------------------

static void drv_init(void) {
    memset(&uac1, 0, sizeof(uac1));
    uac1.vendor_itf = 0xFF;
    uac1.selected_rate = 48000;
    fb_ctrl_init(&fb);
    update_gain();
}

static bool drv_deinit(void) { return true; }

static void drv_reset(uint8_t rhport) {
    (void)rhport;
    uac1.ep_data_open = uac1.ep_fb_open = uac1.notify_ep_open = false;
    uac1.cur_alt = 0;
    uac1.vendor_itf = 0xFF;
    snd.usb_alt = 0;
}

static uint16_t drv_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    if (itf_desc->bInterfaceClass == 0xFF && itf_desc->bAlternateSetting == 0 && itf_desc->bNumEndpoints <= 1) {
        // The vendor interface and its notification endpoint (opened, never
        // armed: the sender sends no notifications).
        uac1.vendor_itf = itf_desc->bInterfaceNumber;
        uint16_t n = tu_desc_len(itf_desc);
        if (itf_desc->bNumEndpoints == 1) {
            uint8_t const *ep = (uint8_t const *)itf_desc + n;
            if (tu_desc_type(ep) == TUSB_DESC_ENDPOINT) {
                TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *)ep));
                uac1.notify_ep_open = true;
                n += tu_desc_len(ep);
            }
        }
        return n;
    }
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_AUDIO);
    TU_VERIFY(itf_desc->bInterfaceSubClass == AUDIO_SUBCLASS_CONTROL);
    TU_VERIFY(itf_desc->bAlternateSetting == 0);
    TU_VERIFY(itf_desc->bInterfaceNumber == ITF_NUM_AUDIO_CONTROL);
    uac1.ac_itf = itf_desc->bInterfaceNumber;

    uint8_t const *p = (uint8_t const *)itf_desc, *end = p + max_len;
    uint16_t len = tu_desc_len(p);
    p += tu_desc_len(p);
    while (p < end && tu_desc_type(p) == TUSB_DESC_CS_INTERFACE) {
        len += tu_desc_len(p);
        p += tu_desc_len(p);
    }
    bool alloc_out = false, alloc_fb = false;
    while (p < end && tu_desc_type(p) == TUSB_DESC_INTERFACE) {
        tusb_desc_interface_t const *as = (tusb_desc_interface_t const *)p;
        if (as->bInterfaceClass != TUSB_CLASS_AUDIO || as->bInterfaceSubClass != AUDIO_SUBCLASS_STREAMING) break;
        uac1.as_itf = as->bInterfaceNumber;
        len += tu_desc_len(p);
        p += tu_desc_len(p);
        while (p < end && tu_desc_type(p) != TUSB_DESC_INTERFACE) {
            if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
                tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;
                if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
                    if (ep->bEndpointAddress == AUDIO_OUT_ENDPOINT && !alloc_out) {
                        usbd_edpt_iso_alloc(rhport, AUDIO_OUT_ENDPOINT, AUDIO_EP_MAX_PKT);
                        alloc_out = true;
                    } else if (ep->bEndpointAddress == AUDIO_IN_ENDPOINT && !alloc_fb) {
                        usbd_edpt_iso_alloc(rhport, AUDIO_IN_ENDPOINT, 4);
                        alloc_fb = true;
                    }
                }
            }
            len += tu_desc_len(p);
            p += tu_desc_len(p);
        }
    }
    usbd_sof_enable(rhport, SOF_CONSUMER_AUDIO, true);
    return len;
}

static inline void arm_data_out(uint8_t rhport) {
    usbd_edpt_xfer(rhport, AUDIO_OUT_ENDPOINT, ep_out_buf, AUDIO_EP_MAX_PKT);
}

static inline void arm_feedback(uint8_t rhport) {
    uint32_t v = snd.feedback_10_14;
    ep_fb_buf[0] = (uint8_t)v;
    ep_fb_buf[1] = (uint8_t)(v >> 8);
    ep_fb_buf[2] = (uint8_t)(v >> 16);
    ep_fb_buf[3] = 0;
    usbd_edpt_xfer(rhport, AUDIO_IN_ENDPOINT, ep_fb_buf, 3);
}

static void close_stream_eps(uint8_t rhport) {
    if (uac1.ep_data_open) usbd_edpt_close(rhport, AUDIO_OUT_ENDPOINT);
    if (uac1.ep_fb_open) usbd_edpt_close(rhport, AUDIO_IN_ENDPOINT);
    uac1.ep_data_open = uac1.ep_fb_open = false;
}

static bool open_stream_eps(uint8_t rhport, uint8_t alt) {
    TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *)usb_audio_data_ep_desc[alt - 1]));
    TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *)usb_audio_fb_ep_desc[alt - 1]));
    // Flush the stale busy/AVAIL state a previous alt leaves (see usb_audio.c).
    usbd_edpt_stall(rhport, AUDIO_OUT_ENDPOINT);
    usbd_edpt_clear_stall(rhport, AUDIO_OUT_ENDPOINT);
    usbd_edpt_stall(rhport, AUDIO_IN_ENDPOINT);
    usbd_edpt_clear_stall(rhport, AUDIO_IN_ENDPOINT);
    uac1.ep_data_open = uac1.ep_fb_open = true;
    arm_data_out(rhport);
    arm_feedback(rhport);
    return true;
}

static bool apply_alt(uint8_t rhport, uint8_t alt) {
    if (alt > NUM_ALTS) return false;
    if (alt == uac1.cur_alt) return true;
    close_stream_eps(rhport);
    uac1.cur_alt = alt;
    snd.usb_channels = alt_fmt[alt].channels;
    uac1.last_packet_us = 0;
    if (alt) {
        set_stream_rate(alt_fmt[alt].rate ? alt_fmt[alt].rate : uac1.selected_rate);
        snd.usb_alt = alt;
        return open_stream_eps(rhport, alt);
    }
    snd.usb_alt = 0;
    return true;
}

static bool fu_get(uint8_t rhport, tusb_control_request_t const *req) {
    uint8_t cs = TU_U16_HIGH(req->wValue);
    static int16_t v;
    static uint8_t m;
    if (cs == UAC1_FU_CTRL_MUTE && req->bRequest == UAC1_REQ_GET_CUR) {
        m = uac1.mute;
        return tud_control_xfer(rhport, req, &m, 1);
    }
    if (cs != UAC1_FU_CTRL_VOLUME) return false;
    switch (req->bRequest) {
    case UAC1_REQ_GET_CUR: v = uac1.volume; break;
    case UAC1_REQ_GET_MIN: v = VOLUME_MIN; break;
    case UAC1_REQ_GET_MAX: v = VOLUME_MAX; break;
    case UAC1_REQ_GET_RES: v = VOLUME_RES; break;
    default: return false;
    }
    return tud_control_xfer(rhport, req, &v, 2);
}

static bool drv_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req) {
    if (stage == CONTROL_STAGE_SETUP) {
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
            uint8_t itf = TU_U16_LOW(req->wIndex);
            if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
                uint8_t alt = TU_U16_LOW(req->wValue);
                if (itf == uac1.as_itf) {
                    if (!apply_alt(rhport, alt)) return false;
                } else if ((itf != uac1.ac_itf && itf != uac1.vendor_itf) || alt != 0) {
                    return false;
                }
                return tud_control_status(rhport, req);
            }
            if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
                static uint8_t r;
                if (itf == uac1.as_itf) r = uac1.cur_alt;
                else if (itf == uac1.ac_itf || itf == uac1.vendor_itf) r = 0;
                else return false;
                return tud_control_xfer(rhport, req, &r, 1);
            }
            return false;
        }
        if (req->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) return false;
        bool get = req->bmRequestType_bit.direction == TUSB_DIR_IN;
        uint8_t rcpt = req->bmRequestType_bit.recipient;
        if (rcpt == TUSB_REQ_RCPT_INTERFACE) {
            if (TU_U16_LOW(req->wIndex) != uac1.ac_itf || TU_U16_HIGH(req->wIndex) != UAC1_FEATURE_UNIT_ID) return false;
            if (get) return fu_get(rhport, req);
        } else if (rcpt == TUSB_REQ_RCPT_ENDPOINT) {
            if (TU_U16_LOW(req->wIndex) != AUDIO_OUT_ENDPOINT) return false;
            if (get) {
                if (req->bRequest != UAC1_REQ_GET_CUR || TU_U16_HIGH(req->wValue) != UAC1_EP_CTRL_SAMPLING_FREQ)
                    return false;
                static uint8_t f[3];
                uint32_t r = snd.usb_rate ? snd.usb_rate : uac1.selected_rate;
                f[0] = (uint8_t)r; f[1] = (uint8_t)(r >> 8); f[2] = (uint8_t)(r >> 16);
                return tud_control_xfer(rhport, req, f, 3);
            }
        } else {
            return false;
        }
        if (req->bRequest != UAC1_REQ_SET_CUR || req->wLength == 0 || req->wLength > sizeof(ctrl_buf)) return false;
        uac1.pending_cs = TU_U16_HIGH(req->wValue);
        uac1.pending_recipient = rcpt;
        return tud_control_xfer(rhport, req, ctrl_buf, req->wLength);
    }
    if (stage == CONTROL_STAGE_DATA && req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
        uint8_t rcpt = uac1.pending_recipient;
        uac1.pending_recipient = 0;
        if (rcpt == TUSB_REQ_RCPT_INTERFACE) {
            if (uac1.pending_cs == UAC1_FU_CTRL_MUTE) {
                uac1.mute = ctrl_buf[0] != 0;
            } else if (uac1.pending_cs == UAC1_FU_CTRL_VOLUME) {
                int16_t v;
                memcpy(&v, ctrl_buf, 2);
                if (v < VOLUME_MIN) v = VOLUME_MIN;
                if (v > VOLUME_MAX) v = VOLUME_MAX;
                uac1.volume = v;
            }
            update_gain();
        } else if (rcpt == TUSB_REQ_RCPT_ENDPOINT && uac1.pending_cs == UAC1_EP_CTRL_SAMPLING_FREQ) {
            uint32_t f = ctrl_buf[0] | ((uint32_t)ctrl_buf[1] << 8) | ((uint32_t)ctrl_buf[2] << 16);
            uint32_t fixed = alt_fmt[uac1.cur_alt].rate;
            bool ok = fixed ? f == fixed : (f == 44100u || f == 48000u || f == 96000u);
            if (!ok) return false;          // STALL: UAC1 rejects, never clamps
            if (!fixed) uac1.selected_rate = f;
            if (uac1.cur_alt) set_stream_rate(f);
        }
    }
    return true;
}

static bool drv_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t n) {
    if (ep_addr == AUDIO_OUT_ENDPOINT) {
        if (result == XFER_RESULT_SUCCESS && n > 0) {
            snd.usb_packets = snd.usb_packets + 1u;
            uint32_t now = time_us_32();
            if (uac1.last_packet_us) {
                uint32_t gap = now - uac1.last_packet_us;
                if (gap > 2000u && gap < 50000u) snd.usb_gaps = snd.usb_gaps + 1u;
            }
            uac1.last_packet_us = now;
            if (!snd.config[0]) push_packet(ep_out_buf, n);     // fan-out mode plays no USB audio
        }
        if (uac1.ep_data_open) arm_data_out(rhport);
        return true;
    }
    if (ep_addr == AUDIO_IN_ENDPOINT) {
        if (uac1.ep_fb_open) arm_feedback(rhport);
        return true;
    }
    if (ep_addr == NOTIFY_IN_ENDPOINT) return true;
    return false;
}

// The feedback servo.  It runs only while the encoder plays USB samples:
// then the samples consumed per USB frame are the link frames sent, exactly
// one per frame; otherwise the host is told the nominal rate.
static void drv_sof(uint8_t rhport, uint32_t frame_count) {
    (void)rhport;
    uint32_t fs = snd.usb_rate ? snd.usb_rate : 48000u;
    uint32_t nominal = (uint32_t)(((uint64_t)fs << 14) / 1000u);
    if (!snd.usb_playing) {
        if (fb.stream_active) fb_ctrl_stream_stop(&fb);
        snd.feedback_10_14 = nominal;
        return;
    }
    if (!fb.stream_active || fb_rate != fs) {
        fb_ctrl_reset(&fb, (uint32_t)(((uint64_t)fs << 16) / 1000u));
        fb_rate = fs;
    }
    uint32_t fill = snd_fifo.wr - snd_fifo.rd;
    fb_ctrl_sof_update(&fb, frame_count, (uint32_t)snd.wire_frame, 14,
                       (fill << 16) / SLOT_FILL_FRAMES_PER_BUCKET,
                       (snd_fifo_target(fs) << 16) / SLOT_FILL_FRAMES_PER_BUCKET);
    uint32_t v = fb_ctrl_get_10_14(&fb);
    snd.feedback_10_14 = v ? v : nominal;
}

static const usbd_class_driver_t uac1_driver = {
    .name = "DSPi_Sender_UAC1",
    .init = drv_init,
    .deinit = drv_deinit,
    .reset = drv_reset,
    .open = drv_open,
    .control_xfer_cb = drv_control_xfer_cb,
    .xfer_cb = drv_xfer_cb,
    .sof = drv_sof,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &uac1_driver;
}

void sender_usb_init(void) {
    snd.feedback_10_14 = (48000u << 14) / 1000u;
    snd.usb_rate = 0;
    snd.usb_gain_q15 = 32768;
    tud_init(0);
}

void sender_usb_task(void) {
    tud_task();
}
