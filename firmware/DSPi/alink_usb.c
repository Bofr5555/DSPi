/*
 * alink_usb.c - USB-direct sync (WP USB1), firmware side.  See alink_usb.h and
 * alink_usb/alink_usb_core.h.
 *
 * RAM: the two USB IRQ entry points (alink_usb_isr_sof, alink_usb_isr_out_done)
 * and the time base.  Everything else (one call per packet in phase mode, the
 * service once per pass, the control plane) runs from flash, like the link's
 * phase glue; check_ram_placement.py whitelists alink_usb_fw_deliver as a cold
 * callee of usb_audio_drain_ring().
 */

#include "alink_usb.h"

#include <stddef.h>
#include <string.h>

#include "audio_pipeline.h"
#include "bulk_params.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "input_servo.h"
#include "notify.h"
#include "output_latency.h"
#include "soft_vcxo.h"
#include "usb_audio.h"
#include "usb_audio_ring.h"   // USB_RING_MAX_PKT
#include "audio_input.h"
#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/structs/usb.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

alink_usb_t alink_usb;
alink_usb_tb_t alink_usb_tb;                // .bss; stamp set to NONE by alink_usb_fw_init()
volatile bool alink_usb_fb_nominal;
const link_apply_limits_t alink_usb_apply_limits = { PRESET_SLOTS, NUM_OUTPUT_CHANNELS, MAX_DELAY_SAMPLES };

#if defined(DSPI_ALINK_USB_GPIO)
// Bench builds only: the sync GPIO is high for one frame at every SOF whose
// frame number is a multiple of 128 (E0: SOF skew between units on the scope)
// and at the SOF of a SYNC_PULSE op's frame.
static volatile uint32_t pulse_frame = ALINK_USB_FRAME_NONE;
#endif

// ---------------------------------------------------------------------------
// USB IRQ (RAM)
// ---------------------------------------------------------------------------

void __not_in_flash_func(alink_usb_isr_sof)(uint32_t frame11) {
    uint32_t now = time_us_32() * 48u;
#if PICO_RP2350
    // The PHY latched the SOF on its 48 MHz counter: exact to 21 ns.
    uint32_t since = (usb_hw->sof_timestamp_raw - usb_hw->sof_timestamp_last) & 0x1FFFFFu;
#else
    uint32_t since = UINT32_MAX;
#endif
    alink_usb_tb_sof(&alink_usb_tb, frame11, now, since);
#if defined(DSPI_ALINK_USB_GPIO)
    uint32_t n = alink_usb_tb.n;
    gpio_put(DSPI_ALINK_USB_GPIO, (n & 127u) == 0u || n == pulse_frame);
#endif
}


// ---------------------------------------------------------------------------
// Port
// ---------------------------------------------------------------------------

uint32_t alink_usb_port_now48(void) { return time_us_32() * 48u; }
uint32_t alink_usb_port_now_us(void) { return time_us_32(); }
uint32_t alink_usb_port_irq_save(void) { return save_and_disable_interrupts(); }
void alink_usb_port_irq_restore(uint32_t s) { restore_interrupts(s); }
uint32_t alink_usb_port_fs(void) { return audio_state.freq; }

bool alink_usb_port_play(float *fill) {
    uint32_t played;
    return slot_ring_play_position(0, fill, &played);
}

uint32_t alink_usb_port_room(void) {
    uint32_t min_frames;
    return producer_block_bounds(&min_frames);
}

static void inputs_zero(uint32_t k) {
    memset(buf_l, 0, k * sizeof(buf_l[0]));
    memset(buf_r, 0, k * sizeof(buf_r[0]));
#if PICO_RP2350
    for (int c = 0; c < NUM_INPUT_CHANNELS - NUM_STEREO_INPUTS; c++) memset(buf_in_ext[c], 0, k * sizeof(float));
#endif
}

static void inputs_shift(uint32_t off, uint32_t k) {
    memmove(buf_l, buf_l + off, k * sizeof(buf_l[0]));
    memmove(buf_r, buf_r + off, k * sizeof(buf_r[0]));
#if PICO_RP2350
    for (int c = 0; c < NUM_INPUT_CHANNELS - NUM_STEREO_INPUTS; c++)
        memmove(buf_in_ext[c], buf_in_ext[c] + off, k * sizeof(float));
#endif
}

static void block_done(void) {
#if PICO_RP2350
    pipeline_click_output = -1;
#endif
}

// Input gain ramp on the decoded samples (review usb1 F1: fades around gaps).
static void inputs_ramp(uint32_t k, float g0, float g1) {
    if (g0 == 1.0f && g1 == 1.0f) return;
    float step = k > 1u ? (g1 - g0) / (float)(k - 1u) : 0.0f;
#if PICO_RP2350
    float *ch[NUM_INPUT_CHANNELS] = { buf_l, buf_r };
    for (int c = 0; c < NUM_INPUT_CHANNELS - NUM_STEREO_INPUTS; c++) ch[NUM_STEREO_INPUTS + c] = buf_in_ext[c];
    for (int c = 0; c < NUM_INPUT_CHANNELS; c++)
        for (uint32_t i = 0; i < k; i++) ch[c][i] *= g0 + step * (float)i;
#else
    for (uint32_t i = 0; i < k; i++) {
        int32_t g = (int32_t)((g0 + step * (float)i) * 32768.0f);
        buf_l[i] = (int32_t)(((int64_t)buf_l[i] * g) >> 15);
        buf_r[i] = (int32_t)(((int64_t)buf_r[i] * g) >> 15);
    }
#endif
}

// The held packet's bytes (alink_usb_port_keep): .bss, one packet.
static uint8_t keep_buf[USB_RING_MAX_PKT] __attribute__((aligned(4)));

const void *alink_usb_port_keep(const alink_usb_pkt_t *p) {
    uint16_t n = p->len <= sizeof(keep_buf) ? p->len : (uint16_t)sizeof(keep_buf);
    memcpy(keep_buf, p->data, n);
    return keep_buf;
}

void alink_usb_port_write_packet(const alink_usb_pkt_t *p, uint32_t off, uint32_t k, float g0, float g1) {
    // A packet written in parts (an apply op splits it) is decoded once per
    // part; only the first part counts in the USB stream bookkeeping (review usb1 F7).
    extern volatile uint64_t total_samples_produced;
    uint64_t produced = total_samples_produced;
    int32_t d = usb_audio_decode_packet((const uint8_t *)p->data, p->len);
    if (off) total_samples_produced = produced;
    if (d <= 0) return;
    uint32_t n = (uint32_t)d;
    if (off >= n) return;
    if (k > n - off) k = n - off;
    if (off) inputs_shift(off, k);
    inputs_ramp(k, g0, g1);
    process_input_block(k);
    block_done();
}

void alink_usb_port_write_silence(uint32_t k) {
    inputs_zero(k);
    process_input_block(k);
    block_done();
}

void alink_usb_port_actuate(float hz, float trim, bool ff_trim) {
    if (ff_trim) input_servo_apply_trim(hz, trim);
    else input_servo_apply(hz, 0);
}

void alink_usb_port_park(void) { input_servo_reset(); }
float alink_usb_port_vcxo_ppm(void) { return soft_vcxo_current_ppm(); }

uint8_t alink_usb_port_apply(const link_apply_op_t *op) {
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
        float ms = ((float)op->value + 0.5f) * 1000.0f / fs;
        channel_delays_ms[CH_OUT_1 + op->arg] = ms;
        dsp_update_delay_samples(fs);
        notify_param_write((uint16_t)(offsetof(WireBulkParams, delays.delay_ms) +
                                      (CH_OUT_1 + op->arg) * sizeof(float)), sizeof(float), &ms);
        return LINK_APPLY_APPLIED;
    }
    case LINK_OP_CLICK:
#if PICO_RP2350
        // -20 dBFS, never on the PDM sub or a muted unit (as the link, 2.8.7).
        if (op->arg >= NUM_SPDIF_INSTANCES * 2 || user_mute || audio_state.mute) return LINK_APPLY_REFUSED;
        pipeline_click_output = (int8_t)op->arg;
        return LINK_APPLY_APPLIED;
#else
        return LINK_APPLY_REFUSED;          // the RP2040's fixed-point pipeline has no click injection
#endif
    case LINK_OP_SYNC_PULSE:
#if defined(DSPI_ALINK_USB_GPIO)
        return LINK_APPLY_APPLIED;          // the ISR pulsed the GPIO at the op's SOF
#else
        return LINK_APPLY_REFUSED;
#endif
    default:
        return LINK_APPLY_REFUSED;
    }
}

void alink_usb_port_apply_result(uint8_t op, uint8_t result, uint32_t frame) {
    notify_push_apply_result(op, result, frame);
}

void alink_usb_port_event(uint8_t state, uint8_t reason, uint32_t frame) {
    uint8_t ps = alink_usb.started || alink_usb.prefilling ? alink_usb.ph.state : LINK_PHASE_FREE;
    notify_push_alink_usb(state, ps, reason, frame);
}

void alink_usb_port_soft_mute(uint32_t samples) { pipeline_request_soft_mute(samples); }

bool alink_usb_port_pulse(uint32_t frame) {
#if defined(DSPI_ALINK_USB_GPIO)
    pulse_frame = frame;
    return true;
#else
    (void)frame;
    return false;
#endif
}

void alink_usb_port_ring_target(void) {
    output_latency_apply(audio_state.freq ? audio_state.freq : 48000u, INPUT_SOURCE_USB);
}

// ---------------------------------------------------------------------------
// Firmware API
// ---------------------------------------------------------------------------

void alink_usb_fw_init(void) {
    alink_usb_tb.stamp = ALINK_USB_FRAME_NONE;
    uint8_t s[ALINK_USB_STORE_LEN];
    alink_usb_store(&alink_usb, s);           // keep values loaded from flash before init
    alink_usb_init(&alink_usb);
    alink_usb_load(&alink_usb, s);
#if PICO_RP2350
    alink_usb.exact = true;                 // SOF times from the PHY counter
#endif
#if defined(DSPI_ALINK_USB_GPIO)
    gpio_init(DSPI_ALINK_USB_GPIO);
    gpio_set_dir(DSPI_ALINK_USB_GPIO, GPIO_OUT);
    gpio_put(DSPI_ALINK_USB_GPIO, 0);
#endif
}

void alink_usb_fw_deliver(uint32_t frame, const uint8_t *data, uint16_t len) {
    alink_usb_pkt_t p = { frame, (uint16_t)usb_audio_packet_samples(len), data, len };
    alink_usb_deliver(&alink_usb, &p);
}

static void refresh_limits(void) {
    uint32_t fs = audio_state.freq ? audio_state.freq : 48000u;
    const OutputLatencyOption *opt = output_latency_resolve(output_latency_requested(), INPUT_SOURCE_USB);
    alink_usb_limits(&alink_usb, fs, output_latency_target_frames(opt, fs), output_latency_block_cap(opt, fs),
                    output_latency_ring_span());
}

void alink_usb_fw_service(void) {
    if (alink_usb.mode != ALINK_USB_MODE_OFF) refresh_limits();
    alink_usb_service(&alink_usb, to_ms_since_boot(get_absolute_time()));
    alink_usb_fb_nominal = alink_usb_nominal_feedback(&alink_usb);
}

void alink_usb_fw_stop(void) {
    alink_usb_stop(&alink_usb, ALINK_USB_RS_DISABLED);
    alink_usb_fb_nominal = false;
}

bool alink_usb_fw_anchor_ready(void) {
    float wait = alink_usb_anchor_wait(&alink_usb);
    if (wait > (float)alink_usb.spf) return false;
    // The last millisecond in place, so the start lands to a fraction of a
    // sample; the verification corrects a late start.
    uint32_t t0 = time_us_32();
    while (wait > 0.0f && time_us_32() - t0 < 1100u) wait = alink_usb_anchor_wait(&alink_usb);
    return true;
}

uint32_t alink_usb_fw_ring_target(uint32_t fs, uint32_t target, uint32_t cap) {
    return alink_usb_ring_target(&alink_usb, fs, target, cap, output_latency_ring_span());
}

bool alink_usb_fw_set_config(const uint8_t p[ALINK_USB_CFG_LEN]) {
    refresh_limits();
    bool ok = alink_usb_set_config(&alink_usb, p);
    if (ok && alink_usb.mode == ALINK_USB_MODE_OFF) alink_usb_fw_stop();
    return ok;
}

void alink_usb_fw_get_config(uint8_t o[ALINK_USB_CFG_LEN]) {
    refresh_limits();
    alink_usb_get_config(&alink_usb, o);
}

void alink_usb_fw_get_status(uint8_t o[ALINK_USB_STATUS_LEN]) {
    refresh_limits();
    alink_usb_get_status(&alink_usb, o);
}

bool alink_usb_fw_time_base_active(void) {
    return active_input_source == INPUT_SOURCE_USB && alink_usb.mode != ALINK_USB_MODE_OFF;
}

void alink_usb_fw_get_frame_counter(uint8_t o[20]) {
    memset(o, 0, 20);
    uint64_t s = 0;
    uint16_t bus = 0;
    uint32_t save = save_and_disable_interrupts();
    bool ok = alink_usb_sample_now(&alink_usb, &s, &bus);
    uint32_t t = time_us_32();
    float fill;
    uint32_t played = 0;
    if (!slot_ring_play_position(0, &fill, &played)) played = 0;
    restore_interrupts(save);
    if (!ok) s = 0;
    for (int i = 0; i < 8; i++) o[i] = (uint8_t)(s >> (8 * i));
    for (int i = 0; i < 4; i++) o[8 + i] = (uint8_t)(t >> (8 * i));
    for (int i = 0; i < 4; i++) o[12 + i] = (uint8_t)(played >> (8 * i));
    o[16] = 1;                              // time base: USB sample index S
    o[18] = (uint8_t)bus;
    o[19] = (uint8_t)(bus >> 8);
}

bool alink_usb_fw_apply(const uint8_t p[LINK_APPLY_LEN]) {
    if (active_input_source != INPUT_SOURCE_USB && p[8] != LINK_APPLY_CANCEL_ALL) return false;
    return alink_usb_apply(&alink_usb, p);
}

void alink_usb_fw_get_apply(uint8_t o[LINK_APPLY_LEN]) {
    link_apply_in(&alink_usb.aq, o);
}

void alink_usb_fw_store(uint8_t s[ALINK_USB_STORE_LEN]) { alink_usb_store(&alink_usb, s); }
void alink_usb_fw_load(const uint8_t s[ALINK_USB_STORE_LEN]) { alink_usb_load(&alink_usb, s); }
