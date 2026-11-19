// sg_unit.c - one unit's real signal generator and time-base aggregator
// (firmware/DSPi/siggen.c, timebase.c) behind the table of sg_unit.h (WP FWTB3).
// Compiled once; run.sh makes the per-unit copies with objcopy (sg_unit.sh).
// Build flags as the host tests' (tests/siggen_host.h): an RP2350 build with
// the ops (DSPI_SIGGEN_SYNC), -I firmware/DSPi -I firmware/tests/stubs.
#ifndef PICO_RP2350
#define PICO_RP2350 1
#endif
#define DSPI_SIGGEN_SYNC 1
#define USB_AUDIO_H              // siggen.c needs only audio_state from usb_audio.h
#include "config.h"
#include "notify.h"
#include <stdint.h>
#include <string.h>
typedef struct { uint32_t freq; bool mute; } AudioStateStub;
static volatile AudioStateStub audio_state = { 48000u, false };
#include "siggen.c"
#include "timebase.c"

#include "sg_unit.h"

MatrixMixer matrix_mixer;
sg_port_t *sg_port;

void notify_push_siggen_state(uint8_t s, uint8_t r, uint8_t t, uint8_t c) { (void)s; (void)r; (void)t; (void)c; }
bool siggen_output_ready_now(void) { return sg_port->ready_now(); }
uint16_t siggen_port_arm_lead_ms(void) { return sg_port->arm_lead_ms(); }
uint32_t siggen_port_slips(void) { return sg_port->slips(); }
uint32_t timebase_port_irq_save(void) { return 0; }
void timebase_port_irq_restore(uint32_t s) { (void)s; }
uint32_t timebase_port_now_us(void) { return sg_port->now_us(); }
bool timebase_port_ring(uint32_t *wr, uint32_t *played) { return sg_port->ring(wr, played); }
void timebase_port_event(uint8_t tb, uint8_t st, uint8_t rs, uint32_t t) { sg_port->event(tb, st, rs, t); }

static float sgu_bufs[NUM_OUTPUT_CHANNELS][AUDIO_BUFFER_SAMPLES];

static bool sgu_running(void) { return siggen_running; }

static void sgu_render(float *out, unsigned ch, uint32_t n, uint32_t fs) {
    if (n > AUDIO_BUFFER_SAMPLES) n = AUDIO_BUFFER_SAMPLES;
    for (unsigned o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
        matrix_mixer.outputs[o].enabled = 1;
        memset(sgu_bufs[o], 0, n * sizeof(float));
    }
    // As process_input_block(): the generator renders only while it runs.
    if (siggen_running) siggen_render(sgu_bufs, n, fs);
    memcpy(out, sgu_bufs[ch < NUM_OUTPUT_CHANNELS ? ch : 0], n * sizeof(float));
}

static void sgu_hold(float *out, uint32_t n) {
    if (n > AUDIO_BUFFER_SAMPLES) n = AUDIO_BUFFER_SAMPLES;
    if (siggen_running) siggen_hold(sgu_bufs, n);
    memset(out, 0, n * sizeof(float));
}

const sg_unit_t sg_unit = {
    .port = &sg_port,
    .stage_config = siggen_stage_config,
    .control = siggen_control,
    .service = siggen_service,
    .wants_output = siggen_wants_output,
    .output_gate = siggen_output_gate,
    .running = sgu_running,
    .render = sgu_render,
    .hold = sgu_hold,
    .tb_accept = siggen_tb_accept,
    .tb_accept_broadcast = siggen_tb_accept_broadcast,
    .tb_poll = siggen_tb_poll,
    .tb_fire = siggen_tb_fire,
    .tb_stop_op = siggen_tb_stop_op,
    .tb_dropped = siggen_tb_dropped,
    .tb_cancel_all = siggen_tb_cancel_all,
    .tb_timeline = siggen_tb_timeline,
    .tb_register = timebase_register,
    .tb_service = timebase_service,
    .tb_read = timebase_read,
    .tb_state = timebase_state,
};
