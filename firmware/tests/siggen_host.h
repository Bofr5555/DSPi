// siggen_host.h - the real siggen.c on the host with the WP FWTB1 port
// (siggen.h "Start on the time base"), for tests that drive the generator
// together with a transport (test_alink_usb.c, test_siggen_dac.c,
// test_timebase.c).  test_siggen_timing.c keeps its own stubs.
//
// The port: siggen_output_ready_now() (main.c's DAC readiness),
// siggen_port_arm_lead_ms() (150 ms, the K340's), siggen_port_slips()
// (output underrun episodes).  siggen_port_drop_op9() and
// siggen_port_tb_now() come from timebase.c, which the test includes too.
#ifndef SIGGEN_HOST_H
#define SIGGEN_HOST_H

#ifndef PICO_RP2350
#define PICO_RP2350 1
#endif
#define DSPI_SIGGEN_SYNC 1
#define USB_AUDIO_H              // siggen.c needs only audio_state from it
#include "config.h"
#include "notify.h"
#include <stdint.h>
typedef struct { uint32_t freq; bool mute; } AudioStateStub;
static volatile AudioStateStub audio_state = { 48000u, false };
#include "siggen.c"

MatrixMixer matrix_mixer;
static int sh_n_siggen_notify;
void notify_push_siggen_state(uint8_t s, uint8_t r, uint8_t t, uint8_t c) {
    (void)s; (void)r; (void)t; (void)c;
    sh_n_siggen_notify++;
}
static bool sh_ready_now = true;
bool siggen_output_ready_now(void) { return sh_ready_now; }
static uint16_t sh_arm_lead_ms = 150u;
uint16_t siggen_port_arm_lead_ms(void) { return sh_arm_lead_ms; }
static uint32_t sh_slips;
uint32_t siggen_port_slips(void) { return sh_slips; }

// EXT_SIGGEN_TIMELINE fields.
typedef struct {
    uint8_t  tb, state, result;
    uint16_t disc, arm_lead_ms;
    uint64_t start, pending;
    uint32_t samples, held, run_id;
} sh_timeline_t;

static inline sh_timeline_t sh_timeline(void) {
    uint8_t o[40];
    siggen_tb_timeline(o);
    sh_timeline_t t;
    t.tb = o[1];
    t.state = o[2];
    t.result = o[3];
    t.disc = (uint16_t)(o[4] | o[5] << 8);
    t.arm_lead_ms = (uint16_t)(o[6] | o[7] << 8);
    t.start = t.pending = 0;
    for (int i = 7; i >= 0; i--) {
        t.start = t.start << 8 | o[8 + i];
        t.pending = t.pending << 8 | o[16 + i];
    }
    t.samples = (uint32_t)o[24] | (uint32_t)o[25] << 8 | (uint32_t)o[26] << 16 | (uint32_t)o[27] << 24;
    t.held = (uint32_t)o[28] | (uint32_t)o[29] << 8 | (uint32_t)o[30] << 16 | (uint32_t)o[31] << 24;
    t.run_id = (uint32_t)o[32] | (uint32_t)o[33] << 8 | (uint32_t)o[34] << 16 | (uint32_t)o[35] << 24;
    return t;
}

// A finite config (log sweeps or impulses) for output `out`.
static inline SiggenConfig sh_cfg(uint8_t type, uint16_t repeat, uint8_t out) {
    SiggenConfig c;
    memset(&c, 0, sizeof c);
    c.version = SIGGEN_CFG_VERSION;
    c.signal_type = type;
    c.channel_mask = (uint16_t)(1u << out);
    c.level_db = -20.0f;
    c.repeat = repeat;
    c.duration_ms = 200;
    c.gap_ms = 10;
    c.p1 = type == SIGGEN_IMPULSE ? 50.0f : 20.0f;
    c.p2 = type == SIGGEN_IMPULSE ? 0.0f : 20000.0f;
    return c;
}

#endif // SIGGEN_HOST_H
