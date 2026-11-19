// Host test for WP FWSIGGEN requirement (1): the signal generator releases the
// DAC hardware mute itself while it plays and restores the previous mute
// behaviour when it goes idle.  Runs the real dac_hw_mute.c (GPIO and clock
// stubbed; the pin level is recorded) under the real siggen_dac.h state
// machine, stepped like main.c siggen_dac_service() once per 100 us pass.
//
// K340 config: enabled, GP11, active low, hold 100 ms, release 100 ms.
#define PICO_RP2350 1
#include "dac_hw_mute.c"
#include "siggen_dac.h"
// WP FWTB1: the real generator for the op 9 cases (arming, hand-back).
#include "siggen_host.h"
static int n_drop9;
static uint8_t drop9_res;
void siggen_port_drop_op9(uint8_t tb, uint8_t r) { (void)tb; n_drop9++; drop9_res = r; }
bool siggen_port_tb_now(uint8_t *tb, uint64_t *t) { (void)tb; (void)t; return false; }

#include <stdio.h>

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

// ---- stubs for dac_hw_mute.c's collaborators --------------------------------
uint64_t stub_time_us;
static int pin_level = -1;      // last driven level (active low: 0 = muted)
static int pin_edges;
void stub_gpio_put(unsigned pin, bool level) {
    (void)pin;
    if (pin_level != (int)level) pin_edges++;
    pin_level = level;
}
uint8_t output_pins[NUM_PIN_OUTPUTS];
uint8_t output_types[NUM_SPDIF_INSTANCES];
uint8_t i2s_bck_pin = 14, i2s_mck_pin = 13;
bool i2s_mck_enabled;
uint8_t spdif_rx_pin = 20;
uint8_t spdif_rx_pin_ext[SPDIF_RX_NUM_INPUTS - 1];
uint8_t spdif_rx_enabled_ext;
uint8_t i2s_clock_pin_mode, i2s_bck_pin_slave;
bool uart_ctrl_owns_pin(uint8_t pin) { (void)pin; return false; }
bool i2c_ctrl_owns_pin(uint8_t pin) { (void)pin; return false; }
bool adat_output_config_enabled(void) { return false; }
uint8_t adat_output_pin(void) { return 0xFF; }
uint8_t adat_input_enabled;
uint8_t adat_input_pin = 0xFF;
void preset_set_dac_hw_mute(const DacHwMuteConfig *cfg) { (void)cfg; }
void notify_param_write(uint16_t off, uint16_t len, const void *data) { (void)off; (void)len; (void)data; }

// ---- harness -----------------------------------------------------------------
#define PASS_US   100u
#define HOLD_MS   100u
#define REL_MS    100u
#define TAIL_US   SIGGEN_DAC_TAIL_BASE_US

static siggen_dac_t sd;
static bool want;            // the generator asks (START pending or running)
static bool playing;         // the engine was started by the gate
static uint64_t started_us, unmuted_us;

static bool muted(void) { return pin_level == 0; }   // active low

static void setup(void) {
    DacHwMuteConfig c = { .enabled = 1, .active_low = 1, .pin = 11, .hold_ms = HOLD_MS, .release_ms = REL_MS };
    stub_time_us = 1000000u;
    dac_hw_mute_init(NULL);
    dac_hw_mute_init(&c);
    siggen_dac_init(&sd);
    dac_hw_mute_siggen_override(false);
    want = playing = false;
    started_us = unmuted_us = 0;
    pin_edges = 0;
}

// One main-loop pass: tick, then siggen_dac_service() as in main.c.
static void pass(void) {
    stub_time_us += PASS_US;
    dac_hw_mute_tick();
    siggen_dac_in_t in = {
        .want = want, .pin_muted = dac_hw_mute_is_asserted(), .bracket = dac_hw_mute_bracket_active(),
        .now_us = (uint32_t)stub_time_us, .settle_us = (HOLD_MS > REL_MS ? HOLD_MS : REL_MS) * 1000u,
        .tail_us = TAIL_US,
    };
    bool was_muted = muted();
    bool ready = siggen_dac_step(&sd, &in);
    dac_hw_mute_siggen_override(sd.override);
    if (was_muted && !muted()) unmuted_us = stub_time_us;
    if (want && ready && !playing) { playing = true; started_us = stub_time_us; }
    if (!want) playing = false;
}

static void run_us(uint64_t us) { for (uint64_t t = 0; t < us; t += PASS_US) pass(); }

// START as siggen_control() does: at once when ready, else wait for the gate.
static void start(void) {
    want = true;
    if (siggen_dac_ready_now(&sd, dac_hw_mute_is_asserted(), dac_hw_mute_bracket_active())) {
        playing = true;
        started_us = stub_time_us;
    }
}

// ---- WP FWTB1: op 9 through the DAC machine ----------------------------------
// The time base runs at 48 kHz from tb0_us; t_now is the sample entering the
// pipeline.  One pass: the transport's arming poll, main.c's
// siggen_dac_service() with the real generator, and at T the transport's
// split block (siggen_tb_fire()).
static uint64_t tb0_us, tb_T;
static bool tb_fired;
static uint8_t tb_fire_res;
static uint64_t tb_now(void) { return (stub_time_us - tb0_us) * 48u / 1000u; }

static void pass_tb(void) {
    stub_time_us += PASS_US;
    dac_hw_mute_tick();
    siggen_tb_poll(TB_ID_LINE, tb_now(), 48000u);
    siggen_dac_in_t in = {
        .want = siggen_wants_output(), .pin_muted = dac_hw_mute_is_asserted(),
        .bracket = dac_hw_mute_bracket_active(), .now_us = (uint32_t)stub_time_us,
        .settle_us = (HOLD_MS > REL_MS ? HOLD_MS : REL_MS) * 1000u, .tail_us = TAIL_US,
    };
    bool was_muted = muted();
    bool ready = siggen_dac_step(&sd, &in);
    dac_hw_mute_siggen_override(sd.override);
    if (was_muted && !muted()) unmuted_us = stub_time_us;
    sh_ready_now = siggen_dac_ready_now(&sd, dac_hw_mute_is_asserted(), dac_hw_mute_bracket_active());
    siggen_output_gate(ready);
    siggen_service();
    if (!tb_fired && tb_T && tb_now() >= tb_T) {
        tb_fired = true;
        tb_fire_res = siggen_tb_fire(TB_ID_LINE, tb_T, false, true);
        if (tb_fire_res == ALINK_APPLY_APPLIED) started_us = stub_time_us;
    }
}

static void run_tb(uint64_t us) { for (uint64_t t = 0; t < us; t += PASS_US) pass_tb(); }

// The USB phase mode out of step: an idle hold mutes the DAC.  A fresh finite
// config; op 9 written lead_ms ahead.
static uint8_t setup_tb(uint32_t lead_ms) {
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);
    tb0_us = stub_time_us;
    tb_fired = false;
    tb_T = 0;
    n_drop9 = 0;
    SiggenConfig c = sh_cfg(SIGGEN_SWEEP_LOG, 2, 1);
    (void)siggen_stage_config(&c, sizeof c);
    uint64_t T = tb_now() + lead_ms * 48u;
    uint8_t r = siggen_tb_accept(TB_ID_LINE, T, tb_now(), 48000u);
    if (r == TB_OK) tb_T = T;
    return r;
}

static void test_fwtb1(void) {
    const uint32_t ARM_US = (HOLD_MS + TB_ARM_MARGIN_MS) * 1000u;    // 150 ms on the K340
    CHECK(siggen_port_arm_lead_ms() == HOLD_MS + TB_ARM_MARGIN_MS);
    // A. Armed at T - arm_lead: the DAC opens then (not at the write), its
    //    un-mute ramp is over at T, and generator sample 0 plays on T.
    CHECK(setup_tb(400) == TB_OK);
    int edges = pin_edges;
    uint64_t t_write = stub_time_us;
    while (!siggen_wants_output() && stub_time_us - t_write < 1000000u) pass_tb();
    uint64_t t_arm = stub_time_us;
    CHECK(t_arm - t_write >= 400000u - ARM_US - PASS_US && t_arm - t_write <= 400000u - ARM_US + 2 * PASS_US);
    CHECK(pin_edges == edges + 1 && unmuted_us == t_arm);   // nothing moved before arming; opened then
    CHECK(sh_timeline().state == TB_TL_ARMED);
    run_tb(t_arm - t_write < 400000u ? 400000u - (t_arm - t_write) + 2 * PASS_US : 0u);
    CHECK(tb_fired && tb_fire_res == ALINK_APPLY_APPLIED && siggen_running);
    CHECK(!muted() && started_us - unmuted_us >= HOLD_MS * 1000u);   // open and settled at T
    CHECK(unmuted_us - t_arm <= 2 * PASS_US);       // opened right at arming
    CHECK(sh_timeline().result == TB_RES_STARTED && sh_timeline().start == tb_T);
    siggen_control(SIGGEN_CTL_STOP_NOW);            // done: the pin is handed back after the tail
    run_tb(TAIL_US + 2 * PASS_US);
    CHECK(muted() && sd.st == SIGGEN_DAC_IDLE && !sd.override);

    // B. Too little lead: refused at the write, no pin edge, nothing armed.
    CHECK(setup_tb(149) == TB_ERR_STATE);
    edges = pin_edges;
    run_tb(300000u);
    CHECK(muted() && pin_edges == edges && !siggen_wants_output() && !tb_fired);

    // C. Cancelled after arming (a SET_CONFIG: CANCELLED_CONFIG): wants output
    //    is false at once; the override closes after its tail and the idle
    //    hold drives the pin again.  The transport drops the queued op.
    CHECK(setup_tb(300) == TB_OK);
    run_tb(300000u - ARM_US + 20000u);              // armed, the pin open 20 ms
    CHECK(siggen_wants_output() && !muted());
    SiggenConfig c2 = sh_cfg(SIGGEN_IMPULSE, 1, 1);
    CHECK(siggen_stage_config(&c2, sizeof c2));
    CHECK(!siggen_wants_output() && n_drop9 == 1 && drop9_res == ALINK_APPLY_CANCELLED);
    CHECK(sh_timeline().result == TB_RES_CANCELLED_CONFIG);
    run_tb(TAIL_US + 2 * PASS_US);
    CHECK(muted() && !sd.override && dac_hw_mute_latched());
    tb_T = 0;

    // D. Refused at T (here: the time base not LOCKED or the unit muted, the
    //    transport's `ready`): not started, the DAC handed back the same way.
    CHECK(setup_tb(300) == TB_OK);
    run_tb(300000u - 1000u);
    CHECK(siggen_wants_output() && !muted() && !tb_fired);
    tb_fired = true;                                // the transport fires it itself
    CHECK(siggen_tb_fire(TB_ID_LINE, tb_T, false, false) == ALINK_APPLY_REFUSED && !siggen_running);
    CHECK(!siggen_wants_output() && sh_timeline().result == TB_RES_REFUSED_NOT_READY);
    run_tb(TAIL_US - 2 * PASS_US);
    CHECK(!muted());                                // the tail first
    run_tb(4 * PASS_US);
    CHECK(muted() && !sd.override);

    // E. Arming with a stale config (a STOP since the SET_CONFIG): refused at
    //    T - arm_lead, the DAC never opens.
    CHECK(setup_tb(300) == TB_OK);
    siggen_control(SIGGEN_CTL_STOP);                // a host stop: cancels it (CANCELLED_HOST)...
    CHECK(sh_timeline().result == TB_RES_CANCELLED_HOST && n_drop9 == 1);
    CHECK(siggen_tb_accept(TB_ID_LINE, tb_now() + 300u * 48u, tb_now(), 48000u) == TB_OK);   // ... a new op 9 on it
    tb_T = 0;
    edges = pin_edges;
    run_tb(300000u);
    CHECK(sh_timeline().result == TB_RES_REFUSED_NOT_READY && n_drop9 == 2 && drop9_res == ALINK_APPLY_REFUSED);
    CHECK(muted() && pin_edges == edges);
}

int main(void) {
    // 1. USB mode 0 (today's K340): the pin is open at idle; START plays at
    //    once and the generator never moves the pin.
    setup();
    CHECK(!muted());
    start();
    CHECK(playing && started_us == stub_time_us);
    run_us(2000000u);
    CHECK(!muted() && pin_edges == 0);
    want = false;
    run_us(100000u);
    CHECK(!muted() && pin_edges == 0 && sd.st == SIGGEN_DAC_IDLE && !sd.override);

    // 2. USB phase mode out of step, no host: an idle hold mutes the DAC.  The
    //    generator opens it itself, starts only after the DAC's un-mute ramp,
    //    keeps it open for the whole run (gaps included) and re-mutes after
    //    its tail because the idle hold is still in force.
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);                         // the hold is long elapsed
    CHECK(muted() && dac_hw_mute_latched() && !dac_hw_mute_bracket_active());
    uint64_t t_start = stub_time_us;
    start();
    CHECK(!playing);                         // muted: START waits (status FADE_IN)
    pass();
    CHECK(!muted());                         // opened on the first pass
    CHECK(unmuted_us == stub_time_us);
    CHECK(dac_hw_mute_latched());            // the idle hold itself is untouched
    run_us(HOLD_MS * 1000u - 2 * PASS_US);
    CHECK(!playing);                         // still inside the un-mute ramp
    run_us(3 * PASS_US);
    CHECK(playing);
    CHECK(started_us - unmuted_us >= HOLD_MS * 1000u && started_us - unmuted_us <= HOLD_MS * 1000u + 2 * PASS_US);
    CHECK(started_us > t_start);
    int edges = pin_edges;
    run_us(5000000u);                        // a 5 s walk
    CHECK(!muted() && pin_edges == edges);
    want = false;                            // the engine went idle (completed)
    run_us(TAIL_US - PASS_US);
    CHECK(!muted());                         // the tail is still on the wire
    run_us(2 * PASS_US);
    CHECK(muted());                          // previous behaviour restored
    CHECK(sd.st == SIGGEN_DAC_IDLE && !sd.override && dac_hw_mute_latched());
    // ... and the restored hold is still the owner's to end (alink_usb releases it).
    dac_hw_mute_release();
    run_us(REL_MS * 1000u + PASS_US);
    CHECK(!muted());

    // 3. The idle hold ends while the generator plays (a host stream anchored):
    //    nothing to restore, the pin stays open after the tail.
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);
    start();
    run_us(300000u);
    CHECK(playing && !muted());
    edges = pin_edges;
    dac_hw_mute_release();                   // alink_usb dac_hold_update: in step
    CHECK(!muted() && pin_edges == edges);   // no release dwell on an open pin
    run_us(REL_MS * 1000u + PASS_US);
    CHECK(!muted() && pin_edges == edges);
    want = false;
    run_us(TAIL_US + PASS_US);
    CHECK(!muted() && !dac_hw_mute_latched());

    // 4. A reset bracket while the generator holds an idle hold open: the
    //    bracket wins at once and gets a FULL hold (the DAC was playing), so
    //    no clock stop happens under an open DAC; its completion releases.
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);
    start();
    run_us(300000u);
    CHECK(playing && !muted());
    dac_hw_mute_assert();                    // pipeline_reset_ready() stage 2
    CHECK(muted() && dac_hw_mute_bracket_active());
    CHECK(!dac_hw_mute_hold_elapsed());      // re-armed
    run_us(HOLD_MS * 1000u - PASS_US);
    CHECK(!dac_hw_mute_hold_elapsed() && muted());
    run_us(2 * PASS_US);
    CHECK(dac_hw_mute_hold_elapsed() && muted());
    dac_hw_mute_release();                   // complete_pipeline_reset() phase 4
    CHECK(muted());                          // release dwell
    run_us(REL_MS * 1000u + PASS_US);
    CHECK(!muted());
    // The phase anchor keeps the mute after a reset (alink_usb_fw_keep_dac_mute
    // marks it idle at the clock restart): the generator may open it again, but
    // only after the restart's release dwell (review fwsiggen F2).
    dac_hw_mute_assert();
    run_us(HOLD_MS * 1000u + PASS_US);
    dac_hw_mute_mark_idle();                 // clocks restarted now
    uint64_t t_restart = stub_time_us;
    pass();
    CHECK(muted() && dac_hw_mute_bracket_active());
    run_us(REL_MS * 1000u - 2 * PASS_US);
    CHECK(muted());                          // still inside the dwell
    while (muted() && stub_time_us - t_restart < 3 * REL_MS * 1000u) pass();
    CHECK(!muted());
    CHECK(stub_time_us - t_restart >= REL_MS * 1000u && stub_time_us - t_restart <= REL_MS * 1000u + 2 * PASS_US);
    // Without the generator the same restart stays muted (the idle hold).
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);
    dac_hw_mute_assert();
    run_us(HOLD_MS * 1000u + PASS_US);
    dac_hw_mute_mark_idle();
    run_us(3 * REL_MS * 1000u);
    CHECK(muted() && dac_hw_mute_latched());

    // 4b. The generator's tail ends (override off re-mutes the idle hold); a
    //     bracket 5 ms later must wait a full hold from the re-mute, not the
    //     long-elapsed hold of the original assert (review fwsiggen F3).
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);
    start();
    run_us(500000u);
    CHECK(playing && !muted());
    want = false;
    while (!muted() && sd.st != SIGGEN_DAC_IDLE) pass();
    CHECK(muted() && sd.st == SIGGEN_DAC_IDLE);
    uint64_t t_remute = stub_time_us;
    run_us(5000u);
    dac_hw_mute_assert();                    // anchor prepare / pipeline_reset_ready
    CHECK(!dac_hw_mute_hold_elapsed());
    while (!dac_hw_mute_hold_elapsed() && stub_time_us - t_remute < 3 * HOLD_MS * 1000u) pass();
    CHECK(stub_time_us - t_remute >= HOLD_MS * 1000u && stub_time_us - t_remute <= HOLD_MS * 1000u + 2 * PASS_US);
    // ... and STOP while START waits re-arms it the same way.
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);
    start();
    run_us(50000u);
    CHECK(!muted());
    want = false;
    pass();
    CHECK(muted());
    run_us(5000u);
    dac_hw_mute_assert();
    CHECK(!dac_hw_mute_hold_elapsed());

    // 5. A bracket in force at START: the generator does not override it and
    //    starts only after the bracket released and the DAC settled.
    setup();
    dac_hw_mute_assert();
    start();
    run_us(500000u);
    CHECK(!playing && muted());
    dac_hw_mute_release();
    run_us(REL_MS * 1000u);                  // release dwell
    CHECK(!playing);
    run_us(HOLD_MS * 1000u + 3 * PASS_US);
    CHECK(playing && !muted());

    // 6. STOP while START waits: nothing played, the override goes at once and
    //    the idle hold keeps the pin muted.
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);
    start();
    run_us(50000u);
    CHECK(!playing && !muted());
    want = false;
    pass();
    CHECK(muted() && sd.st == SIGGEN_DAC_IDLE && !sd.override);

    // 7. Feature off: the generator never waits and no pin moves.
    setup();
    dac_hw_mute_init(NULL);
    start();
    CHECK(playing);
    run_us(100000u);
    CHECK(pin_edges == 0);

    // 8. A diagnostic test pulse is never overridden.
    setup();
    dac_hw_mute_assert_idle();
    run_us(200000u);
    start();
    run_us(HOLD_MS * 1000u + 1000u);
    CHECK(playing && !muted());
    (void)dac_hw_mute_test_start();
    CHECK(muted());
    run_us(1000000u + 1000u);
    CHECK(!muted());

    test_fwtb1();

    printf("test_siggen_dac: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
