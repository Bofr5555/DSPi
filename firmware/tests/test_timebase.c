// Host test for WP FWTB1's time-base contract plane (timebase.h, timebase.c;
// alink_protocol_spec.md 11, extension_channel_spec.md 2.11): the contract
// state of every line and USB-direct state (11.2), the disturbance counter
// (C4: every listed event exactly once), event 0xE4 once per transition with
// the 250 ms merge per time base, the EXT_TIME_BASE layout (2.11.1's example
// byte for byte, followers first), the atomic read's invariant over 1000
// reads in a locked simulation (R7c), and the generator's side of a loss on
// the line (R3: the stop of a timed run, the cancel of a pending op 9, the
// discontinuities of a run).  Built by run.sh.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "siggen_host.h"
#include "timebase.c"

static int failures, checks;
#define CHECK(c, ...)                                                                                                \
    do {                                                                                                             \
        checks++;                                                                                                    \
        if (!(c)) {                                                                                                  \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);                                                     \
            fprintf(stderr, __VA_ARGS__);                                                                            \
            fprintf(stderr, "\n");                                                                                   \
            failures++;                                                                                              \
        }                                                                                                            \
    } while (0)

// ---- timebase.c's port: a slot-0 ring and the events -------------------------
static uint32_t sim_wr, sim_played, sim_us;
static bool sim_ring = true;
static int n_ev;
static struct { uint8_t tb, st, rs; uint32_t t; } ev[64];
uint32_t timebase_port_irq_save(void) { return 0; }
void timebase_port_irq_restore(uint32_t s) { (void)s; }
uint32_t timebase_port_now_us(void) { return sim_us; }
bool timebase_port_ring(uint32_t *wr, uint32_t *played) {
    *wr = sim_wr;
    *played = sim_played;
    return sim_ring;
}
void timebase_port_event(uint8_t tb, uint8_t st, uint8_t rs, uint32_t t) {
    if (n_ev < 64) { ev[n_ev].tb = tb; ev[n_ev].st = st; ev[n_ev].rs = rs; ev[n_ev].t = t; }
    n_ev++;
}

// ---- a line follower as alink_input.c's provider -----------------------------
static uint8_t ln_alink = 2, ln_phase = 2, ln_sender = 0x42;
static bool ln_mode1 = true, ln_active = true, ln_started = true, ln_present = true;
static uint64_t ln_next_w = 1000000u;
static int32_t ln_d = 128;
static tb_track_t ln_track;
static int n_ln_drop;
static uint8_t ln_state(void) {
    return ln_present ? tb_line_state(ln_alink, ln_mode1, ln_active, ln_phase, ln_started) : TB_ST_NONE;
}
static bool ln_is_present(void) { return ln_present; }
static void ln_read(tb_entry_t *e, bool ring, uint32_t wr, uint32_t played) {
    e->domain = ln_sender;
    e->d = (uint16_t)ln_d;
    e->phase_error = 0.25f;
    e->t = ring ? tb_t_at_output(ln_next_w, wr, played, ln_d) : 0;
}
static bool ln_next(uint64_t *t) { *t = ln_next_w; return true; }
static void ln_drop(uint8_t r) { (void)r; n_ln_drop++; }
static const tb_provider_t ln_prov = { TB_ID_LINE, TB_ROLE_FOLLOWER, &ln_track, ln_state, ln_is_present,
                                       ln_read, ln_next, ln_drop };
static void ln_note(uint8_t reason) { (void)tb_track_note(&ln_track, ln_state(), reason, (uint32_t)ln_next_w); }

// ---- the example's two entries: a USB follower and an S/PDIF source ----------
static tb_track_t ex_usb_track, ex_sp_track;
static bool ex_on;
static uint8_t ex_usb_state(void) { return ex_on ? TB_ST_LOCKED : TB_ST_NONE; }
static bool ex_present(void) { return ex_on; }
static void ex_usb_read(tb_entry_t *e, bool ring, uint32_t wr, uint32_t played) {
    (void)ring; (void)wr; (void)played;
    e->domain = 0;
    e->t = 80000000u;
    e->d = 192;
    e->phase_error = 0.125f;
}
static void ex_sp_read(tb_entry_t *e, bool ring, uint32_t wr, uint32_t played) {
    (void)ring; (void)wr; (void)played;
    e->domain = 0x033742u;
    e->t = 960000u;
    e->d = 0;
    e->phase_error = 0.0f;
}
static const tb_provider_t ex_usb = { TB_ID_USB, TB_ROLE_FOLLOWER, &ex_usb_track, ex_usb_state, ex_present,
                                      ex_usb_read, NULL, NULL };
static const tb_provider_t ex_sp = { TB_ID_SPDIF, TB_ROLE_SOURCE, &ex_sp_track, ex_usb_state, ex_present,
                                     ex_sp_read, NULL, NULL };

// ---------------------------------------------------------------------------

static void test_states(void) {
    // The line (11.2): EXT_ALINK_STATUS.state / EXT_ALINK_PHASE.state.
    CHECK(tb_line_state(0, true, true, 0, false) == TB_ST_NONE, "line off: NONE");
    CHECK(tb_line_state(2, false, true, 2, true) == TB_ST_NONE, "line mode 0 (fill servo): NONE");
    CHECK(tb_line_state(1, false, true, 0, false) == TB_ST_NONE, "line mode 0 searching: NONE");
    CHECK(tb_line_state(1, true, true, 0, false) == TB_ST_ACQUIRING, "line 1: ACQUIRING");
    CHECK(tb_line_state(2, true, true, 1, true) == TB_ST_ANCHORING, "line 2/1: ANCHORING");
    CHECK(tb_line_state(2, true, true, 0, false) == TB_ST_ANCHORING, "line 2, draining: ANCHORING");
    CHECK(tb_line_state(2, true, true, 2, true) == TB_ST_LOCKED, "line 2/2: LOCKED");
    CHECK(tb_line_state(2, true, true, 2, false) == TB_ST_ANCHORING, "line 2/2 re-anchoring (not started): ANCHORING");
    CHECK(tb_line_state(3, true, true, 2, true) == TB_ST_HOLDOVER, "line 3: HOLDOVER");
    CHECK(tb_line_state(2, true, true, 3, true) == TB_ST_LOCK_ERROR, "line 2/3 pending: LOCK_ERROR");
    CHECK(tb_line_state(2, true, false, 0, false) == TB_ST_LOCK_ERROR, "line mode 1, D out of range: LOCK_ERROR");
    // USB-direct (11.2): EXT_ALINK_USB_STATUS.state.
    for (uint8_t st = 0; st <= 6; st++) {
        uint8_t want[7] = { TB_ST_NONE, TB_ST_NONE, TB_ST_ACQUIRING, TB_ST_NONE, TB_ST_ANCHORING, TB_ST_LOCKED,
                            TB_ST_LOCK_ERROR };
        CHECK(tb_usb_state(st, st == 5 ? 2 : 1, st == 5, false, false) == want[st], "USB state %u", st);
    }
    CHECK(tb_usb_state(5, 2, true, true, false) == TB_ST_HOLDOVER, "USB 5 + HOLDOVER flag: HOLDOVER");
    CHECK(tb_usb_state(3, 0, false, false, true) == TB_ST_LOCK_ERROR, "USB 3 CONFIG + LOCK_ERROR flag");
    CHECK(tb_usb_state(5, 3, true, false, true) == TB_ST_LOCK_ERROR, "USB 5 pending: LOCK_ERROR");
    CHECK(tb_usb_state(5, 1, true, false, false) == TB_ST_ANCHORING, "USB 5 verifying (soft re-anchor): ANCHORING");
}

// C4: every listed event exactly once; a transition with no reason gets one.
static void test_disturbances(void) {
    tb_track_t k;
    tb_track_init(&k);
    CHECK(!tb_track_note(&k, TB_ST_ACQUIRING, TB_RS_NONE, 1) && k.dist == 0, "NONE -> ACQUIRING: no disturbance");
    CHECK(!tb_track_note(&k, TB_ST_ANCHORING, TB_RS_NONE, 2) && k.dist == 0, "-> ANCHORING");
    CHECK(!tb_track_note(&k, TB_ST_LOCKED, TB_RS_NONE, 3) && k.last_reason == TB_RS_LOCKED, "-> LOCKED: reason LOCKED");
    CHECK(tb_track_note(&k, TB_ST_LOCKED, TB_RS_CORRECTION, 4) && k.dist == 1, "a correction: 1");
    CHECK(tb_track_note(&k, TB_ST_HOLDOVER, TB_RS_NONE, 5) && k.dist == 2 && k.last_reason == TB_RS_HOLDOVER,
          "a holdover that leaves LOCKED counts once (2)");
    CHECK(k.lost, "leaving LOCKED is a loss for the generator");
    k.lost = false;
    CHECK(tb_track_note(&k, TB_ST_LOCKED, TB_RS_NONE, 6) && k.dist == 3 && k.last_reason == TB_RS_RESUMED,
          "a resume counts (3), reason RESUMED");
    CHECK(tb_track_note(&k, TB_ST_HOLDOVER, TB_RS_NONE, 7) && k.dist == 4, "holdover again (4)");
    CHECK(!tb_track_note(&k, TB_ST_ANCHORING, TB_RS_NONE, 8) && k.dist == 4, "relock placed by F: anchoring");
    CHECK(tb_track_note(&k, TB_ST_LOCKED, TB_RS_NONE, 9) && k.dist == 5 && k.last_reason == TB_RS_RESUMED,
          "verified after a holdover: a resume (5)");
    CHECK(tb_track_note(&k, TB_ST_ANCHORING, TB_RS_REANCHOR, 10) && k.dist == 6, "a re-anchor leaving LOCKED: once (6)");
    CHECK(!tb_track_note(&k, TB_ST_LOCKED, TB_RS_NONE, 11) && k.dist == 6 && k.last_reason == TB_RS_LOCKED,
          "anchored again: LOCKED, not counted");
    CHECK(tb_track_note(&k, TB_ST_LOCKED, TB_RS_EPOCH, 12) && k.dist == 7, "an epoch change (7)");
    CHECK(tb_track_note(&k, TB_ST_ACQUIRING, TB_RS_NONE, 13) && k.dist == 8 && k.last_reason == TB_RS_LOST,
          "a loss (8), reason LOST");
    CHECK(!tb_track_note(&k, TB_ST_LOCK_ERROR, TB_RS_NONE, 14) && k.last_reason == TB_RS_LOCK_ERROR,
          "lock error from ACQUIRING: reason LOCK_ERROR, no disturbance");
    CHECK(!tb_track_note(&k, TB_ST_NONE, TB_RS_DISABLED, 15) && k.dist == 8, "disabled outside LOCKED: no disturbance");
    // A wrap of the u16 counter.
    k.dist = 0xFFFFu;
    k.state = TB_ST_LOCKED;
    CHECK(tb_track_note(&k, TB_ST_LOCKED, TB_RS_CORRECTION, 16) && k.dist == 0, "the counter wraps");
}

// 0xE4: once per transition, at most one per 250 ms per time base, the latest
// state and reason; time bases never merge.
static void test_events(void) {
    memset(&ln_track, 0, sizeof ln_track);
    tb_track_init(&ln_track);
    tb_track_init(&ex_usb_track);
    ex_on = false;
    timebase_register(TB_SLOT_LINE, &ln_prov);
    timebase_register(TB_SLOT_USB, &ex_usb);
    timebase_register(TB_SLOT_SPDIF, NULL);
    ln_present = true;
    ln_alink = 1; ln_phase = 0; ln_started = false;
    n_ev = 0;
    timebase_service(1000);
    CHECK(n_ev == 1 && ev[0].tb == TB_ID_LINE && ev[0].st == TB_ST_ACQUIRING, "ACQUIRING sent (%d)", n_ev);
    timebase_service(1001);
    CHECK(n_ev == 1, "no transition: nothing");
    ln_alink = 2; ln_phase = 1; ln_started = true;
    timebase_service(1100);                         // ANCHORING inside the window: held
    CHECK(n_ev == 1, "held by the 250 ms limit");
    ln_phase = 2;
    timebase_service(1200);                         // LOCKED, still inside: merged
    CHECK(n_ev == 1, "still held");
    ex_on = true;
    timebase_service(1210);                         // another time base: its own window
    CHECK(n_ev == 2 && ev[1].tb == TB_ID_USB && ev[1].st == TB_ST_LOCKED && ev[1].rs == TB_RS_LOCKED,
          "the USB time base is not merged with the line's");
    timebase_service(1250);
    CHECK(n_ev == 3 && ev[2].tb == TB_ID_LINE && ev[2].st == TB_ST_LOCKED && ev[2].rs == TB_RS_LOCKED,
          "one event for the collapsed burst: latest state and reason");
    ln_note(TB_RS_CORRECTION);
    timebase_service(1300);
    CHECK(n_ev == 3, "a correction 50 ms later: held");
    timebase_service(1500);
    CHECK(n_ev == 4 && ev[3].st == TB_ST_LOCKED && ev[3].rs == TB_RS_CORRECTION,
          "a disturbance with no state change is an event (CORRECTION)");
    timebase_service(2000);
    CHECK(n_ev == 4, "nothing pending: nothing");
    ex_on = false;
    timebase_register(TB_SLOT_USB, NULL);
}

// 2.11.1's example, byte for byte: a USB-direct follower (locked, D 192, phase
// error 0.125, one disturbance) that is also an S/PDIF source (epoch 3, source
// 0x3742, last reason EPOCH, four disturbances), time_us 100 000 000, output
// frames 4 800 000.  Followers first, whatever the slot order.
static void test_example(void) {
    timebase_register(TB_SLOT_LINE, NULL);
    timebase_register(TB_SLOT_USB, &ex_sp);     // the source in the first slot ...
    timebase_register(TB_SLOT_SPDIF, &ex_usb);  // ... the follower after it
    tb_track_init(&ex_usb_track);
    tb_track_init(&ex_sp_track);
    ex_on = true;
    (void)tb_track_note(&ex_usb_track, TB_ST_LOCKED, TB_RS_NONE, 0);
    ex_usb_track.dist = 1;
    (void)tb_track_note(&ex_sp_track, TB_ST_LOCKED, TB_RS_EPOCH, 0);
    ex_sp_track.dist = 4;
    sim_us = 100000000u;
    sim_played = 4800000u;
    uint8_t o[TB_READ_LEN];
    timebase_read(o);
    static const uint8_t want[64] = {
        0x01, 0x02, 0x00, 0x00, 0x00, 0xe1, 0xf5, 0x05, 0x00, 0x3e, 0x49, 0x00,
        0x01, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0xb4, 0xc4, 0x04, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x3e,
        0x03, 0x01, 0x03, 0x0b, 0x42, 0x37, 0x03, 0x03, 0x00, 0xa6, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
    };
    int bad = -1;
    for (int i = 0; i < 64 && bad < 0; i++)
        if (o[i] != want[i]) bad = i;
    CHECK(bad < 0, "2.11.1 example: first difference at byte %d (%02x, want %02x)", bad, bad >= 0 ? o[bad] : 0,
          bad >= 0 ? want[bad] : 0);
    ex_on = false;
    timebase_read(o);
    CHECK(o[0] == 1 && o[1] == 0 && !memcmp(o + 12, (uint8_t[52]){ 0 }, 52), "no entries: count 0, entries zero");
    timebase_register(TB_SLOT_USB, NULL);
    timebase_register(TB_SLOT_SPDIF, NULL);
}

// R7c: t - output_frames identical over 1000 reads in a locked simulation:
// blocks of random size written (next_w and wr together), the DAC consuming
// in between, reads at random moments.  A correction moves it, and counts.
static void test_invariant(void) {
    timebase_register(TB_SLOT_LINE, &ln_prov);
    tb_track_init(&ln_track);
    ln_present = true;
    ln_alink = 2; ln_phase = 2; ln_started = true; ln_mode1 = true; ln_active = true;
    ln_d = 128;
    timebase_service(5000);
    CHECK(ln_track.state == TB_ST_LOCKED, "locked");
    ln_next_w = (1ull << 40) + 12345u;
    sim_wr = 4000000000u;                           // wraps during the run
    sim_played = sim_wr - 100u;
    uint32_t rng = 777u;
    uint32_t inv = 0;
    bool same = true;
    uint16_t d0 = ln_track.dist;
    for (int r = 0; r < 1000; r++) {
        for (int k = (int)((rng >> 8) % 4u); k >= 0; k--) {
            rng = rng * 1103515245u + 12345u;
            uint32_t n = 1u + (rng >> 9) % 49u;
            ln_next_w += n;                         // process_input_block(n): rows of F next_w..
            sim_wr += n;
            rng = rng * 1103515245u + 12345u;
            uint32_t c = (rng >> 9) % (sim_wr - sim_played + 1u);
            sim_played += c;                        // the DAC played some
        }
        uint8_t o[TB_READ_LEN];
        timebase_read(o);
        uint32_t of = (uint32_t)o[8] | (uint32_t)o[9] << 8 | (uint32_t)o[10] << 16 | (uint32_t)o[11] << 24;
        uint32_t t = (uint32_t)o[20] | (uint32_t)o[21] << 8 | (uint32_t)o[22] << 16 | (uint32_t)o[23] << 24;
        if (r == 0) inv = t - of;
        else if (t - of != inv) same = false;
        uint16_t dist = (uint16_t)(o[28] | o[29] << 8);
        if (dist != d0) same = false;
    }
    CHECK(same, "t - output_frames identical over 1000 reads, disturbances unchanged");
    // A whole-frame correction (rows dropped: next_w moves without a write).
    ln_next_w += 3u;
    ln_note(TB_RS_CORRECTION);
    uint8_t o[TB_READ_LEN];
    timebase_read(o);
    uint32_t of = (uint32_t)o[8] | (uint32_t)o[9] << 8 | (uint32_t)o[10] << 16 | (uint32_t)o[11] << 24;
    uint32_t t = (uint32_t)o[20] | (uint32_t)o[21] << 8 | (uint32_t)o[22] << 16 | (uint32_t)o[23] << 24;
    CHECK(t - of == inv + 3u && (uint16_t)(o[28] | o[29] << 8) == (uint16_t)(d0 + 1u),
          "a correction moves the invariant and counts once");
    CHECK(o[12] == TB_ID_LINE && o[13] == TB_ROLE_FOLLOWER && o[14] == TB_ST_LOCKED && o[16] == 0x42 && o[19] == 0 &&
          o[30] == 128 && o[31] == 0, "line entry: time base 0, follower, LOCKED, domain = sender, d");
}

// R3 / R4 on the line, through the aggregator: a holdover stops a run op 9
// started (STOPPED_LOSS) and cancels a pending op 9 (CANCELLED_LOSS); a plain
// START plays on; discontinuities count an underrun, a re-anchor and a
// holdover entered while running.
static void line_lock(uint32_t ms) {
    ln_alink = 2; ln_phase = 2; ln_started = true;
    timebase_service(ms);
}

static void test_line_loss(void) {
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) matrix_mixer.outputs[o].enabled = 1;
    timebase_register(TB_SLOT_LINE, &ln_prov);
    tb_track_init(&ln_track);
    line_lock(10000);
    SiggenConfig c = sh_cfg(SIGGEN_SWEEP_LOG, 3, 1);
    CHECK(siggen_stage_config(&c, sizeof c), "stage");
    uint64_t T = ln_next_w + 9600u;
    CHECK(siggen_tb_accept(TB_ID_LINE, T, ln_next_w, 48000u) == TB_OK, "op 9 accepted");
    siggen_tb_poll(TB_ID_LINE, T - 7200u, 48000u);
    siggen_output_gate(true);
    CHECK(siggen_tb_fire(TB_ID_LINE, T, false, ln_track.state == TB_ST_LOCKED) == ALINK_APPLY_APPLIED && siggen_running,
          "started on T");
    float buf[NUM_OUTPUT_CHANNELS][AUDIO_BUFFER_SAMPLES];
    siggen_render(buf, 48, 48000u);
    // The line enters holdover (a loss while phase-locked).
    ln_alink = 3;
    timebase_service(10100);
    sh_timeline_t tl = sh_timeline();
    CHECK(!siggen_running && tl.result == TB_RES_STOPPED_LOSS && tl.disc == 1 && tl.samples == 48,
          "line holdover: STOPPED_LOSS (disc %u, samples %u)", tl.disc, tl.samples);
    // A pending op 9 on the line, then a holdover: CANCELLED_LOSS, dropped.
    line_lock(10200);
    CHECK(siggen_stage_config(&c, sizeof c), "restage");
    n_ln_drop = 0;
    CHECK(siggen_tb_accept(TB_ID_LINE, ln_next_w + 48000u, ln_next_w, 48000u) == TB_OK, "op 9 pending");
    ln_alink = 3;
    timebase_service(10300);
    CHECK(sh_timeline().result == TB_RES_CANCELLED_LOSS && n_ln_drop == 1 && sh_timeline().state == TB_TL_IDLE,
          "a holdover cancels a pending op 9 (CANCELLED_LOSS), the line drops it");
    // ... and the loss cleared the fresh flag: an op 9 on the same config is
    // refused at arming.
    line_lock(10400);
    CHECK(siggen_tb_accept(TB_ID_LINE, ln_next_w + 9600u, ln_next_w, 48000u) == TB_OK, "op 9 again");
    siggen_tb_poll(TB_ID_LINE, ln_next_w + 2400u, 48000u);
    CHECK(sh_timeline().result == TB_RES_REFUSED_NOT_READY && n_ln_drop == 2, "stale after a loss: refused at arming");
    // A plain START plays on through the loss, its discontinuities counted.
    line_lock(10500);
    CHECK(siggen_stage_config(&c, sizeof c) && siggen_control(SIGGEN_CTL_START) && siggen_running, "plain START");
    tl = sh_timeline();
    CHECK(tl.tb == TB_ID_LINE && tl.start == ln_next_w && tl.disc == 0, "stamped on the line's T");
    sh_slips++;                                     // an output underrun's re-anchor
    siggen_service();
    CHECK(sh_timeline().disc == 1, "an underrun: 1");
    ln_started = false;                             // reanchor(): not started, then the event
    ln_note(TB_RS_REANCHOR);
    CHECK(ln_track.dist == 1u + ln_track.dist_seen, "a re-anchor leaving LOCKED counts once");
    timebase_service(10600);
    CHECK(sh_timeline().disc == 2 && siggen_running, "a re-anchor: 2, still playing");
    line_lock(10700);
    ln_alink = 3;
    timebase_service(10800);
    CHECK(sh_timeline().disc == 3 && siggen_running, "a holdover while running: 3, still playing (plain START)");
    siggen_tb_pipeline_reset();
    CHECK(sh_timeline().disc == 4, "a pipeline reset: 4");
    siggen_control(SIGGEN_CTL_STOP_NOW);
    siggen_service();
    CHECK(sh_timeline().disc == 4 && !siggen_running, "stopped; the record stays");
    timebase_register(TB_SLOT_LINE, NULL);
}

// R4: op 9 is refused NOT_READY at arming unless the staged config is
// finite and written since the last of boot, STOP, STOP_NOW, op 10,
// CANCEL_ALL, a preset load and a loss.  First in main(): nothing staged yet.
static int arm_case(void) {
    // A pending op 9 750 ms ahead, then the arming poll: 1 armed, 0 refused.
    if (siggen_tb_accept(TB_ID_LINE, 1000000u + 36000u, 1000000u, 48000u) != TB_OK) return -1;
    siggen_tb_poll(TB_ID_LINE, 1000000u + 36000u - 7200u, 48000u);
    bool armed = siggen_wants_output();
    if (armed) siggen_tb_cancel_all();
    return armed ? 1 : 0;
}

static void test_preconditions(void) {
    timebase_register(TB_SLOT_LINE, NULL);
    CHECK(arm_case() == 0 && sh_timeline().result == TB_RES_REFUSED_NOT_READY, "since boot: nothing staged, refused");
    SiggenConfig fin = sh_cfg(SIGGEN_SWEEP_LOG, 2, 1);
    SiggenConfig sine = sh_cfg(SIGGEN_SINE, 1, 1);
    sine.duration_ms = 1000;
    SiggenConfig raw = fin;
    raw.flags = SIGGEN_FLAG_RAW;
    SiggenConfig forever = fin;
    forever.repeat = 0;
    SiggenConfig walk = sh_cfg(SIGGEN_PINK, 2, 1);
    walk.flags = SIGGEN_FLAG_WALK;
    walk.channel_mask = 0x3;
    SiggenConfig imp = sh_cfg(SIGGEN_IMPULSE, 4, 1);
    CHECK(siggen_stage_config(&fin, sizeof fin) && arm_case() == 1, "a fresh finite sweep arms");
    // (the CANCEL_ALL in arm_case cleared it: staged again before each case)
    CHECK(siggen_stage_config(&imp, sizeof imp) && arm_case() == 1, "a fresh finite pattern arms");
    CHECK(siggen_stage_config(&sine, sizeof sine) && arm_case() == 0, "a continuous type (even time-limited): refused");
    CHECK(siggen_stage_config(&walk, sizeof walk) && arm_case() == 0, "a walked continuous type: refused");
    CHECK(siggen_stage_config(&raw, sizeof raw) && arm_case() == 0, "RAW: refused");
    CHECK(siggen_stage_config(&forever, sizeof forever) && arm_case() == 0, "repeat 0 (infinite): refused");
    CHECK(siggen_stage_config(&fin, sizeof fin), "stage");
    siggen_control(SIGGEN_CTL_STOP);
    CHECK(arm_case() == 0, "after STOP: refused");
    CHECK(siggen_stage_config(&fin, sizeof fin), "stage");
    siggen_control(SIGGEN_CTL_STOP_NOW);
    CHECK(arm_case() == 0, "after STOP_NOW: refused");
    CHECK(siggen_stage_config(&fin, sizeof fin), "stage");
    siggen_tb_stop_op(0);
    CHECK(arm_case() == 0, "after an applied op 10: refused");
    CHECK(siggen_stage_config(&fin, sizeof fin), "stage");
    siggen_tb_cancel_all();
    CHECK(arm_case() == 0, "after CANCEL_ALL: refused");
    CHECK(siggen_stage_config(&fin, sizeof fin), "stage");
    siggen_stop_immediate(SIGGEN_STOP_PRESET);
    CHECK(arm_case() == 0, "after a preset load: refused");
    CHECK(siggen_stage_config(&fin, sizeof fin), "stage");
    siggen_tb_loss(TB_ID_USB, 1);
    CHECK(arm_case() == 0, "after a loss of any time base: refused");
    CHECK(siggen_stage_config(&fin, sizeof fin) && arm_case() == 1, "staged again: arms");
}

int main(void) {
    test_preconditions();
    test_states();
    test_disturbances();
    test_events();
    test_example();
    test_invariant();
    test_line_loss();
    printf("test_timebase: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
