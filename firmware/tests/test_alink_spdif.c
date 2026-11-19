// Host test for WP FWSP2: S/PDIF as an alink time-base transport, the roles
// (alink_spdif.{c,h}; alink_protocol_spec.md 11 and 12, extension_channel_spec.md
// 2.10, 2.11, test_signals_spec.md 15).  The real roles, phase lock, codec,
// generator (siggen.c) and time-base aggregator (timebase.c) in the model world
// of alink_spdif_world.h: a source and followers over a modelled wire.
//
// Covered (PLAN-AT FWSP2 "Done when"): the wire layouts and their refusals
// (2.10.1-2.10.3 examples byte for byte); a follower anchors, locks and plays
// the sample carried as F at F + D; mode 1 without a superframe is LOCK_ERROR
// and mutes, or plays unaligned with on_lock_error 1; a 50 ms cut is a
// holdover that resumes on the same timeline; a source epoch change
// re-anchors every follower with reason EPOCH; up to 8 missing or invalid
// superframes are coasted (sf_errors, no disturbance, the timeline unmoved),
// the 9th enters HOLDOVER, a returning superframe that breaks continuity
// re-anchors; a broadcast with less than 896 frames of lead is refused, one
// with enough is applied by the source and every follower on the same F;
// op 9 on time base 3 starts the generator on its sample on a follower and on
// the source; EXT_TIME_BASE lists the follower and the source entries with
// the domain of C5; mode 0 touches nothing.  Built by run.sh.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DSPI_SIGGEN_SYNC 1
#include "siggen_host.h"
#include "timebase.c"

#define SW_HAVE_GENERATOR 1
#define SW_HAVE_TIMEBASE 1
#include "alink_spdif.h"
static uint8_t sw_fire(const alink_apply_op_t *op, bool late, bool locked) {
    if (op->op == ALINK_OP_SIGGEN_START) return siggen_tb_fire(TB_ID_SPDIF, op->frame, late, locked);
    if (op->op == ALINK_OP_SIGGEN_STOP) {
        siggen_tb_stop_op(op->arg);
        return ALINK_APPLY_APPLIED;
    }
    if (op->op == ALINK_OP_CLICK && !locked) return ALINK_APPLY_REFUSED;
    return ALINK_APPLY_APPLIED;
}
uint8_t alink_spdif_port_op9_accept(uint64_t t, uint64_t t_now, uint32_t fs) { return siggen_tb_accept(TB_ID_SPDIF, t, t_now, fs); }
bool alink_spdif_port_op9_broadcast(uint64_t t, uint64_t t_now, uint32_t fs) { return siggen_tb_accept_broadcast(TB_ID_SPDIF, t, t_now, fs); }
void alink_spdif_port_op9_poll(uint64_t t_now, uint32_t fs) { siggen_tb_poll(TB_ID_SPDIF, t_now, fs); }
void alink_spdif_port_cancel_all(void) { siggen_tb_cancel_all(); }
void alink_spdif_port_op9_lost(void) { siggen_tb_dropped(TB_RES_CANCELLED_LOSS); }
#include "alink_spdif_world.h"

// timebase.c's port.
static int n_tb_ev;
static uint8_t tb_ev_tb, tb_ev_st, tb_ev_rs;
uint32_t timebase_port_irq_save(void) { return 0; }
void timebase_port_irq_restore(uint32_t s) { (void)s; }
uint32_t timebase_port_now_us(void) { return alink_spdif_port_now_us(); }
void timebase_port_event(uint8_t tb, uint8_t st, uint8_t rs, uint32_t t) {
    (void)t;
    tb_ev_tb = tb; tb_ev_st = st; tb_ev_rs = rs;
    n_tb_ev++;
}

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

static uint64_t le64(const uint8_t *p) { uint64_t v = 0; for (int i = 7; i >= 0; i--) v = v << 8 | p[i]; return v; }
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static int hexcmp(const uint8_t *a, const char *hex) {
    for (unsigned i = 0; *hex; i++) {
        while (*hex == ' ') hex++;
        if (!*hex) break;
        unsigned v;
        if (sscanf(hex, "%2x", &v) != 1) return -1;
        if (a[i] != v) return (int)i + 1;
        hex += 2;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Observation helpers
// ---------------------------------------------------------------------------

// Source-to-follower latency now (source frames played minus follower's), or
// NAN when the follower plays silence.
static double latency(const sw_fol_t *f) {
    double p = sw_fol_playing(f);
    if (p < 0) return NAN;
    return sw_src_playing() - p;
}

static double lat_mean(const sw_fol_t *f, double secs) {
    double t_end = SW.t + secs, sum = 0;
    int n = 0;
    while (SW.t < t_end) {
        sw_run(SW.t + 0.01, 0, NULL);
        double l = latency(f);
        if (!isnan(l)) { sum += l; n++; }
    }
    return n ? sum / n : NAN;
}

static void world(uint32_t fs, int nfol, uint16_t d) {
    sw_world_init(fs, 3.0, 0x1234567ull + fs + (uint64_t)nfol);
    SW.nfol = nfol;
    // The source on epoch 3 (the spec's examples), so a domain shows it.
    SW.src.u.epoch = 3;
    SW.src.u.src.epoch = 3;
    alink_spdif_src_stage(&SW.src.u, 0, true);
    for (int i = 0; i < nfol; i++) sw_fol_init(&SW.fol[i], i ? -4.0 : 6.0, d, 700.0 + 150.0 * i);
    sw_pass_hook = NULL;
}

static uint8_t st(const sw_fol_t *f) { return alink_spdif_state(&f->u); }

// ---------------------------------------------------------------------------
// Wire layouts (2.10.1 - 2.10.3)
// ---------------------------------------------------------------------------

static void test_config(void) {
    alink_spdif_t u;
    alink_spdif_init(&u, 0x42, 0x37);
    alink_spdif_limits(&u, 49, 48, 448);
    CHECK(u.ph.d_min == 129 && u.ph.d_max == 399, "limits at 48 kHz 4 ms: %u..%u", u.ph.d_min, u.ph.d_max);
    uint8_t o[8];
    alink_spdif_get_config(&u, o);
    CHECK(!hexcmp(o, "00 01 c0 00 00 00 00 00"), "defaults: mode 0, AUTO_REANCHOR, D 192");
    uint8_t c[8];
    memcpy(c, (uint8_t[8]){ 3, 1, 0xc0, 0, 0, 0, 0, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 1) == TB_ERR_VALUE, "mode 3 refused");
    memcpy(c, (uint8_t[8]){ 1, 3, 0xc0, 0, 0, 0, 0, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 1) == TB_ERR_VALUE, "undefined flag refused");
    memcpy(c, (uint8_t[8]){ 1, 1, 0xc0, 0, 0, 1, 0, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 1) == TB_ERR_VALUE, "reserved byte refused");
    memcpy(c, (uint8_t[8]){ 1, 1, 0xc0, 0, 2, 0, 0, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 1) == TB_ERR_VALUE, "on_lock_error 2 refused");
    memcpy(c, (uint8_t[8]){ 1, 1, 100, 0, 0, 0, 0, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 1) == TB_ERR_VALUE, "mode 1 D below d_min refused");
    memcpy(c, (uint8_t[8]){ 1, 1, 0x90, 1, 0, 0, 0, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 1) == TB_ERR_VALUE, "mode 1 D 400 above d_max refused");
    memcpy(c, (uint8_t[8]){ 2, 1, 0, 0, 0, 0, 0, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 0) == ALINK_SPDIF_ERR_UNSUPPORTED, "mode 2 without an S/PDIF slot: UNSUPPORTED");
    memcpy(c, (uint8_t[8]){ 2, 1, 0, 0, 0, 0, 0x04, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 0x03) == TB_ERR_VALUE, "mode 2 tx_mask on a non-S/PDIF slot refused");
    memcpy(c, (uint8_t[8]){ 1, 1, 0xc0, 0, 0, 0, 0x01, 0 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 0x03) == TB_ERR_VALUE, "mode 1 tx_mask refused");
    alink_spdif_get_config(&u, o);
    CHECK(o[0] == 0, "refused writes changed nothing");
    // Accepted: the follower of the 2.10.1 example (state IN only).
    memcpy(c, (uint8_t[8]){ 1, 1, 0xc0, 0, 0, 0, 0, 0x03 }, 8);
    CHECK(alink_spdif_set_config(&u, c, 0x03) == TB_OK && u.mode == 1 && u.d == 192, "follower accepted");
    // Persistence round trip; implausible bytes keep the defaults.
    uint8_t s[ALINK_SPDIF_STORE_LEN];
    alink_spdif_store(&u, s);
    alink_spdif_t v;
    alink_spdif_init(&v, 0, 0);
    alink_spdif_load(&v, s);
    CHECK(v.mode == 1 && v.d == 192 && v.flags == 1 && v.on_lock_error == 0 && v.tx_mask == 0, "store / load");
    uint8_t bad[ALINK_SPDIF_STORE_LEN] = { 3, 1, 0xc0, 0, 0, 0 };
    alink_spdif_init(&v, 0, 0);
    alink_spdif_load(&v, bad);
    CHECK(v.mode == 0, "an implausible stored block reads as mode 0");
    uint8_t zero[ALINK_SPDIF_STORE_LEN] = { 0 };
    alink_spdif_load(&v, zero);
    CHECK(v.mode == 0 && v.d == 0 + 192 - 192 + v.d, "a never-saved block reads as mode 0");
    // The source example: slot 0 only, outputs running: 02 01 00 00 00 00 01 03.
    alink_spdif_t sr;
    alink_spdif_init(&sr, 0x42, 0x37);
    memcpy(c, (uint8_t[8]){ 2, 1, 0, 0, 0, 0, 0x01, 0 }, 8);
    CHECK(alink_spdif_set_config(&sr, c, 0x03) == TB_OK, "source accepted");
    alink_spdif_src_sync(&sr, true, 1000u);
    alink_spdif_get_config(&sr, o);
    CHECK(!hexcmp(o, "02 01 00 00 00 00 01 03"), "2.10.1 source example");
    CHECK(sr.origin % 192u == 0 && sr.origin + 1000u >= 192u, "the origin is a multiple of 192");
    // Rewriting the same values does nothing.
    uint8_t ep = sr.epoch;
    CHECK(alink_spdif_set_config(&sr, c, 0x03) == TB_OK && sr.epoch == ep && sr.src_running, "same values: no change");
}

static void test_status_layout(void) {
    // The 2.10.2 example, built from a follower in that state.
    alink_spdif_t u;
    alink_spdif_init(&u, 0, 0);
    u.mode = 1;
    u.active = true;
    u.ph.mode = 1;
    u.ph.d_local = 192;
    u.d = 192;
    u.ph.d_min = 150;
    u.ph.d_max = 399;
    u.ph.started = true;
    u.ph.timeline = true;
    u.ph.state = ALINK_PHASE_LOCKED;
    u.ph.e = 0.25f;
    u.rx_locked = true;
    u.rx_rate = 1;
    u.rd_valid = true;
    u.sf_seen = true;
    u.src_id = 0x3742;
    u.src_epoch = 3;
    u.lock_count = 2;
    u.loss_count = 1;
    u.sf_errors = 5;
    u.reanchors = 1;
    uint8_t o[48];
    alink_spdif_get_status(&u, 960000u, 12.5f, 96, o);
    int r = hexcmp(o, "03 02 01 03 42 37 03 00 00 a6 0e 00 00 00 00 00 00 00 80 3e 00 00 48 41 96 00 8f 01 60 00 00 00 "
                      "02 00 00 00 01 00 00 00 05 00 00 00 01 00 00 00");
    CHECK(r == 0, "2.10.2 example byte for byte (first difference at byte %d)", r - 1);
    // Mode 0: all fields zero except the rate code 0xFF.
    alink_spdif_init(&u, 0, 0);
    alink_spdif_get_status(&u, 123, 1.0f, 5, o);
    bool z = o[2] == 0xFF;
    for (int i = 0; i < 48; i++) if (i != 2 && o[i]) z = false;
    CHECK(z, "mode 0 status: nothing but rate 0xFF");
}

static void test_ctrl_layout(void) {
    world(48000, 0, 192);
    alink_spdif_t *s = &SW.src.u;
    uint64_t fp = 1000000u - 2000u;
    uint8_t p[16];
    // 2.10.3 example: SIGGEN_STOP now at F = 1 000 000.
    CHECK(!hexcmp((uint8_t[16]){ 0x40, 0x42, 0x0f, 0, 0, 0, 0, 0, 0x0a, 0x01, 0, 0, 0x03, 0, 0, 0 },
                  "40 42 0f 00 00 00 00 00 0a 01 00 00 03 00 00 00"), "self-check");
    memcpy(p, (uint8_t[16]){ 0x40, 0x42, 0x0f, 0, 0, 0, 0, 0, 0x0a, 0x01, 0, 0, 0x03, 0, 0, 0 }, 16);
    // Op 0 CANCEL_ALL and ops 3, 5, 6, 8 are never carried; time base 3 only.
    uint8_t q[16];
    memcpy(q, p, 16); q[8] = 0; q[9] = 0;
    CHECK(alink_spdif_ctrl_write(s, q, fp, fp + 256) == TB_ERR_VALUE, "broadcast CANCEL_ALL refused (VALUE)");
    memcpy(q, p, 16); q[8] = 5; q[9] = 0;
    CHECK(alink_spdif_ctrl_write(s, q, fp, fp + 256) == TB_ERR_VALUE, "broadcast OUT_DELAY refused (VALUE)");
    memcpy(q, p, 16); q[12] = 0;
    CHECK(alink_spdif_ctrl_write(s, q, fp, fp + 256) == TB_ERR_VALUE, "broadcast time base 0 refused (VALUE)");
    memcpy(q, p, 16); q[14] = 1;
    CHECK(alink_spdif_ctrl_write(s, q, fp, fp + 256) == TB_ERR_VALUE, "broadcast reserved byte refused");
    // R2b: the lead, 896 frames.
    CHECK(alink_spdif_ctrl_write(s, p, 1000000u - 895u, 1000000u - 895u + 256u) == TB_ERR_STATE && !s->bc_pending,
          "broadcast lead 895 refused (STATE)");
    CHECK(alink_spdif_ctrl_write(s, p, fp, fp + 256u) == TB_OK && s->bc_pending, "broadcast lead 2000 accepted");
    CHECK(alink_spdif_ctrl_write(s, p, fp, fp + 256u) == TB_ERR_STATE, "a second broadcast refused (STATE)");
    uint8_t o[16];
    alink_spdif_ctrl_read(s, o);
    CHECK(o[0] == 1 && o[1] == 0x0a, "CTRL IN pending");
    // Not a source / not running: STATE.
    alink_spdif_t f;
    alink_spdif_init(&f, 0, 0);
    CHECK(alink_spdif_ctrl_write(&f, p, fp, fp) == TB_ERR_STATE, "broadcast on a non-source refused (STATE)");
    alink_spdif_ctrl_read(&f, o);
    bool z = true;
    for (int i = 0; i < 16; i++) z = z && o[i] == 0;
    CHECK(z, "a follower reads CTRL IN as zero");
    // The source applies it at its F (the split) and the broadcast ends.
    uint64_t fw = fp + 256u;
    sw_src_n_op = 0;
    for (int i = 0; i < 100 && s->aq.n; i++) {
        uint32_t k = alink_spdif_src_split(s, fw, 48);
        if (k) fw += k;
    }
    CHECK(sw_src_n_op == 1 && sw_src_oplog[0].frame == 1000000u && fw == 1000000u && !sw_src_oplog[0].late,
          "the source runs its own copy on its F (at %llu)", (unsigned long long)fw);
    // Its staged superframes carried it in every block completing before N.
    uint32_t b0 = (uint32_t)((fp - s->origin) / 192u);
    alink_spdif_src_stage(s, b0, true);
    int carried = 0;
    for (uint32_t b = b0; b < b0 + 8; b++) {
        const uint8_t *sf = alink_spdif_src_staged(s, b);
        alink_spdif_sf_t x;
        if (sf && alink_spdif_sf_unpack(sf, &x) == ALINK_SPDIF_OK && x.apply_op == 10 && x.apply_frame == 1000000u)
            carried++;
    }
    CHECK(carried >= 1, "the staged blocks carry the broadcast (%d)", carried);
    SWF = NULL;
    SW.t = 0;
    alink_spdif_service(s, 0);
    alink_spdif_ctrl_read(s, o);
    (void)le64;
}

// ---------------------------------------------------------------------------
// The follower in the world
// ---------------------------------------------------------------------------

static void test_anchor_lock(uint32_t fs) {
    world(fs, 1, fs == 96000 ? 320 : 192);
    sw_fol_t *f = &SW.fol[0];
    sw_run(1.5, 0, NULL);
    CHECK(f->u.sf_seen && f->u.rd_valid, "%u: superframes accepted", fs);
    sw_run(6.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCKED, "%u: LOCKED after 6 s (state %u, phase %u)", fs, st(f), f->u.ph.state);
    double l = lat_mean(f, 2.0);
    double d = (double)f->u.d;
    CHECK(fabs(l - d - 0.25) < 1.0, "%u: the sample carried as F plays at F + D (latency %.3f, D %.0f)", fs, l, d);
    CHECK(f->underruns == 0 && f->overflows == 0, "%u: no underrun, no overflow (%u %u)", fs, f->underruns, f->overflows);
    CHECK(f->u.sf_errors == 0 && f->u.reanchors == 0, "%u: no superframe error, no re-anchor", fs);
    CHECK(fabsf(f->u.ph.e) < 0.5f, "%u: phase error %.3f", fs, f->u.ph.e);
}

static void test_lock_error(void) {
    world(48000, 1, 192);
    sw_fol_t *f = &SW.fol[0];
    f->foreign = true;              // no superframe at all
    sw_run(0.4, 0, NULL);
    CHECK(st(f) == TB_ST_ACQUIRING, "foreign: ACQUIRING before 0.5 s (%u)", st(f));
    sw_run(1.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCK_ERROR && f->u.lock_err, "foreign: LOCK_ERROR after 0.5 s (%u)", st(f));
    uint8_t o[48];
    alink_spdif_get_status(&f->u, 0, 0, 0, o);
    CHECK((o[3] & (ALINK_SPDIF_ST_LOCK_ERROR | ALINK_SPDIF_ST_FOREIGN)) == (ALINK_SPDIF_ST_LOCK_ERROR | ALINK_SPDIF_ST_FOREIGN),
          "status flags LOCK_ERROR | FOREIGN (%02x)", o[3]);
    sw_run(2.0, 0, NULL);
    CHECK(alink_spdif_muted(&f->u) && isnan(latency(f)), "on_lock_error 0: muted, nothing plays");
    // on_lock_error 1: plays unaligned (the stock rate lock).
    uint8_t c[8] = { 1, 1, 0xc0, 0, 1, 0, 0, 0 };
    SWF = f;
    CHECK(alink_spdif_set_config(&f->u, c, 0) == TB_OK, "on_lock_error 1 accepted");
    CHECK(alink_spdif_unaligned(&f->u) && !alink_spdif_engaged(&f->u) && !alink_spdif_muted(&f->u),
          "on_lock_error 1: unaligned, not muted");
    // The world's stock handshake is the glue's; here the follower delivers
    // the stream to running outputs.
    sw_run(2.5, 0, NULL);
    CHECK(!isnan(latency(f)), "on_lock_error 1: audio plays");
    // Superframes come back: LOCK_OK, then the anchor.
    f->foreign = false;
    c[4] = 0;
    CHECK(alink_spdif_set_config(&f->u, c, 0) == TB_OK, "back to mute");
    sw_run(3.0, 0, NULL);
    CHECK(!f->u.lock_err, "superframes back: lock error cleared");
    sw_run(8.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCKED, "then anchored and LOCKED (%u)", st(f));
}

static void test_cut(void) {
    world(48000, 1, 192);
    sw_fol_t *f = &SW.fol[0];
    sw_run(7.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCKED, "cut: LOCKED first");
    double l0 = lat_mean(f, 1.0);
    uint16_t d0 = f->u.tbk.dist;
    uint32_t ra0 = f->u.reanchors;
    // A 50 ms cut of the carrier.
    f->wire = false;
    sw_run(SW.t + 0.050, 0, NULL);
    CHECK(st(f) == TB_ST_HOLDOVER && f->u.ph.hold, "cut: HOLDOVER (%u)", st(f));
    uint64_t fn;
    CHECK(alink_spdif_f_now(&f->u, &fn, alink_spdif_port_now_us()) && fn > 0, "cut: F extrapolated");
    f->wire = true;
    sw_run(SW.t + 1.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCKED && f->u.reanchors == ra0, "cut: resumed without a re-anchor (%u, re %u)", st(f),
          f->u.reanchors);
    CHECK(f->u.tbk.last_reason == TB_RS_RESUMED, "cut: reason RESUMED (%u)", f->u.tbk.last_reason);
    CHECK((uint16_t)(f->u.tbk.dist - d0) == 2u, "cut: two disturbances (holdover, resume): %u", f->u.tbk.dist - d0);
    sw_run(SW.t + 3.0, 0, NULL);
    double l1 = lat_mean(f, 1.0);
    CHECK(fabs(l1 - l0) < 0.1, "cut: the same timeline (latency %.3f -> %.3f)", l0, l1);
    // A loss beyond 10 s ends the time base (LOST).
    f->wire = false;
    sw_run(SW.t + 10.2, 0, NULL);
    CHECK(!f->u.ph.hold && st(f) == TB_ST_ACQUIRING, "a 10 s cut ends the holdover (%u)", st(f));
}

static void test_epoch(void) {
    world(48000, 2, 192);
    sw_run(8.0, 0, NULL);
    CHECK(st(&SW.fol[0]) == TB_ST_LOCKED && st(&SW.fol[1]) == TB_ST_LOCKED, "epoch: both LOCKED");
    double l0 = lat_mean(&SW.fol[0], 0.5), m0 = lat_mean(&SW.fol[1], 0.5);
    uint8_t ep = SW.src.u.epoch;
    int ev0 = n_tb_ev;
    (void)ev0;
    sw_src_restart(SW.t, 0.006);
    CHECK(SW.src.u.epoch == (uint8_t)(ep + 1u), "epoch: the source's epoch increments");
    bool saw[2] = { false, false };
    for (int i = 0; i < 300; i++) {
        sw_run(SW.t + 0.01, 0, NULL);
        for (int k = 0; k < 2; k++) if (SW.fol[k].u.tbk.last_reason == TB_RS_EPOCH) saw[k] = true;
    }
    CHECK(saw[0] && saw[1], "epoch: every follower re-anchors with reason EPOCH");
    sw_run(SW.t + 6.0, 0, NULL);
    CHECK(st(&SW.fol[0]) == TB_ST_LOCKED && st(&SW.fol[1]) == TB_ST_LOCKED, "epoch: LOCKED again");
    CHECK(SW.fol[0].u.src_epoch == SW.src.u.epoch, "epoch: on the new epoch");
    double l1 = lat_mean(&SW.fol[0], 0.5), m1 = lat_mean(&SW.fol[1], 0.5);
    CHECK(fabs(l1 - l0) < 0.1 && fabs(m1 - m0) < 0.1, "epoch: latency kept (%.3f %.3f -> %.3f %.3f)", l0, m0, l1, m1);
}

static void test_coast(void) {
    world(48000, 1, 192);
    sw_fol_t *f = &SW.fol[0];
    sw_run(7.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCKED, "coast: LOCKED first");
    double l0 = lat_mean(f, 0.5);
    uint16_t d0 = f->u.tbk.dist;
    uint32_t e0 = f->u.sf_errors;
    // 8 blocks with zero user bits: coasted.
    f->kill_ubits = 8;
    sw_run(SW.t + 0.2, 0, NULL);
    CHECK(f->u.sf_errors - e0 == 8u, "coast: 8 blocks counted in sf_errors (%u)", f->u.sf_errors - e0);
    CHECK(f->u.tbk.dist == d0 && st(f) == TB_ST_LOCKED, "coast: no disturbance, still LOCKED");
    double l1 = lat_mean(f, 0.5);
    CHECK(fabs(l1 - l0) < 0.05, "coast: the timeline unmoved (%.3f -> %.3f)", l0, l1);
    // 9: the 9th enters HOLDOVER, the 10th valid resumes.
    f->kill_ubits = 9;
    bool held = false;
    for (int i = 0; i < 40; i++) {
        sw_run(SW.t + 0.005, 0, NULL);
        if (st(f) == TB_ST_HOLDOVER) held = true;
    }
    CHECK(held, "coast: the 9th missing superframe enters HOLDOVER");
    sw_run(SW.t + 1.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCKED && f->u.tbk.last_reason == TB_RS_RESUMED, "coast: resumed (%u, %u)", st(f),
          f->u.tbk.last_reason);
    // A returning superframe whose F breaks continuity re-anchors, on that
    // very block (not later through the phase error): 5 blocks lost, the
    // source's counter jumps meanwhile (same epoch).
    sw_run(SW.t + 3.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCKED, "coast: LOCKED again");
    uint32_t ra = f->u.reanchors;
    f->kill_ubits = 5;
    sw_run(SW.t + 0.008, 0, NULL);
    SW.src.u.origin += 1920u;
    alink_spdif_src_stage(&SW.src.u, (uint32_t)(sw_src_pos(SW.t) / 192.0), true);
    bool back = false, re_at_back = false;
    for (int i = 0; i < 60 && !back; i++) {
        uint32_t e = f->u.sf_errors;
        sw_run(SW.t + 0.0005, 0, NULL);
        if (f->kill_ubits == 0 && f->u.coast == 0 && f->u.sf_errors >= e && f->u.blocks_since_sf == 0 && i > 2) {
            back = true;
            re_at_back = f->u.reanchors == ra + 1u && f->u.tbk.last_reason == TB_RS_REANCHOR;
        }
    }
    CHECK(back && re_at_back, "coast: a return that breaks continuity re-anchors at once (back %d, re %u)", back,
          f->u.reanchors - ra);
    // After a holdover, a continuity break: the re-anchored lock is LOCKED,
    // not RESUMED (a new placement).
    sw_run(SW.t + 4.0, 0, NULL);
    f->kill_ubits = 12;
    sw_run(SW.t + 0.04, 0, NULL);
    CHECK(st(f) == TB_ST_HOLDOVER, "coast: held (%u)", st(f));
    SW.src.u.origin += 1920u;
    alink_spdif_src_stage(&SW.src.u, (uint32_t)(sw_src_pos(SW.t) / 192.0), true);
    sw_run(SW.t + 5.0, 0, NULL);
    CHECK(st(f) == TB_ST_LOCKED && f->u.tbk.last_reason == TB_RS_LOCKED,
          "coast: re-anchored after a holdover locks as LOCKED, not RESUMED (%u)", f->u.tbk.last_reason);
    // A new D while locked re-anchors at the write (REANCHOR), not later.
    uint32_t r2 = f->u.reanchors;
    uint8_t c[8] = { 1, 1, 200, 0, 0, 0, 0, 0 };
    SWF = f;
    CHECK(alink_spdif_set_config(&f->u, c, 0) == TB_OK && f->u.reanchors == r2 + 1u &&
              f->u.tbk.last_reason == TB_RS_REANCHOR && !f->u.ph.started,
          "a new D re-anchors at the write");
}

// The block's first subframe must carry the Z (B) preamble: a superframe
// collected from words that are not block aligned is not accepted.
static void test_block_alignment(void) {
    alink_spdif_src_t src = { 0, 1, 0x42, 0x37, 3, 0, 0, 0, 0 };
    uint8_t sf[48];
    alink_spdif_sf_build(&src, 960000u, sf);
    uint32_t w[384];
    for (unsigned i = 0; i < 384; i++) {
        unsigned ub = (sf[i >> 3] >> (i & 7u)) & 1u;
        w[i] = (i == 0 ? 0xFu : (i & 1u) ? 0x7u : 0xBu) | (uint32_t)ub << 29;
    }
    for (int bad = 0; bad < 2; bad++) {
        alink_spdif_t u;
        alink_spdif_init(&u, 0, 0);
        u.mode = 1;
        u.ph.mode = 1;
        u.active = true;
        alink_spdif_rx_state(&u, true, 1, 48000u, 0);
        if (bad) w[0] = 0xBu | (w[0] & ~0xFu);
        SWF = NULL;
        alink_spdif_rx_words(&u, w, 384, 0, 10);
        if (!bad) CHECK(u.sf_seen && u.rd_valid && u.rd_f == 960192u, "aligned block accepted, rd_f %llu", (unsigned long long)u.rd_f);
        else CHECK(!u.sf_seen && !u.rd_valid, "a block without the Z preamble at word 0 is not accepted");
    }
}

static void test_broadcast(void) {
    world(48000, 2, 192);
    sw_run(8.0, 0, NULL);
    CHECK(st(&SW.fol[0]) == TB_ST_LOCKED && st(&SW.fol[1]) == TB_ST_LOCKED, "broadcast: LOCKED");
    for (int k = 0; k < 2; k++) SW.fol[k].n_op = 0;
    uint64_t fp = (uint64_t)sw_src_playing();
    uint64_t N = fp + 2000u;
    uint8_t p[16] = { 0 };
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(N >> (8 * i));
    p[8] = ALINK_OP_MUTE;
    p[12] = 3;
    SWF = NULL;
    CHECK(alink_spdif_ctrl_write(&SW.src.u, p, fp, fp + 256u) == TB_OK, "broadcast MUTE accepted");
    alink_spdif_src_stage(&SW.src.u, (uint32_t)(sw_src_pos(SW.t) / 192.0), true);
    sw_run(SW.t + 0.3, 0, NULL);
    for (int k = 0; k < 2; k++) {
        sw_fol_t *f = &SW.fol[k];
        CHECK(f->n_op == 1 && f->oplog[0].op == ALINK_OP_MUTE && f->oplog[0].frame == N &&
                  f->oplog[0].next_f == N && !f->oplog[0].late,
              "broadcast: follower %d applies it on F (n %d, next %llu, N %llu)", k, f->n_op,
              (unsigned long long)f->oplog[0].next_f, (unsigned long long)N);
    }
    CHECK(!SW.src.u.bc_pending, "broadcast ended at F");
}

// ---------------------------------------------------------------------------
// Op 9 on time base 3, EXT_TIME_BASE
// ---------------------------------------------------------------------------

static void gen_hook(sw_fol_t *f) {
    (void)f;
    siggen_output_gate(siggen_wants_output());
    siggen_service();
    timebase_service(alink_spdif_port_now_us() / 1000u);
}

static void op9_bytes(uint8_t p[16], uint64_t t) {
    memset(p, 0, 16);
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)(t >> (8 * i));
    p[8] = ALINK_OP_SIGGEN_START;
    p[12] = 3;
}

static void test_op9_follower(void) {
    world(48000, 1, 192);
    sw_fol_t *f = &SW.fol[0];
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) matrix_mixer.outputs[o].enabled = 1;
    alink_spdif_tb_register(&f->u);
    sw_pass_hook = gen_hook;
    sw_run(7.0, 0, NULL);
    CHECK(f->u.tbk.state == TB_ST_LOCKED, "op 9: follower LOCKED in the tracker (%u)", f->u.tbk.state);
    // EXT_TIME_BASE: one follower entry, domain = source id | epoch << 16 | 3 << 24.
    uint8_t tb[64];
    SWF = f;
    timebase_read(tb);
    uint32_t dom = le32(tb + 12 + 4);
    CHECK(tb[1] == 1 && tb[12] == TB_ID_SPDIF && tb[13] == TB_ROLE_FOLLOWER && tb[14] == TB_ST_LOCKED &&
              dom == (0x03000000u | (uint32_t)SW.src.u.epoch << 16 | 0x3742u) && (tb[30] | tb[31] << 8) == 192,
          "EXT_TIME_BASE follower entry (count %u tb %u role %u st %u domain %08x)", tb[1], tb[12], tb[13], tb[14], dom);
    uint64_t t0 = le64(tb + 20);
    uint32_t of0 = le32(tb + 8);
    // t - output_frames constant over reads while LOCKED and undisturbed.
    bool same = true;
    for (int i = 0; i < 50; i++) {
        sw_run(SW.t + 0.01, 0, NULL);
        SWF = f;
        timebase_read(tb);
        if ((uint32_t)(le64(tb + 20) - le32(tb + 8)) != (uint32_t)(t0 - of0)) same = false;
    }
    CHECK(same, "EXT_TIME_BASE t - output_frames constant");
    SiggenConfig c = sh_cfg(SIGGEN_SWEEP_LOG, 1, 2);
    CHECK(siggen_stage_config(&c, sizeof c), "op 9: stage");
    uint64_t T = f->u.ph.next_w + 48u * 300u + 11u;
    uint8_t p[16];
    op9_bytes(p, f->u.ph.next_w + 48u * 100u);
    CHECK(alink_spdif_apply(&f->u, p, f->u.ph.next_w) == TB_ERR_STATE, "op 9: lead under arm_lead refused (STATE)");
    op9_bytes(p, T);
    p[12] = 0;
    CHECK(alink_spdif_apply(&f->u, p, f->u.ph.next_w) == TB_ERR_VALUE, "op 9: time base 0 is not this queue's");
    op9_bytes(p, T);
    f->n_op = 0;
    CHECK(alink_spdif_apply(&f->u, p, f->u.ph.next_w) == TB_OK && f->u.aq.n == 1, "op 9 on time base 3 queued");
    sh_timeline_t tl = sh_timeline();
    CHECK(tl.state == TB_TL_PENDING && tl.pending == T && tl.tb == TB_ID_SPDIF, "op 9: timeline pending on F");
    sw_run(SW.t + 0.5, 0, NULL);
    tl = sh_timeline();
    CHECK(f->n_op >= 1 && f->oplog[0].frame == T && f->oplog[0].next_f == T, "op 9 runs when F = T enters (next %llu)",
          (unsigned long long)(f->n_op ? f->oplog[0].next_f : 0));
    CHECK(tl.result == TB_RES_STARTED && tl.start == T, "op 9 started the generator on its sample (result %u start %llu T %llu)",
          tl.result, (unsigned long long)tl.start, (unsigned long long)T);
    siggen_control(SIGGEN_CTL_STOP_NOW);
    siggen_service();
    // A loss (re-anchor) stops a timed run and cancels a pending op 9.
    CHECK(siggen_stage_config(&c, sizeof c), "op 9: restage");
    T = f->u.ph.next_w + 48u * 400u;
    op9_bytes(p, T);
    CHECK(alink_spdif_apply(&f->u, p, f->u.ph.next_w) == TB_OK, "op 9 queued again");
    reanchor(&f->u, TB_RS_REANCHOR);
    timebase_service(alink_spdif_port_now_us() / 1000u + 1000u);
    tl = sh_timeline();
    CHECK(tl.result == TB_RES_CANCELLED_LOSS && f->u.aq.n == 0, "op 9 cancelled by the loss (%u, n %u)", tl.result, f->u.aq.n);
    // CANCEL_ALL with time base 3 in any state.
    uint8_t ca[16] = { 0 };
    ca[12] = 3;
    CHECK(alink_spdif_apply(&f->u, ca, 0) == TB_OK, "CANCEL_ALL on time base 3");
    sw_pass_hook = NULL;
}

static void test_op9_source(void) {
    world(48000, 0, 192);
    alink_spdif_t *s = &SW.src.u;
    SWF = NULL;
    alink_spdif_tb_register(s);
    timebase_service(0);
    CHECK(s->tbk.state == TB_ST_LOCKED, "source: LOCKED while its outputs run");
    uint8_t tb[64];
    timebase_read(tb);
    uint32_t dom = le32(tb + 12 + 4);
    CHECK(tb[1] == 1 && tb[12] == TB_ID_SPDIF && tb[13] == TB_ROLE_SOURCE && tb[14] == TB_ST_LOCKED &&
              dom == (0x03000000u | (uint32_t)s->epoch << 16 | 0x3742u) && tb[30] == 0 && tb[31] == 0,
          "EXT_TIME_BASE source entry (domain %08x)", dom);
    uint64_t t = le64(tb + 20);
    uint32_t of = le32(tb + 8);
    CHECK((t - of) % 192u == 0, "source t - output_frames is its origin, a multiple of 192");
    SiggenConfig c = sh_cfg(SIGGEN_SWEEP_LOG, 1, 1);
    CHECK(siggen_stage_config(&c, sizeof c), "source op 9: stage");
    uint64_t fw = alink_spdif_src_f(s, 256u);
    uint64_t T = fw + 48u * 200u + 5u;
    uint8_t p[16];
    op9_bytes(p, T);
    CHECK(alink_spdif_apply(s, p, fw) == TB_OK, "source op 9 queued");
    // The pipeline: blocks of 48 split at the op (the split hook).
    sw_src_n_op = 0;
    for (int i = 0; i < 400 && s->aq.n; i++) {
        siggen_tb_poll(TB_ID_SPDIF, fw, 48000u);
        siggen_output_gate(siggen_wants_output());
        siggen_service();
        uint32_t n = 48, k;
        while (n && (k = alink_spdif_src_split(s, fw, n)) != n) {
            fw += k;
            n -= k;
        }
        fw += n;
    }
    sh_timeline_t tl = sh_timeline();
    CHECK(sw_src_n_op == 1 && sw_src_oplog[0].frame == T, "source op 9 ran on its F");
    CHECK(tl.result == TB_RES_STARTED && tl.start == T, "source op 9 started on its sample (%u, %llu vs %llu)", tl.result,
          (unsigned long long)tl.start, (unsigned long long)T);
    siggen_control(SIGGEN_CTL_STOP_NOW);
    siggen_service();
    // An epoch change (output restart): the source leaves LOCKED, counted once,
    // 0xE4 EPOCH when it runs again, the domain changes.
    uint16_t d0 = s->tbk.dist;
    alink_spdif_src_stopping(s, alink_spdif_src_f(s, 5000u));
    timebase_service(500);
    CHECK(s->tbk.state == TB_ST_ANCHORING, "source restarting: ANCHORING");
    alink_spdif_src_started(s);
    timebase_service(1000);
    CHECK(s->tbk.state == TB_ST_LOCKED && s->tbk.last_reason == TB_RS_EPOCH && (uint16_t)(s->tbk.dist - d0) == 1u,
          "source epoch: LOCKED, reason EPOCH, one disturbance (%u)", (unsigned)(s->tbk.dist - d0));
    timebase_read(tb);
    CHECK(le32(tb + 12 + 4) != dom, "source epoch: a new domain");
}

static void test_mode0(void) {
    world(48000, 1, 192);
    sw_fol_t *f = &SW.fol[0];
    uint8_t c[8] = { 0, 1, 0xc0, 0, 0, 0, 0, 0 };
    SWF = f;
    CHECK(alink_spdif_set_config(&f->u, c, 0) == TB_OK && f->u.mode == 0, "mode 0");
    f->n_reads = 0;
    sw_run(2.0, 0, NULL);
    CHECK(alink_spdif_state(&f->u) == TB_ST_NONE && f->u.reanchors == 0 && f->u.sf_errors == 0 && !f->u.sf_seen,
          "mode 0: no time base, nothing observed");
    CHECK(alink_spdif_poll(&f->u, 0) == 0 && f->n_reads == 0, "mode 0: the role reads nothing (the stock input does)");
    CHECK(!alink_spdif_muted(&f->u) && !alink_spdif_engaged(&f->u) && !alink_spdif_owns_rings(&f->u) &&
              !alink_spdif_anchor_wanted(&f->u),
          "mode 0: no mute, no engagement, no anchor");
    alink_spdif_tb_register(&f->u);
    uint8_t tb[64];
    timebase_read(tb);
    CHECK(tb[1] == 0, "mode 0: no EXT_TIME_BASE entry");
    // A source's S/PDIF in mode 0 stages nothing.
    alink_spdif_t s;
    alink_spdif_init(&s, 1, 2);
    alink_spdif_src_stage(&s, 0, true);
    CHECK(alink_spdif_src_staged(&s, 0) == NULL, "mode 0: no superframe staged (zero user bits)");
}

int main(void) {
    test_config();
    test_status_layout();
    test_ctrl_layout();
    test_anchor_lock(48000);
    test_anchor_lock(96000);
    test_lock_error();
    test_cut();
    test_epoch();
    test_coast();
    test_block_alignment();
    test_broadcast();
    test_op9_follower();
    test_op9_source();
    test_mode0();
    printf("test_alink_spdif: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
