/*
 * alink_usb_core.c - USB-direct sync engine (WP USB1).  See alink_usb_core.h.
 *
 * Main-loop code only (control plane, one call per packet in phase mode,
 * one service per pass, one servo tick per 75 ms): the firmware leaves it
 * in flash.  The SOF and packet-stamp parts are inline in the header.
 */

#include "alink_usb_core.h"

#include <string.h>

static uint16_t au_get16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static void au_put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void au_put32(uint8_t *p, uint32_t v) { au_put16(p, v); au_put16(p + 2, v >> 16); }
static void au_putf(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); au_put32(p, u); }
static float fabs_(float x) { return x < 0.0f ? -x : x; }

// ---------------------------------------------------------------------------
// Estimator
// ---------------------------------------------------------------------------

void alink_usb_est_reset(alink_usb_est_t *e) {
    memset(e, 0, sizeof(*e));
}

// The quick estimate (WP USB3): block means of the clean pairs.
static void est_quick(alink_usb_est_t *e, uint32_t n, uint32_t t) {
    uint32_t x = n - e->q_n0;
    if (x < 2u * ALINK_USB_EST_QUICK) {
        int32_t y = (int32_t)(t - e->q_t0 - x * ALINK_USB_T48_FRAME);
        if (y > 480000 || y < -480000) {        // 10 ms off in half a second: not one time base
            alink_usb_est_reset(e);
            return;
        }
        int b = x < ALINK_USB_EST_QUICK ? 0 : 1;
        e->q_x[b] += (int32_t)x;
        e->q_y[b] += y;
        e->q_cnt[b]++;
        return;
    }
    if (e->quick || e->q_cnt[0] < ALINK_USB_EST_QUICK_MIN || e->q_cnt[1] < ALINK_USB_EST_QUICK_MIN) return;
    float c0 = (float)e->q_cnt[0], c1 = (float)e->q_cnt[1];
    float dx = (float)e->q_x[1] / c1 - (float)e->q_x[0] / c0;
    float dy = (float)e->q_y[1] / c1 - (float)e->q_y[0] / c0;
    if (dx < 1.0f) return;
    float slope = dy / dx;                      // t48 per frame beyond the nominal 48000
    float ppm = -slope / ((float)ALINK_USB_T48_FRAME + slope) * 1e6f;
    if (ppm > 2000.0f || ppm < -2000.0f) {
        alink_usb_est_reset(e);
        return;
    }
    e->ppm = ppm;
    e->valid = true;
    e->quick = true;
}

uint32_t alink_usb_est_span(const alink_usb_est_t *e) {
    return e->valid ? e->last_n - e->n[0] : 0u;
}

void alink_usb_est_sample(alink_usb_est_t *e, uint32_t n, uint32_t t) {
    if (e->seeded && n == e->last_n) return;
    if (e->seeded && e->rebase) {
        // A ride-through return with a deviation (review usb2 N1): keep the
        // rate, restart the anchors on the first clean pair after the return,
        // so the deviation stays out of it.
        if ((int32_t)(t - e->rebase_t) <= 0) return;
        e->n[0] = e->n[1] = e->last_n = n;
        e->t[0] = e->t[1] = t;
        e->rebase = false;
        e->q_off = true;
        e->quick = false;
        return;
    }
    // Seed, or reseed after a gap without clean pairs (over 12 s): the
    // anchors stay exact either way, but a long gap is where the frame
    // extension can be wrong.
    if (!e->seeded || n - e->last_n > ALINK_USB_EST_GAP_MAX) {
        memset(e, 0, sizeof(*e));
        e->n[0] = e->n[1] = e->last_n = n;
        e->t[0] = e->t[1] = t;
        e->q_n0 = n;
        e->q_t0 = t;
        e->seeded = true;
        return;
    }
    e->last_n = n;
    if (n - e->n[1] >= ALINK_USB_EST_HALF) {
        e->n[0] = e->n[1];
        e->t[0] = e->t[1];
        e->n[1] = n;
        e->t[1] = t;
    }
    uint32_t span = n - e->n[0];
    if (span < ALINK_USB_EST_MIN) {
        if (!e->q_off) est_quick(e, n, t);
        return;
    }
    uint32_t dt = t - e->t[0];
    int64_t num = (int64_t)span * ALINK_USB_T48_FRAME - (int64_t)dt;
    float ppm = (float)num * 1e6f / (float)dt;
    if (ppm > 2000.0f || ppm < -2000.0f) {      // garbage (a wrong extension): start over
        alink_usb_est_reset(e);
        return;
    }
    e->ppm = ppm;
    e->valid = true;
    e->quick = false;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void alink_usb_init(alink_usb_t *us) {
    memset(us, 0, sizeof(*us));
    us->mode = ALINK_USB_MODE_OFF;
    us->flags = ALINK_USB_FLAGS_DEFAULT;
    us->d = ALINK_USB_D_DEFAULT;
    us->d_min = 0;
    us->d_max = 0xFFFF;
    us->fs = 48000u;
    us->spf = 48u;
    us->block_cap = 48u;
    us->last_frame = ALINK_USB_FRAME_NONE;
    alink_phase_init(&us->ph);
    us->ph.mode = 0;
    us->ph.d_local = us->d;
    alink_apply_init(&us->aq);
    us->diag.last_cause = 0xFF;
    us->parked = true;
}

static void sync_ph(alink_usb_t *us) {
    us->ph.mode = us->mode == ALINK_USB_MODE_PHASE ? 1u : 0u;
    us->ph.d_local = us->d;
    us->ph.hops_offset = 0;
    us->ph.d_min = us->d_min;
    us->ph.d_max = us->d_max;
}

void alink_usb_limits(alink_usb_t *us, uint32_t fs, uint32_t target, uint32_t cap, uint32_t span) {
    (void)target;
    uint32_t spf = fs / 1000u;
    // The rings run D - lag samples ahead of the arrival.  The newest packet is
    // held back a frame (so a gap can fade out), and a packet may come up to
    // 1.2 frames after its SOF before the hold, at lag > D - cap - 16, decides
    // it is lost: D >= 2.2 spf + cap + 16.
    uint32_t lo = 2u * spf + spf / 5u + cap + ALINK_USB_DECODE_MARGIN;
    uint32_t hi = span > cap ? span - cap : 0;
    us->d_min = (uint16_t)(lo < 0xFFFFu ? lo : 0xFFFFu);
    us->d_max = (uint16_t)(hi < 0xFFFFu ? hi : 0xFFFFu);
    us->block_cap = cap ? cap : spf;
    sync_ph(us);
}

bool alink_usb_phase_possible(const alink_usb_t *us, uint32_t fs) {
    return us->mode == ALINK_USB_MODE_PHASE && (fs == 48000u || fs == 96000u) && us->d >= us->d_min &&
           us->d <= us->d_max;
}

uint32_t alink_usb_ring_target(alink_usb_t *us, uint32_t fs, uint32_t target, uint32_t cap, uint32_t span) {
    alink_usb_limits(us, fs, target, cap, span);
    // The stock target unless a phase timeline can run now: not while rejected
    // or without a time base (review usb1 R5).
    if (!alink_usb_phase_possible(us, fs) || us->rejected || !us->tb_ok) return target;
    // D less the block cap and the margin: the hold threshold at the start.
    return alink_phase_prefill(&us->ph, target, cap, span);
}

static void timeline_end(alink_usb_t *us);
static bool rate_ready(const alink_usb_t *us);
static void end_cause(alink_usb_t *us, uint8_t cause);

bool alink_usb_set_config(alink_usb_t *us, const uint8_t p[ALINK_USB_CFG_LEN]) {
    uint8_t mode = p[0], flags = p[1];
    uint16_t d = au_get16(p + 2);
    if (mode > ALINK_USB_MODE_PHASE || (flags & ~ALINK_USB_FLAG_AUTO_REANCHOR) || p[4] > ALINK_USB_LOCKERR_PLAY || p[5])
        return false;
    if (d == 0) return false;
    if (mode == ALINK_USB_MODE_PHASE && (d < us->d_min || d > us->d_max)) return false;
    bool d_changed = d != us->d;
    uint8_t was = us->mode;
    us->mode = mode;
    us->flags = flags;
    us->d = d;
    us->on_lock_error = p[4];
    sync_ph(us);
    // Leaving mode 2: the stock ring target at once (review usb1 F4).
    if (was == ALINK_USB_MODE_PHASE && mode != ALINK_USB_MODE_PHASE) alink_usb_port_ring_target();
    // A running timeline that the new values move: re-anchor (main loop).
    // Leaving mode 2 ends it without a re-anchor (WP USB3b: it counted as a
    // re-anchor "other" in the DIAG and STATUS before; now STOP).
    if ((us->started || us->prefilling) && (mode != was || d_changed)) {
        bool again = mode == ALINK_USB_MODE_PHASE;
        end_cause(us, again ? ALINK_USB_END_RE_OTHER : ALINK_USB_END_STOP);
        timeline_end(us);
        if (again) {
            us->reanchors++;
            us->ev_pending = true;
            us->ev_reason = ALINK_USB_RS_REANCHOR;
            us->ev_gap_last = false;
        }
    }
    return true;
}

void alink_usb_get_config(const alink_usb_t *us, uint8_t o[ALINK_USB_CFG_LEN]) {
    memset(o, 0, ALINK_USB_CFG_LEN);
    o[0] = us->mode;
    o[1] = us->flags;
    au_put16(o + 2, us->d);
    uint8_t sf = 0;
    if (us->mode == ALINK_USB_MODE_PHASE) {
        if (us->d < us->d_min) sf |= ALINK_USB_SF_D_BELOW_MIN;
        if (us->d > us->d_max) sf |= ALINK_USB_SF_D_ABOVE_MAX;
        if (us->fs != 48000u && us->fs != 96000u) sf |= ALINK_USB_SF_RATE_UNSUPP;
    }
    o[4] = us->on_lock_error;
    o[6] = sf;
    o[7] = us->state;
}

void alink_usb_store(const alink_usb_t *us, uint8_t s[ALINK_USB_STORE_LEN]) {
    s[0] = us->mode;
    s[1] = us->flags;
    au_put16(s + 2, us->d);
}

void alink_usb_load(alink_usb_t *us, const uint8_t s[ALINK_USB_STORE_LEN]) {
    uint16_t d = au_get16(s + 2);
    if (s[0] > ALINK_USB_MODE_PHASE || (s[1] & ~ALINK_USB_FLAG_AUTO_REANCHOR) || d == 0) return;
    us->mode = s[0];
    us->flags = s[1];
    us->d = d;
    sync_ph(us);
}

bool alink_usb_nominal_feedback(const alink_usb_t *us) {
    return us->mode != ALINK_USB_MODE_OFF && us->tb_ok && !us->rejected && us->rate_locked;
}

bool alink_usb_lock_error(const alink_usb_t *us) {
    if (us->mode != ALINK_USB_MODE_PHASE) return false;
    return us->rejected || !alink_usb_phase_possible(us, us->fs) ||
           (us->started && us->ph.state == ALINK_PHASE_PENDING);
}

bool alink_usb_out_of_step(const alink_usb_t *us) {
    if (us->mode != ALINK_USB_MODE_PHASE) return false;
    if (alink_usb_lock_error(us)) return us->on_lock_error == ALINK_USB_LOCKERR_MUTE;
    return !(us->started && us->ph.state == ALINK_PHASE_LOCKED);
}

bool alink_usb_outputs_orphaned(const alink_usb_t *us) {
    if (!us->drained || us->prefilling || us->started) return false;
    // An anchor will come and start them: mode 2, phase lock possible, host in range.
    return us->mode != ALINK_USB_MODE_PHASE || us->rejected || !alink_usb_phase_possible(us, us->fs);
}

void alink_usb_on_restarted(alink_usb_t *us) { us->drained = false; }

uint8_t alink_usb_policy_byte(const alink_usb_t *us) { return us->on_lock_error; }

void alink_usb_policy_load(alink_usb_t *us, uint8_t b) {
    us->on_lock_error = b <= ALINK_USB_LOCKERR_PLAY ? b : ALINK_USB_LOCKERR_MUTE;
}

// ---------------------------------------------------------------------------
// The stored host rate (WP USB3)
// ---------------------------------------------------------------------------

void alink_usb_seed_load(alink_usb_t *us, const uint8_t s[ALINK_USB_SEED_LEN]) {
    int16_t v = (int16_t)au_get16(s);
    float ppm = (float)v * 0.01f;
    us->seed_ok = s[2] == (uint8_t)(0xA5u ^ s[0] ^ s[1]) && fabs_(ppm) <= ALINK_USB_HOST_MAX_PPM;
    us->seed_ppm = us->seed_ok ? ppm : 0.0f;
}

static void seed_encode(float ppm, uint8_t s[ALINK_USB_SEED_LEN]) {
    float f = ppm * 100.0f;
    int32_t v = (int32_t)(f < 0.0f ? f - 0.5f : f + 0.5f);
    au_put16(s, (uint32_t)(uint16_t)(int16_t)v);
    s[2] = (uint8_t)(0xA5u ^ s[0] ^ s[1]);
}

void alink_usb_seed_bytes(const alink_usb_t *us, uint8_t s[ALINK_USB_SEED_LEN]) {
    if (!us->seed_ok) {
        s[0] = s[1] = s[2] = 0;
        return;
    }
    seed_encode(us->seed_ppm, s);
}

static bool rate_ready(const alink_usb_t *us);

bool alink_usb_seed_save_due(alink_usb_t *us, uint8_t s[ALINK_USB_SEED_LEN]) {
    // A settled rate over a long span, with nothing playing for a while: the
    // directory write (about 45 ms with interrupts masked, then the pipeline
    // restart) re-anchors a phase timeline, inaudibly when nothing plays.
    if (us->mode == ALINK_USB_MODE_OFF || !us->tb_ok || us->rejected || us->dark || !us->rate_locked) return false;
    if (!us->est.valid || us->est.quick || alink_usb_est_span(&us->est) < ALINK_USB_SEED_SPAN || !rate_ready(us))
        return false;
    if (us->stream || !us->idle_ms || us->now_ms - us->idle_ms < ALINK_USB_SEED_IDLE_MS) return false;
    if (us->prefilling || (us->started && us->ph.state == ALINK_PHASE_ANCHORING)) return false;
    if (us->seed_saves >= ALINK_USB_SEED_SAVES_MAX) return false;
    if (us->seed_saves && us->now_ms - us->seed_save_ms < ALINK_USB_SEED_GAP_MS) return false;
    if (us->seed_ok && fabs_(us->est.ppm - us->seed_ppm) < ALINK_USB_SEED_SAVE_PPM) return false;
    seed_encode(us->est.ppm, s);
    return true;
}

void alink_usb_seed_saved(alink_usb_t *us, const uint8_t s[ALINK_USB_SEED_LEN]) {
    alink_usb_seed_load(us, s);
    if (us->seed_saves < 0xFFu) us->seed_saves++;
    us->seed_save_ms = us->now_ms;
}

// ---------------------------------------------------------------------------
// Time base reads
// ---------------------------------------------------------------------------

// F_arr - x in samples.  Call with interrupts off when it is paired with the
// ring fill.  The latest clean pair, extrapolated at the measured host rate.
static bool arrival(const alink_usb_t *us, uint64_t x, float *since) {
    if (!alink_usb_tb.have_c) return false;
    uint32_t n = alink_usb_tb.n_c, t = alink_usb_tb.t_c;
    uint32_t now = alink_usb_port_now48();
    int64_t base = (int64_t)((uint64_t)us->spf * n - x);
    float frames = (float)(int32_t)(now - t) / (float)ALINK_USB_T48_FRAME;
    if (us->est.valid) frames *= 1.0f + us->est.ppm * 1e-6f;
    *since = (float)base + (float)us->spf * frames;
    return true;
}

bool alink_usb_sample_now(alink_usb_t *us, uint64_t *s, uint16_t *bus) {
    if (!us->tb_ok) return false;
    uint32_t sv = alink_usb_port_irq_save();
    uint32_t nc = alink_usb_tb.n_c, n = alink_usb_tb.n;
    float frac;
    bool ok = arrival(us, (uint64_t)us->spf * nc, &frac);
    alink_usb_port_irq_restore(sv);
    if (!ok) return false;
    (void)n;
    *s = (uint64_t)us->spf * nc + (uint64_t)(frac > 0.0f ? frac : 0.0f);
    // The bus frame of S itself, so the two always agree (review usb1 F7).
    if (bus) *bus = (uint16_t)((*s / us->spf) & 0x7FFu);
    return true;
}

uint32_t alink_usb_bus_form(const alink_usb_t *us, uint64_t s) {
    uint64_t n = s / us->spf;
    return (uint32_t)(n & 0x7FFu) | (uint32_t)(s - n * us->spf) << 16;
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

static void set_state(alink_usb_t *us, uint8_t st, uint8_t reason) {
    if (st != us->state) {
        us->state = st;
        us->ev_pending = true;
        us->ev_gap_last = false;
    }
    if (reason != ALINK_USB_RS_NONE) {
        us->ev_reason = reason;
        us->ev_pending = true;
        us->ev_gap_last = false;
    }
}

static void event_service(alink_usb_t *us, uint32_t now_ms) {
    if (!us->ev_pending) return;
    if (us->ev_once && now_ms - us->ev_last_ms < ALINK_USB_EVENT_MIN_MS) return;
    uint64_t s = 0;
    alink_usb_sample_now(us, &s, NULL);
    alink_usb_port_event(us->state, us->ev_reason, (uint32_t)s);
    us->ev_state = us->state;
    us->ev_pending = false;
    us->ev_reason = ALINK_USB_RS_NONE;
    us->ev_gap_last = false;
    us->ev_last_ms = now_ms;
    us->ev_once = true;
}

// ---------------------------------------------------------------------------
// Apply at frame
// ---------------------------------------------------------------------------

static void apply_report(alink_usb_t *us, uint8_t op, uint8_t result, uint64_t s) {
    alink_apply_done(&us->aq, op, result, s);
    alink_usb_port_apply_result(op, result, alink_usb_bus_form(us, s));
}

static void apply_run(alink_usb_t *us, bool late) {
    alink_apply_op_t op;
    alink_apply_pop(&us->aq, &op);
    uint8_t r = alink_usb_port_apply(&op);
    if (late && r == ALINK_APPLY_APPLIED) r = ALINK_APPLY_LATE;
    if (op.op == ALINK_OP_SYNC_PULSE && r != ALINK_APPLY_REFUSED) us->sync_pulses++;
    apply_report(us, op.op, r, op.frame);
}

void alink_usb_cancel_all(alink_usb_t *us) {
    while (us->aq.n) {
        alink_apply_op_t op;
        alink_apply_pop(&us->aq, &op);
        apply_report(us, op.op, ALINK_APPLY_CANCELLED, op.frame);
    }
}

bool alink_usb_apply(alink_usb_t *us, const uint8_t p[ALINK_APPLY_LEN]) {
    uint8_t q[ALINK_APPLY_LEN];
    memcpy(q, p, ALINK_APPLY_LEN);
    uint8_t tb = q[12];
    q[12] = 0;
    // The limits of alink_apply_args_ok(); the firmware's values (presets,
    // outputs, delay) are the same on the alink.
    alink_apply_op_t op;
    if (!alink_apply_parse(&alink_usb_apply_limits, q, &op)) return false;
    if (op.op == ALINK_APPLY_CANCEL_ALL) {
        if (tb > 2u) return false;
        alink_usb_cancel_all(us);
        return true;
    }
    if (tb != 1u && tb != 2u) return false;
    if (!us->started || us->ph.state == ALINK_PHASE_ANCHORING) return false;     // no timeline, no S
    uint64_t now;
    if (!alink_usb_sample_now(us, &now, NULL)) return false;
    if (tb == 2u) {
        // Bus frame (bits 0-10) and the sample in it (bits 16-23): the next
        // occurrence of that frame; one in the last 64 frames is past (late).
        if (op.frame & ~(uint64_t)0x00FF07FFu) return false;
        uint32_t bus = (uint32_t)op.frame & 0x7FFu, k = (uint32_t)(op.frame >> 16) & 0xFFu;
        if (k >= us->spf) return false;
        uint32_t n_now = (uint32_t)(now / us->spf);
        uint32_t delta = (bus - n_now) & 0x7FFu;
        uint32_t n = delta >= 2048u - 64u ? n_now - (2048u - delta) : n_now + delta;
        op.frame = (uint64_t)us->spf * n + k;
    }
    if (!alink_apply_push(&us->aq, &op)) return false;
    // One pending pulse: a later SYNC_PULSE moves the GPIO's frame (review usb1 F7).
    if (op.op == ALINK_OP_SYNC_PULSE) (void)alink_usb_port_pulse((uint32_t)(op.frame / us->spf));
    return true;
}

// ---------------------------------------------------------------------------
// Timeline writes
// ---------------------------------------------------------------------------

// Gain ramps of a packet write (review usb1 F1): a lost packet is never a hard
// hole.  The packet before a gap fades out over its own samples, the packet
// after it fades in; both act on the input samples, so the DSP's filters and
// delays smooth the edge further.
#define RAMP_NONE 0
#define RAMP_IN   1
#define RAMP_OUT  2

// Gain at packet sample j of a ramp over [a, b).
static float ramp_gain(int ramp, uint32_t a, uint32_t b, uint32_t j) {
    float len = (float)(b - a);
    if (ramp == RAMP_IN) return (float)(j - a + 1u) / len;
    if (ramp == RAMP_OUT) return (float)(b - 1u - j) / len;
    return 1.0f;
}

// n samples at next_w: the packet from `off` (with `ramp` over [off, off+n)),
// or silence (p NULL).  Split where an apply op is due (it acts on exactly
// its sample) and at the block cap.
static void write_span(alink_usb_t *us, const alink_usb_pkt_t *p, uint32_t off, uint32_t n, int ramp) {
    uint32_t done = 0, cap = us->block_cap ? us->block_cap : us->spf;
    while (done < n) {
        uint64_t here = us->ph.next_w, at = alink_apply_next(&us->aq);
        if (at <= here) {
            apply_run(us, at < here);
            continue;
        }
        uint32_t k = n - done;
        if (k > cap) k = cap;
        if (at - here < k) k = (uint32_t)(at - here);
        if (p) {
            uint32_t j = off + done;
            alink_usb_port_write_packet(p, j, k, ramp_gain(ramp, off, off + n, j),
                                        ramp_gain(ramp, off, off + n, j + k - 1u));
        } else {
            alink_usb_port_write_silence(k);
        }
        alink_phase_written(&us->ph, here, k);
        done += k;
    }
}

static void timeline_end(alink_usb_t *us) {
    us->soft = false;
    us->started = false;
    us->prefilling = false;
    us->muted_gap = false;
    us->held = false;
    us->fade_in = false;
    us->pkt_off = 0;
    alink_phase_drain(&us->ph);
}

// Diagnostics: one timeline end of this cause (EXT_ALINK_USB_DIAG).
static void end_cause(alink_usb_t *us, uint8_t cause) {
    if (us->diag.ends[cause] != 0xFFFFu) us->diag.ends[cause]++;
    us->diag.last_cause = cause;
}

static void reanchor_for(alink_usb_t *us, uint8_t cause);
static void reanchor(alink_usb_t *us) { reanchor_for(us, ALINK_USB_END_RE_OTHER); }

static void reanchor_for(alink_usb_t *us, uint8_t cause) {
    end_cause(us, cause);
    timeline_end(us);
    us->reanchors++;
    uint32_t fs = us->fs ? us->fs : 48000u;
    alink_usb_port_soft_mute(fs / 1000u * ALINK_USB_ANCHOR_MUTE_MS);
    set_state(us, ALINK_USB_ST_ANCHORING, ALINK_USB_RS_REANCHOR);
}

// A gap follows the stream's audio: one GAP each (review usb1 F1).
static void count_gap(alink_usb_t *us) {
    if (!us->stream || !us->stream_seen) return;
    // Remembered so that a stream stop right after it can take it back (R3).
    us->gap_n = alink_usb_tb.n;
    us->gap_missed0 = us->missed;
    us->gap_recent = true;
    us->gaps++;
    us->ev_gap_pending0 = us->ev_pending;
    us->ev_gap_reason0 = us->ev_reason;
    us->ev_gap_last = true;
    us->ev_pending = true;
    us->ev_reason = ALINK_USB_RS_GAP;
}

// Write the held packet (the newest placed one) at next_w; fade it out when a
// gap follows, fade it in when it follows one.
static void flush_held(alink_usb_t *us, bool gap_follows) {
    if (!us->held) return;
    us->held = false;
    const alink_usb_pkt_t *h = &us->hp;
    uint32_t n = h->samples, off = us->pkt_off;
    uint64_t hs = (uint64_t)us->spf * h->frame + off;
    // A whole-sample correction may have moved next_w since it was placed.
    if (hs < us->ph.next_w) {
        uint64_t k = us->ph.next_w - hs;
        off = k < (uint64_t)(n - off) ? off + (uint32_t)k : n;
    } else if (hs > us->ph.next_w && hs - us->ph.next_w < 4096u) {
        write_span(us, NULL, 0, (uint32_t)(hs - us->ph.next_w), RAMP_NONE);
    }
    if (off >= n) return;
    bool in = us->fade_in;
    us->fade_in = false;
    if (in && gap_follows && n - off >= 2u) {
        uint32_t mid = off + (n - off) / 2u;
        write_span(us, h, off, mid - off, RAMP_IN);
        write_span(us, h, mid, n - mid, RAMP_OUT);
    } else {
        write_span(us, h, off, n - off, in ? RAMP_IN : gap_follows ? RAMP_OUT : RAMP_NONE);
    }
    if (gap_follows) {
        us->fade_in = true;
        count_gap(us);
    }
}

// One packet.  The newest placed packet is held back one frame and written
// when the next one arrives (or the hold decides that it will not): the
// packet before a gap can then fade out.
bool alink_usb_deliver(alink_usb_t *us, const alink_usb_pkt_t *p) {
    if (!us->started) return true;              // prefill: the outputs are stopped, muted; dropped
    uint32_t frame = p->frame;
    if (frame == ALINK_USB_FRAME_NONE) {
        frame = us->last_frame + 1u;
        us->unstamped++;
    }
    us->last_frame = frame;
    uint32_t n = p->samples, off = 0, span = 448u;
    uint64_t s = (uint64_t)us->spf * frame;
    if (us->held) {
        uint64_t hend = (uint64_t)us->spf * us->hp.frame + us->hp.samples;
        flush_held(us, s > hend);
    }
    bool first = us->stream && !us->stream_seen;
    while (off < n) {
        int64_t gap = (int64_t)(s - us->ph.next_w);
        if (!alink_phase_gap_ok(gap, span)) {
            // Another time base (a bus reset, a wrong stamp far off): start over.
            reanchor(us);
            return true;
        }
        if (gap < 0) {
            uint32_t k = (uint64_t)-gap < (uint64_t)(n - off) ? (uint32_t)-gap : n - off;
            off += k;
            s += k;
            if (!first) {
                us->late += k;
                us->stream_shift -= (int32_t)k;
            }
            continue;
        }
        if (gap > 0) {
            // Silence up to the packet: idle timeline before a stream's first
            // packet, a missing packet after it.
            uint32_t k = (uint64_t)gap < 4096u ? (uint32_t)gap : 4096u;
            write_span(us, NULL, 0, k, RAMP_NONE);
            if (!first && us->stream_seen) {
                us->missed += k;
                us->stream_shift += (int32_t)k;
                us->fade_in = true;
            }
            continue;
        }
        if (first) {
            us->stream_seen = true;
            us->stream_first = (uint32_t)(s / us->spf);
            us->stream_shift = 0;
            us->stream_samples = 0;
            us->fade_in = true;                 // the stream starts on silence: fade in
        }
        us->muted_gap = false;
        // In place: hold it (its bytes copied by the port; the USB slot is reused).
        us->hp = *p;
        us->hp.frame = frame;
        us->hp.data = alink_usb_port_keep(p);
        us->pkt_off = off;
        us->held = true;
        us->stream_samples += n - off;
        break;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Phase measurement, verification, hold
// ---------------------------------------------------------------------------

static bool phase_error(alink_usb_t *us, float *e) {
    if (!us->ph.timeline) return false;
    float fill, since;
    uint32_t sv = alink_usb_port_irq_save();
    bool ok = alink_usb_port_play(&fill) && arrival(us, us->ph.next_w, &since);
    alink_usb_port_irq_restore(sv);
    if (!ok) return false;
    *e = alink_phase_error(&us->ph, since, fill);
    return true;
}

// A soft re-anchor (WP USB3): a phase-locked timeline that is off by more
// than 2 samples but at most ALINK_PHASE_CORRECT_MAX is moved back by whole
// samples while muted, as the anchor's verification does, with the outputs
// running: no drain, no DAC hold, no prefill (about 15 ms muted instead of
// the hold, the prefill and the DAC release).  A larger error, a placement
// far off, a stall or a verification that fails re-anchors in full.
static void soft_reanchor(alink_usb_t *us) {
    end_cause(us, ALINK_USB_END_RE_PHASE);
    us->reanchors++;
    alink_usb_port_soft_mute(us->spf * ALINK_USB_ANCHOR_MUTE_MS);
    us->ph.state = ALINK_PHASE_ANCHORING;
    us->ph.corrections = 0;
    us->ph.e_valid = false;
    alink_phase_pi_reset(&us->ph.pi);
    us->soft = true;
    us->soft_ms = us->now_ms + ALINK_USB_SOFT_FADE_MS;
    set_state(us, ALINK_USB_ST_ANCHORING, ALINK_USB_RS_REANCHOR);
}

static void phase_act(alink_usb_t *us, int act, int32_t k, float e) {
    if (act == ALINK_PHASE_ACT_REANCHOR) {
        // A phase error or a failed verification is a phase re-anchor (it was
        // counted as "other" before WP USB3b).
        if (us->ph.state == ALINK_PHASE_LOCKED && fabs_(e) <= (float)ALINK_PHASE_CORRECT_MAX) soft_reanchor(us);
        else reanchor_for(us, ALINK_USB_END_RE_PHASE);
    } else if (act == ALINK_PHASE_ACT_CORRECT) {
        if (k > 0) {
            us->ph.next_w += (uint32_t)k;       // skip: those samples never reach the rings
        } else {
            // Silence that delays the timeline: not samples of it (review lk6 F2).
            uint32_t m = (uint32_t)-k, cap = us->block_cap ? us->block_cap : us->spf;
            while (m) {
                uint32_t c = m < cap ? m : cap;
                alink_usb_port_write_silence(c);
                m -= c;
            }
        }
    }
}

static bool auto_reanchor(const alink_usb_t *us) { return (us->flags & ALINK_USB_FLAG_AUTO_REANCHOR) != 0; }

// Keep the timeline running while no packet comes: the held packet first
// (faded out: the next one is late or lost), then silence, so the rings never
// run more than D - (block cap + margin) behind the arrival.
static void hold(alink_usb_t *us) {
    float h = (float)us->d - (float)(us->block_cap + ALINK_USB_DECODE_MARGIN);
    for (int pass = 0; pass < 2; pass++) {
        float since;
        uint32_t sv = alink_usb_port_irq_save();
        bool ok = arrival(us, us->ph.next_w, &since);
        alink_usb_port_irq_restore(sv);
        if (!ok || since <= h) return;
        if (pass == 0 && us->held) {
            flush_held(us, true);
            continue;
        }
        uint32_t k = (uint32_t)(since - h);
        if (!k) return;
        if (k > 4096u) {                        // the main loop stood still far too long
            reanchor_for(us, ALINK_USB_END_RE_PHASE);
            return;
        }
        write_span(us, NULL, 0, k, RAMP_NONE);
        if (us->stream && us->stream_seen) {
            // A longer loss: the stream is also muted, then fades back in.
            us->missed += k;
            us->stream_shift += (int32_t)k;
            us->fade_in = true;
            us->muted_gap = true;
            alink_usb_port_soft_mute(us->spf * ALINK_USB_GAP_MUTE_MS);
        }
        return;
    }
}

// ---------------------------------------------------------------------------
// Anchor handshake
// ---------------------------------------------------------------------------

static bool rate_ready(const alink_usb_t *us) {
    if (!us->est.valid || !us->ticked) return false;
    float d = alink_usb_port_vcxo_ppm() - us->ff_ppm;
    return d < 2.0f * ALINK_USB_READY_PPM && d > -2.0f * ALINK_USB_READY_PPM;
}

bool alink_usb_anchor_wanted(const alink_usb_t *us) {
    // The VCXO at the feed-forward now, not only once since the last reset: a
    // re-anchor while they disagree fails at once and loops (review usb1 F3).
    // After a fast VCXO step (WP USB3) the outputs' receivers settle first.
    if (us->settling && (int32_t)(us->now_ms - us->settle_ms) < 0) return false;
    return us->tb_ok && !us->rejected && us->rate_locked && rate_ready(us) && !us->started && !us->prefilling &&
           alink_usb_phase_possible(us, us->fs);
}

void alink_usb_on_drain(alink_usb_t *us) {
    us->drained = true;
    alink_phase_drain(&us->ph);
    us->held = false;
    us->prefilling = true;
    us->started = false;
    us->pkt_off = 0;
    alink_usb_port_ring_target();                // prefill target = D - cap - margin
    uint64_t s;
    if (!alink_usb_sample_now(us, &s, NULL)) s = 0;
    us->ph.next_w = s;                          // the first silence written sets `first`
    us->ph.state = ALINK_PHASE_ANCHORING;
    set_state(us, ALINK_USB_ST_ANCHORING, ALINK_USB_RS_NONE);
}

static void prefill(alink_usb_t *us) {
    uint32_t room = alink_usb_port_room();
    if (room) write_span(us, NULL, 0, room, RAMP_NONE);
}

float alink_usb_anchor_wait(alink_usb_t *us) {
    if (!us->prefilling || !us->ph.timeline) return 1e9f;
    if (alink_usb_port_room()) return 1e9f;      // not at the prefill target yet
    float since;
    uint32_t sv = alink_usb_port_irq_save();
    bool ok = arrival(us, us->ph.first, &since);
    alink_usb_port_irq_restore(sv);
    if (!ok) return 1e9f;
    return alink_phase_anchor_wait(&us->ph, since);
}

void alink_usb_on_enabled(alink_usb_t *us) {
    us->drained = false;
    alink_phase_started(&us->ph);
    us->held = false;
    us->fade_in = false;
    us->prefilling = false;
    us->started = true;
    us->anchors++;
    us->stream_seen = false;                    // a running stream is placed anew
}

void alink_usb_outputs_reset(alink_usb_t *us) {
    // Our own drain (main.c) uses drain_and_disable_outputs(), not this.
    if (us->started || us->prefilling) reanchor_for(us, ALINK_USB_END_RE_OUTPUTS);
}

void alink_usb_stream(alink_usb_t *us, bool on) {
    if (!on && us->gap_recent && alink_usb_tb.n - us->gap_n <= ALINK_USB_STOP_FRAMES) {
        // A host sends its last packet a few frames before SET_INTERFACE 0: the
        // hold then saw the end of the stream as a gap.  A normal stop is not a
        // GAP: take the count and its silence back (review usb1 R3).
        us->gaps--;
        us->missed = us->gap_missed0;
        if (us->ev_gap_last) {
            // Nothing else changed since that GAP: the event is as before it, so
            // a GAP that was all there was to send sends nothing (no 0xE3 with
            // reason NONE and no state change, spec 2.9.5; review usb1 N1).
            us->ev_pending = us->ev_gap_pending0;
            us->ev_reason = us->ev_gap_reason0;
            us->ev_gap_last = false;
        } else if (us->ev_reason == ALINK_USB_RS_GAP) {
            us->ev_reason = ALINK_USB_RS_NONE;
        }
    }
    us->gap_recent = false;
    if (on && us->held) {
        // A new stream or format (alt change without alt 0): the held packet
        // belongs to the old layout and cannot be decoded with the new one;
        // dropped, the new stream fades in (review usb1 R4).
        us->held = false;
        us->fade_in = true;
    }
    us->stream = on;
    us->stream_seen = false;
    if (!on) us->muted_gap = false;
}

void alink_usb_stop(alink_usb_t *us, uint8_t reason) {
    if (us->started || us->prefilling) end_cause(us, ALINK_USB_END_STOP);
    us->dark = false;
    us->settling = false;
    alink_usb_cancel_all(us);
    timeline_end(us);
    bool was = us->state != ALINK_USB_ST_OFF;
    us->rejected = false;
    us->tb_ok = false;
    us->ticked = false;
    us->rate_locked = false;
    alink_usb_est_reset(&us->est);
    if (was) {
        us->seed_used = false;                  // parked: the next start may preset it again
        alink_usb_port_park();
        us->parked = true;
        alink_usb_port_ring_target();           // the stock target (review usb1 F4)
        set_state(us, ALINK_USB_ST_OFF, reason);
    }
}

// ---------------------------------------------------------------------------
// Service
// ---------------------------------------------------------------------------

static void service_body(alink_usb_t *us, uint32_t now_ms);

void alink_usb_service(alink_usb_t *us, uint32_t now_ms) {
    us->now_ms = now_ms;
    if (us->mode != ALINK_USB_MODE_OFF) {
        if (us->stream) us->idle_ms = 0;
        else if (!us->idle_ms) us->idle_ms = now_ms ? now_ms : 1u;
        // The stored host rate (WP USB3): a parked VCXO starts there instead
        // of at the crystal, before the first SOF, while nothing is audible
        // (mode 2 mutes out of step; mode 1 only without a stream).  The quick
        // estimate replaces it 0.5 s after the SOF starts; a different host
        // is then reached by the slew from here.
        if (us->seed_ok && !us->seed_used && !us->ticked && !us->rate_locked && !us->rejected &&
            (alink_usb_out_of_step(us) || !us->stream) && us->parked) {
            alink_usb_port_preset(us->seed_ppm);
            us->seed_used = true;
            us->parked = false;
        }
    }
    service_body(us, now_ms);
    // Lock error (WP USB3b): reported on entry and exit, counted in the DIAG.
    bool le = alink_usb_lock_error(us);
    if (le != us->lock_err) {
        us->lock_err = le;
        if (le && us->diag.lock_errors != 0xFFFFu) us->diag.lock_errors++;
        set_state(us, us->state, le ? ALINK_USB_RS_LOCK_ERROR : ALINK_USB_RS_LOCK_OK);
        event_service(us, now_ms);
    }
    // Owner decision 2026-11-14: muted while out of step in mode 2, every
    // pass (the request is refreshed, never accumulated).  Owner 2026-11-18
    // (WP FWSIGGEN): not while the signal generator plays with no host stream
    // (the generator disables the mute itself; nothing else is audible then).
    if (alink_usb_out_of_step(us) && !(us->siggen_exempt && !us->stream))
        alink_usb_port_soft_mute((us->fs / 1000u) * ALINK_USB_ANCHOR_MUTE_MS);
}

static void service_body(alink_usb_t *us, uint32_t now_ms) {
    if (us->mode == ALINK_USB_MODE_OFF) {
        if (us->state != ALINK_USB_ST_OFF) alink_usb_stop(us, ALINK_USB_RS_DISABLED);
        event_service(us, now_ms);
        return;
    }
    uint32_t fs = alink_usb_port_fs();
    if (!fs) fs = 48000u;

    // Time base.
    uint32_t sv = alink_usb_port_irq_save();
    bool valid = alink_usb_tb.valid, have_c = alink_usb_tb.have_c;
    uint32_t t = alink_usb_tb.t, n_c = alink_usb_tb.n_c, t_c = alink_usb_tb.t_c, n = alink_usb_tb.n;
    uint32_t isr = alink_usb_tb.isr;
    uint32_t now = alink_usb_port_now48();
    // WP USB3b: a SOF whose time lies more than ALINK_USB_SINCE_MAX48 before
    // the interrupt entry that took it is a bad PHY reading (RAW behind LAST:
    // the 21-bit difference wrapped to about 43.7 ms; a real SOF is under a
    // frame before its entry, as LAST is the latest SOF).  Use the entry time
    // (a few us late); if it also made the clean pair, keep that pair out of
    // the estimator.  The next SOF overwrites it anyway.
    if (valid && (int32_t)(isr - t) > (int32_t)ALINK_USB_SINCE_MAX48) {
        if (have_c && n_c == n && t_c == t) {
            alink_usb_tb.t_c = t_c = isr;
            us->bad_c = true;
            us->bad_n = n;
        }
        alink_usb_tb.t = t = isr;
        if (us->diag.bad_sof != 0xFFFFu) us->diag.bad_sof++;
    }
    alink_usb_port_irq_restore(sv);
    uint32_t now_us = alink_usb_port_now_us();

    // Diagnostics (WP USB2): the main-loop pass, the time without a SOF
    // interrupt entry, and how late the latest entry ran after its SOF.
    alink_usb_diag_t *dg = &us->diag;
    if (dg->last_pass_us) {
        uint32_t pass = now_us - dg->last_pass_us;
        if (pass > dg->max_pass_us) dg->max_pass_us = pass;
    }
    dg->last_pass_us = now_us ? now_us : 1u;
    if (valid) {
        int32_t quiet = (int32_t)(now - isr), late = (int32_t)(isr - t);
        if (quiet > 0 && (uint32_t)quiet / 48u > dg->max_isr_quiet_us) dg->max_isr_quiet_us = (uint32_t)quiet / 48u;
        if (late > 0 && (uint32_t)late / 48u > dg->max_isr_late_us) dg->max_isr_late_us = (uint32_t)late / 48u;
    }

    // Signed: the main loop reads whole microseconds, so a SOF stamped exactly
    // (RP2350) can be a few ticks "after" now.
    bool sof_ok = valid && (int32_t)(now - t) < (int32_t)(ALINK_USB_SOF_LOST_MS * ALINK_USB_T48_FRAME);
    int64_t lim = us->exact ? 4800 : 24000;
    int end = -1;                               // ALINK_USB_END_* when the time base ends now

    if (us->dark) {
        if (!sof_ok) {
            // Riding through: the timeline runs on the extrapolated arrival.
            if (now_us - us->dark_us > ALINK_USB_RIDE_MAX_MS * 1000u) {
                end = ALINK_USB_END_SOF_LOST;
                dg->last_frames = 0;
                dg->last_us = now_us - us->dark_us;
            }
        } else {
            // Back: the frames counted since the last SOF must match the time
            // at the measured host rate, modulo the 2048-frame wraps the 11-bit
            // frame number cannot show over a long absence.
            // Float and int32 only (no double or 64-bit division helpers, which
            // the SDK places in RAM): 10 s is 4.8e8 ticks, float resolves 32.
            int32_t dt = (int32_t)(t - us->dark_t);
            int32_t dn = (int32_t)(n - us->dark_n);
            float per = (float)ALINK_USB_T48_FRAME / (1.0f + us->est.ppm * 1e-6f);
            float exp_frames = (float)dt / per;
            float kf = (exp_frames - (float)dn) / 2048.0f;
            int32_t k = (int32_t)(kf >= 0.0f ? kf + 0.5f : kf - 0.5f);
            int32_t frames = dn + 2048 * k;
            float dev = (float)dt - (float)frames * per;
            dg->last_frames = (uint32_t)frames;
            dg->last_us = (uint32_t)dt / 48u;
            dg->last_dev_us = (int32_t)(dev / 48.0f);
            if (k < 0 || dt < 0 || dev > (float)lim || dev < -(float)lim) {
                end = ALINK_USB_END_DISCONT;
            } else {
                // Within the F2 limit but beyond the extrapolation's own error:
                // keep the timeline and the rate, restart the estimator's
                // anchors after the return (review usb2 N1).
                float keep = (float)(ALINK_USB_RIDE_KEEP_US * 48);
                if (dev > keep || dev < -keep) {
                    us->est.rebase = true;
                    us->est.rebase_t = us->dark_t;
                }
                if (k) {
                    // Restore the wraps in the ISR's count (it keeps N mod 2048).
                    uint32_t add = (uint32_t)k * 2048u;
                    uint32_t sv2 = alink_usb_port_irq_save();
                    alink_usb_tb.n += add;
                    // The clean pair only if it was taken after the return; one
                    // from before the absence is already right.
                    if ((int32_t)(alink_usb_tb.t_c - us->dark_t) > 0) alink_usb_tb.n_c += add;
                    n = alink_usb_tb.n;
                    n_c = alink_usb_tb.n_c;
                    alink_usb_port_irq_restore(sv2);
                }
                us->dark = false;
                if (dg->rides != 0xFFFFu) dg->rides++;
                if (dg->last_us > dg->longest_ride_us) dg->longest_ride_us = dg->last_us;
                set_state(us, us->state, ALINK_USB_RS_RESUMED);
                us->seen_valid = true;
                us->seen_n = n;
                us->seen_t = t;
            }
        }
    } else {
        // A frame count and clock that disagree since the last pass (a SOF
        // stall with the frame number frozen, a small frame jump) is a
        // discontinuity (review usb1 F2): RP2350 100 us, RP2040 0.5 ms.
        if (valid && us->seen_valid && n != us->seen_n) {
            int64_t dt = (int64_t)(int32_t)(t - us->seen_t);
            int64_t dev = dt - (int64_t)(uint32_t)(n - us->seen_n) * ALINK_USB_T48_FRAME;
            if (dt > 1000LL * ALINK_USB_T48_FRAME) end = ALINK_USB_END_LONG_GAP;
            else if (dev > lim || dev < -lim) end = ALINK_USB_END_DISCONT;
            if (end >= 0) {
                dg->last_frames = n - us->seen_n;
                dg->last_us = (uint32_t)(dt / 48);
                dg->last_dev_us = (int32_t)(dev / 48);
            }
        }
        if (end < 0 && !sof_ok && us->tb_ok) {
            // No SOF for 50 ms: ride through on the estimate if there is one and
            // no anchor is half done (WP USB2); else the time base ends.
            if (dg->holdovers != 0xFFFFu) dg->holdovers++;
            bool anchoring = us->prefilling || (us->started && us->ph.state == ALINK_PHASE_ANCHORING);
            if (us->est.valid && !anchoring) {
                us->dark = true;
                us->dark_n = n;
                us->dark_t = t;
                us->dark_us = now_us - (uint32_t)((int32_t)(now - t) / 48);
                set_state(us, us->state, ALINK_USB_RS_HOLDOVER);
            } else {
                end = ALINK_USB_END_SOF_LOST;
                dg->last_frames = 0;
                dg->last_us = (uint32_t)((int32_t)(now - t) / 48);
            }
        }
    }
    if (!us->dark) {
        us->seen_valid = valid;
        us->seen_n = n;
        us->seen_t = t;
    }
    if (end >= 0 || (!sof_ok && !us->dark)) {
        if (us->tb_ok || us->state != ALINK_USB_ST_NO_SOF) {
            if (us->started || us->prefilling) {
                end_cause(us, (uint8_t)(end >= 0 ? end : ALINK_USB_END_SOF_LOST));
                alink_usb_cancel_all(us);
            }
            timeline_end(us);
            us->dark = false;
            us->tb_ok = false;
            us->ticked = false;
            us->rate_locked = false;
            alink_usb_est_reset(&us->est);
            set_state(us, ALINK_USB_ST_NO_SOF, us->state == ALINK_USB_ST_OFF ? ALINK_USB_RS_NONE : ALINK_USB_RS_SOF_LOST);
            alink_usb_port_ring_target();       // the stock target while dark (R5)
        }
        event_service(us, now_ms);
        return;                                 // the VCXO holds its trim
    }
    if (us->dark) {
        // Holdover: no new rate information, the VCXO holds; the timeline runs
        // on silence at the extrapolated arrival (hold()).
        if (us->started) hold(us);
        event_service(us, now_ms);
        return;
    }
    us->tb_ok = true;
    // Only a fresh clean pair (WP USB3): after a time-base end the latest clean
    // pair can still be the one from before the absence (the first SOFs after
    // a long one are not clean), and seeding on it made the estimator span the
    // absence with a frame count that lost its 2048-frame wraps, a garbage
    // rate, a reset and 2 s more before the next estimate.
    if (have_c && (int32_t)(now - t_c) < (int32_t)(ALINK_USB_SOF_LOST_MS * ALINK_USB_T48_FRAME) &&
        !(us->bad_c && n_c == us->bad_n))
        alink_usb_est_sample(&us->est, n_c, t_c);

    // Host range.
    if (us->est.valid) {
        float a = fabs_(us->est.ppm);
        if (!us->rejected && a > ALINK_USB_HOST_MAX_PPM) {
            if (us->started || us->prefilling) end_cause(us, ALINK_USB_END_REJECTED);
            alink_usb_cancel_all(us);
            timeline_end(us);
            us->rejected = true;
            us->ticked = false;
            us->rate_locked = false;
            alink_usb_port_park();
            us->parked = true;
            set_state(us, ALINK_USB_ST_REJECTED, ALINK_USB_RS_HOST_RANGE);
            alink_usb_port_ring_target();       // the stock target while rejected (R5)
        } else if (us->rejected && a < ALINK_USB_HOST_MAX_PPM - ALINK_USB_HOST_HYST_PPM) {
            us->rejected = false;
            set_state(us, ALINK_USB_ST_PULL_IN, ALINK_USB_RS_HOST_RANGE);
        }
    }
    if (us->rejected) {
        event_service(us, now_ms);
        return;
    }

    // Rate and configuration: a timeline at another rate, or one the mode or
    // D no longer allows, ends (the rate change itself resets the outputs).
    bool phase = alink_usb_phase_possible(us, fs);
    if ((us->started || us->prefilling) && (!phase || fs != us->fs)) {
        end_cause(us, ALINK_USB_END_CONFIG);
        alink_usb_cancel_all(us);
        timeline_end(us);
        set_state(us, us->state, phase ? ALINK_USB_RS_REANCHOR : ALINK_USB_RS_CONFIG);
    }
    if (fs != us->fs) {
        us->fs = fs;
        us->spf = fs / 1000u;
    }

    // Servo tick: the host rate as feed-forward.  Pull-in (and the anchor's
    // prefill): feed-forward only, while the asynchronous feedback still keeps
    // the ring fill.  Rate-locked: the fill servo (mode 1, mode 2 before the
    // anchor).  Phase-locked: the feed-forward plus PI(e).
    if (us->est.valid && (uint32_t)(alink_usb_port_now_us() - us->tick_last_us) >= ALINK_USB_TICK_US) {
        us->tick_last_us = alink_usb_port_now_us();
        float hz = (float)fs * (1.0f + us->est.ppm * 1e-6f);
        float trim = 0.0f;
        bool ff_only = true;
        if (us->started) {
            float e;
            if (phase_error(us, &e)) {
                int act = ALINK_PHASE_ACT_NONE;
                int32_t k = 0;
                if (us->ph.state != ALINK_PHASE_ANCHORING) {
                    act = alink_phase_check(&us->ph, e, auto_reanchor(us), &k);
                    phase_act(us, act, k, e);
                }
                if (act == ALINK_PHASE_ACT_REANCHOR) alink_phase_pi_reset(&us->ph.pi);
                else if (us->ph.state == ALINK_PHASE_LOCKED || us->ph.state == ALINK_PHASE_PENDING)
                    trim = alink_phase_servo(&us->ph, e);
            }
        } else if (us->rate_locked && !us->prefilling && us->stream && alink_usb_port_stream_active()) {
            // The fill servo only while a stream fills the rings: with none
            // (alt 0) the ring sits empty and the fill term would pull the VCXO
            // off the feed-forward by -10..-20 ppm, and in mode 2 keep
            // rate_ready() false so the anchor never comes (hardware H1/H2).
            // Gated on packets arriving, not only on the alt: a stream stalled
            // with the alt still set (a paused application, a host xrun) empties
            // the ring the same way (review usb1 N2).
            ff_only = false;
        }
        // Muted out of step and not yet on a timeline: a faster, bounded slew
        // (WP USB3); an anchor after it waits for the outputs to settle.
        float v0 = alink_usb_port_vcxo_ppm(), off = us->est.ppm + trim - v0;
        bool fast = alink_usb_out_of_step(us) && !us->started &&
                    (off > ALINK_USB_FAST_MIN_PPM || off < -ALINK_USB_FAST_MIN_PPM);
        float slew = fast ? ALINK_USB_FAST_SLEW_PPM : ALINK_USB_SLEW_PPM;
        alink_usb_port_actuate(hz, trim, ff_only, slew);
        us->parked = false;
        float dv = alink_usb_port_vcxo_ppm() - v0;
        if (dv > 2.0f * ALINK_USB_SLEW_PPM || dv < -2.0f * ALINK_USB_SLEW_PPM) {
            us->settling = true;
            us->settle_ms = now_ms + ALINK_USB_FAST_SETTLE_MS;
        }
        us->ff_ppm = us->est.ppm;
        us->ticked = true;
        if (!us->rate_locked && rate_ready(us)) us->rate_locked = true;
    }

    // Anchor, verification, hold, state.
    uint32_t mute = (us->fs / 1000u) * ALINK_USB_ANCHOR_MUTE_MS;
    if (us->prefilling) {
        prefill(us);
        alink_usb_port_soft_mute(mute);
        set_state(us, ALINK_USB_ST_ANCHORING, ALINK_USB_RS_NONE);
    } else if (us->started) {
        // A soft re-anchor corrects only once its mute has faded the audio out.
        bool wait = us->soft && (int32_t)(now_ms - us->soft_ms) < 0;
        if (us->ph.state == ALINK_PHASE_ANCHORING && !wait) {
            float e;
            if (phase_error(us, &e)) {
                int32_t k = 0;
                int act = alink_phase_check(&us->ph, e, auto_reanchor(us), &k);
                phase_act(us, act, k, e);
                if (us->ph.state != ALINK_PHASE_ANCHORING) us->soft = false;
            }
        }
        if (us->started) hold(us);
        if (us->started && us->ph.state == ALINK_PHASE_ANCHORING) {
            alink_usb_port_soft_mute(mute);
            set_state(us, ALINK_USB_ST_ANCHORING, ALINK_USB_RS_NONE);
        } else if (us->started && us->state != ALINK_USB_ST_PHASE_LOCK) {
            set_state(us, ALINK_USB_ST_PHASE_LOCK, ALINK_USB_RS_LOCKED);
        }
    }
    if (!us->started && !us->prefilling) {
        if (!us->rate_locked) {
            set_state(us, ALINK_USB_ST_PULL_IN, ALINK_USB_RS_NONE);
        } else if (phase) {
            set_state(us, ALINK_USB_ST_ANCHORING, ALINK_USB_RS_NONE);
        } else if (us->state != ALINK_USB_ST_RATE_LOCK) {
            set_state(us, ALINK_USB_ST_RATE_LOCK,
                      us->mode == ALINK_USB_MODE_PHASE ? ALINK_USB_RS_CONFIG : ALINK_USB_RS_LOCKED);
        }
    }
    event_service(us, now_ms);
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

void alink_usb_get_status(alink_usb_t *us, uint8_t o[ALINK_USB_STATUS_LEN]) {
    memset(o, 0, ALINK_USB_STATUS_LEN);
    o[0] = us->state;
    o[1] = us->started || us->prefilling ? us->ph.state : ALINK_PHASE_FREE;
    o[2] = us->fs == 44100u ? 0 : us->fs == 48000u ? 1 : us->fs == 96000u ? 2 : 0xFF;
    uint8_t f = 0;
#if defined(ALINK_USB_HW_TIMESTAMP) && ALINK_USB_HW_TIMESTAMP
    f |= 0x01;
#endif
    if (us->stream) f |= 0x02;
    if (alink_usb_nominal_feedback(us)) f |= 0x04;
    if (us->tb_ok) f |= 0x08;
    if (us->dark) f |= 0x10;                    // holdover: no SOF, extrapolating (WP USB2)
    if (us->seed_used) f |= 0x20;               // the VCXO started at the stored host rate (WP USB3)
    if (us->lock_err) f |= 0x40;                // mode 2 in lock error (WP USB3b)
    o[3] = f;
    au_putf(o + 4, us->est.valid ? us->est.ppm : 0.0f);
    au_putf(o + 8, alink_usb_port_vcxo_ppm());
    au_putf(o + 12, us->started && (us->ph.state == ALINK_PHASE_LOCKED || us->ph.state == ALINK_PHASE_PENDING) &&
                         us->ph.e_valid ? us->ph.e : 0.0f);
    uint64_t s = 0;
    uint16_t bus = 0;
    if (us->tb_ok) alink_usb_sample_now(us, &s, &bus);
    au_put32(o + 16, (uint32_t)s);
    au_put32(o + 20, (uint32_t)(s >> 32));
    au_put16(o + 24, bus);
    au_put16(o + 26, us->d_min);
    au_put16(o + 28, us->d_max);
    float fill = 0.0f;
    if (!alink_usb_port_play(&fill) || fill < 0.0f) fill = 0.0f;
    au_put16(o + 30, (uint32_t)fill);
    if (us->stream_seen) {
        au_put16(o + 32, us->stream_first & 0x7FFu);
        au_put32(o + 36, alink_usb_tb.n - us->stream_first);
        au_put32(o + 40, (uint32_t)us->stream_shift);
        au_put32(o + 44, us->stream_samples);
    } else {
        au_put16(o + 32, 0xFFFF);
    }
    au_put16(o + 34, us->gaps);                // u16, wraps
    au_put32(o + 48, us->anchors);
    au_put32(o + 52, us->reanchors);
    au_put32(o + 56, us->missed);
    au_put32(o + 60, us->late);
}

// ---------------------------------------------------------------------------
// Diagnostics (WP USB2)
// ---------------------------------------------------------------------------

void alink_usb_get_diag(const alink_usb_t *us, uint8_t o[ALINK_USB_DIAG_LEN]) {
    const alink_usb_diag_t *d = &us->diag;
    memset(o, 0, ALINK_USB_DIAG_LEN);
    o[0] = 2;                                   // layout version (2: WP USB3b, 44-47 changed)
    o[1] = d->last_cause;
    au_put16(o + 2, d->rides);
    au_put16(o + 4, d->holdovers);
    for (int i = 0; i < ALINK_USB_END_N; i++) au_put16(o + 6 + 2 * i, d->ends[i]);
    // 6 + 2 * 9 = 24
    au_put16(o + 24, d->suspends);
    au_put16(o + 26, d->resumes);
    au_put16(o + 28, d->mounts);
    au_put16(o + 30, d->umounts);
    // v2 (WP USB3b): the last absence's frames as u16 (saturated) and the
    // repaired SOF readings.
    au_put16(o + 32, d->last_frames < 0xFFFFu ? d->last_frames : 0xFFFFu);
    au_put16(o + 34, d->bad_sof);
    au_put32(o + 36, d->last_us);
    au_put32(o + 40, (uint32_t)d->last_dev_us);
    // v2: the longest ride in ms (u16: rides end at 10 s) and the lock-error count.
    uint32_t lr = d->longest_ride_us / 1000u;
    au_put16(o + 44, lr < 0xFFFFu ? lr : 0xFFFFu);
    au_put16(o + 46, d->lock_errors);
    au_put32(o + 48, d->max_pass_us);
    au_put32(o + 52, d->max_isr_quiet_us);
    au_put32(o + 56, d->max_isr_late_us);
    au_put32(o + 60, us->dark ? alink_usb_port_now_us() - us->dark_us : 0u);
}

bool alink_usb_clear_diag(alink_usb_t *us, uint8_t v) {
    if (v != 1u) return false;
    uint32_t last = us->diag.last_pass_us;
    memset(&us->diag, 0, sizeof(us->diag));
    us->diag.last_cause = 0xFF;
    us->diag.last_pass_us = last;
    return true;
}
