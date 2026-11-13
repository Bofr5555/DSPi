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

void alink_usb_est_sample(alink_usb_est_t *e, uint32_t n, uint32_t t) {
    if (e->seeded && n == e->last_n) return;
    // Seed, or reseed after a gap without clean pairs (over 2 s): the anchors
    // stay exact either way, but a long gap is where the frame extension can
    // be wrong (alink_usb_tb_t.long_gap).
    if (!e->seeded || n - e->last_n > 2048u) {
        e->n[0] = e->n[1] = e->last_n = n;
        e->t[0] = e->t[1] = t;
        e->seeded = true;
        e->valid = false;
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
    if (span < ALINK_USB_EST_MIN) return;
    uint32_t dt = t - e->t[0];
    int64_t num = (int64_t)span * ALINK_USB_T48_FRAME - (int64_t)dt;
    float ppm = (float)num * 1e6f / (float)dt;
    if (ppm > 2000.0f || ppm < -2000.0f) {      // garbage (a wrong extension): start over
        alink_usb_est_reset(e);
        return;
    }
    e->ppm = ppm;
    e->valid = true;
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
    link_phase_init(&us->ph);
    us->ph.mode = 0;
    us->ph.d_local = us->d;
    link_apply_init(&us->aq);
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
    // The rings run D - lag samples ahead of the arrival, lag being how late
    // a packet is processed after its SOF (up to a frame on the wire plus a
    // main-loop pass).  The hold keeps lag <= D - cap - 16 by writing silence,
    // so D must leave room for one and a half frames of lag above that.
    uint32_t lo = spf + spf / 2u + cap + 2u * ALINK_USB_DECODE_MARGIN;
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
    if (!alink_usb_phase_possible(us, fs)) return target;
    // D less the block cap and the margin: the hold threshold at the start.
    return link_phase_prefill(&us->ph, target, cap, span);
}

static void timeline_end(alink_usb_t *us);

bool alink_usb_set_config(alink_usb_t *us, const uint8_t p[ALINK_USB_CFG_LEN]) {
    uint8_t mode = p[0], flags = p[1];
    uint16_t d = au_get16(p + 2);
    if (mode > ALINK_USB_MODE_PHASE || (flags & ~ALINK_USB_FLAG_AUTO_REANCHOR) || p[4] || p[5]) return false;
    if (d == 0) return false;
    if (mode == ALINK_USB_MODE_PHASE && (d < us->d_min || d > us->d_max)) return false;
    bool d_changed = d != us->d;
    uint8_t was = us->mode;
    us->mode = mode;
    us->flags = flags;
    us->d = d;
    sync_ph(us);
    // A running timeline that the new values move: re-anchor (main loop).
    if ((us->started || us->prefilling) && (mode != was || d_changed)) {
        timeline_end(us);
        us->reanchors++;
        us->ev_pending = true;
        us->ev_reason = ALINK_USB_RS_REANCHOR;
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
    *s = (uint64_t)us->spf * nc + (uint64_t)(frac > 0.0f ? frac : 0.0f);
    if (bus) *bus = (uint16_t)(n & 0x7FFu);
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
    }
    if (reason != ALINK_USB_RS_NONE) {
        us->ev_reason = reason;
        us->ev_pending = true;
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
    us->ev_last_ms = now_ms;
    us->ev_once = true;
}

// ---------------------------------------------------------------------------
// Apply at frame
// ---------------------------------------------------------------------------

static void apply_report(alink_usb_t *us, uint8_t op, uint8_t result, uint64_t s) {
    link_apply_done(&us->aq, op, result, s);
    alink_usb_port_apply_result(op, result, alink_usb_bus_form(us, s));
}

static void apply_run(alink_usb_t *us, bool late) {
    link_apply_op_t op;
    link_apply_pop(&us->aq, &op);
    uint8_t r = alink_usb_port_apply(&op);
    if (late && r == LINK_APPLY_APPLIED) r = LINK_APPLY_LATE;
    if (op.op == LINK_OP_SYNC_PULSE && r != LINK_APPLY_REFUSED) us->sync_pulses++;
    apply_report(us, op.op, r, op.frame);
}

void alink_usb_cancel_all(alink_usb_t *us) {
    while (us->aq.n) {
        link_apply_op_t op;
        link_apply_pop(&us->aq, &op);
        apply_report(us, op.op, LINK_APPLY_CANCELLED, op.frame);
    }
}

bool alink_usb_apply(alink_usb_t *us, const uint8_t p[LINK_APPLY_LEN]) {
    uint8_t q[LINK_APPLY_LEN];
    memcpy(q, p, LINK_APPLY_LEN);
    uint8_t tb = q[12];
    q[12] = 0;
    // The limits of link_apply_args_ok(); the firmware's values (presets,
    // outputs, delay) are the same on the link.
    link_apply_op_t op;
    if (!link_apply_parse(&alink_usb_apply_limits, q, &op)) return false;
    if (op.op == LINK_APPLY_CANCEL_ALL) {
        if (tb > 2u) return false;
        alink_usb_cancel_all(us);
        return true;
    }
    if (tb != 1u && tb != 2u) return false;
    if (!us->started || us->ph.state == LINK_PHASE_ANCHORING) return false;      // no timeline, no S
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
    if (op.op == LINK_OP_SYNC_PULSE) (void)alink_usb_port_pulse((uint32_t)(op.frame / us->spf));
    return link_apply_push(&us->aq, &op);
}

// ---------------------------------------------------------------------------
// Timeline writes
// ---------------------------------------------------------------------------

// n samples at next_w: the packet from `off`, or silence (p NULL).  Split
// where an apply op is due (it acts on exactly its sample) and at the block cap.
static void write_span(alink_usb_t *us, const alink_usb_pkt_t *p, uint32_t off, uint32_t n) {
    uint32_t done = 0, cap = us->block_cap ? us->block_cap : us->spf;
    while (done < n) {
        uint64_t here = us->ph.next_w, at = link_apply_next(&us->aq);
        if (at <= here) {
            apply_run(us, at < here);
            continue;
        }
        uint32_t k = n - done;
        if (k > cap) k = cap;
        if (at - here < k) k = (uint32_t)(at - here);
        if (p) alink_usb_port_write_packet(p, off + done, k);
        else alink_usb_port_write_silence(k);
        link_phase_written(&us->ph, here, k);
        done += k;
    }
}

static void timeline_end(alink_usb_t *us) {
    us->started = false;
    us->prefilling = false;
    us->muted_gap = false;
    us->pkt_off = 0;
    link_phase_drain(&us->ph);
}

static void reanchor(alink_usb_t *us) {
    timeline_end(us);
    us->reanchors++;
    uint32_t fs = us->fs ? us->fs : 48000u;
    alink_usb_port_soft_mute(fs / 1000u * ALINK_USB_ANCHOR_MUTE_MS);
    set_state(us, ALINK_USB_ST_ANCHORING, ALINK_USB_RS_REANCHOR);
}

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
    bool first = us->stream && !us->stream_seen;
    while (off < n) {
        int64_t gap = (int64_t)(s - us->ph.next_w);
        if (!link_phase_gap_ok(gap, span)) {
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
            write_span(us, NULL, 0, k);
            if (!first && us->stream_seen) {
                us->missed += k;
                us->stream_shift += (int32_t)k;
            }
            continue;
        }
        if (first) {
            us->stream_seen = true;
            us->stream_first = (uint32_t)(s / us->spf);
            us->stream_shift = 0;
            us->stream_samples = 0;
            us->muted_gap = true;
            first = false;
        }
        if (us->muted_gap) {
            // The stream (re)starts on silence: one packet muted, then the
            // pipeline's 8 ms fade in.
            alink_usb_port_soft_mute(us->spf);
            us->muted_gap = false;
        }
        uint32_t k = n - off;
        write_span(us, p, off, k);
        us->stream_samples += k;
        off += k;
        s += k;
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
    *e = link_phase_error(&us->ph, since, fill);
    return true;
}

static void phase_act(alink_usb_t *us, int act, int32_t k) {
    if (act == LINK_PHASE_ACT_REANCHOR) {
        reanchor(us);
    } else if (act == LINK_PHASE_ACT_CORRECT) {
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

// Keep the timeline running on silence while no packet comes: never let the
// rings fall below D - (block cap + margin) behind the arrival.
static void hold(alink_usb_t *us) {
    float since;
    uint32_t sv = alink_usb_port_irq_save();
    bool ok = arrival(us, us->ph.next_w, &since);
    alink_usb_port_irq_restore(sv);
    if (!ok) return;
    float h = (float)us->d - (float)(us->block_cap + ALINK_USB_DECODE_MARGIN);
    if (since <= h) return;
    uint32_t k = (uint32_t)(since - h);
    if (!k) return;
    if (k > 4096u) {                            // the main loop stood still far too long
        reanchor(us);
        return;
    }
    write_span(us, NULL, 0, k);
    if (us->stream && us->stream_seen) {
        // A packet is missing while streaming: mute, fade back in.
        us->missed += k;
        us->stream_shift += (int32_t)k;
        if (!us->muted_gap) {
            us->gaps++;
            us->ev_pending = true;
            us->ev_reason = ALINK_USB_RS_GAP;
        }
        us->muted_gap = true;
        alink_usb_port_soft_mute(us->spf * ALINK_USB_GAP_MUTE_MS);
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
    return us->tb_ok && !us->rejected && us->rate_locked && !us->started && !us->prefilling &&
           alink_usb_phase_possible(us, us->fs);
}

void alink_usb_on_drain(alink_usb_t *us) {
    link_phase_drain(&us->ph);
    us->prefilling = true;
    us->started = false;
    us->pkt_off = 0;
    alink_usb_port_ring_target();                // prefill target = D - cap - margin
    uint64_t s;
    if (!alink_usb_sample_now(us, &s, NULL)) s = 0;
    us->ph.next_w = s;                          // the first silence written sets `first`
    us->ph.state = LINK_PHASE_ANCHORING;
    set_state(us, ALINK_USB_ST_ANCHORING, ALINK_USB_RS_NONE);
}

static void prefill(alink_usb_t *us) {
    uint32_t room = alink_usb_port_room();
    if (room) write_span(us, NULL, 0, room);
}

float alink_usb_anchor_wait(alink_usb_t *us) {
    if (!us->prefilling || !us->ph.timeline) return 1e9f;
    if (alink_usb_port_room()) return 1e9f;      // not at the prefill target yet
    float since;
    uint32_t sv = alink_usb_port_irq_save();
    bool ok = arrival(us, us->ph.first, &since);
    alink_usb_port_irq_restore(sv);
    if (!ok) return 1e9f;
    return link_phase_anchor_wait(&us->ph, since);
}

void alink_usb_on_enabled(alink_usb_t *us) {
    link_phase_started(&us->ph);
    us->prefilling = false;
    us->started = true;
    us->anchors++;
    us->stream_seen = false;                    // a running stream is placed anew
}

void alink_usb_outputs_reset(alink_usb_t *us) {
    // Our own drain (main.c) uses drain_and_disable_outputs(), not this.
    if (us->started || us->prefilling) reanchor(us);
}

void alink_usb_stream(alink_usb_t *us, bool on) {
    us->stream = on;
    us->stream_seen = false;
    if (!on) us->muted_gap = false;
}

void alink_usb_stop(alink_usb_t *us, uint8_t reason) {
    alink_usb_cancel_all(us);
    timeline_end(us);
    bool was = us->state != ALINK_USB_ST_OFF;
    us->rejected = false;
    us->tb_ok = false;
    us->ticked = false;
    us->rate_locked = false;
    alink_usb_est_reset(&us->est);
    if (was) {
        alink_usb_port_park();
        set_state(us, ALINK_USB_ST_OFF, reason);
    }
}

// ---------------------------------------------------------------------------
// Service
// ---------------------------------------------------------------------------

void alink_usb_service(alink_usb_t *us, uint32_t now_ms) {
    if (us->mode == ALINK_USB_MODE_OFF) {
        if (us->state != ALINK_USB_ST_OFF) alink_usb_stop(us, ALINK_USB_RS_DISABLED);
        event_service(us, now_ms);
        return;
    }
    uint32_t fs = alink_usb_port_fs();
    if (!fs) fs = 48000u;

    // Time base.
    uint32_t sv = alink_usb_port_irq_save();
    bool valid = alink_usb_tb.valid, have_c = alink_usb_tb.have_c, long_gap = alink_usb_tb.long_gap;
    uint32_t t = alink_usb_tb.t, n_c = alink_usb_tb.n_c, t_c = alink_usb_tb.t_c;
    alink_usb_tb.long_gap = false;
    uint32_t now = alink_usb_port_now48();
    alink_usb_port_irq_restore(sv);
    // Signed: the main loop reads whole microseconds, so a SOF stamped exactly
    // (RP2350) can be a few ticks "after" now.
    bool sof_ok = valid && (int32_t)(now - t) < (int32_t)(ALINK_USB_SOF_LOST_MS * ALINK_USB_T48_FRAME);
    if (!sof_ok || long_gap) {
        if (us->tb_ok || us->state != ALINK_USB_ST_NO_SOF) {
            if (us->started || us->prefilling) alink_usb_cancel_all(us);
            timeline_end(us);
            us->tb_ok = false;
            us->ticked = false;
    us->rate_locked = false;
            alink_usb_est_reset(&us->est);
            set_state(us, ALINK_USB_ST_NO_SOF, us->state == ALINK_USB_ST_OFF ? ALINK_USB_RS_NONE : ALINK_USB_RS_SOF_LOST);
        }
        event_service(us, now_ms);
        return;                                 // the VCXO holds its trim
    }
    us->tb_ok = true;
    if (have_c) alink_usb_est_sample(&us->est, n_c, t_c);

    // Host range.
    if (us->est.valid) {
        float a = fabs_(us->est.ppm);
        if (!us->rejected && a > ALINK_USB_HOST_MAX_PPM) {
            alink_usb_cancel_all(us);
            timeline_end(us);
            us->rejected = true;
            us->ticked = false;
    us->rate_locked = false;
            alink_usb_port_park();
            set_state(us, ALINK_USB_ST_REJECTED, ALINK_USB_RS_HOST_RANGE);
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
                int act = LINK_PHASE_ACT_NONE;
                int32_t k = 0;
                if (us->ph.state != LINK_PHASE_ANCHORING) {
                    act = link_phase_check(&us->ph, e, auto_reanchor(us), &k);
                    phase_act(us, act, k);
                }
                if (act == LINK_PHASE_ACT_REANCHOR) link_phase_pi_reset(&us->ph.pi);
                else if (us->ph.state == LINK_PHASE_LOCKED || us->ph.state == LINK_PHASE_PENDING)
                    trim = link_phase_servo(&us->ph, e);
            }
        } else if (us->rate_locked && !us->prefilling) {
            ff_only = false;
        }
        alink_usb_port_actuate(hz, trim, ff_only);
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
        if (us->ph.state == LINK_PHASE_ANCHORING) {
            float e;
            if (phase_error(us, &e)) {
                int32_t k = 0;
                int act = link_phase_check(&us->ph, e, auto_reanchor(us), &k);
                phase_act(us, act, k);
            }
        }
        if (us->started) hold(us);
        if (us->started && us->ph.state == LINK_PHASE_ANCHORING) {
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
    o[1] = us->started || us->prefilling ? us->ph.state : LINK_PHASE_FREE;
    o[2] = us->fs == 44100u ? 0 : us->fs == 48000u ? 1 : us->fs == 96000u ? 2 : 0xFF;
    uint8_t f = 0;
#if defined(ALINK_USB_HW_TIMESTAMP) && ALINK_USB_HW_TIMESTAMP
    f |= 0x01;
#endif
    if (us->stream) f |= 0x02;
    if (alink_usb_nominal_feedback(us)) f |= 0x04;
    if (us->tb_ok) f |= 0x08;
    o[3] = f;
    au_putf(o + 4, us->est.valid ? us->est.ppm : 0.0f);
    au_putf(o + 8, alink_usb_port_vcxo_ppm());
    au_putf(o + 12, us->started && (us->ph.state == LINK_PHASE_LOCKED || us->ph.state == LINK_PHASE_PENDING) &&
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
    au_put32(o + 48, us->anchors);
    au_put32(o + 52, us->reanchors);
    au_put32(o + 56, us->missed);
    au_put32(o + 60, us->late);
}
