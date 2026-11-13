/*
 * alink_phase.c - DSPi alink protocol v1: phase lock and apply-at-frame (WP LK6)
 *
 * See alink_phase.h. Everything here runs off the per-sample path (control
 * plane, servo tick, anchor, loss), so the firmware leaves it in flash.
 */

#include "alink_phase.h"

#include <string.h>

static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

void alink_phase_init(alink_phase_t *ph) {
    memset(ph, 0, sizeof(*ph));
    ph->mode = 1;
    ph->d_local = ALINK_PHASE_D_DEFAULT;
    ph->d_min = 0;
    ph->d_max = 0xFFFF;
}

void alink_phase_limits(alink_phase_t *ph, uint32_t target, uint32_t block_cap, uint32_t span) {
    uint32_t lo = target + block_cap + ALINK_PHASE_DECODE_MARGIN;
    uint32_t hi = span > block_cap ? span - block_cap : 0;
    ph->d_min = (uint16_t)(lo < 0xFFFFu ? lo : 0xFFFFu);
    ph->d_max = (uint16_t)(hi < 0xFFFFu ? hi : 0xFFFFu);
}

bool alink_phase_active(const alink_phase_t *ph) {
    int32_t d = alink_phase_delay(ph);
    return ph->mode == 1 && d >= ph->d_min && d <= ph->d_max;
}

uint32_t alink_phase_prefill(const alink_phase_t *ph, uint32_t target, uint32_t block_cap, uint32_t span) {
    if (!alink_phase_active(ph)) return target;
    int32_t w = alink_phase_delay(ph) - (int32_t)(block_cap + ALINK_PHASE_DECODE_MARGIN);
    int32_t ceil = (int32_t)span - (int32_t)block_cap;
    if (w > ceil) w = ceil;
    if (w < (int32_t)target) w = (int32_t)target;
    return (uint32_t)w;
}

int alink_phase_set(alink_phase_t *ph, const uint8_t p[ALINK_PHASE_LEN]) {
    uint8_t mode = p[0];
    uint16_t d = get16(p + 2);
    int16_t hops = (int16_t)get16(p + 4);
    int32_t eff = (int32_t)d + hops;
    if (mode > 1 || p[1] || p[6] || hops < -ALINK_PHASE_HOPS_MAX || hops > ALINK_PHASE_HOPS_MAX) return -1;
    if (eff < ph->d_min || eff > ph->d_max) return -1;
    bool changed = mode != ph->mode || eff != alink_phase_delay(ph);
    ph->mode = mode;
    ph->d_local = d;
    ph->hops_offset = hops;
    return changed ? 1 : 0;
}

void alink_phase_get(const alink_phase_t *ph, uint8_t o[ALINK_PHASE_LEN]) {
    memset(o, 0, ALINK_PHASE_LEN);
    o[0] = ph->mode;
    put16(o + 2, ph->d_local);
    put16(o + 4, (uint16_t)ph->hops_offset);
    if (ph->mode == 1 && alink_phase_delay(ph) < ph->d_min) o[6] = ALINK_PHASE_FLAG_BELOW_MIN;
    o[7] = ph->state;
}

void alink_phase_drain(alink_phase_t *ph) {
    ph->timeline = false;
    ph->started = false;
    ph->hold = false;
    ph->resync = false;
    ph->corrections = 0;
    ph->e_valid = false;
    ph->state = alink_phase_active(ph) ? ALINK_PHASE_ANCHORING : ALINK_PHASE_FREE;
}

void alink_phase_started(alink_phase_t *ph) {
    ph->started = true;
    ph->corrections = 0;
    ph->e_valid = false;
    alink_phase_pi_reset(&ph->pi);
    ph->state = alink_phase_active(ph) ? ALINK_PHASE_ANCHORING : ALINK_PHASE_FREE;
}

static int32_t round_frames(float e) {
    return (int32_t)(e < 0.0f ? e - 0.5f : e + 0.5f);
}

int alink_phase_check(alink_phase_t *ph, float e, bool auto_reanchor, int32_t *frames) {
    ph->e = e;
    ph->e_valid = true;
    float a = e < 0.0f ? -e : e;
    switch (ph->state) {
    case ALINK_PHASE_ANCHORING:
        if (a < ALINK_PHASE_LOCK_FRAMES) {
            ph->state = ALINK_PHASE_LOCKED;
            return ALINK_PHASE_ACT_RESUME;
        }
        if (ph->corrections < ALINK_PHASE_MAX_CORRECTIONS && a <= (float)ALINK_PHASE_CORRECT_MAX) {
            ph->corrections++;
            *frames = round_frames(e);
            return ALINK_PHASE_ACT_CORRECT;
        }
        if (auto_reanchor) return ALINK_PHASE_ACT_REANCHOR;
        ph->state = ALINK_PHASE_PENDING;    // play, and let the servo pull it in
        return ALINK_PHASE_ACT_RESUME;
    case ALINK_PHASE_LOCKED:
        if (a <= ALINK_PHASE_REANCHOR_FRAMES) return ALINK_PHASE_ACT_NONE;
        if (auto_reanchor) return ALINK_PHASE_ACT_REANCHOR;
        ph->state = ALINK_PHASE_PENDING;
        return ALINK_PHASE_ACT_NONE;
    case ALINK_PHASE_PENDING:
        if (a < ALINK_PHASE_LOCK_FRAMES) ph->state = ALINK_PHASE_LOCKED;
        return ALINK_PHASE_ACT_NONE;
    default:
        return ALINK_PHASE_ACT_NONE;
    }
}

bool alink_phase_loss(alink_phase_t *ph, uint32_t fill) {
    if (!ph->started || !ph->timeline || ph->state == ALINK_PHASE_FREE) return false;
    ph->hold = true;
    ph->resync = false;
    ph->hold_fill = fill;
    ph->e_valid = false;
    return true;
}

bool alink_phase_relock(alink_phase_t *ph, uint64_t f, uint64_t f_expect, uint32_t span) {
    ph->hold = false;
    if (!alink_phase_gap_ok((int64_t)(f - f_expect), span)) {
        ph->resync = false;
        ph->started = false;
        ph->e_valid = false;
        ph->state = ALINK_PHASE_ANCHORING;
        return false;
    }
    ph->resync = true;
    ph->corrections = 0;
    ph->e_valid = false;
    ph->state = ALINK_PHASE_ANCHORING;
    return true;
}

void alink_phase_pi_reset(alink_phase_pi_t *pi) {
    memset(pi, 0, sizeof(*pi));
}

float alink_phase_servo(alink_phase_t *ph, float e) {
    alink_phase_pi_t *pi = &ph->pi;
    if (!pi->seeded) {
        pi->e_s = e;
        pi->seeded = true;
    } else {
        pi->e_s += ALINK_PHASE_ALPHA * (e - pi->e_s);
    }
    pi->integ += pi->e_s * ALINK_PHASE_KI_PPM;
    if (pi->integ > ALINK_PHASE_ICLAMP_PPM) pi->integ = ALINK_PHASE_ICLAMP_PPM;
    if (pi->integ < -ALINK_PHASE_ICLAMP_PPM) pi->integ = -ALINK_PHASE_ICLAMP_PPM;
    float t = pi->e_s * ALINK_PHASE_KP_PPM + pi->integ;
    if (t > ALINK_PHASE_TRIM_MAX_PPM) t = ALINK_PHASE_TRIM_MAX_PPM;
    if (t < -ALINK_PHASE_TRIM_MAX_PPM) t = -ALINK_PHASE_TRIM_MAX_PPM;
    pi->trim = t;
    return t;
}

// ---------------------------------------------------------------------------
// Apply at frame N
// ---------------------------------------------------------------------------

void alink_apply_init(alink_apply_q_t *q) {
    memset(q, 0, sizeof(*q));
}

bool alink_apply_args_ok(const alink_apply_limits_t *lim, uint8_t op, uint8_t arg, int16_t value) {
    switch (op) {
    case ALINK_OP_MUTE:
    case ALINK_OP_UNMUTE:
    case ALINK_OP_SYNC_PULSE:
        return arg == 0 && value == 0;
    case ALINK_OP_PRESET_LOAD:
        return arg < lim->presets && value == 0;
    case ALINK_OP_MASTER_VOLUME:
        return arg == 0 && value >= -12800 && value <= 0;
    case ALINK_OP_OUT_DELAY:
        return arg < lim->outputs && value >= 0 && value < (int32_t)lim->max_delay;
    case ALINK_OP_CLICK:
        return arg < lim->outputs && value == 0;
    default:
        return false;
    }
}

bool alink_apply_parse(const alink_apply_limits_t *lim, const uint8_t p[ALINK_APPLY_LEN], alink_apply_op_t *op) {
    if (p[12] | p[13] | p[14] | p[15]) return false;
    uint64_t f = 0;
    for (int i = 7; i >= 0; i--) f = f << 8 | p[i];
    op->frame = f;
    op->op = p[8];
    op->arg = p[9];
    op->value = (int16_t)get16(p + 10);
    if (op->op == ALINK_APPLY_CANCEL_ALL) return op->arg == 0 && op->value == 0;
    return alink_apply_args_ok(lim, op->op, op->arg, op->value);
}

bool alink_apply_push(alink_apply_q_t *q, const alink_apply_op_t *op) {
    if (q->n >= ALINK_APPLY_DEPTH) return false;
    unsigned i = q->n;
    while (i && q->q[i - 1].frame > op->frame) {
        q->q[i] = q->q[i - 1];
        i--;
    }
    q->q[i] = *op;
    q->n++;
    return true;
}

void alink_apply_pop(alink_apply_q_t *q, alink_apply_op_t *op) {
    *op = q->q[0];
    q->n--;
    memmove(&q->q[0], &q->q[1], q->n * sizeof(q->q[0]));
}

void alink_apply_done(alink_apply_q_t *q, uint8_t op, uint8_t result, uint64_t frame) {
    q->last_op = op;
    q->last_result = result;
    q->last_frame = frame;
}

int alink_apply_take_sf(alink_apply_q_t *q, const alink_apply_limits_t *lim, const alink_sf_t *sf) {
    if (sf->apply_op == ALINK_OP_NONE || sf->apply_op == ALINK_OP_RATE || sf->apply_frame == 0) return 0;
    if (q->sf_have && q->sf.frame == sf->apply_frame && q->sf.op == sf->apply_op && q->sf.arg == sf->apply_arg &&
        q->sf.value == sf->apply_value)
        return 0;
    alink_apply_op_t op = { sf->apply_frame, sf->apply_op, sf->apply_arg, sf->apply_value };
    q->sf = op;
    q->sf_have = true;
    if (!alink_apply_args_ok(lim, op.op, op.arg, op.value) || !alink_apply_push(q, &op)) return 2;
    return 1;
}

void alink_apply_in(const alink_apply_q_t *q, uint8_t o[ALINK_APPLY_LEN]) {
    memset(o, 0, ALINK_APPLY_LEN);
    o[0] = q->n;
    o[1] = q->last_op;
    o[2] = q->last_result;
    for (int i = 0; i < 8; i++) o[4 + i] = (uint8_t)(q->last_frame >> (8 * i));
}
