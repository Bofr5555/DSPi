/*
 * alink_spdif.c - S/PDIF as an alink time-base transport: the roles (WP FWSP2)
 *
 * See alink_spdif.h.  Portable (the host test and tools/alink_spdif_sim
 * include this file); the firmware's port is alink_spdif_port.c.  Flash.
 */

#include "alink_spdif.h"

#include <string.h>

static uint16_t sp_get16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static void sp_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void sp_put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void sp_put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void sp_putf(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); sp_put32(p, u); }

static void tb_note(alink_spdif_t *u, uint8_t reason);
static void tb_role(void);
static void reanchor(alink_spdif_t *u, uint8_t reason);
static void apply_report(alink_spdif_t *u, uint8_t op, uint8_t result, uint64_t frame);

void alink_spdif_init(alink_spdif_t *u, uint8_t serial0, uint8_t serial1) {
    memset(u, 0, sizeof(*u));
    u->mode = ALINK_SPDIF_MODE_OFF;
    u->flags = ALINK_SPDIF_FLAGS_DEFAULT;
    u->d = ALINK_SPDIF_D_DEFAULT;
    u->on_lock_error = ALINK_SPDIF_LOCKERR_MUTE;
    u->serial0 = serial0;
    u->serial1 = serial1;
    tb_track_init(&u->tbk);
    alink_apply_init(&u->aq);
    alink_phase_init(&u->ph);
    u->ph.mode = 0;
    u->ph.d_local = u->d;
    u->ph.d_min = 0;
    u->ph.d_max = 0xFFFF;
    u->src.sender = serial0;
    u->src.sender2 = serial1;
    u->rx_rate = ALINK_SPDIF_RATE_UNKNOWN;
    u->fs = 48000u;
    u->fifo_words = 8u * ALINK_SPDIF_BLOCK_WORDS;
}

// ---------------------------------------------------------------------------
// Configuration (2.10.1)
// ---------------------------------------------------------------------------

void alink_spdif_limits(alink_spdif_t *u, uint32_t cap, uint32_t batch, uint32_t span) {
    uint32_t lo = cap + batch + ALINK_SPDIF_RX_MARGIN;
    uint32_t hi = span > cap ? span - cap : 0;
    u->ph.d_min = (uint16_t)(lo < 0xFFFFu ? lo : 0xFFFFu);
    u->ph.d_max = (uint16_t)(hi < 0xFFFFu ? hi : 0xFFFFu);
}

uint32_t alink_spdif_ring_target(const alink_spdif_t *u, uint32_t target, uint32_t cap, uint32_t batch,
                                 uint32_t span) {
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER || !alink_phase_active(&u->ph)) return target;
    int32_t w = (int32_t)u->d - (int32_t)(batch + ALINK_SPDIF_RX_MARGIN / 2u);
    int32_t ceil = (int32_t)span - (int32_t)cap;
    if (w > ceil) w = ceil;
    if (w < (int32_t)cap) w = (int32_t)cap;
    return (uint32_t)w;
}

uint8_t alink_spdif_set_config(alink_spdif_t *u, const uint8_t p[ALINK_SPDIF_CFG_LEN], uint8_t spdif_slots) {
    uint8_t mode = p[0], flags = p[1], ole = p[4], mask = p[6];
    uint16_t d = sp_get16(p + 2);
    if (mode > ALINK_SPDIF_MODE_SOURCE) return TB_ERR_VALUE;
    if (flags & ~ALINK_SPDIF_FLAG_AUTO_REANCHOR) return TB_ERR_VALUE;
    if (ole > ALINK_SPDIF_LOCKERR_PLAY || p[5]) return TB_ERR_VALUE;
    if (mode == ALINK_SPDIF_MODE_FOLLOWER && (d < u->ph.d_min || d > u->ph.d_max)) return TB_ERR_VALUE;
    if (mode == ALINK_SPDIF_MODE_SOURCE) {
        if (!spdif_slots) return ALINK_SPDIF_ERR_UNSUPPORTED;
        if (mask & ~spdif_slots) return TB_ERR_VALUE;
    } else if (mask) {
        return TB_ERR_VALUE;                // tx_mask is the source's: 0 otherwise
    }
    bool same = mode == u->mode && flags == u->flags && d == u->d && ole == u->on_lock_error && mask == u->tx_mask;
    if (same) return TB_OK;                 // rewriting the same values does nothing
    uint8_t was = u->mode;
    bool relock = was == ALINK_SPDIF_MODE_FOLLOWER && mode == ALINK_SPDIF_MODE_FOLLOWER && d != u->d;
    if (was != mode) {
        // Leaving a role: its queue is cancelled and the time base ends here.
        alink_spdif_stop(u, mode == ALINK_SPDIF_MODE_OFF ? TB_RS_DISABLED : TB_RS_NONE);
    }
    u->mode = mode;
    u->flags = flags;
    u->d = d;
    u->on_lock_error = ole;
    u->tx_mask = mask;
    u->ph.mode = mode == ALINK_SPDIF_MODE_FOLLOWER ? 1u : 0u;
    u->ph.d_local = d;
    u->ph.hops_offset = 0;
    if (relock && (u->ph.started || u->prefilling)) reanchor(u, TB_RS_REANCHOR);   // a new D while locked
    tb_role();
    return TB_OK;
}

void alink_spdif_get_config(const alink_spdif_t *u, uint8_t o[ALINK_SPDIF_CFG_LEN]) {
    memset(o, 0, ALINK_SPDIF_CFG_LEN);
    o[0] = u->mode;
    o[1] = u->flags;
    sp_put16(o + 2, u->d);
    o[4] = u->on_lock_error;
    o[6] = u->tx_mask;
    o[7] = alink_spdif_state(u);
}

void alink_spdif_store(const alink_spdif_t *u, uint8_t s[ALINK_SPDIF_STORE_LEN]) {
    s[0] = u->mode;
    s[1] = u->flags;
    sp_put16(s + 2, u->d);
    s[4] = u->on_lock_error;
    s[5] = u->tx_mask;
}

void alink_spdif_load(alink_spdif_t *u, const uint8_t s[ALINK_SPDIF_STORE_LEN]) {
    // The limits are not known at boot: D is checked when mode 1 runs
    // (out of range there is a lock error, CONFIG).
    if (s[0] > ALINK_SPDIF_MODE_SOURCE || (s[1] & ~ALINK_SPDIF_FLAG_AUTO_REANCHOR) || s[4] > 1u) return;
    if (s[0] != ALINK_SPDIF_MODE_SOURCE && s[5]) return;
    u->mode = s[0];
    u->flags = s[1];
    u->d = sp_get16(s + 2);
    u->on_lock_error = s[4];
    u->tx_mask = s[5];
    u->ph.mode = u->mode == ALINK_SPDIF_MODE_FOLLOWER ? 1u : 0u;
    u->ph.d_local = u->d;
}

// ---------------------------------------------------------------------------
// The contract (11.2)
// ---------------------------------------------------------------------------

// Mode 1 that cannot lock: no valid superframe for 0.5 s on a locked stream,
// or D outside d_min..d_max (CONFIG).
static bool lock_error_now(const alink_spdif_t *u) {
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER || !u->active) return false;
    return u->lock_err || (u->rx_locked && !alink_phase_active(&u->ph));
}

bool alink_spdif_unaligned(const alink_spdif_t *u) {
    return lock_error_now(u) && u->on_lock_error == ALINK_SPDIF_LOCKERR_PLAY;
}

bool alink_spdif_engaged(const alink_spdif_t *u) {
    return u->mode == ALINK_SPDIF_MODE_FOLLOWER && u->active && !alink_spdif_unaligned(u);
}

uint8_t alink_spdif_state(const alink_spdif_t *u) {
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) return u->src_running ? TB_ST_LOCKED : TB_ST_ANCHORING;
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER || !u->active) return TB_ST_NONE;
    if (lock_error_now(u)) return TB_ST_LOCK_ERROR;
    if (u->ph.hold) return TB_ST_HOLDOVER;
    if (u->resuming && u->ph.started) {
        if (u->ph.state != ALINK_PHASE_LOCKED || u->ph.resync) return TB_ST_HOLDOVER;
    }
    if (u->ph.started && u->ph.state == ALINK_PHASE_PENDING) return TB_ST_LOCK_ERROR;
    if (u->ph.started && u->ph.state == ALINK_PHASE_LOCKED && !u->ph.resync) return TB_ST_LOCKED;
    if (u->rd_valid && u->rx_locked) return TB_ST_ANCHORING;
    return TB_ST_ACQUIRING;
}

bool alink_spdif_muted(const alink_spdif_t *u) {
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER || !u->active) return false;
    if (alink_spdif_unaligned(u)) return false;
    return alink_spdif_state(u) != TB_ST_LOCKED;
}

static void tb_note(alink_spdif_t *u, uint8_t reason) {
    uint64_t t = u->mode == ALINK_SPDIF_MODE_SOURCE ? u->origin : u->ph.next_w;
    (void)tb_track_note(&u->tbk, alink_spdif_state(u), reason, (uint32_t)t);
}

// The tracker sees the state now (timebase_service() polls it too), and the
// lock and loss counts (2.10.2): entries into LOCKED, transitions out of it.
static void count_locks(alink_spdif_t *u) {
    tb_note(u, TB_RS_NONE);
    bool now = alink_spdif_state(u) == TB_ST_LOCKED;
    if (now && !u->was_locked) u->lock_count++;
    if (!now && u->was_locked) u->loss_count++;
    u->was_locked = now;
}

void alink_spdif_get_status(const alink_spdif_t *u, uint64_t f_now, float vcxo_ppm, uint16_t fill,
                            uint8_t o[ALINK_SPDIF_STATUS_LEN]) {
    memset(o, 0, ALINK_SPDIF_STATUS_LEN);
    uint8_t st = alink_spdif_state(u);
    o[0] = st;
    uint8_t fl = 0;
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) {
        o[2] = u->src.rate;
        fl |= ALINK_SPDIF_ST_SOURCE;
        if (st == TB_ST_LOCKED) fl |= ALINK_SPDIF_ST_TIME_BASE;
        sp_put16(o + 4, (uint16_t)(u->serial0 | u->serial1 << 8));
        o[6] = u->epoch;
        if (u->src_running) sp_put64(o + 8, f_now);
        sp_putf(o + 20, vcxo_ppm);
        sp_put16(o + 28, fill);
    } else if (u->mode == ALINK_SPDIF_MODE_FOLLOWER && u->active) {
        o[1] = u->ph.started ? u->ph.state : (u->prefilling ? ALINK_PHASE_ANCHORING : ALINK_PHASE_FREE);
        o[2] = u->rx_locked ? u->rx_rate : ALINK_SPDIF_RATE_UNKNOWN;
        bool recent = u->sf_seen && u->blocks_since_sf <= ALINK_SPDIF_SF_RECENT;
        if (recent) fl |= ALINK_SPDIF_ST_SUPERFRAME;
        if (st == TB_ST_LOCKED || st == TB_ST_HOLDOVER) fl |= ALINK_SPDIF_ST_TIME_BASE;
        if (u->ph.hold) fl |= ALINK_SPDIF_ST_HOLDOVER;
        if (st == TB_ST_LOCK_ERROR) fl |= ALINK_SPDIF_ST_LOCK_ERROR;
        if (u->rx_locked && !recent) fl |= ALINK_SPDIF_ST_FOREIGN;
        if (u->sf_seen) {
            sp_put16(o + 4, u->src_id);
            o[6] = u->src_epoch;
        }
        if (st == TB_ST_LOCKED || st == TB_ST_HOLDOVER || st == TB_ST_ANCHORING) sp_put64(o + 8, f_now);
        if (u->ph.started && (u->ph.state == ALINK_PHASE_LOCKED || u->ph.state == ALINK_PHASE_PENDING))
            sp_putf(o + 16, u->ph.e);
        sp_putf(o + 20, vcxo_ppm);
        sp_put16(o + 24, u->ph.d_min);
        sp_put16(o + 26, u->ph.d_max);
        sp_put16(o + 28, fill);
        sp_put32(o + 40, u->sf_errors);
        sp_put32(o + 44, u->reanchors);
    } else {
        o[2] = ALINK_SPDIF_RATE_UNKNOWN;
    }
    o[3] = fl;
    if (u->mode != ALINK_SPDIF_MODE_OFF) {
        sp_put32(o + 32, u->lock_count);
        sp_put32(o + 36, u->loss_count);
    }
}

// ---------------------------------------------------------------------------
// Apply queue of time base 3 (2.10.4)
// ---------------------------------------------------------------------------

static void apply_report(alink_spdif_t *u, uint8_t op, uint8_t result, uint64_t frame) {
    // A pending op 9 the role cancelled itself (it stopped) is a loss; one
    // the generator gave up was cleared there already (a no-op then).
    if (op == ALINK_OP_SIGGEN_START && result == ALINK_APPLY_CANCELLED) alink_spdif_port_op9_lost();
    alink_apply_done(&u->aq, op, result, frame);
    if (u->mode == ALINK_SPDIF_MODE_SOURCE && u->bc_last_frame == frame && u->bc_last_op == op)
        u->bc_last_result = result;
    alink_spdif_port_apply_result(op, result, (uint32_t)frame);
}

static void apply_run(alink_spdif_t *u, bool late) {
    alink_apply_op_t op;
    alink_apply_pop(&u->aq, &op);
    uint8_t r = alink_spdif_port_apply(&op, late, alink_spdif_state(u) == TB_ST_LOCKED && !u->sf_mute);
    if (late && r == ALINK_APPLY_APPLIED) r = ALINK_APPLY_LATE;
    apply_report(u, op.op, r, op.frame);
}

static void cancel_queue(alink_spdif_t *u) {
    alink_apply_op_t op;
    while (u->aq.n) {
        alink_apply_pop(&u->aq, &op);
        apply_report(u, op.op, ALINK_APPLY_CANCELLED, op.frame);
    }
}

void alink_spdif_cancel_all(alink_spdif_t *u) {
    // CANCEL_ALL of any time base: the generator first (CANCELLED_HOST, no
    // later START on the staged config), then the queue.
    alink_spdif_port_cancel_all();
    cancel_queue(u);
}

void alink_spdif_drop_op9(alink_spdif_t *u, uint8_t apply_result) {
    alink_apply_op_t op;
    if (alink_apply_remove(&u->aq, ALINK_OP_SIGGEN_START, &op)) {
        alink_apply_done(&u->aq, op.op, apply_result, op.frame);
        alink_spdif_port_apply_result(op.op, apply_result, (uint32_t)op.frame);
    }
}

static bool parse_tb3(const uint8_t p[ALINK_APPLY_LEN], alink_apply_op_t *op) {
    if (p[12] != TB_ID_SPDIF || p[13] || p[14] || p[15]) return false;
    uint8_t q[ALINK_APPLY_LEN];
    memcpy(q, p, ALINK_APPLY_LEN);
    q[12] = 0;
    return alink_apply_parse(&alink_spdif_apply_limits, q, op);
}

uint8_t alink_spdif_apply(alink_spdif_t *u, const uint8_t p[ALINK_APPLY_LEN], uint64_t t_write) {
    alink_apply_op_t op;
    if (!parse_tb3(p, &op)) return TB_ERR_VALUE;
    if (op.op == ALINK_APPLY_CANCEL_ALL) {
        alink_spdif_cancel_all(u);          // in any state
        return TB_OK;
    }
    if (alink_spdif_state(u) != TB_ST_LOCKED) return TB_ERR_STATE;
    if (u->aq.n >= ALINK_APPLY_DEPTH) return TB_ERR_STATE;
    if (op.op == ALINK_OP_SIGGEN_START) {
        uint8_t r = alink_spdif_port_op9_accept(op.frame, t_write, u->fs);
        if (r != TB_OK) return r;
    }
    return alink_apply_push(&u->aq, &op) ? TB_OK : TB_ERR_STATE;
}

// ---------------------------------------------------------------------------
// The source (12.3, 12.4; 2.10.3)
// ---------------------------------------------------------------------------

static bool ring_now(uint32_t *wr, uint32_t *played) {
    uint32_t s = alink_spdif_port_irq_save();
    bool ok = timebase_port_ring(wr, played);
    alink_spdif_port_irq_restore(s);
    return ok;
}

static uint64_t align_up_192(uint64_t f) {
    uint64_t r = f % ALINK_SPDIF_BLOCK_FRAMES;
    return r ? f + (ALINK_SPDIF_BLOCK_FRAMES - r) : f;
}

static void stage_invalidate(alink_spdif_t *u) {
    for (unsigned i = 0; i < ALINK_SPDIF_STAGE_BLOCKS; i++) u->stage[i].valid = false;
}

void alink_spdif_src_stopping(alink_spdif_t *u, uint64_t f_now) {
    if (f_now > u->f_seen) u->f_seen = f_now;
    if (u->mode != ALINK_SPDIF_MODE_SOURCE) return;
    bool was = u->src_running;
    u->src_running = false;
    u->stage_on = false;
    stage_invalidate(u);
    if (u->src_epoch_open) {
        // The next start is a new epoch: an origin past every F played, the
        // epoch byte incremented; the timeline's queue is void.
        u->epoch++;
        u->src.epoch = u->epoch;
        u->src_epoch_pending = true;
    }
    u->origin = align_up_192(u->f_seen + ALINK_SPDIF_BLOCK_FRAMES);
    u->src_epoch_open = true;
    u->bc_pending = false;
    u->src.apply_frame = 0;
    u->src.apply_op = ALINK_SPDIF_OP_NONE;
    cancel_queue(u);
    if (was) tb_note(u, TB_RS_NONE);        // leaves LOCKED: LOST, one disturbance
    count_locks(u);
    u->stage_on = true;                     // the prefill stamps the new epoch's blocks
}

void alink_spdif_src_started(alink_spdif_t *u) {
    if (u->mode != ALINK_SPDIF_MODE_SOURCE) return;
    if (!u->src_epoch_open) {
        u->origin = align_up_192(u->f_seen + ALINK_SPDIF_BLOCK_FRAMES);
        u->src_epoch_open = true;
    }
    bool epoch = u->src_epoch_pending;
    u->src_epoch_pending = false;
    u->src_running = true;
    u->stage_on = true;
    tb_note(u, TB_RS_NONE);
    if (epoch) {
        // 0xE4 EPOCH (counted once, when the old epoch left LOCKED).
        u->tbk.last_reason = TB_RS_EPOCH;
        if (u->tbk.ev_pending) u->tbk.ev_reason = TB_RS_EPOCH;
    }
    count_locks(u);
}

// Mode 2 entered with the outputs already running at ring position `played`:
// a new epoch starts here, F of the frame playing now beyond every F before.
static void src_enter(alink_spdif_t *u, bool running, uint32_t played) {
    u->src.sender = u->serial0;
    u->src.sender2 = u->serial1;
    u->epoch++;
    u->src.epoch = u->epoch;
    u->src_epoch_pending = false;
    stage_invalidate(u);
    if (running) {
        uint64_t want = u->f_seen + ALINK_SPDIF_BLOCK_FRAMES;
        uint64_t base = want > played ? want - played : 0;
        u->origin = align_up_192(base);
        u->src_epoch_open = true;
        u->src_running = true;
    } else {
        u->origin = align_up_192(u->f_seen + ALINK_SPDIF_BLOCK_FRAMES);
        u->src_epoch_open = true;
        u->src_running = false;
    }
    u->stage_on = true;
    tb_note(u, TB_RS_NONE);
    count_locks(u);
}

void alink_spdif_src_sync(alink_spdif_t *u, bool running, uint32_t played) {
    if (u->mode == ALINK_SPDIF_MODE_SOURCE && !u->src_epoch_open) src_enter(u, running, played);
}

void alink_spdif_src_stage(alink_spdif_t *u, uint32_t reader_block, bool restage) {
    if (u->mode != ALINK_SPDIF_MODE_SOURCE || !u->stage_on) return;
    u->src.rate = alink_spdif_rate_code(u->fs);
    for (uint32_t b = reader_block; b != reader_block + ALINK_SPDIF_STAGE_BLOCKS; b++) {
        alink_spdif_stage_t *s = &u->stage[b & (ALINK_SPDIF_STAGE_BLOCKS - 1u)];
        if (!restage && s->valid && s->block == b) continue;
        // Invalid while it is written: the filler (a stamper, possibly the
        // stall interrupt) never copies a half-built entry.
        s->valid = false;
        s->block = b;
        alink_spdif_sf_build(&u->src, u->origin + (uint64_t)b * ALINK_SPDIF_BLOCK_FRAMES, s->sf);
        s->valid = true;
    }
}

uint8_t alink_spdif_ctrl_write(alink_spdif_t *u, const uint8_t p[ALINK_SPDIF_CTRL_LEN], uint64_t f_play,
                               uint64_t f_write) {
    alink_apply_op_t op;
    if (!parse_tb3(p, &op)) return TB_ERR_VALUE;
    if (!alink_spdif_op_allowed(op.op)) return TB_ERR_VALUE;    // CANCEL_ALL and 3, 5, 6, 8 included
    if (u->mode != ALINK_SPDIF_MODE_SOURCE || alink_spdif_state(u) != TB_ST_LOCKED) return TB_ERR_STATE;
    if (u->bc_pending) return TB_ERR_STATE;                     // one at a time
    if (!alink_spdif_lead_ok(f_play, op.frame)) return TB_ERR_STATE;
    if (u->aq.n >= ALINK_APPLY_DEPTH) return TB_ERR_STATE;
    if (op.op == ALINK_OP_SIGGEN_START) {
        // The source plays it too: its own generator takes it as a write.
        uint8_t r = alink_spdif_port_op9_accept(op.frame, f_write, u->fs);
        if (r != TB_OK) return r;
    }
    if (!alink_apply_push(&u->aq, &op)) return TB_ERR_STATE;
    u->src.apply_frame = op.frame;
    u->src.apply_op = op.op;
    u->src.apply_arg = op.arg;
    u->src.apply_value = op.value;
    u->bc_pending = true;
    u->bc_last_op = op.op;
    u->bc_last_frame = op.frame;
    u->bc_last_result = 0;
    return TB_OK;
}

void alink_spdif_ctrl_read(const alink_spdif_t *u, uint8_t o[ALINK_SPDIF_CTRL_LEN]) {
    memset(o, 0, ALINK_SPDIF_CTRL_LEN);
    if (u->mode != ALINK_SPDIF_MODE_SOURCE) return;     // a follower reads all zero
    o[0] = u->bc_pending ? 1u : 0u;
    o[1] = u->bc_last_op;
    o[2] = u->bc_last_result;
    sp_put64(o + 4, u->bc_last_frame);
}

uint32_t alink_spdif_src_split(alink_spdif_t *u, uint64_t f, uint32_t n) {
    uint64_t at = alink_apply_next(&u->aq);
    if (at == UINT64_MAX) return n;
    if (at <= f) {
        while (u->aq.n && alink_apply_next(&u->aq) <= f) apply_run(u, alink_apply_next(&u->aq) < f);
        return 0;
    }
    return at - f < n ? (uint32_t)(at - f) : n;
}

// ---------------------------------------------------------------------------
// The follower: the word stream and the superframes (12.4 d, 12.5)
// ---------------------------------------------------------------------------

static void end_hold(alink_spdif_t *u) {
    u->resuming = false;
    u->ph.hold = false;
    u->ph.resync = false;
    u->ph.started = false;
    u->ph.e_valid = false;
    u->hold_rx = false;
}

static void reanchor(alink_spdif_t *u, uint8_t reason) {
    u->drop_rows = 0;
    u->ph.resync = false;
    u->ph.hold = false;
    u->hold_rx = false;
    u->ph.started = false;      // no verification or servo on the dead timeline
    u->ph.e_valid = false;
    u->prefilling = false;      // (drained stays: the handshake drains again)
    u->resuming = false;
    u->reanchor_req = true;     // the main loop mutes, drains, prefills, starts in sync
    u->reanchors++;
    tb_note(u, reason);
    u->tbk.from_hold = false;   // a new placement: the next LOCKED is not a resume
    count_locks(u);
}

// Coasted blocks ran out (12.4 d) or the carrier went (12.5): hold the
// timeline on silence.  False: nothing to hold (not playing it yet).
static bool enter_hold(alink_spdif_t *u, bool keep_rx, uint32_t now_us) {
    uint32_t fill = alink_spdif_port_fill();
    if (!alink_phase_active(&u->ph) || !alink_phase_loss(&u->ph, fill)) return false;
    u->hold_f = u->ph.next_w - fill + (uint64_t)u->d;
    u->hold_us = now_us;
    u->hold_rx = keep_rx;
    u->drop_rows = 0;
    tb_note(u, TB_RS_NONE);     // HOLDOVER (one disturbance)
    count_locks(u);
    return true;
}

static uint64_t hold_extrapolate(const alink_spdif_t *u, uint32_t now_us) {
    return u->hold_f + (uint64_t)(now_us - u->hold_us) * u->fs / 1000000u;
}

// The superframe a valid block carries: its apply (the broadcast path).
static void take_broadcast(alink_spdif_t *u, const alink_spdif_sf_t *sf) {
    if (sf->apply_op == ALINK_SPDIF_OP_NONE || sf->apply_frame == 0) return;
    if (u->sf_taken_have && u->sf_taken.frame == sf->apply_frame && u->sf_taken.op == sf->apply_op &&
        u->sf_taken.arg == sf->apply_arg && u->sf_taken.value == sf->apply_value)
        return;                 // taken already (every carrying block repeats it)
    alink_apply_op_t op = { sf->apply_frame, sf->apply_op, sf->apply_arg, sf->apply_value };
    u->sf_taken = op;
    u->sf_taken_have = true;
    if (alink_spdif_state(u) != TB_ST_LOCKED) {
        apply_report(u, op.op, ALINK_APPLY_REFUSED, op.frame);   // no lock, no F
        return;
    }
    bool ok = alink_apply_args_ok(&alink_spdif_apply_limits, op.op, op.arg, op.value) &&
              u->aq.n < ALINK_APPLY_DEPTH;
    // A broadcast op 9: refused at once when one is pending or it was first
    // seen less than arm_lead before T.
    if (ok && op.op == ALINK_OP_SIGGEN_START)
        ok = alink_spdif_port_op9_broadcast(op.frame, u->ph.next_w, u->fs);
    if (!ok || !alink_apply_push(&u->aq, &op)) apply_report(u, op.op, ALINK_APPLY_REFUSED, op.frame);
}

static void block_bad(alink_spdif_t *u, uint32_t now_us) {
    u->blocks_since_sf++;
    if (!u->rd_valid) return;
    u->sf_errors++;
    // Ridden through on continuity (rd_f counts on), for at most 8 blocks.
    if (++u->coast > ALINK_SPDIF_COAST_BLOCKS && !u->ph.hold) (void)enter_hold(u, true, now_us);
}

static void block_done(alink_spdif_t *u, bool whole, uint32_t now_us) {
    alink_spdif_sf_t sf;
    if (!whole || alink_spdif_sf_unpack(u->acc, &sf) != ALINK_SPDIF_OK || sf.rate != u->rx_rate) {
        block_bad(u, now_us);
        return;
    }
    uint64_t f_end = sf.frame + ALINK_SPDIF_BLOCK_FRAMES;   // F of the next pair read
    uint16_t id = alink_spdif_source_id(&sf);
    bool same = u->sf_seen && id == u->src_id && sf.epoch == u->src_epoch;
    bool first = !u->rd_valid;
    u->rd_f = f_end;
    u->rd_valid = true;
    if (first) {
        if (u->ph.hold) {
            // Back from a carrier holdover: placed by F on the held timeline
            // when the source and epoch are the same and F is where the
            // holdover's extrapolation says (the rings' span).
            bool ok = same && alink_phase_relock(&u->ph, f_end, hold_extrapolate(u, now_us),
                                                  alink_spdif_port_ring_span());
            u->hold_rx = false;
            if (ok) u->resuming = true;
            else reanchor(u, same ? TB_RS_REANCHOR : TB_RS_EPOCH);
        } else if (u->sf_seen && !same && (u->ph.started || u->prefilling)) {
            reanchor(u, TB_RS_EPOCH);
        }
    } else if (!same) {
        reanchor(u, TB_RS_EPOCH);   // another source or a new epoch: a new domain
    } else if (f_end != u->rd_f_expect) {
        // The block's F breaks continuity (a returning superframe that does
        // not confirm the coasted F, or words lost in the receiver).
        reanchor(u, TB_RS_REANCHOR);
    } else if (u->ph.hold && u->hold_rx) {
        // A coast holdover confirmed: resume on the same timeline.
        (void)alink_phase_relock(&u->ph, f_end, f_end, alink_spdif_port_ring_span());
        u->hold_rx = false;
        u->resuming = true;
    }
    if (u->lock_err) {
        u->lock_err = false;
        tb_note(u, TB_RS_LOCK_OK);
    }
    u->src_id = id;
    u->src_epoch = sf.epoch;
    u->sf_seen = true;
    u->sf_last = sf;
    u->sf_mute = (sf.flags & ALINK_SPDIF_FLAG_MUTE) != 0;
    u->coast = 0;
    u->blocks_since_sf = 0;
    u->last_sf_us = now_us;
    take_broadcast(u, &sf);
}

void alink_spdif_rx_words(alink_spdif_t *u, const uint32_t *w, uint32_t n, uint32_t idx, uint32_t now_us) {
    if (u->rd_have && idx != u->rd_idx) {
        // Words the receiver disposed of (an overflow): the stream moved on.
        uint32_t skip = (idx + u->fifo_words - u->rd_idx) % u->fifo_words;
        if (u->rd_valid) u->rd_f += skip / 2u;
        u->acc_ok = false;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t pos = (idx + i) % ALINK_SPDIF_BLOCK_WORDS;
        uint32_t x = w[i];
        if (pos == 0) {
            u->acc_ok = (x & ALINK_SPDIF_SYNC_MASK) == ALINK_SPDIF_SYNC_B;
            memset(u->acc, 0, sizeof(u->acc));
        } else if (pos != u->acc_pos) {
            u->acc_ok = false;      // joined mid-block, or words missing
        }
        u->acc[pos >> 3] |= (uint8_t)(((x >> ALINK_SPDIF_UBIT_SHIFT) & 1u) << (pos & 7u));
        u->acc_pos = (uint16_t)(pos + 1u);
        if (pos & 1u) u->rd_f++;    // a whole frame read
        if (pos == ALINK_SPDIF_BLOCK_WORDS - 1u) {
            u->rd_f_expect = u->rd_f;
            block_done(u, u->acc_ok, now_us);
            u->acc_ok = false;
            u->acc_pos = 0;
        }
    }
    u->rd_idx = (idx + n) % u->fifo_words;
    u->rd_have = true;
}

void alink_spdif_rx_state(alink_spdif_t *u, bool locked, uint8_t rate, uint32_t fs, uint32_t now_us) {
    if (fs) u->fs = fs;
    if (locked == u->rx_locked && (!locked || rate == u->rx_rate)) return;
    if (locked) {
        // A (new) lock: the FIFO restarted, the word stream is new.
        u->rx_locked = true;
        u->rx_rate = rate;
        u->rx_lock_us = now_us;
        u->last_sf_us = now_us;     // the lock-error timer starts at the lock
        u->rd_have = false;
        u->rd_valid = false;
        u->acc_ok = false;
        u->acc_pos = 0;
        u->coast = 0;
        u->blocks_since_sf = 0;
        return;
    }
    // The carrier went: hold the timeline while it is played, else no T.
    u->rx_locked = false;
    u->rx_rate = ALINK_SPDIF_RATE_UNKNOWN;
    u->rd_have = false;
    u->rd_valid = false;
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER || !u->active) return;
    if (u->ph.hold) {
        u->hold_rx = false;         // a coast holdover now on the extrapolation only
    } else if (u->ph.started && !enter_hold(u, false, now_us)) {
        u->ph.started = false;
    }
    tb_note(u, TB_RS_NONE);
    count_locks(u);
}

// ---------------------------------------------------------------------------
// The follower: delivery on the timeline (mirrors the line's alink_input.c)
// ---------------------------------------------------------------------------

// Read up to n frames from the FIFO; deliver them (or discard).  Returns
// frames read.  *f0: F of the first one (rd_f before the read).
static uint32_t rx_take(alink_spdif_t *u, uint32_t n, bool deliver, bool silent, uint32_t now_us) {
    uint32_t got = 0;
    while (got < n) {
        const uint32_t *w;
        uint32_t idx;
        uint32_t k = alink_spdif_port_rx_read(n - got, &w, &idx);
        if (k == 0) break;
        alink_spdif_rx_words(u, w, 2u * k, idx, now_us);
        if (deliver) alink_spdif_port_deliver(silent ? NULL : w, k);
        got += k;
    }
    return got;
}

static void discard_all(alink_spdif_t *u, uint32_t now_us) {
    uint32_t a = alink_spdif_port_rx_avail();
    if (a) (void)rx_take(u, a, false, false, now_us);
}

// n frames from the stream (or silence), split where an op is due so that it
// acts on exactly its frame; an op whose frame passed runs at once (late).
static uint32_t deliver_span(alink_spdif_t *u, bool rows, uint32_t n, uint32_t now_us) {
    uint32_t done = 0;
    while (done < n) {
        uint64_t here = u->ph.next_w;
        uint64_t at = alink_apply_next(&u->aq);
        if (at <= here) {
            apply_run(u, at < here);
            continue;
        }
        uint32_t k = n - done;
        if (at - here < k) k = (uint32_t)(at - here);
        if (rows) {
            uint64_t f = u->rd_f;
            uint32_t got = rx_take(u, k, true, u->sf_mute, now_us);
            if (got == 0) break;
            alink_phase_written(&u->ph, f, got);
            k = got;
        } else {
            alink_spdif_port_deliver(NULL, k);
            alink_phase_written(&u->ph, u->ph.next_w, k);
        }
        done += k;
        if (rows && !(u->ph.started || u->prefilling)) break;   // re-anchored meanwhile
    }
    return done;
}

// Rows against the timeline before they play: an anchor correction drops
// whole frames; after a holdover the first rows are placed by F (late rows
// dropped, a gap filled with silence).  False: nothing more this pass.
static bool deliver_place(alink_spdif_t *u, uint32_t *avail, uint32_t now_us) {
    if (u->drop_rows) {
        uint64_t f = u->rd_f;
        uint32_t k = u->drop_rows < *avail ? u->drop_rows : *avail;
        uint32_t got = rx_take(u, k, false, false, now_us);
        alink_phase_written(&u->ph, f, got);    // skipped: the timeline moves on
        u->drop_rows -= got;
        *avail -= got;
        if (u->drop_rows) return false;
    }
    if (!u->ph.resync) return true;
    int64_t gap = alink_phase_gap(&u->ph, u->rd_f);
    if (!alink_phase_gap_ok(gap, alink_spdif_port_ring_span())) {
        reanchor(u, TB_RS_REANCHOR);
        return false;
    }
    if (gap < 0) {
        uint32_t k = (uint64_t)-gap < *avail ? (uint32_t)-gap : *avail;
        uint32_t got = rx_take(u, k, false, false, now_us);
        *avail -= got;
        if ((int64_t)got < -gap) return false;
    } else if (gap > 0) {
        uint32_t min, max = alink_spdif_port_bounds(&min);
        uint32_t k = (uint64_t)gap < max ? (uint32_t)gap : max;
        if (k) deliver_span(u, false, k, now_us);
        return false;
    }
    u->ph.resync = false;       // in place: the next check verifies the phase
    return true;
}

// The phase error now (frames): the arrival against the timeline and the live
// ring fill, read together.  False without a timeline or a live position.
static bool phase_error(alink_spdif_t *u, float *e) {
    if (!u->ph.timeline || u->ph.hold || u->ph.resync || !u->rd_valid) return false;
    float fill;
    uint32_t played, words;
    uint32_t s = alink_spdif_port_irq_save();
    bool ok = alink_spdif_port_play(&fill, &played) && alink_spdif_port_rx_ahead(&words);
    alink_spdif_port_irq_restore(s);
    if (!ok) return false;
    float arr = (float)(int64_t)(u->rd_f - u->ph.next_w) + 0.5f * (float)words;
    *e = alink_phase_error(&u->ph, arr, fill);
    return true;
}

static void phase_act(alink_spdif_t *u, int act, int32_t frames) {
    if (u->ph.state == ALINK_PHASE_LOCKED) u->resuming = false;     // verified: back on the timeline
    if (act == ALINK_PHASE_ACT_REANCHOR) {
        reanchor(u, TB_RS_REANCHOR);
    } else if (act == ALINK_PHASE_ACT_CORRECT) {
        tb_note(u, TB_RS_CORRECTION);       // a disturbance of the timeline
        if (frames > 0) {
            u->drop_rows += (uint32_t)frames;
        } else {
            // Silence that delays the stream: not frames of it, so next_w
            // stays and e reads the corrected value at once.
            uint32_t k = (uint32_t)-frames, min, cap = alink_spdif_port_bounds(&min);
            if (cap == 0) cap = 1;
            while (k) {
                uint32_t n = k < cap ? k : cap;
                alink_spdif_port_deliver(NULL, n);
                k -= n;
            }
        }
    }
    count_locks(u);
}

static void phase_verify(alink_spdif_t *u) {
    float e;
    if (!phase_error(u, &e)) return;
    int32_t k = 0;
    int act = alink_phase_check(&u->ph, e, (u->flags & ALINK_SPDIF_FLAG_AUTO_REANCHOR) != 0, &k);
    phase_act(u, act, k);
}

uint32_t alink_spdif_poll(alink_spdif_t *u, uint32_t now_us) {
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER || !u->active) return 0;
    uint32_t min, max = alink_spdif_port_bounds(&min);
    if (alink_spdif_unaligned(u)) {
        // Lock error, play unaligned: the stock delivery (rate lock), the
        // superframes still watched.
        uint32_t a = alink_spdif_port_rx_avail();
        if (max == 0 || a < min || a == 0) return 0;
        uint32_t n = a < max ? a : max;
        return rx_take(u, n, true, false, now_us);
    }
    bool place = u->rd_valid && u->rx_locked && !u->ph.hold && (u->prefilling || u->ph.started) &&
                 u->rx_rate == alink_spdif_rate_code(u->fs) && !u->reanchor_req;
    if (!place) {
        discard_all(u, now_us);     // nothing plays; the words still count
        return 0;
    }
    uint32_t avail = alink_spdif_port_rx_avail();
    if ((u->ph.resync || u->drop_rows) && !deliver_place(u, &avail, now_us)) return 0;
    if (max == 0 || avail < min || avail == 0) return 0;
    uint32_t n = avail < max ? avail : max;
    uint32_t got = deliver_span(u, true, n, now_us);
    if (u->ph.started && u->ph.state == ALINK_PHASE_ANCHORING && !u->ph.hold && !u->ph.resync && !u->drop_rows)
        phase_verify(u);
    return got;
}

void alink_spdif_service(alink_spdif_t *u, uint32_t now_us) {
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) {
        uint32_t wr, played;
        if (u->src_running && ring_now(&wr, &played)) {
            uint64_t f_play = alink_spdif_src_f(u, played);
            if (f_play > u->f_seen) u->f_seen = f_play;
            if (u->bc_pending && f_play >= u->bc_last_frame) {
                // The broadcast ends once F reaches its frame.
                u->bc_pending = false;
                u->src.apply_frame = 0;
                u->src.apply_op = ALINK_SPDIF_OP_NONE;
            }
            alink_spdif_port_op9_poll(alink_spdif_src_f(u, wr), u->fs);
        }
        u->src.flags = alink_spdif_port_usb_live() ? ALINK_SPDIF_FLAG_USB_LIVE : 0u;
        u->src.rate = alink_spdif_rate_code(u->fs);
        return;
    }
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER || !u->active) return;
    // Lock error: a locked stream with no valid superframe for 0.5 s (since
    // the lock or the last valid one, last_sf_us), in any state.
    if (u->rx_locked && !u->lock_err && now_us - u->last_sf_us >= ALINK_SPDIF_LOCKERR_US) {
        bool had = u->ph.started || u->ph.hold;
        if (u->ph.hold) end_hold(u);
        u->ph.started = false;
        u->resuming = false;
        u->rd_valid = false;
        u->lock_err = true;
        if (had) tb_note(u, TB_RS_LOST);
        tb_note(u, TB_RS_LOCK_ERROR);
        count_locks(u);
    }
    // Holdover ends after 10 s (LOST): no T any more.
    if (u->ph.hold && now_us - u->hold_us >= ALINK_SPDIF_HOLD_MAX_US) {
        end_hold(u);
        tb_note(u, TB_RS_LOST);
        count_locks(u);
    }
    // The holdover keeps the rings at their fill with silence (ops run on it).
    if (u->ph.hold) {
        if (u->hold_rx) discard_all(u, now_us);
        uint32_t min, max = alink_spdif_port_bounds(&min);
        uint32_t n = alink_phase_hold_frames(&u->ph, alink_spdif_port_fill(), max);
        if (n) deliver_span(u, false, n, now_us);
    }
    // An op 9 pending on F arms at T - arm_lead.
    if (u->ph.timeline && (u->ph.started || u->ph.hold)) alink_spdif_port_op9_poll(u->ph.next_w, u->fs);
    count_locks(u);
}

// ---------------------------------------------------------------------------
// The follower: the anchor handshake, the servo, the stop
// ---------------------------------------------------------------------------

bool alink_spdif_anchor_wanted(const alink_spdif_t *u) {
    if (!alink_spdif_engaged(u) || lock_error_now(u)) return false;
    if (!u->rd_valid || !u->rx_locked || u->ph.hold || u->prefilling) return false;
    if (u->rx_rate != alink_spdif_rate_code(u->fs)) return false;
    if (u->ph.started && !u->reanchor_req) return false;
    return alink_spdif_port_rate_ready();
}

void alink_spdif_on_drain(alink_spdif_t *u) {
    alink_phase_drain(&u->ph);
    u->drop_rows = 0;
    u->reanchor_req = false;
    u->prefilling = true;
    u->drained = true;
    alink_spdif_port_ring_target();         // the prefill target follows D
    if (alink_phase_active(&u->ph)) u->ph.state = ALINK_PHASE_ANCHORING;
}

float alink_spdif_anchor_wait(alink_spdif_t *u) {
    if (!u->prefilling || !u->ph.timeline || !u->rd_valid) return 1e9f;
    uint32_t words;
    uint32_t s = alink_spdif_port_irq_save();
    bool ok = alink_spdif_port_rx_ahead(&words);
    alink_spdif_port_irq_restore(s);
    if (!ok) return 1e9f;
    float since = (float)(int64_t)(u->rd_f - u->ph.first) + 0.5f * (float)words;
    return alink_phase_anchor_wait(&u->ph, since);
}

void alink_spdif_on_enabled(alink_spdif_t *u) {
    // The servo's integrator holds the residual of the rate feed-forward (the
    // crystals' ratio, which a re-anchor or an epoch does not change): kept,
    // so a re-anchored follower is back on the same fraction of a frame at
    // once instead of after the integrator's tens of seconds.
    float integ = u->ph.pi.integ;
    alink_phase_started(&u->ph);
    u->ph.pi.integ = integ;
    u->prefilling = false;
    u->drained = false;
    tb_note(u, TB_RS_NONE);
}

bool alink_spdif_outputs_orphaned(const alink_spdif_t *u) {
    return u->drained && (!alink_spdif_engaged(u) || lock_error_now(u) || !u->rd_valid || !u->rx_locked);
}

void alink_spdif_on_restarted(alink_spdif_t *u) {
    u->drained = false;
    u->prefilling = false;
    u->ph.started = false;
}

void alink_spdif_outputs_reset(alink_spdif_t *u) {
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER) return;
    if (u->ph.started || u->ph.hold || u->prefilling) reanchor(u, TB_RS_REANCHOR);
    u->drained = false;         // the reset started them
}

void alink_spdif_stop(alink_spdif_t *u, uint8_t reason) {
    bool had_tb = alink_spdif_state(u) != TB_ST_NONE;
    cancel_queue(u);
    u->ph.hold = false;
    u->ph.resync = false;
    u->ph.started = false;
    u->ph.timeline = false;
    u->prefilling = false;
    u->drained = false;
    u->drop_rows = 0;
    u->reanchor_req = false;
    u->hold_rx = false;
    u->lock_err = false;
    u->src_running = false;
    u->src_epoch_open = false;
    u->src_epoch_pending = false;
    u->stage_on = false;
    u->bc_pending = false;
    u->src.apply_frame = 0;
    u->src.apply_op = ALINK_SPDIF_OP_NONE;
    stage_invalidate(u);
    uint8_t m = u->mode;
    u->mode = ALINK_SPDIF_MODE_OFF;         // the tracker reads NONE now
    if (had_tb) tb_note(u, reason);
    count_locks(u);
    u->mode = m;
}

bool alink_spdif_servo(alink_spdif_t *u, float *trim, bool *hold) {
    *trim = 0.0f;
    *hold = false;
    if (!alink_spdif_engaged(u) || !alink_phase_active(&u->ph)) return false;
    if (u->ph.hold) {
        *hold = true;           // the VCXO stays where it was
        return true;
    }
    float e;
    if (u->ph.started && phase_error(u, &e)) {
        int32_t k = 0;
        int act = ALINK_PHASE_ACT_NONE;
        if (u->ph.state != ALINK_PHASE_ANCHORING) {
            act = alink_phase_check(&u->ph, e, (u->flags & ALINK_SPDIF_FLAG_AUTO_REANCHOR) != 0, &k);
            phase_act(u, act, k);
        }
        if (act == ALINK_PHASE_ACT_REANCHOR) alink_phase_pi_reset(&u->ph.pi);
        else if (u->ph.state == ALINK_PHASE_LOCKED || u->ph.state == ALINK_PHASE_PENDING)
            *trim = alink_phase_servo(&u->ph, e);
    }
    return true;
}

bool alink_spdif_f_now(alink_spdif_t *u, uint64_t *f, uint32_t now_us) {
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) {
        uint32_t wr, played;
        if (!u->src_running || !ring_now(&wr, &played)) return false;
        *f = alink_spdif_src_f(u, played);
        return true;
    }
    if (u->ph.hold) {
        *f = hold_extrapolate(u, now_us);
        return true;
    }
    if (!u->rd_valid) return false;
    uint32_t words = 0;
    (void)alink_spdif_port_rx_ahead(&words);
    *f = u->rd_f + words / 2u;
    return true;
}

// ---------------------------------------------------------------------------
// The time-base provider (timebase.c)
// ---------------------------------------------------------------------------

static alink_spdif_t *tb_unit;
static tb_provider_t tb_prov;

static void tb_role(void) {
    if (tb_unit) tb_prov.role = tb_unit->mode == ALINK_SPDIF_MODE_SOURCE ? TB_ROLE_SOURCE : TB_ROLE_FOLLOWER;
}

static bool tb_present(void) {
    return tb_unit && (tb_unit->mode == ALINK_SPDIF_MODE_SOURCE ||
                       (tb_unit->mode == ALINK_SPDIF_MODE_FOLLOWER && tb_unit->active));
}

static uint8_t tb_now(void) {
    tb_role();
    if (!tb_present()) return TB_ST_NONE;
    return alink_spdif_state(tb_unit);
}

static void tb_read(tb_entry_t *e, bool ring, uint32_t wr, uint32_t played) {
    alink_spdif_t *u = tb_unit;
    e->role = u->mode == ALINK_SPDIF_MODE_SOURCE ? TB_ROLE_SOURCE : TB_ROLE_FOLLOWER;
    uint8_t st = u->tbk.state;
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) {
        e->domain = (uint32_t)u->serial0 | (uint32_t)u->serial1 << 8 | (uint32_t)u->epoch << 16;
        e->d = 0;
        e->t = (u->src_running && ring) ? tb_t_at_output(alink_spdif_src_f(u, wr), wr, played, 0) : 0;
        return;
    }
    e->domain = u->sf_seen ? ((uint32_t)u->src_id | (uint32_t)u->src_epoch << 16) : 0u;
    e->d = u->d;
    e->phase_error = u->ph.e;
    if (st == TB_ST_NONE || st == TB_ST_ACQUIRING) {
        e->t = 0;
    } else if (u->ph.timeline && u->ph.started && !u->ph.hold && ring) {
        e->t = tb_t_at_output(u->ph.next_w, wr, played, (int32_t)u->d);   // whole samples at the output instant
    } else {
        uint64_t f;
        e->t = alink_spdif_f_now(u, &f, alink_spdif_port_now_us()) ? f : 0;
    }
}

static bool tb_next(uint64_t *t) {
    alink_spdif_t *u = tb_unit;
    if (!u) return false;
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) {
        uint32_t wr, played;
        if (!u->src_running || !ring_now(&wr, &played)) return false;
        *t = alink_spdif_src_f(u, wr);
        return true;
    }
    if (u->mode != ALINK_SPDIF_MODE_FOLLOWER || !u->ph.timeline) return false;
    *t = u->ph.next_w;
    return true;
}

static void tb_drop(uint8_t r) {
    if (tb_unit) alink_spdif_drop_op9(tb_unit, r);
}

void alink_spdif_tb_register(alink_spdif_t *u) {
    tb_unit = u;
    tb_prov.tb = TB_ID_SPDIF;
    tb_prov.role = TB_ROLE_FOLLOWER;
    tb_prov.track = &u->tbk;
    tb_prov.state_now = tb_now;
    tb_prov.present = tb_present;
    tb_prov.read = tb_read;
    tb_prov.t_next = tb_next;
    tb_prov.drop_op9 = tb_drop;
    timebase_register(TB_SLOT_SPDIF, &tb_prov);
}
