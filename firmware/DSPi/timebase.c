/*
 * timebase.c - the EXT_TIME_BASE aggregator and the 0xE4 emitter (WP FWTB1).
 * See timebase.h.  Portable: the firmware's port is timebase_port.c, the
 * host tests supply their own.  Flash-resident (nothing here is hot).
 */
#include "timebase.h"

#include <string.h>

#include "siggen.h"

static const tb_provider_t *prov[TB_SLOTS];

void timebase_register(unsigned slot, const tb_provider_t *p) {
    if (slot < TB_SLOTS) prov[slot] = p;
}

static const tb_provider_t *find(uint8_t tb) {
    for (unsigned i = 0; i < TB_SLOTS; i++)
        if (prov[i] && prov[i]->tb == tb) return prov[i];
    return NULL;
}

uint8_t timebase_state(uint8_t tb) {
    const tb_provider_t *p = find(tb);
    return p ? p->track->state : TB_ST_NONE;
}

void timebase_service(uint32_t now_ms) {
    for (unsigned i = 0; i < TB_SLOTS; i++) {
        const tb_provider_t *p = prov[i];
        if (!p) continue;
        tb_track_t *k = p->track;
        uint64_t tn = 0;
        if (!p->t_next || !p->t_next(&tn)) tn = 0;      // T of an event found by this poll
        (void)tb_track_note(k, p->state_now(), TB_RS_NONE, (uint32_t)tn);
        // The generator (R3, R4): a loss stops a timed run and cancels a
        // pending op 9; a disturbance is a discontinuity of the running run.
        // Here, at the top of the main loop, never inside a transport's
        // delivery (which may be the one holding the queue).
        uint16_t n = (uint16_t)(k->dist - k->dist_seen);
        k->dist_seen = k->dist;
        if (k->lost) {
            k->lost = false;
            siggen_tb_loss(p->tb, n);
        } else if (n) {
            siggen_tb_disturbed(p->tb, n);
        }
        uint8_t reason;
        uint32_t t;
        if (tb_track_event_due(k, now_ms, &reason, &t)) timebase_port_event(p->tb, k->state, reason, t);
    }
}

static void tb_put16(uint8_t *o, uint16_t v) { o[0] = (uint8_t)v; o[1] = (uint8_t)(v >> 8); }
static void tb_put32(uint8_t *o, uint32_t v) { for (int i = 0; i < 4; i++) o[i] = (uint8_t)(v >> (8 * i)); }

void timebase_read(uint8_t o[TB_READ_LEN]) {
    memset(o, 0, TB_READ_LEN);
    o[0] = TB_VERSION;
    tb_entry_t e[TB_SLOTS];
    const tb_provider_t *src[TB_SLOTS];
    unsigned n = 0;
    uint32_t wr = 0, played = 0;
    // R7c: t and output_frames at one output instant, interrupts masked.
    uint32_t s = timebase_port_irq_save();
    bool ring = timebase_port_ring(&wr, &played);
    uint32_t us = timebase_port_now_us();
    // Followers first (2.11.1), then sources.
    for (unsigned pass = 0; pass < 2; pass++) {
        uint8_t role = pass ? TB_ROLE_SOURCE : TB_ROLE_FOLLOWER;
        for (unsigned i = 0; i < TB_SLOTS && n < TB_MAX_ENTRIES; i++) {
            const tb_provider_t *p = prov[i];
            if (!p || p->role != role || !p->present()) continue;
            memset(&e[n], 0, sizeof(e[n]));
            e[n].tb = p->tb;
            e[n].role = p->role;
            p->read(&e[n], ring, wr, played);
            src[n++] = p;
        }
    }
    timebase_port_irq_restore(s);
    o[1] = (uint8_t)n;
    tb_put32(o + 4, us);
    tb_put32(o + 8, played);
    for (unsigned i = 0; i < n; i++) {
        uint8_t *x = o + TB_ENTRY_OFF + TB_ENTRY_LEN * i;
        const tb_track_t *k = src[i]->track;
        x[0] = e[i].tb;
        x[1] = e[i].role;
        x[2] = k->state;
        x[3] = k->last_reason;
        tb_put32(x + 4, (e[i].domain & 0x00FFFFFFu) | (uint32_t)e[i].tb << 24);
        tb_put32(x + 8, (uint32_t)e[i].t);
        tb_put32(x + 12, (uint32_t)(e[i].t >> 32));
        tb_put16(x + 16, k->dist);
        tb_put16(x + 18, e[i].d);
        float pe = k->state == TB_ST_LOCKED ? e[i].phase_error : 0.0f;
        memcpy(x + 20, &pe, 4);
    }
}

// The signal generator's view of the time bases (siggen.h port).
void siggen_port_drop_op9(uint8_t tb, uint8_t apply_result) {
    const tb_provider_t *p = find(tb);
    if (p && p->drop_op9) p->drop_op9(apply_result);
}

bool siggen_port_tb_now(uint8_t *tb, uint64_t *t) {
    for (unsigned i = 0; i < TB_SLOTS; i++) {
        const tb_provider_t *p = prov[i];
        if (!p || p->track->state != TB_ST_LOCKED || !p->t_next) continue;
        if (p->t_next(t)) {
            *tb = p->tb;
            return true;
        }
    }
    return false;
}
