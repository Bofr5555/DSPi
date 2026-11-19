// sg_unit.h - several units' signal generators in one simulation (WP FWTB3).
//
// firmware/DSPi/siggen.c and timebase.c keep their state in file statics:
// one generator and one time-base aggregator per unit.  sg_unit.c includes
// both, the sims' run.sh compile it once and make SG_UNITS copies of the
// object with every global symbol renamed (objcopy --redefine-syms, prefix
// sgu<k>_), so unit k of a simulation runs its own, real generator and
// aggregator: the arming at T - arm_lead, the R4 preconditions, the never-late
// rule, the loss cancel and stop, the timeline record, the 0xE4 tracker.
//
// Each copy exports one table, sgu<k>_sg_unit; the simulation reaches the
// copy's port (the DAC gate, the slips, timebase.c's port) through *port,
// which it sets before the first call.  Everything is single-threaded (the
// firmware's Core 0 main loop).
#ifndef SG_UNIT_H
#define SG_UNIT_H

#include <stdbool.h>
#include <stdint.h>

#include "timebase.h"

#define SG_UNITS        5u      // four units and a reference
#define SG_TIMELINE_LEN 40u

typedef struct {
    bool     (*ready_now)(void);                 // siggen_output_ready_now(): may a plain START play at once?
    uint16_t (*arm_lead_ms)(void);               // siggen_port_arm_lead_ms()
    uint32_t (*slips)(void);                     // siggen_port_slips(): output underrun episodes
    uint32_t (*now_us)(void);                    // timebase_port_now_us()
    bool     (*ring)(uint32_t *wr, uint32_t *played);                  // timebase_port_ring()
    void     (*event)(uint8_t tb, uint8_t st, uint8_t rs, uint32_t t); // timebase_port_event(): 0xE4
} sg_port_t;

typedef struct {
    sg_port_t **port;                            // the copy's port pointer (set it first)
    // siggen.h
    bool    (*stage_config)(const void *payload, uint16_t len);
    bool    (*control)(uint8_t action);
    void    (*service)(void);
    bool    (*wants_output)(void);
    void    (*output_gate)(bool ready);
    bool    (*running)(void);
    // One block of n samples (n <= 192): output channel `ch` of the generator's
    // render (0 where it does not play) into out[0..n), or the hold of a
    // block the output rings refused (out zeroed, the engine not advanced).
    void    (*render)(float *out, unsigned ch, uint32_t n, uint32_t fs);
    void    (*hold)(float *out, uint32_t n);
    uint8_t (*tb_accept)(uint8_t tb, uint64_t t, uint64_t t_now, uint32_t fs);
    bool    (*tb_accept_broadcast)(uint8_t tb, uint64_t t, uint64_t t_now, uint32_t fs);
    void    (*tb_poll)(uint8_t tb, uint64_t t_now, uint32_t fs);
    uint8_t (*tb_fire)(uint8_t tb, uint64_t t, bool late, bool ready);
    void    (*tb_stop_op)(uint8_t arg);
    void    (*tb_dropped)(uint8_t result);
    void    (*tb_cancel_all)(void);
    void    (*tb_timeline)(uint8_t out[SG_TIMELINE_LEN]);
    // timebase.h
    void    (*tb_register)(unsigned slot, const tb_provider_t *p);
    void    (*tb_service)(uint32_t now_ms);
    void    (*tb_read)(uint8_t out[TB_READ_LEN]);
    uint8_t (*tb_state)(uint8_t tb);
} sg_unit_t;

extern const sg_unit_t sgu0_sg_unit, sgu1_sg_unit, sgu2_sg_unit, sgu3_sg_unit, sgu4_sg_unit;

// EXT_SIGGEN_TIMELINE decoded (extension_channel_spec.md 2.11.2).
typedef struct {
    uint8_t  tb, state, result;
    uint16_t disc, arm_lead_ms;
    uint64_t start, pending;
    uint32_t samples, held, run_id;
} sg_timeline_t;

static inline sg_timeline_t sg_timeline(const sg_unit_t *g) {
    uint8_t o[SG_TIMELINE_LEN];
    g->tb_timeline(o);
    sg_timeline_t t;
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

#endif
