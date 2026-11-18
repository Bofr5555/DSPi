/*
 * siggen_dac.h -- the signal generator owns the DAC hardware mute while it
 * plays (WP FWSIGGEN).  Pure logic, host-tested (tests/test_siggen_dac.c);
 * the glue is siggen_dac_service() in main.c.
 *
 * The generator plays with no host stream, so nothing else would open a DAC
 * that an IDLE hold keeps muted (the USB phase mode out of step holds the pin
 * while it waits for a host, alink_usb.c dac_hold_update()).  The rule:
 *
 *   OPEN    START with the pin muted: the generator sets the override
 *           (dac_hw_mute_siggen_override(true)); the idle hold no longer
 *           drives the pin.  The engine start waits until the pin is
 *           actually open, no reset bracket / release dwell is in force, and
 *           (when the pin was muted at START) the DAC's own un-mute ramp has
 *           had settle_us (max(hold_ms, release_ms)): the first cycle is not
 *           played into the ramp.  The outputs play silence meanwhile, so the
 *           pin edge is click-free.  A START with the pin open starts at once.
 *   PLAY    the override stays on.  Reset brackets keep full priority (they
 *           re-mute and re-arm a full hold, dac_hw_mute.c).
 *   TAIL    the generator went idle: the override stays on for tail_us, the
 *           time the rings and delay lines need to play the faded-out tail,
 *           then goes off.  An idle hold still latched mutes the pin again;
 *           one that ended meanwhile leaves it open.  So the previous mute
 *           behaviour is back exactly, and the analog mute never cuts audio.
 *
 * Times are uint32_t microseconds (wrap-safe differences).
 */
#ifndef SIGGEN_DAC_H
#define SIGGEN_DAC_H

#include <stdbool.h>
#include <stdint.h>

enum { SIGGEN_DAC_IDLE = 0, SIGGEN_DAC_OPENING, SIGGEN_DAC_PLAY, SIGGEN_DAC_TAIL };

// The generator's stop fade has left the wire after this long: one slot ring
// (512 frames, 11.6 ms at 44.1 kHz) plus margin.  Delay lines add their own
// length (siggen_dac_tail_us()).
#define SIGGEN_DAC_TAIL_BASE_US 15000u

typedef struct {
    uint8_t  st;
    bool     was_muted;   // the pin was muted when the generator asked
    bool     override;    // what dac_hw_mute_siggen_override() must be
    uint32_t t0_us;       // OPENING: pin seen open since; TAIL: idle since
} siggen_dac_t;

typedef struct {
    bool     want;         // the generator asks for the outputs (start pending or running)
    bool     pin_muted;    // dac_hw_mute_is_asserted()
    bool     bracket;      // dac_hw_mute_bracket_active()
    uint32_t now_us;
    uint32_t settle_us;    // DAC un-mute ramp: max(hold_ms, release_ms) * 1000
    uint32_t tail_us;      // see SIGGEN_DAC_TAIL_BASE_US
} siggen_dac_in_t;

static inline void siggen_dac_init(siggen_dac_t *d) {
    d->st = SIGGEN_DAC_IDLE;
    d->was_muted = false;
    d->override = false;
    d->t0_us = 0;
}

// One step; returns true when the engine may start (or keep playing).
static inline bool siggen_dac_step(siggen_dac_t *d, const siggen_dac_in_t *in) {
    switch (d->st) {
    case SIGGEN_DAC_IDLE:
        if (!in->want) return false;
        d->override = true;
        d->was_muted = in->pin_muted;
        d->st = SIGGEN_DAC_OPENING;
        d->t0_us = in->now_us;
        /* fall through */
    case SIGGEN_DAC_OPENING:
        if (!in->want) {
            // START cancelled before it played: nothing was audible.
            d->st = SIGGEN_DAC_IDLE;
            d->override = false;
            return false;
        }
        if (in->pin_muted || in->bracket) {
            d->was_muted = true;
            d->t0_us = in->now_us;           // the ramp counts from the open pin
            return false;
        }
        if (d->was_muted && (uint32_t)(in->now_us - d->t0_us) < in->settle_us) return false;
        d->st = SIGGEN_DAC_PLAY;
        return true;
    case SIGGEN_DAC_PLAY:
        if (in->want) return true;
        d->st = SIGGEN_DAC_TAIL;
        d->t0_us = in->now_us;
        return false;
    case SIGGEN_DAC_TAIL:
        if (in->want) {
            // A new START during the tail: the DAC is still open.
            if (in->pin_muted || in->bracket) {
                d->st = SIGGEN_DAC_OPENING;
                d->was_muted = true;
                d->t0_us = in->now_us;
                return false;
            }
            d->st = SIGGEN_DAC_PLAY;
            return true;
        }
        if ((uint32_t)(in->now_us - d->t0_us) >= in->tail_us) {
            d->st = SIGGEN_DAC_IDLE;
            d->override = false;
        }
        return false;
    default:
        siggen_dac_init(d);
        return false;
    }
}

// Ready for a START right now without a step (the vendor path asks before the
// next main-loop pass): only when the pin is open and nothing settles.
static inline bool siggen_dac_ready_now(const siggen_dac_t *d, bool pin_muted, bool bracket) {
    if (pin_muted || bracket) return false;
    return d->st == SIGGEN_DAC_IDLE || d->st == SIGGEN_DAC_PLAY || d->st == SIGGEN_DAC_TAIL;
}

#endif // SIGGEN_DAC_H
