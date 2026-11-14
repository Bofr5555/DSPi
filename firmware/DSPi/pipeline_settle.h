/*
 * pipeline_settle.h - the decision part of the pre-reset fade-to-silence
 * settle (main.c pipeline_fade_to_silence_poll), free of SDK and hardware
 * so the host tests can run it.  main.c owns the side effects (soft-mute
 * request, envelope latch, ring silence mark); this header only decides.
 */
#ifndef PIPELINE_SETTLE_H
#define PIPELINE_SETTLE_H

#include <stdbool.h>
#include <stdint.h>

// Defensive cap on the whole settle (fade + drain), for a producer that
// stalls after being declared live.  Comfortably above the worst real case
// (8 ms fade + ~30 ms of queue + 42 ms of delay line).
#define PIPELINE_FADE_CAP_US    200000u
// A fade that has not been polled for this long belongs to an abandoned
// operation; the next poll starts a fresh one rather than inheriting its
// "already silent" verdict.
#define PIPELINE_FADE_STALE_US   50000u

typedef struct {
    uint64_t start_us;       // first poll of this settle (0: none in flight)
    uint64_t last_poll_us;   // latest poll (0: none in flight)
    bool     drain_armed;    // envelope reached zero, ring silence mark armed
} pipeline_settle_t;

typedef enum {
    PIPELINE_SETTLE_WAIT_ENVELOPE,  // producer live, envelope not yet at zero
    PIPELINE_SETTLE_ARM_DRAIN,      // envelope at zero now: arm the ring silence mark
    PIPELINE_SETTLE_WAIT_DRAIN,     // mark armed: done once the rings play past it
    PIPELINE_SETTLE_LATCH,          // nothing will move the envelope: latch it at zero
    PIPELINE_SETTLE_SILENT,         // the wire is silent from an earlier settle
} pipeline_settle_act_t;

static inline void pipeline_settle_reset(pipeline_settle_t *s) {
    s->start_us = 0;
    s->last_poll_us = 0;
    s->drain_armed = false;
}

// One poll at now_us.  wire_silent: an earlier settle completed and the
// envelope has stayed at zero since.  producer_streaming: something is
// feeding blocks that move the envelope (pipeline_producer_is_streaming()).
// mute_silent: the envelope is at zero.
static inline pipeline_settle_act_t pipeline_settle_step(pipeline_settle_t *s, uint64_t now_us,
                                                         bool wire_silent, bool producer_streaming,
                                                         bool mute_silent) {
    if (s->last_poll_us != 0 && (now_us - s->last_poll_us) > PIPELINE_FADE_STALE_US)
        pipeline_settle_reset(s);
    if (s->start_us == 0) s->start_us = now_us;
    s->last_poll_us = now_us;

    if (wire_silent) return PIPELINE_SETTLE_SILENT;
    // Nothing to fade, and no block will arrive to advance the envelope.
    if (!producer_streaming) return PIPELINE_SETTLE_LATCH;
    // Defensive: a producer that was declared live but stopped delivering.
    if ((now_us - s->start_us) > PIPELINE_FADE_CAP_US) return PIPELINE_SETTLE_LATCH;
    if (s->drain_armed) return PIPELINE_SETTLE_WAIT_DRAIN;
    if (!mute_silent) return PIPELINE_SETTLE_WAIT_ENVELOPE;
    // The envelope reached zero; delay lines still hold pre-fade audio.
    s->drain_armed = true;
    return PIPELINE_SETTLE_ARM_DRAIN;
}

// USB liveness for the settle: a packet within threshold_us.  sync_started
// alone is not enough: the stream's gap check runs only on the next packet,
// so a host that stops leaves it latched with nothing arriving.
static inline bool pipeline_usb_packets_recent(bool sync_started, uint64_t last_packet_us,
                                               uint64_t now_us, uint32_t threshold_us) {
    return sync_started && last_packet_us > 0 && (now_us - last_packet_us) <= threshold_us;
}

#endif // PIPELINE_SETTLE_H
