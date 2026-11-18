/*
 * siggen_pump.h -- pacing rule of the generator's zero-input pump (pure,
 * host-tested in tests/test_siggen_timing.c; used by siggen_pump() in
 * audio_pipeline.c).
 *
 * With no input source owning the rings, the main loop renders zero-input
 * blocks through process_input_block() while the slot-0 ring is below the
 * output latency target: a block is written only when the DAC's own DMA has
 * consumed room for it.  The DAC clock alone paces the generator; main-loop
 * jitter only moves WHEN a block is written, never how many frames the
 * generator emits or where they land in the DAC frame sequence.  Nothing
 * resamples, drops or inserts while the ring never runs dry.  A ring that
 * does run dry (a main-loop stall longer than the fill) is the one slip
 * path: output_rings_pre_give() re-anchors it, counted as an underrun
 * (overruns / dry episode) and in the ring's underrun_frames.
 */
#ifndef SIGGEN_PUMP_H
#define SIGGEN_PUMP_H

#include <stdbool.h>
#include <stdint.h>

// Blocks per pump call: bounded so the main loop stays responsive; the loop
// spins far faster than one block period at any supported rate.
#define SIGGEN_PUMP_MAX_BLOCKS 2u

typedef struct {
    bool running;               // siggen_running
    bool source_owns_rings;     // pipeline_source_owns_rings()
    bool preset_loading;
    bool source_is_usb;
    bool source_change_pending; // input_source_change_pending
    bool type_switch;           // output_type_switch_in_progress
    bool slots_ready;           // the slot instances exist (usb_sound_card_init ran)
} siggen_pump_gate_t;

// May the pump run this pass?  An input source that delivers blocks (or keeps
// its own timeline, the USB phase mode) owns the rings: a second producer
// would add frames the DAC never asked for.  preset_loading refuses only for
// non-USB sources, where it is the lock / prefill handshake; on USB the pump
// is the only block source and must pump through the mute.
static inline bool siggen_pump_may_run(const siggen_pump_gate_t *g) {
    if (!g->running) return false;
    if (g->preset_loading && !g->source_is_usb) return false;
    if (g->source_change_pending || g->type_switch || !g->slots_ready) return false;
    return !g->source_owns_rings;
}

// One more block now?  fill = live slot-0 ring fill (frames), re-read before
// every block because the DMA keeps consuming.
static inline bool siggen_pump_wants_block(uint32_t fill, uint32_t target, uint32_t blocks_done) {
    return blocks_done < SIGGEN_PUMP_MAX_BLOCKS && fill < target;
}

#endif // SIGGEN_PUMP_H
