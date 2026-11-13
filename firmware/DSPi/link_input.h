/*
 * link_input.h - the multi-unit audio link receiver: INPUT_SOURCE_LINK (WP LK5)
 *
 * RP2350 link builds only (DSPI_LINK).  The receiver takes the S/PDIF
 * receiver's resources, which are free while LINK is the source: PIO2 SM0
 * runs link_rx_N (LK2) on the configured GPIO, DMA channels 5 + 6 write a
 * 32 KB raw ring in the input capture arena (data channel plus write-address
 * reload), and link_fast (link/link_fast.h, LK5 milestone 1) decodes it.
 *
 * Link-up, all in non-blocking steps of the main-loop poll:
 *   1. class detection: link_rx_20, _10, _5 in turn (or the forced class),
 *      about 12 ms each, until the decoder locks (the superframe's class byte
 *      confirms; a wrong-class program never produces a comma);
 *   2. the trim scan (LK2): every sampling trim the program allows, a few
 *      hundred frames each, then the centre of the longest clean run; the
 *      result is EXT_LINK_DIAG offsets 32-39;
 *   3. lock with that trim, then audio: the unit's 8 slots (EXT_LINK_SLOTS)
 *      go to the pipeline's 8 input channels, like the 8-channel ADAT input.
 * A loss keeps class and trim and re-locks; a loss that lasts goes back to 1.
 *
 * Clock: the sender owns it.  The wire rate is measured from the RX DMA
 * position (one PIO bit per line bit, against the crystal timer; the decode
 * service publishes each position with the time it read it, and the
 * measurement uses that pair: link/link_ring_clock.h) and fed to
 * input_servo_apply() exactly as the S/PDIF and ADAT slave inputs do.  The
 * superframe's rate code drives perform_rate_change() through
 * link_input_check_rate_change().
 *
 * Phase lock (WP LK6, link/link_phase.h): in EXT_LINK_PHASE mode 1 the
 * outputs play link frame F exactly d_local + hops_offset frames after it
 * arrived.  The anchor prefills the rings to about that delay and starts the
 * outputs when the delay is reached (link_input_anchor_ready()); the servo
 * then holds the measured phase error at zero instead of the ring fill.  A
 * loss while phase-locked is a muted holdover on the same timeline; the
 * relock resumes without a re-anchor when the phase still matches.  Apply at
 * frame N (EXT_APPLY_AT_FRAME, the superframe's apply) acts on that frame's
 * samples as they enter the pipeline.
 *
 * Decoder placement: the decode service fills a row FIFO that the core-0 poll
 * drains into the pipeline.  By default core 0 runs it inside the poll; with
 * DSPI_LINK_DECODE_CORE1 the Core 1 EQ worker runs it while it waits for work
 * (core 0 takes over whenever Core 1 is not in EQ-worker mode).
 */

#ifndef LINK_INPUT_H
#define LINK_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "audio_input.h"
#include "link/link_phase.h"

// EXT_LINK_STATUS.state values (extension_channel_spec.md 2.8.2).
typedef enum {
    LINK_INPUT_OFF       = 0,   // not running (source not LINK, or enable 0)
    LINK_INPUT_SEARCHING = 1,   // class detection, trim scan, comma search, waiting for a superframe
    LINK_INPUT_LOCKED    = 2,   // decoding audio
    LINK_INPUT_HOLDOVER  = 3,   // phase-locked loss: muted, silence on the timeline, VCXO held
} LinkInputState;

// NOTIFY_EVT_LINK_STATE reasons (2.8.12).
#define LINK_REASON_NONE         0
#define LINK_REASON_LOCKED       1
#define LINK_REASON_LOS          2
#define LINK_REASON_FRAME_ERRORS 3
#define LINK_REASON_COMMA_LOST   4
#define LINK_REASON_REANCHOR     5
#define LINK_REASON_VCXO_LIMIT   6
#define LINK_REASON_DISABLED     7

// Reply sizes of the link sub-commands this build serves.
#define LINK_INFO_LEN       32
#define LINK_STATUS_LEN     52
#define LINK_CTRL_MSG_LEN   24
#define LINK_DIAG_LEN       64
#define LINK_FRAME_COUNTER_LEN 16

// Deferred restart (main loop): set when EXT_LINK_CONFIG changes something the
// running receiver uses, or enable flips while LINK is the source.
extern volatile bool link_input_restart_pending;

#if DSPI_LINK

void link_input_init(void);

// Start/stop the receiver (main-loop source switch).  Start claims PIO2 SM0,
// DMA 5/6 and the arena only while EXT_LINK_CONFIG.enable is 1; otherwise the
// source plays silence and the state reads OFF.
void link_input_start(void);
void link_input_stop(void);
bool link_input_is_running(void);

// Main-loop poll: link-up steps, decoding (unless Core 1 owns it), delivery
// to process_input_block().  Returns frames delivered.
uint32_t link_input_poll(void);

LinkInputState link_input_get_state(void);

// True once after every loss of lock (and after a rate change in the stream):
// the main loop mutes and re-prefills.  Clears itself.
bool link_input_take_loss(void);

// Drop decoded frames not yet delivered (prefill start).
void link_input_discard_queued(void);

// Servo the outputs to the measured wire rate (LOCKED only; rate-limited).
void link_input_update_clock_servo(void);

// Arm a deferred pipeline rate change if the stream's rate differs from
// audio_state.freq.  True if one was armed.
bool link_input_check_rate_change(void);
uint32_t link_input_get_detected_rate(void);

// True if `pin` is owned by the running receiver (RX, LOS).
bool link_input_owns_pin(uint8_t pin);

// Pushes the pending NOTIFY_EVT_LINK_STATE when 250 ms have passed since the
// last one.  Main loop, every pass.
void link_input_event_service(uint32_t now_ms);

// Control plane (vendor_commands.c validates the pins against the live pin
// table first).  Writes return false for a refused payload and change nothing.
bool link_input_set_config(const uint8_t cfg[LINK_CFG_LEN]);
bool link_input_set_slots(const uint8_t slots[LINK_SLOTS_LEN]);
bool link_input_stats_reset(uint8_t v);
void link_input_get_info(uint8_t out[LINK_INFO_LEN]);
void link_input_get_status(uint8_t out[LINK_STATUS_LEN]);
void link_input_get_ctrl_msg(uint8_t out[LINK_CTRL_MSG_LEN]);
void link_input_get_diag(uint8_t out[LINK_DIAG_LEN]);

// The decode service: link_fast over the ring into the row FIFO.  Core 0's
// poll calls it; with DSPI_LINK_DECODE_CORE1 the Core 1 EQ worker does while
// in EQ-worker mode (see the file comment).
void link_input_decode_service(void);
#if DSPI_LINK_DECODE_CORE1
void link_input_core1_service(void);
#endif

// F of the frame the decoder is on, valid while LOCKED.
uint64_t link_input_get_frame(bool *valid);

// --- Phase lock and apply at frame (WP LK6) ---------------------------------
// Boot, after preset_load_link_config(): the stored EXT_LINK_PHASE.
void link_input_load_phase(void);
bool link_input_set_phase(const uint8_t p[LINK_PHASE_LEN]);
void link_input_get_phase(uint8_t out[LINK_PHASE_LEN]);
void link_input_get_frame_counter(uint8_t out[LINK_FRAME_COUNTER_LEN]);
bool link_input_apply(const uint8_t p[LINK_APPLY_LEN]);
void link_input_get_apply(uint8_t out[LINK_APPLY_LEN]);
// output_latency_apply() for the LINK source: the ring target to prefill
// (the phase-locked delay less the block cap and decode margin, or target).
uint32_t link_input_ring_target(uint32_t target, uint32_t block_cap);
// Main loop, LINK prefill handshake: the outputs were drained (the rings are
// empty); then, once the prefill is at its target, true when the outputs may
// start (phase-locked: spins up to a few frames for the exact moment); and
// the outputs were started.
// Before the drain: phase-locked, true once the VCXO has pulled in to the
// stream's rate (SYS2 3.3: pull-in, then anchor), so the anchor does not
// drift away while the VCXO is still slewing.  Always true otherwise.
bool link_input_rate_ready(void);
void link_input_on_drain(void);
bool link_input_anchor_ready(void);
void link_input_on_enabled(void);

#endif // DSPI_LINK

#endif // LINK_INPUT_H
