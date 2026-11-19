/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_AUDIO_SPDIF_H
#define _PICO_AUDIO_SPDIF_H

#include "pico.h"

/** \file audio_spdif.h
 *  \defgroup pico_audio_spdif pico_audio_spdif
 *  S/PDIF audio output using the PIO
 *
 * This library uses the \ref pio system to implement a S/PDIF audio interface.
 * Multiple instances can operate concurrently on independent PIO SMs/DMA
 * channels/GPIO pins.
 */

#ifdef __cplusplus
extern "C" {
#endif


#ifndef PICO_AUDIO_SPDIF_PIO
#ifdef PICO_AUDIO_PIO
#define PICO_AUDIO_SPDIF_PIO PICO_AUDIO_PIO
#else
#define PICO_AUDIO_SPDIF_PIO 0
#endif
#endif


// IEC 60958 block length in frames (channel-status / preamble period).
#define PICO_AUDIO_SPDIF_BLOCK_SAMPLE_COUNT 192u


#ifndef PICO_AUDIO_SPDIF_PIN
#define PICO_AUDIO_SPDIF_PIN 0
#endif

#define PICO_AUDIO_SPDIF_MAX_INSTANCES 4

// -- WP FWSP1 -- user bits: the alink S/PDIF superframe (alink_protocol_spec.md
// 12), one 48-byte superframe per 192-frame block from a 4-entry table keyed
// by block index.  The DSPi build turns it on with DSPI_ALINK_SPDIF (RP2350);
// off, the stampers are the code before FWSP1 (RP2040: .data unchanged).
#ifndef PICO_AUDIO_SPDIF_UBITS
#define PICO_AUDIO_SPDIF_UBITS 0
#endif
#define PICO_AUDIO_SPDIF_UBITS_LEN          48u     // 384 user bits per block
#define PICO_AUDIO_SPDIF_UBITS_TABLE_BLOCKS 4u      // SPDIF_TX_TABLE_BLOCKS

#include "hardware/pio.h"

/** \brief Per-instance state for an S/PDIF output
 * \ingroup audio_spdif
 */
// Ring geometry: one encoded ring per slot.
// Power of 2 (hardware read-address wrap on RP2350); 512 frames = 10.7 ms
// at 48 kHz. The writer keeps ERASE_AHEAD frames of valid silence stamped
// ahead of real audio so a stalled producer plays silence, never a loop of
// stale ring content. Both values bound the selectable output latency.
#define PICO_AUDIO_SPDIF_RING_FRAMES     512u
#define PICO_AUDIO_SPDIF_RING_BYTES      (PICO_AUDIO_SPDIF_RING_FRAMES * 2u * 8u)
#define PICO_AUDIO_SPDIF_ERASE_AHEAD     64u    // the idle pump extends it to a full ring

typedef struct audio_spdif_instance {
    // Hardware config (set in setup, immutable after)
    PIO pio;
    uint8_t pio_sm;
    uint8_t dma_channel;
    uint8_t dma_reload_channel; // RP2040 only: free-running reload pair partner
    uint8_t pin;
    uint8_t instance_index;     // Stable slot identity (== dma_channel)

    // Runtime state
    uint32_t freq;
    bool enabled;
    // Ring word format of the block's loaded program: PIO biphase encoder
    // (control + data words) or software NRZI transition words. Fixed at setup.
    bool pio_bmc;

    // Ring state. Frame counters are monotonic since the last ring reset;
    // uint32 wrap is harmless (all consumers use differences).
    uint32_t *ring;                     // slot ring base (caller-owned words)
    volatile uint32_t wr_frames;        // frames of real audio written
    uint32_t erased_frames;             // silence stamped through this total
    volatile uint32_t underrun_frames;  // silence frames exposed to the wire
    volatile uint32_t drop_frames;      // producer frames dropped on ring-full

    // Consumed-side unwrap state; mutated only inside
    // audio_spdif_ring_consumed_words() under a brief IRQ-off section.
    uint32_t last_read_addr;
    uint32_t last_read_time_us;          // selects whole laps across long polls
    volatile uint32_t consumed_words;   // words; USB feedback only
    volatile uint32_t consumed_frames;  // frames; ALL fill/deficit math
    uint8_t consumed_word_rem;          // mid-frame carry between reads

    // Cumulative-count snapshot latched at the end of each block write:
    // a coherent (constant-production-phase) observation pair for the
    // clock servo, free of the block-arrival sawtooth.
    volatile uint32_t snap_produced_frames;
    volatile uint32_t snap_consumed_frames;
    // Fill immediately before the most recent successful block write.
    // UINT32_MAX means no successful write has occurred since ring reset.
    volatile uint32_t recovery_prewrite_fill_frames;
#if PICO_AUDIO_SPDIF_UBITS
    // User bits (WP FWSP1).  ub_carry: this slot stamps the table's
    // superframes (EXT_ALINK_SPDIF.tx_mask); off, it stamps zero user bits.
    // ub_ref_frame/ub_ref_block: a block start in the 32-bit frame counter
    // and its block index since the ring reset, moved by the stampers; they
    // unwrap the counter so the block phase is continuous at 2^32.
    bool ub_carry;
    uint32_t ub_ref_frame;
    uint32_t ub_ref_block;
#endif
} audio_spdif_instance_t;

/** \brief Configuration for an S/PDIF output instance
 * \ingroup audio_spdif
 */
typedef struct audio_spdif_config {
    uint8_t pin;
    uint8_t dma_channel;
    uint8_t dma_reload_channel; // RP2040 only: reload partner (ignored on RP2350)
    uint8_t pio_sm;
    uint8_t pio;        // PIO block index (0, 1, or 2 on RP2350)
    void   *ring_base;  // caller-owned per-slot ring storage,
                        // PICO_AUDIO_SPDIF_RING_BYTES, aligned to its size;
                        // shared with the slot's I2S instance across retypes
} audio_spdif_config_t;

/** \brief Set up an S/PDIF audio output instance
 * \ingroup audio_spdif
 *
 * Resets the slot ring and programs the nominal divider for \p sample_freq.
 * Audio reaches the instance only through audio_spdif_ring_write_s32().
 *
 * \param inst The instance to initialize (caller-allocated, zero-initialized)
 * \param sample_freq Output sample rate in Hz
 * \param config The hardware configuration to apply
 */
void audio_spdif_setup(audio_spdif_instance_t *inst, uint32_t sample_freq,
                       const audio_spdif_config_t *config);

/** \brief Enable or disable an S/PDIF output instance
 * \ingroup audio_spdif
 *
 * \param inst The S/PDIF instance
 * \param enabled true to enable, false to disable
 */
void audio_spdif_set_enabled(audio_spdif_instance_t *inst, bool enabled);

/** \brief Change the GPIO pin of an S/PDIF output instance
 * \ingroup audio_spdif
 *
 * The instance must be disabled before calling this function.
 * Aborts any stale DMA, releases the old pin to high-Z, reinitializes the
 * PIO state machine with the new pin, and restores the clock divider.
 *
 * \param inst The S/PDIF instance (must be disabled)
 * \param new_pin The new GPIO pin number
 */
void audio_spdif_change_pin(audio_spdif_instance_t *inst, uint new_pin);

/** \brief Move an S/PDIF output instance onto a new GPIO without releasing the old one
 * \ingroup audio_spdif
 *
 * Same as audio_spdif_change_pin() except the old pin is left untouched, for
 * callers that release outgoing pins themselves before claiming incoming ones
 * (so a pin swap between outputs cannot release a pin another output just took).
 *
 * \param inst The S/PDIF instance (must be disabled)
 * \param new_pin The new GPIO pin number
 */
void audio_spdif_attach_pin(audio_spdif_instance_t *inst, uint new_pin);

/** \brief Fully tear down an S/PDIF output instance for output-type switching
 * \ingroup audio_spdif
 *
 * Counterpart to audio_spdif_setup() and mirror of audio_i2s_teardown().
 * Releases the DMA channel, PIO SM, pin, and registry slot so this slot's DMA
 * channel can be re-claimed by its I2S instance.  The caller-owned ring
 * storage is kept for the next setup.
 *
 * \param inst The S/PDIF instance to tear down
 */
void audio_spdif_teardown(audio_spdif_instance_t *inst);

// ---------------------------------------------------------------------------
// Ring output API
// ---------------------------------------------------------------------------

/** \brief Encode a block of interleaved S32 L/R frames into the slot ring.
 *
 * Stamps IEC preamble/channel-status per frame from the absolute frame
 * index (phase = total frames mod 192), then extends the erase-ahead
 * silence region. Frames beyond the writable window are dropped and
 * counted in drop_frames. Latches the cumulative produced/consumed
 * snapshot pair on return.
 */
void audio_spdif_ring_write_s32(audio_spdif_instance_t *inst,
                                const int32_t *interleaved_lr, uint32_t frames);

/** \brief Monotonic DMA words consumed since the last ring reset.
 *
 * Unwraps the DMA read pointer. Elapsed time selects any whole ring laps
 * between calls; the hardware pointer supplies the exact in-ring remainder.
 */
uint32_t audio_spdif_ring_consumed_words(audio_spdif_instance_t *inst);

// Frames consumed since ring reset. Wrap-coherent with wr_frames at 2^32;
// the words counter is NOT (divided down it wraps early), so all fill and
// deficit arithmetic must use this one.
uint32_t audio_spdif_ring_consumed_frames(audio_spdif_instance_t *inst);

/** \brief Frames of real audio written since the last ring reset. */
static inline uint32_t audio_spdif_ring_produced_frames(audio_spdif_instance_t *inst) {
    return inst->wr_frames;
}

/** \brief Current fill in frames (produced minus consumed). */
uint32_t audio_spdif_ring_fill_frames(audio_spdif_instance_t *inst);

/** \brief Reset ring pointers and accounting. DMA must be stopped. */
void audio_spdif_ring_reset(audio_spdif_instance_t *inst);

/** \brief Group re-anchor primitive: advance the write position by
 * `frames` without writing audio (the skipped region already carries
 * erase-ahead silence). Counted in underrun_frames. The caller applies
 * the SAME skip to every enabled slot so inter-slot alignment holds.
 */
void audio_spdif_ring_skip(audio_spdif_instance_t *inst, uint32_t frames);

// Extend the erased (silence) region toward one full ring ahead of the
// reader, bounded by max_frames per call. Keeps a stalled producer from
// ever looping stale ring content; cheap no-op while streaming.
void audio_spdif_ring_silence_pump(audio_spdif_instance_t *inst, uint32_t max_frames);

#if PICO_AUDIO_SPDIF_UBITS
// ---------------------------------------------------------------------------
// User bits (WP FWSP1, alink_protocol_spec.md 12.1 and 12.4)
//
// Block index b = frames since the slot's ring reset / 192 (continuous at the
// 32-bit frame counter wrap); block b's first frame carries the Z preamble.
// Every carrying slot must share one frame index (started in sync), so one
// table serves them all.  An entry is frozen once any frame of its block is
// stamped by any stamper; a block whose entry was not filled for it by then
// stamps zero user bits, and stays zero.
// ---------------------------------------------------------------------------

// audio_spdif_ubits_fill() results.
#define AUDIO_SPDIF_UBITS_OK      0
#define AUDIO_SPDIF_UBITS_FROZEN  1   // the block is (partly) stamped or older: too late
#define AUDIO_SPDIF_UBITS_EARLY   2   // its entry still serves a block not yet played: too early

/** \brief Called by a stamper immediately before the first stamp of block
 * `block` whose entry is not filled for it, so the entry can be filled in the
 * same pass (audio_spdif_ubits_fill()).  It runs inside the stampers, from
 * the main loop and from the flash-blackout timer interrupt: it must be
 * RAM-resident and must not fill while its own state is unavailable (a flash
 * blackout), which leaves those blocks zero (spec 12.4 c). */
typedef void (*audio_spdif_ubits_filler_t)(uint32_t block);

/** \brief Fill block `block`'s entry with its 48-byte superframe.  Refused
 * (FROZEN) for a block any frame of which is already stamped (inside the
 * erase-ahead) or that is older than the entry's; refused (EARLY) while the
 * entry still serves block - 4 or later, i.e. until the reader has left it.
 * IRQ-safe and RAM-resident. */
int audio_spdif_ubits_fill(uint32_t block, const uint8_t sf[PICO_AUDIO_SPDIF_UBITS_LEN]);

/** \brief Blocks fillable now, over the enabled carrying slots: the first
 * block not yet stamped to the reader's block + 3.  False when no carrying
 * slot is enabled or the window is empty (the stamped frontier is past
 * the table's end). */
bool audio_spdif_ubits_window(uint32_t *first, uint32_t *last);

/** \brief The reader's block (the oldest over the enabled carrying slots),
 * also while the window above is empty.  False when no carrying slot is
 * enabled.  (Review fwbc F4: a source stages ahead of the reader whether or
 * not a block can be filled this pass.) */
bool audio_spdif_ubits_reader(uint32_t *reader);

/** \brief Block index of an instance's frame (the stampers' numbering). */
uint32_t audio_spdif_ubits_block_of(const audio_spdif_instance_t *inst, uint32_t frame);

/** \brief Empty the table (an emptied table stamps zero user bits). */
void audio_spdif_ubits_clear(void);

/** \brief Set or clear the filler (NULL: the table holds only what is filled). */
void audio_spdif_ubits_set_filler(audio_spdif_ubits_filler_t fn);

/** \brief Whether this slot stamps the table's bits (default off: zero user bits). */
void audio_spdif_ubits_set_carry(audio_spdif_instance_t *inst, bool carry);
#endif

/** \brief Select a block's encoder: true = 16-instruction PIO biphase encoder,
 * false = 4-instruction NRZI program with software words. Applied at the next
 * program load (first setup after the block's last teardown); instances already
 * set up keep their format. Cannot share a block with the I2S extclk program. */
void audio_spdif_select_encoder(uint pio_index, bool pio_bmc);

/** \brief Encoder currently loaded on a block: 1 = PIO encoder, 0 = NRZI
 * program, -1 = no program loaded (no instance set up on that block). */
int audio_spdif_loaded_encoder(uint pio_index);

/** \brief Enable multiple S/PDIF instances with synchronized PIO start
 * \ingroup audio_spdif
 *
 * Primes DMA for all instances, then starts all PIO state machines
 * simultaneously using pio_enable_sm_mask_in_sync(). SMs on the same
 * PIO block start on the exact same clock cycle.
 *
 * \param instances Array of pointers to initialized instances
 * \param count Number of instances
 */
void audio_spdif_enable_sync(audio_spdif_instance_t *instances[], uint count);

/*! \brief Prepare-only half of audio_spdif_enable_sync
 * \ingroup pico_audio_spdif_multi
 *
 * Rewinds the SMs, arms the ring DMAs and marks the instances enabled without
 * starting the SMs; returns the SM mask for the shared PIO block so the
 * caller can perform one combined pio_enable_sm_mask_in_sync (used by the
 * I2S clock-slave path to gate the start on an external LRCLK edge).
 */
uint32_t audio_spdif_enable_sync_prepare(audio_spdif_instance_t *instances[], uint count);

/*! \brief Program the instance's PIO clock divider for a sample rate
 * \ingroup pico_audio_spdif_multi
 *
 * Recomputes and writes the nominal divider for \p sample_freq and updates
 * the instance's rate bookkeeping (inst->freq) and the IEC 60958-3 channel
 * status rate byte.  Nothing retunes lazily, so a rate change must call this
 * for every set-up instance and then restart the outputs in sync.
 */
void audio_spdif_apply_pio_frequency(audio_spdif_instance_t *inst, uint32_t sample_freq);

/** \brief Reset DMA-starvation counters
 * \ingroup audio_spdif
 */
void audio_spdif_reset_dma_starvations(void);

/** \brief Get total DMA-starvation events across all instances
 * \ingroup audio_spdif
 */
uint32_t audio_spdif_get_dma_starvations(void);

/** \brief Get DMA-starvation events for one instance index (0..3)
 * \ingroup audio_spdif
 */
uint32_t audio_spdif_get_dma_starvations_instance(uint index);

#ifdef __cplusplus
}
#endif

#endif //_AUDIO_SPDIF_H
