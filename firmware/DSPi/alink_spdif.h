/*
 * alink_spdif.h - S/PDIF as an alink time-base transport: the roles (WP FWSP2)
 *
 * Normative text: alink_protocol_spec.md sections 11 and 12 (the contract, the
 * carriage, the source's counter and epochs, the transmit rules, the
 * follower), extension_channel_spec.md 2.10 (EXT_ALINK_SPDIF 0x8020,
 * EXT_ALINK_SPDIF_STATUS 0x8021, EXT_ALINK_SPDIF_CTRL 0x8022, EXT_APPLY_AT_FRAME
 * time base 3) and 2.11 (EXT_TIME_BASE, 0xE4), test_signals_spec.md 15 (ops 9
 * and 10 on time base 3).  The carriage itself (the 48-byte superframe, its
 * validity, the transmit table) is WP FWSP1 (alink/alink_spdif_codec.h,
 * pico_audio_spdif_multi's user-bit table).
 *
 * Portable C11 like alink_usb/alink_usb_core.c: no SDK, the firmware glue
 * (alink_spdif_port.c) and the host test / simulation
 * (tests/test_alink_spdif.c, tools/alink_spdif_sim) run the same code.  The
 * platform comes in through the alink_spdif_port_*() functions at the end.
 * Nothing here is on the per-sample path of mode 0, and nothing here is
 * RAM-resident: the only RAM code of the roles is the transmit filler in
 * alink_spdif_port.c (it copies a block's staged superframe, built here).
 *
 * ONE ROLE PER UNIT (2.10): EXT_ALINK_SPDIF.mode 0 off, 1 follower, 2 source.
 *
 * SOURCE (mode 2).  F counts the frames the outputs play: F of the frame at
 * output-ring position x since the synchronized start is origin + x, origin a
 * multiple of 192, so block b of the transmit table (frames since the ring
 * reset / 192) starts at F = origin + 192 b.  Every output restart is a new
 * epoch: a new origin (past every F played before), byte 28 incremented, a
 * disturbance and 0xE4 EPOCH.  The superframes are built here, from the main
 * loop, into a small staging table eight blocks ahead of the reader
 * (alink_spdif_src_stage()); the firmware's RAM filler copies a block's entry
 * into the transmit table in the same pass as its first stamp.  A broadcast
 * (EXT_ALINK_SPDIF_CTRL) restages the blocks not yet stamped.  The source's
 * own apply queue (time base 3, and its copy of a broadcast) acts on the
 * sample written at output position F: the pipeline splits its block there
 * (alink_spdif_src_split()).
 *
 * FOLLOWER (mode 1, the active input is S/PDIF).  The firmware reads the
 * receiver's raw subframe words through alink_spdif_rx_words(), which collects
 * the U bits of every 384-word block (the receive FIFO is block aligned: word
 * index mod 384 is the subframe's place in its block) and counts the F of the
 * next word read (rd_f).  A completed block is accepted as the line receiver
 * accepts a superframe (validity, the detected rate, the source id, the epoch
 * and F continuity); its F then fixes rd_f exactly.  The phase lock is
 * alink_phase.c's: e = F_arr - (next_w - fill) - D, F_arr = rd_f + the words
 * the receive DMA has written and the reader has not yet taken (half a frame
 * resolution).  Anchor, verify, whole-frame corrections, PI, holdover (on a
 * carrier loss, or after 8 coasted blocks: 12.4 d), resume or re-anchor,
 * lock error (0.5 s without a valid superframe on a locked stream, D out of
 * range, a rate it cannot lock at) with on_lock_error 0 mute / 1 unaligned.
 */

#ifndef ALINK_SPDIF_H
#define ALINK_SPDIF_H

#include <stdbool.h>
#include <stdint.h>

#include "alink/alink_phase.h"
#include "alink/alink_spdif_codec.h"
#include "timebase.h"

// ---------------------------------------------------------------------------
// Wire (extension_channel_spec.md 2.10)
// ---------------------------------------------------------------------------

#define ALINK_SPDIF_CFG_LEN         8u      // EXT_ALINK_SPDIF
#define ALINK_SPDIF_STATUS_LEN      48u     // EXT_ALINK_SPDIF_STATUS
#define ALINK_SPDIF_CTRL_LEN        16u     // EXT_ALINK_SPDIF_CTRL (the EXT_APPLY_AT_FRAME layout)
#define ALINK_SPDIF_STORE_LEN       6u      // persisted: mode, flags, d (u16 LE), on_lock_error, tx_mask

#define ALINK_SPDIF_MODE_OFF        0u
#define ALINK_SPDIF_MODE_FOLLOWER   1u
#define ALINK_SPDIF_MODE_SOURCE     2u

#define ALINK_SPDIF_FLAG_AUTO_REANCHOR 0x01u
#define ALINK_SPDIF_FLAGS_DEFAULT   ALINK_SPDIF_FLAG_AUTO_REANCHOR
#define ALINK_SPDIF_D_DEFAULT       192u
#define ALINK_SPDIF_LOCKERR_MUTE    0u
#define ALINK_SPDIF_LOCKERR_PLAY    1u

// EXT_ALINK_SPDIF_STATUS.flags
#define ALINK_SPDIF_ST_SUPERFRAME   0x01u   // a valid superframe within the last 8 blocks
#define ALINK_SPDIF_ST_TIME_BASE    0x02u   // LOCKED or HOLDOVER
#define ALINK_SPDIF_ST_HOLDOVER     0x04u
#define ALINK_SPDIF_ST_LOCK_ERROR   0x08u
#define ALINK_SPDIF_ST_SOURCE       0x10u
#define ALINK_SPDIF_ST_FOREIGN      0x20u   // locked to a stream with no valid superframe

#define ALINK_SPDIF_RATE_UNKNOWN    0xFFu

// Timing (alink_protocol_spec.md 12.4 d, 12.5).
#define ALINK_SPDIF_LOCKERR_US      500000u     // 0.5 s without a valid superframe on a locked stream
#define ALINK_SPDIF_HOLD_MAX_US     10000000u   // holdover lasts at most 10 s
#define ALINK_SPDIF_SF_RECENT       8u          // STATUS flag SUPERFRAME: within the last 8 blocks
// d_min (2.10.2): the block cap, the input batch and this margin (the
// reader's lag behind the live DMA position and main-loop jitter, frames).
#define ALINK_SPDIF_RX_MARGIN       32u
// The receive FIFO's block: 384 subframe words (spdif_rx.h SPDIF_BLOCK_SIZE).
#define ALINK_SPDIF_BLOCK_WORDS     (2u * ALINK_SPDIF_BLOCK_FRAMES)
// The raw word's Z (B) preamble code in bits 0-3 and its U bit (bit 29).
#define ALINK_SPDIF_SYNC_MASK       0xFu
#define ALINK_SPDIF_SYNC_B          0xFu
#define ALINK_SPDIF_UBIT_SHIFT      29u
// The source's staging table (blocks built ahead of the reader).
#define ALINK_SPDIF_STAGE_BLOCKS    8u
_Static_assert((ALINK_SPDIF_STAGE_BLOCKS & (ALINK_SPDIF_STAGE_BLOCKS - 1u)) == 0, "a power of two");

// ---------------------------------------------------------------------------
// The unit
// ---------------------------------------------------------------------------

typedef struct {
    volatile uint32_t block;        // block index (since the ring reset)
    volatile bool     valid;
    uint8_t  sf[ALINK_SPDIF_SF_LEN];
} alink_spdif_stage_t;

typedef struct {
    // EXT_ALINK_SPDIF (persisted)
    uint8_t  mode, flags, on_lock_error, tx_mask;
    uint16_t d;
    // This unit's serial bytes 0-1 (the unique board id's bytes 7 and 6).
    uint8_t  serial0, serial1;
    // The contract (timebase.h): state, disturbances, 0xE4 of time base 3.
    tb_track_t tbk;
    // The apply queue of time base 3 (the source's own, or the follower's).
    alink_apply_q_t aq;
    // Counters (from boot; EXT_ALINK_SPDIF_STATUS)
    uint32_t lock_count, loss_count, sf_errors, reanchors;
    bool     was_locked;                    // the last state seen was LOCKED (lock/loss counts)

    // ---- source (mode 2) ----
    bool     src_running;                   // the outputs play the current epoch
    bool     src_epoch_open;                // an origin is set (the next start is a new epoch)
    bool     src_epoch_pending;             // a restart is in progress (outputs stopped)
    uint64_t origin;                        // F of output-ring frame 0 of this epoch
    uint64_t f_seen;                        // the highest F known played (the next origin is beyond it)
    uint8_t  epoch;
    alink_spdif_src_t src;                  // the codec's view: flags, rate, ids, epoch, the broadcast
    bool     bc_pending;                    // a broadcast is carried
    uint8_t  bc_last_op, bc_last_result;
    uint64_t bc_last_frame;
    bool     stage_on;                      // the filler may serve the staging table
    alink_spdif_stage_t stage[ALINK_SPDIF_STAGE_BLOCKS];

    // ---- follower (mode 1) ----
    alink_phase_t ph;                       // the phase lock (D = ph.d_local, hops 0)
    bool     active;                        // the input source is S/PDIF (set by the glue)
    bool     rx_locked;                     // the receiver is locked to a stream at a rate it can play
    uint8_t  rx_rate;                       // rate code of the stream (ALINK_SPDIF_RATE_*, 0xFF unknown)
    uint32_t fs;                            // pipeline rate (frames per second)
    // The receive word stream: the next word's FIFO index, U bit collection.
    bool     rd_have;                       // rd_idx is known
    uint32_t rd_idx;                        // FIFO word index of the next word (mod fifo_words)
    uint32_t fifo_words;
    uint16_t acc_pos;                       // words of the current block collected (0..384)
    bool     acc_ok;                        // ... contiguous, starting at a Z preamble
    uint8_t  acc[ALINK_SPDIF_SF_LEN];
    // The timeline of the stream: F of the next word pair read.
    bool     rd_valid;
    uint64_t rd_f;
    uint64_t rd_f_expect;                   // rd_f at the end of the block just completed
    uint16_t src_id;
    uint8_t  src_epoch;
    bool     sf_seen;                       // a valid superframe since the receiver locked
    uint32_t blocks_since_sf;               // blocks completed since the last valid one
    uint32_t coast;                         // consecutive coasted blocks
    uint32_t last_sf_us;                    // when the last valid superframe was accepted
    uint32_t rx_lock_us;                    // when the receiver locked (lock error timer)
    bool     sf_mute;                       // the source's flag b0 (system mute)
    alink_spdif_sf_t sf_last;
    alink_apply_op_t sf_taken;              // the last broadcast taken (taken once)
    bool     sf_taken_have;
    // Phase-lock run state (mirrors the line's alink_input.c)
    bool     prefilling;                    // the outputs are drained; the timeline prefills
    bool     drained;                       // drained by the anchor, not started yet
    uint32_t drop_rows;                     // whole frames still to drop (anchor correction)
    bool     lock_err;                      // in lock error (status flag, events 9/10)
    uint8_t  lock_err_reason;               // TB_RS_CONFIG / TB_RS_LOCK_ERROR
    uint32_t hold_us;                       // holdover since
    uint64_t hold_f;                        // F arriving at hold_us
    bool     hold_rx;                       // the holdover keeps the receiver's word count (coast holdover)
    bool     reanchor_req;                  // re-anchor asked (the main loop's handshake)
    bool     resuming;                      // placed back on the held timeline, verifying (HOLDOVER)
    uint32_t tick_ms;                       // last service call
} alink_spdif_t;

void alink_spdif_init(alink_spdif_t *u, uint8_t serial0, uint8_t serial1);

// EXT_ALINK_SPDIF.  set: 0 accepted, else the refusal (TB_ERR_VALUE, or
// TB_ERR_UNSUPPORTED for mode 2 without an S/PDIF-type slot, given by
// spdif_slots: bit n = output slot n is S/PDIF-type).  The limits of mode 1's
// D are the follower's d_min..d_max now (alink_spdif_limits()).
#define ALINK_SPDIF_ERR_UNSUPPORTED 0x15u   // VC_ERR_UNSUPPORTED
uint8_t alink_spdif_set_config(alink_spdif_t *u, const uint8_t p[ALINK_SPDIF_CFG_LEN], uint8_t spdif_slots);
void    alink_spdif_get_config(const alink_spdif_t *u, uint8_t o[ALINK_SPDIF_CFG_LEN]);
void    alink_spdif_store(const alink_spdif_t *u, uint8_t s[ALINK_SPDIF_STORE_LEN]);
// Implausible stored bytes keep the defaults (mode 0).
void    alink_spdif_load(alink_spdif_t *u, const uint8_t s[ALINK_SPDIF_STORE_LEN]);

// d_min / d_max of the follower at block cap `cap`, input batch `batch` and
// the rings' writable span `span` (2.10.2): d_min = cap + batch + 32,
// d_max = span - cap.
void alink_spdif_limits(alink_spdif_t *u, uint32_t cap, uint32_t batch, uint32_t span);
// The ring prefill of the follower's anchor: D less the batch and the
// margin, within [cap, span - cap]; `target` when the phase lock does not run.
uint32_t alink_spdif_ring_target(const alink_spdif_t *u, uint32_t target, uint32_t cap, uint32_t batch,
                                 uint32_t span);

// The contract state of the role (11.2); TB_ST_NONE in mode 0.
uint8_t alink_spdif_state(const alink_spdif_t *u);
// EXT_ALINK_SPDIF_STATUS (48 bytes); f_now: F now (follower: arriving, source:
// playing), valid or 0.
void alink_spdif_get_status(const alink_spdif_t *u, uint64_t f_now, float vcxo_ppm, uint16_t fill,
                            uint8_t o[ALINK_SPDIF_STATUS_LEN]);

// Rate code of a rate in Hz (0xFF: not one S/PDIF carries).
static inline uint8_t alink_spdif_rate_code(uint32_t hz) {
    return hz == 44100u ? ALINK_SPDIF_RATE_44K1 : hz == 48000u ? ALINK_SPDIF_RATE_48K :
           hz == 96000u ? ALINK_SPDIF_RATE_96K : ALINK_SPDIF_RATE_UNKNOWN;
}

// ---- the source ----

// The outputs are about to restart (teardown): a new epoch follows.  F
// played now is f_now; the transmit table is emptied by the caller.
void alink_spdif_src_stopping(alink_spdif_t *u, uint64_t f_now);
// The outputs started in sync (every slot from ring position 0).
void alink_spdif_src_started(alink_spdif_t *u);
// Mode 2 just written or loaded: the outputs run (`running`, slot 0 at ring
// position `played`) or not; a first epoch starts.
void alink_spdif_src_sync(alink_spdif_t *u, bool running, uint32_t played);
// F of output-ring position `pos` (slot 0's frames written or played).
static inline uint64_t alink_spdif_src_f(const alink_spdif_t *u, uint32_t pos) { return u->origin + pos; }
// Build the staged superframes of blocks reader_block .. reader_block + 7
// that are not staged yet (or all of them: `restage`, after a broadcast).
// Main loop.
void alink_spdif_src_stage(alink_spdif_t *u, uint32_t reader_block, bool restage);
// The staged superframe of block b, or NULL (the RAM filler's lookup).
static inline const uint8_t *alink_spdif_src_staged(const alink_spdif_t *u, uint32_t b) {
    const alink_spdif_stage_t *s = &u->stage[b & (ALINK_SPDIF_STAGE_BLOCKS - 1u)];
    return (u->stage_on && s->valid && s->block == b) ? s->sf : 0;
}
// EXT_ALINK_SPDIF_CTRL OUT (16 bytes): f_play = F the source plays now,
// f_write = F of the next sample written.  0 accepted, else TB_ERR_*.
uint8_t alink_spdif_ctrl_write(alink_spdif_t *u, const uint8_t p[ALINK_SPDIF_CTRL_LEN], uint64_t f_play,
                               uint64_t f_write);
void    alink_spdif_ctrl_read(const alink_spdif_t *u, uint8_t o[ALINK_SPDIF_CTRL_LEN]);
// The pipeline processes `n` samples written from output position F = f:
// returns k <= n, the samples to process before the next op is due (k = n:
// none in this block); with k = 0 the due op(s) are run first (late ones
// report late), so the caller loops: run k, shift, ask again.
uint32_t alink_spdif_src_split(alink_spdif_t *u, uint64_t f, uint32_t n);
// Any op of time base 3 pending on the source (the pipeline's split hook).
static inline bool alink_spdif_src_armed(const alink_spdif_t *u) {
    return u->mode == ALINK_SPDIF_MODE_SOURCE && u->aq.n != 0;
}

// ---- the follower ----

// The receiver's state, once per main-loop pass before the service: locked
// to a stream (at a rate code `rate`), the pipeline rate fs.
void alink_spdif_rx_state(alink_spdif_t *u, bool locked, uint8_t rate, uint32_t fs, uint32_t now_us);
// Raw subframe words just read from the receive FIFO at word index idx
// (mod fifo_words): U bits, the block, F continuity, superframe acceptance.
void alink_spdif_rx_words(alink_spdif_t *u, const uint32_t *w, uint32_t n, uint32_t idx, uint32_t now_us);
// One delivery pass of the follower (the receiver locked and debounced):
// reads the FIFO through the port, places and delivers frames on the
// timeline (or discards them), splits blocks at due ops.  Returns frames
// delivered.
uint32_t alink_spdif_poll(alink_spdif_t *u, uint32_t now_us);
// Main loop, every pass in mode 1 (also while the receiver is not locked):
// the lock-error and holdover timers, holdover silence, the generator poll,
// the contract tracker.
void alink_spdif_service(alink_spdif_t *u, uint32_t now_us);
// The follower runs the phase lock (mode 1, S/PDIF the source, D in range,
// a rate it can lock at, not unaligned by on_lock_error).
bool alink_spdif_engaged(const alink_spdif_t *u);
// Mode 1 playing unaligned (lock error with on_lock_error 1): the stock
// S/PDIF flow plays, the superframes are still watched.
bool alink_spdif_unaligned(const alink_spdif_t *u);
// The phase lock owns the output rings (pipeline_source_owns_rings()).
static inline bool alink_spdif_owns_rings(const alink_spdif_t *u) {
    return u->ph.started || u->prefilling;
}
// Muted by the role (anchoring, holdover, waiting for a superframe, lock
// error with on_lock_error 0): the glue keeps the soft mute refreshed.
bool alink_spdif_muted(const alink_spdif_t *u);

// The main loop's anchor handshake (as the line's): wanted -> mute and drain
// -> on_drain -> prefill (poll) -> anchor_wait() <= 0 -> start the outputs in
// sync -> on_enabled -> verify (poll) -> LOCKED.
bool  alink_spdif_anchor_wanted(const alink_spdif_t *u);
void  alink_spdif_on_drain(alink_spdif_t *u);
// Frames still to wait before the start (<= 0: start now); 1e9 while the
// prefill or the timeline is missing.
float alink_spdif_anchor_wait(alink_spdif_t *u);
void  alink_spdif_on_enabled(alink_spdif_t *u);
// The anchor drained the outputs and will not start them now (the stream
// left, lock error): the main loop restarts them and calls on_restarted.
bool  alink_spdif_outputs_orphaned(const alink_spdif_t *u);
void  alink_spdif_on_restarted(alink_spdif_t *u);
// Somebody else restarted the output rings (preset load, flash write, rate
// or type change): the follower's timeline is gone (re-anchor).
void  alink_spdif_outputs_reset(alink_spdif_t *u);
// The input left S/PDIF, or mode changed away from 1: back to stock.
void  alink_spdif_stop(alink_spdif_t *u, uint8_t reason);
// One servo tick (75 ms) in phase lock: *trim (ppm, added to the rate
// feed-forward).  False: the stock fill servo runs (not engaged); *hold:
// a holdover, leave the VCXO where it is.
bool  alink_spdif_servo(alink_spdif_t *u, float *trim, bool *hold);
// F arriving now (follower) for the status and EXT_TIME_BASE; false: none.
bool  alink_spdif_f_now(alink_spdif_t *u, uint64_t *f, uint32_t now_us);

// ---- apply (EXT_APPLY_AT_FRAME, time base 3) ----

// OUT with byte 12 = 3: 0 accepted, else TB_ERR_*.  t_write: T of the next
// sample entering the pipeline (follower: next_w; source: F of the next
// sample written).
uint8_t alink_spdif_apply(alink_spdif_t *u, const uint8_t p[ALINK_APPLY_LEN], uint64_t t_write);
void    alink_spdif_cancel_all(alink_spdif_t *u);
void    alink_spdif_drop_op9(alink_spdif_t *u, uint8_t apply_result);

// ---- the time-base provider (timebase.c slot TB_SLOT_SPDIF) ----
void alink_spdif_tb_register(alink_spdif_t *u);

// ---------------------------------------------------------------------------
// Port: the firmware (alink_spdif_port.c) or the host test / simulation
// ---------------------------------------------------------------------------

extern const alink_apply_limits_t alink_spdif_apply_limits;
uint32_t alink_spdif_port_irq_save(void);
void     alink_spdif_port_irq_restore(uint32_t s);
uint32_t alink_spdif_port_now_us(void);
// Follower receive FIFO: frames readable now; read up to n frames (whole
// pairs, contiguous): *w the words, *idx the FIFO word index of w[0].
uint32_t alink_spdif_port_rx_avail(void);
uint32_t alink_spdif_port_rx_read(uint32_t n, const uint32_t **w, uint32_t *idx);
// The receive DMA's live write position: words written and not yet read
// (with interrupts masked by the caller); false: none known now.
bool     alink_spdif_port_rx_ahead(uint32_t *words);
// Deliver k frames of raw words into the pipeline (preamp,
// process_input_block), or k frames of silence (w NULL).
void     alink_spdif_port_deliver(const uint32_t *w, uint32_t k);
// producer_block_bounds(): most frames per call now, *min the batch.
uint32_t alink_spdif_port_bounds(uint32_t *min);
// Slot 0's ring: fractional fill and frames played (interrupts masked by
// the caller), whole fill.
bool     alink_spdif_port_play(float *fill, uint32_t *played);
uint32_t alink_spdif_port_fill(void);
uint32_t alink_spdif_port_ring_span(void);
// Run an op of time base 3 at its sample (late: it had passed).
uint8_t  alink_spdif_port_apply(const alink_apply_op_t *op, bool late, bool locked);
void     alink_spdif_port_apply_result(uint8_t op, uint8_t result, uint32_t frame);
// The generator (siggen.h).  op 9 written: 0 accepted, else the refusal.
uint8_t  alink_spdif_port_op9_accept(uint64_t t, uint64_t t_now, uint32_t fs);
bool     alink_spdif_port_op9_broadcast(uint64_t t, uint64_t t_now, uint32_t fs);
void     alink_spdif_port_op9_poll(uint64_t t_now, uint32_t fs);
void     alink_spdif_port_cancel_all(void);
// The role cancelled a pending op 9 itself (it stopped): CANCELLED_LOSS.
void     alink_spdif_port_op9_lost(void);
// The prefill target follows D (output_latency_apply()).
void     alink_spdif_port_ring_target(void);
// The source's flag b3 (a live USB stream).
bool     alink_spdif_port_usb_live(void);
// The follower's VCXO has pulled in to the stream's rate (the long rate
// window valid, within 1 ppm of the feed-forward): the anchor may run.
bool     alink_spdif_port_rate_ready(void);

#endif // ALINK_SPDIF_H
