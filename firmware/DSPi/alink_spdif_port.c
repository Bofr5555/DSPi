/*
 * alink_spdif_port.c - the firmware's port and glue of the S/PDIF time-base
 * roles (WP FWSP2; alink_spdif.h).  Flash, except the transmit filler.
 *
 * The roles' unit is alink_spdif (.bss).  The main loop calls
 * alink_spdif_fw_service() every pass; spdif_input.c hands the follower's
 * deliveries and servo ticks here (alink_spdif_fw_poll(),
 * alink_spdif_fw_servo()); main.c runs the anchor handshake and reports every
 * output stop and start (alink_spdif_fw_outputs_stopping(), _started()); the
 * pipeline's split hook (process_input_block(), audio_pipeline.c) calls
 * alink_spdif_fw_src_block() while a source op of time base 3 is pending.
 *
 * RAM: only alink_spdif_tx_filler() (an indirect call from the transmit
 * library's stampers, which also run from the flash-blackout timer interrupt)
 * and the armed flag.  The filler copies a block's superframe that the main
 * loop staged (alink_spdif_src_stage()); it builds nothing and touches no
 * flash, so it serves the staging table in a blackout too, as far as it
 * reaches (eight blocks; spec 12.4 c: beyond it, zero user bits).
 */

#include "alink_spdif_port.h"

#include <stddef.h>
#include <string.h>

#include "audio_input.h"
#include "audio_pipeline.h"
#include "bulk_params.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "input_capture_arena.h"
#include "notify.h"
#include "output_latency.h"
#include "siggen.h"
#include "spdif_input.h"
#include "usb_audio.h"
#include "input_servo.h"
#include "soft_vcxo.h"
#include "pico/audio_i2s_multi.h"
#include "pico/audio_spdif.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "spdif_rx.h"
#if DSPI_ALINK
#include "alink_input.h"
#endif
#if DSPI_ALINK_USB
#include "alink_usb.h"
#endif

alink_spdif_t alink_spdif;
volatile bool alink_spdif_split_armed;      // a source op of time base 3 pending (the pipeline's hook)

extern uint8_t output_types[];
extern audio_spdif_instance_t *spdif_instance_ptrs[];
extern audio_i2s_instance_t *i2s_instance_ptrs[];

const alink_apply_limits_t alink_spdif_apply_limits = { PRESET_SLOTS, NUM_OUTPUT_CHANNELS, MAX_DELAY_SAMPLES, true };

#define SPDIF_MUTE_HOLD_MS      5u          // soft-mute refresh while muted by the role
#define SPDIF_ANCHOR_SPIN_US    1100u       // the anchor waits in place for the last ms or so

// ---------------------------------------------------------------------------
// The transmit filler (RAM): the staged superframe of block `block`
// ---------------------------------------------------------------------------

static void __not_in_flash_func(alink_spdif_tx_filler)(uint32_t block) {
    const uint8_t *sf = alink_spdif_src_staged(&alink_spdif, block);
    if (sf) (void)audio_spdif_ubits_fill(block, sf);
}

// ---------------------------------------------------------------------------
// Port (alink_spdif.h)
// ---------------------------------------------------------------------------

uint32_t alink_spdif_port_irq_save(void) { return save_and_disable_interrupts(); }
void alink_spdif_port_irq_restore(uint32_t s) { restore_interrupts(s); }
uint32_t alink_spdif_port_now_us(void) { return time_us_32(); }

uint32_t alink_spdif_port_rx_avail(void) { return spdif_rx_get_fifo_count_live() / 2u; }

uint32_t alink_spdif_port_rx_read(uint32_t n, const uint32_t **w, uint32_t *idx) {
    uint32_t *buf;
    uint32_t got = spdif_rx_read_fifo_live(&buf, 2u * n) & ~1u;
    *w = buf;
    *idx = (uint32_t)(buf - input_capture_arena.spdif_fifo);
    return got / 2u;
}

// The live write position: the busy receive DMA channel's write address (the
// library's own live count reads 0 in the window between a block's DMA
// completion and its interrupt).  Interrupts masked by the caller.
bool alink_spdif_port_rx_ahead(uint32_t *words) {
    if (!alink_spdif.rd_have) return false;
    const uint8_t chs[2] = { PICO_SPDIF_RX_DMA_CH0, PICO_SPDIF_RX_DMA_CH1 };
    uintptr_t base = (uintptr_t)input_capture_arena.spdif_fifo;
    for (int i = 0; i < 2; i++) {
        if (!dma_channel_is_busy(chs[i])) continue;
        uint32_t idx = (uint32_t)((dma_hw->ch[chs[i]].write_addr - base) / sizeof(uint32_t));
        if (idx > SPDIF_RX_FIFO_SIZE) return false;
        uint32_t d = (idx + SPDIF_RX_FIFO_SIZE - alink_spdif.rd_idx) % SPDIF_RX_FIFO_SIZE;
        if (d > SPDIF_RX_FIFO_SIZE / 2u) return false;      // not ahead of the reader: no position
        *words = d;
        return true;
    }
    return false;
}

void alink_spdif_port_deliver(const uint32_t *w, uint32_t k) {
    spdif_input_deliver_words(w, k);
}

uint32_t alink_spdif_port_bounds(uint32_t *min) { return producer_block_bounds(min); }
bool alink_spdif_port_play(float *fill, uint32_t *played) { return slot_ring_play_position(0, fill, played); }
uint32_t alink_spdif_port_fill(void) { return slot_ring_fill_frames(0); }
uint32_t alink_spdif_port_ring_span(void) { return output_latency_ring_span(); }

uint8_t alink_spdif_port_apply(const alink_apply_op_t *op, bool late, bool locked) {
    switch (op->op) {
    case ALINK_OP_SIGGEN_START:
        // The generator's sample 0 is this sample; not ready muted or
        // without a LOCKED time base; never late.
        return siggen_tb_fire(TB_ID_SPDIF, op->frame, late, locked && !user_mute);
    case ALINK_OP_SIGGEN_STOP:
        siggen_tb_stop_op(op->arg);
        return ALINK_APPLY_APPLIED;
    case ALINK_OP_MUTE:
    case ALINK_OP_UNMUTE: {
        user_mute = op->op == ALINK_OP_MUTE;
        uint8_t v = user_mute ? 1 : 0;
        notify_param_write(offsetof(WireBulkParams, user_volume.user_mute), sizeof(uint8_t), &v);
        return ALINK_APPLY_APPLIED;
    }
    case ALINK_OP_PRESET_LOAD:
        pending_preset_load_slot = op->arg;
        __dmb();
        preset_load_pending = true;
        return ALINK_APPLY_APPLIED;
    case ALINK_OP_MASTER_VOLUME:
        update_master_volume((float)op->value * 0.01f);
        return ALINK_APPLY_APPLIED;
    case ALINK_OP_OUT_DELAY: {
        float fs = (float)(audio_state.freq ? audio_state.freq : 48000u);
        float ms = ((float)op->value + 0.5f) * 1000.0f / fs;
        channel_delays_ms[CH_OUT_1 + op->arg] = ms;
        dsp_update_delay_samples(fs);
        notify_param_write((uint16_t)(offsetof(WireBulkParams, delays.delay_ms) +
                                      (CH_OUT_1 + op->arg) * sizeof(float)), sizeof(float), &ms);
        return ALINK_APPLY_APPLIED;
    }
    case ALINK_OP_CLICK:
        // Never on the PDM sub, never a muted unit (the source's system-mute
        // flag is the caller's `locked`).
        if (op->arg >= NUM_SPDIF_INSTANCES * 2 || user_mute || !locked) return ALINK_APPLY_REFUSED;
        pipeline_click_output = (int8_t)op->arg;
        return ALINK_APPLY_APPLIED;
    default:
        (void)late;
        return ALINK_APPLY_REFUSED;         // SYNC_PULSE: no sync GPIO here
    }
}

void alink_spdif_port_apply_result(uint8_t op, uint8_t result, uint32_t frame) {
    notify_push_apply_result(op, result, frame);
}

uint8_t alink_spdif_port_op9_accept(uint64_t t, uint64_t t_now, uint32_t fs) {
    return siggen_tb_accept(TB_ID_SPDIF, t, t_now, fs);
}
bool alink_spdif_port_op9_broadcast(uint64_t t, uint64_t t_now, uint32_t fs) {
    return siggen_tb_accept_broadcast(TB_ID_SPDIF, t, t_now, fs);
}
void alink_spdif_port_op9_poll(uint64_t t_now, uint32_t fs) { siggen_tb_poll(TB_ID_SPDIF, t_now, fs); }
void alink_spdif_port_cancel_all(void) { siggen_tb_cancel_all(); }
void alink_spdif_port_op9_lost(void) { siggen_tb_dropped(TB_RES_CANCELLED_LOSS); }

void alink_spdif_port_ring_target(void) {
    output_latency_apply(audio_state.freq ? audio_state.freq : 48000u, active_input_source);
}

bool alink_spdif_port_usb_live(void) {
    return active_input_source == INPUT_SOURCE_USB && usb_audio_stream_active();
}

bool alink_spdif_port_rate_ready(void) { return spdif_input_rate_ready(); }

// ---------------------------------------------------------------------------
// Glue
// ---------------------------------------------------------------------------

// Output slots of S/PDIF type, bit n = slot n.
static uint8_t spdif_slot_mask(void) {
    uint8_t m = 0;
    for (unsigned i = 0; i < NUM_SPDIF_INSTANCES && i < 8; i++)
        if (output_types[i] != OUTPUT_TYPE_I2S) m |= (uint8_t)(1u << i);
    return m;
}

static bool ring_counters(uint32_t *wr, uint32_t *played) {
    uint32_t s = save_and_disable_interrupts();
    bool ok = slot_ring_counters(0, wr, played);
    restore_interrupts(s);
    return ok;
}

static bool outputs_running(void) {
    if (output_types[0] == OUTPUT_TYPE_I2S) {
        audio_i2s_instance_t *inst = i2s_instance_ptrs[0];
        return inst && inst->ring && inst->enabled;
    }
    audio_spdif_instance_t *inst = spdif_instance_ptrs[0];
    return inst && inst->ring && inst->enabled;
}

static void refresh_limits(void) {
    uint32_t fs = audio_state.freq ? audio_state.freq : 48000u;
    const OutputLatencyOption *opt = output_latency_resolve(output_latency_requested(), INPUT_SOURCE_SPDIF);
    uint32_t cap = output_latency_block_cap(opt, fs);
    uint32_t batch = cap > 1u ? cap - 1u : cap;    // the block cap is the batch plus one frame
    alink_spdif_limits(&alink_spdif, cap, batch, output_latency_ring_span());
}

uint32_t alink_spdif_fw_ring_target(uint32_t fs, uint32_t target, uint32_t cap) {
    (void)fs;
    uint32_t batch = cap > 1u ? cap - 1u : cap;
    alink_spdif_limits(&alink_spdif, cap, batch, output_latency_ring_span());
    return alink_spdif_ring_target(&alink_spdif, target, cap, batch, output_latency_ring_span());
}

// Which S/PDIF slots carry the superframe (mode 2: tx_mask, 0 = every
// S/PDIF-type slot; otherwise none: zero user bits).
static void apply_carry(void) {
    uint8_t m = 0;
    if (alink_spdif.mode == ALINK_SPDIF_MODE_SOURCE) m = alink_spdif.tx_mask ? alink_spdif.tx_mask : spdif_slot_mask();
    for (unsigned i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) continue;
        audio_spdif_instance_t *inst = spdif_instance_ptrs[i];
        if (inst) audio_spdif_ubits_set_carry(inst, (m >> i) & 1u);
    }
}

// The source's table: stage eight blocks ahead of the reader and fill the
// transmit table's window (the reader's block .. + 3) from it.  The staging
// follows the reader in every pass, also while the window is empty (the
// stamped frontier in the table's last block: nothing to fill, but the
// stampers' filler still needs the blocks ahead; review fwbc F4).
static void src_stage(bool restage) {
    uint32_t first, last, reader;
    if (!audio_spdif_ubits_reader(&reader)) {
        // No carrying slot enabled (outputs stopped, prefilling): stage from
        // block 0 of the coming epoch.
        if (alink_spdif.mode == ALINK_SPDIF_MODE_SOURCE && !alink_spdif.src_running)
            alink_spdif_src_stage(&alink_spdif, 0, restage);
        return;
    }
    alink_spdif_src_stage(&alink_spdif, reader, restage);
    if (!audio_spdif_ubits_window(&first, &last)) return;
    for (uint32_t b = first; b != last + 1u; b++) {
        const uint8_t *sf = alink_spdif_src_staged(&alink_spdif, b);
        if (sf) (void)audio_spdif_ubits_fill(b, sf);
    }
}

// Slot 0's ring position `played` in the transmit library's unwrapped frame
// numbering (192 per block from the ring reset): the 32-bit ring counter wraps
// after 24.8 h at 48 kHz, the block index does not (review fwbc F1).  The
// carrying slots must be set (apply_carry()); the counter itself when none
// runs.
static uint64_t ring_pos64(uint32_t played) {
    uint32_t reader;
    if (!audio_spdif_ubits_reader(&reader)) return played;
    uint64_t base = (uint64_t)reader * ALINK_SPDIF_BLOCK_FRAMES;
    return base + (uint64_t)(int64_t)(int32_t)(played - (uint32_t)base);
}

static void update_armed(void) {
    alink_spdif_split_armed = alink_spdif_src_armed(&alink_spdif);
}

void alink_spdif_fw_init(void) {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    alink_spdif_init(&alink_spdif, id.id[7], id.id[6]);
    alink_spdif.fifo_words = SPDIF_RX_FIFO_SIZE;
    audio_spdif_ubits_set_filler(alink_spdif_tx_filler);
}

void alink_spdif_fw_tb_register(void) { alink_spdif_tb_register(&alink_spdif); }

bool alink_spdif_fw_following(void) {
    return alink_spdif.mode == ALINK_SPDIF_MODE_FOLLOWER && input_source_is_spdif(active_input_source);
}

bool alink_spdif_fw_engaged(void) { return alink_spdif_fw_following() && alink_spdif_engaged(&alink_spdif); }
bool alink_spdif_fw_owns_rings(void) {
    return alink_spdif_fw_engaged() && (alink_spdif_owns_rings(&alink_spdif) || alink_spdif.ph.hold);
}

void alink_spdif_fw_service(void) {
    alink_spdif_t *u = &alink_spdif;
    uint32_t now = time_us_32();
    uint32_t fs = audio_state.freq ? audio_state.freq : 48000u;
    u->fs = fs;
    bool follow = alink_spdif_fw_following();
    if (u->active && !follow) {
        // The input left S/PDIF: the follower stops (a mode change stopped
        // it at the write already).
        if (u->mode == ALINK_SPDIF_MODE_FOLLOWER) alink_spdif_stop(u, TB_RS_DISABLED);
        u->rx_locked = false;
    }
    u->active = follow;
    if (follow) {
        bool locked = spdif_input_get_state() == SPDIF_INPUT_LOCKED;
        alink_spdif_rx_state(u, locked, locked ? alink_spdif_rate_code(spdif_input_get_sample_rate()) : 0xFFu,
                             fs, now);
    }
    apply_carry();
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) {
        if (!u->src_epoch_open) {
            uint32_t wr = 0, played = 0;
            bool run = outputs_running() && ring_counters(&wr, &played);
            alink_spdif_src_sync(u, run, ring_pos64(played));
        }
        src_stage(false);
    }
    alink_spdif_service(u, now);
    if (follow && alink_spdif_muted(u)) pipeline_request_soft_mute(fs / 1000u * SPDIF_MUTE_HOLD_MS);
    update_armed();
}

uint32_t alink_spdif_fw_poll(void) {
    uint32_t n = alink_spdif_poll(&alink_spdif, time_us_32());
    update_armed();
    return n;
}

// One servo tick: the S/PDIF rate (hz, from the long window when long_ok)
// plus the phase trim.  False: the stock fill servo runs instead.
bool alink_spdif_fw_servo(float hz, bool long_ok) {
    float trim;
    bool hold;
    if (!alink_spdif_fw_engaged() || !alink_spdif_servo(&alink_spdif, &trim, &hold)) return false;
    if (hold) return true;                  // the holdover keeps the VCXO where it was
    // Phase-locked: the 32 ms windows of the first second after a relock
    // would drag the VCXO by several ppm (as the line's, review lk6 F4).
    if (!long_ok && alink_spdif.ph.started) return true;
    float v0 = soft_vcxo_current_ppm(), off = input_servo_feedforward_ppm() - v0;
    bool fast = !alink_spdif.ph.started && alink_spdif_muted(&alink_spdif) && (off > 4.0f || off < -4.0f);
    input_servo_apply_trim_slew(hz, trim, fast ? 2.0f : INPUT_SERVO_SLEW_PPM_PER_TICK);
    return true;
}

bool alink_spdif_fw_anchor_ready(void) {
    uint32_t fs = audio_state.freq ? audio_state.freq : 48000u;
    float wait = alink_spdif_anchor_wait(&alink_spdif);
    if (wait > (float)(fs / 1000u)) return false;
    // The last millisecond in place (main-loop passes are shorter): the start
    // lands on the delay to a fraction of a frame.
    uint32_t t0 = time_us_32();
    while (wait > 0.0f && time_us_32() - t0 < SPDIF_ANCHOR_SPIN_US) wait = alink_spdif_anchor_wait(&alink_spdif);
    return wait <= 0.0f;
}

// Every output stop / start (main.c: drain_and_disable_outputs(),
// complete_pipeline_reset(), enable_outputs_in_sync()).
void alink_spdif_fw_outputs_stopping(void) {
    alink_spdif_t *u = &alink_spdif;
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) {
        uint32_t wr, played;
        uint64_t f = (u->src_running && ring_counters(&wr, &played)) ? alink_spdif_src_f(u, played) : 0;
        audio_spdif_ubits_clear();          // before the group restart (FWSP1: no stale block)
        alink_spdif_src_stopping(u, f);
        alink_spdif_src_stage(u, 0, true);  // the prefill stamps the new epoch's blocks
        update_armed();
    }
}

void alink_spdif_fw_outputs_started(void) {
    alink_spdif_t *u = &alink_spdif;
    apply_carry();
    if (u->mode == ALINK_SPDIF_MODE_SOURCE) alink_spdif_src_started(u);
}

// complete_pipeline_reset(): somebody else restarted the rings under the
// follower's timeline.
void alink_spdif_fw_outputs_reset(void) { alink_spdif_outputs_reset(&alink_spdif); }

// The pipeline's split hook (audio_pipeline.c process_input_block(), while
// alink_spdif_split_armed): run the block in pieces so that every due op of
// the source acts on the sample written at its F.  True: the block was
// processed here.
bool alink_spdif_fw_src_block(uint32_t n) {
    alink_spdif_t *u = &alink_spdif;
    uint32_t wr, played;
    if (!u->src_running || !ring_counters(&wr, &played)) return false;
    alink_spdif_split_armed = false;        // the pieces below go straight through
    uint64_t f = alink_spdif_src_f(u, wr);
    uint32_t off = 0;
    bool split = false;
    int guard = 0;
    while (off < n && guard++ < 16) {
        uint32_t k = alink_spdif_src_split(u, f, n - off);
        if (k == 0) continue;               // due op(s) ran: their sample is next
        if (!split && k == n) break;        // nothing due inside this block
        split = true;
        if (off) {
            memmove(buf_l, buf_l + off, k * sizeof(buf_l[0]));
            memmove(buf_r, buf_r + off, k * sizeof(buf_r[0]));
#if PICO_RP2350
            for (unsigned c = 0; c < NUM_INPUT_CHANNELS - NUM_STEREO_INPUTS; c++)
                memmove(buf_in_ext[c], buf_in_ext[c] + off, k * sizeof(buf_in_ext[c][0]));
#endif
        }
        process_input_block(k);
        f += k;
        off += k;
    }
    update_armed();
    if (!split) return false;
    return true;
}

// ---------------------------------------------------------------------------
// The control plane (extension_channel_spec.md 2.10)
// ---------------------------------------------------------------------------

uint8_t alink_spdif_fw_set_config(const uint8_t p[ALINK_SPDIF_CFG_LEN]) {
    refresh_limits();
    uint8_t r = alink_spdif_set_config(&alink_spdif, p, spdif_slot_mask());
    if (r == TB_OK) {
        uint32_t wr = 0, played = 0;
        bool run = outputs_running() && ring_counters(&wr, &played);
        apply_carry();                      // the reader's block of the carrying slots (ring_pos64())
        alink_spdif_src_sync(&alink_spdif, run, ring_pos64(played));
        if (alink_spdif.mode == ALINK_SPDIF_MODE_SOURCE) src_stage(true);
        update_armed();
    }
    return r;
}

void alink_spdif_fw_get_config(uint8_t o[ALINK_SPDIF_CFG_LEN]) { alink_spdif_get_config(&alink_spdif, o); }

void alink_spdif_fw_get_status(uint8_t o[ALINK_SPDIF_STATUS_LEN]) {
    refresh_limits();
    uint64_t f = 0;
    uint32_t s = save_and_disable_interrupts();
    if (!alink_spdif_f_now(&alink_spdif, &f, time_us_32())) f = 0;
    restore_interrupts(s);
    uint32_t fill = slot_ring_fill_frames(0);
    alink_spdif_get_status(&alink_spdif, f, soft_vcxo_current_ppm(), (uint16_t)(fill < 0xFFFFu ? fill : 0xFFFFu), o);
}

static void src_now(uint64_t *f_play, uint64_t *f_write) {
    uint32_t wr = 0, played = 0;
    (void)ring_counters(&wr, &played);
    *f_play = alink_spdif_src_f(&alink_spdif, played);
    *f_write = alink_spdif_src_f(&alink_spdif, wr);
}

uint8_t alink_spdif_fw_ctrl_write(const uint8_t p[ALINK_SPDIF_CTRL_LEN]) {
    uint64_t fp, fw;
    src_now(&fp, &fw);
    uint8_t r = alink_spdif_ctrl_write(&alink_spdif, p, fp, fw);
    if (r == TB_OK) src_stage(true);        // the blocks not yet stamped carry it
    update_armed();
    return r;
}

void alink_spdif_fw_ctrl_read(uint8_t o[ALINK_SPDIF_CTRL_LEN]) { alink_spdif_ctrl_read(&alink_spdif, o); }

uint8_t alink_spdif_fw_apply(const uint8_t p[ALINK_APPLY_LEN]) {
    uint64_t t;
    if (alink_spdif.mode == ALINK_SPDIF_MODE_SOURCE) {
        uint64_t fp;
        src_now(&fp, &t);
    } else {
        t = alink_spdif.ph.next_w;
    }
    uint8_t r = alink_spdif_apply(&alink_spdif, p, t);
    update_armed();
    return r;
}

void alink_spdif_fw_get_apply(uint8_t o[ALINK_APPLY_LEN]) { alink_apply_in(&alink_spdif.aq, o); }

// EXT_APPLY_AT_FRAME IN reads the queue of the first EXT_TIME_BASE entry
// (2.10.4): the S/PDIF queue when that entry is time base 3.
bool alink_spdif_fw_first_entry(void) {
    if (alink_spdif_fw_following()) return true;
    if (alink_spdif.mode != ALINK_SPDIF_MODE_SOURCE) return false;
#if DSPI_ALINK_USB
    if (alink_usb_fw_time_base_active()) return false;
#endif
#if DSPI_ALINK
    if (active_input_source == INPUT_SOURCE_ALINK && alink_config[ALINK_CFG_ENABLE]) return false;
#endif
    return true;
}

void alink_spdif_fw_store(uint8_t s[ALINK_SPDIF_STORE_LEN]) { alink_spdif_store(&alink_spdif, s); }
void alink_spdif_fw_load(const uint8_t s[ALINK_SPDIF_STORE_LEN]) { alink_spdif_load(&alink_spdif, s); }
