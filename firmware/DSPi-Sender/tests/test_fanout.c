/*
 * test_fanout.c - host tests for the fan-out core (WP LK4)
 *
 * An upstream sender's stream (snd_frame, proven equal to LK1's reference
 * encoder) goes through snd_fo_* into a second sink, and LK1's reference
 * receiver decodes that.  Checked: the same F, samples and layout come out,
 * hops + 1, the outgoing class, valid superframes, a RATE passing through,
 * class conversion (L3 -> L1, L1 -> L2) at 48 and 44.1 kHz, and damage:
 * a corrupted bank stays a CRC failure downstream, a corrupted head never
 * becomes a valid superframe downstream (no laundering), slips re-align
 * without loss, three lost commas are a loss and a fresh acquisition
 * restarts the output.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alink_codec.h"
#include "alink_decoder.h"
#include "sender_fanout.h"
#include "sender_alink.h"

static int failures, checks;

#define CHECK(cond, ...) do {                                        \
        checks++;                                                    \
        if (!(cond)) {                                               \
            failures++;                                              \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);              \
            printf(__VA_ARGS__);                                     \
            printf("\n");                                            \
        }                                                            \
    } while (0)

static snd_tables_t T;
static snd_dectab_t D;

static int32_t sample_of(uint64_t f, unsigned ch) {
    uint32_t u = (uint32_t)(f * 2654435761u) ^ (ch * 0x9E3779B9u);
    return (int32_t)((u & 0xFFFFFFu) ^ 0x800000u) - 0x800000;
}

// ---------------------------------------------------------------------------
// Upstream stream with damage
// ---------------------------------------------------------------------------

#define UP_WORDS (1u << 20)
static uint32_t up_words[UP_WORDS];     // the clean stream
static uint32_t in_words[UP_WORDS];     // after damage
static uint32_t out_words[UP_WORDS];
static uint64_t frame_bit[20000];       // upstream comma position of frame index i

typedef struct {
    unsigned kind;      // 1 flip bit at frame+offset, 2 delete bits (slip -), 3 insert bits (slip +)
    unsigned frame;     // index into the stream
    unsigned offset;    // bit offset inside the frame
    unsigned n;
} damage_t;

static uint64_t build(unsigned cls, unsigned rate, unsigned c, uint64_t f0, unsigned nframes,
                      uint64_t rate_switch_at, unsigned new_rate, unsigned new_c, const damage_t *dm, unsigned ndm) {
    alink_tx_t tx;
    alink_tx_init(&tx, cls, rate, c, f0, 0x33);
    tx.hops = 2;
    tx.flags = ALINK_SF_FLAG_USB_LIVE;
    snd_sink_t s;
    snd_sink_init(&s, up_words, UP_WORDS, NULL);
    int32_t smp[ALINK_MAX_CHANNELS];
    for (unsigned i = 0; i < nframes; i++) {
        if (rate_switch_at && tx.frame == f0 + 100) {
            uint64_t at = rate_switch_at;
            CHECK(alink_tx_schedule(&tx, at, ALINK_OP_RATE, (uint8_t)new_rate, (int16_t)new_c), "schedule RATE");
        }
        frame_bit[i] = snd_sink_bits(&s);
        for (unsigned ch = 0; ch < tx.channels; ch++) smp[ch] = sample_of(tx.frame, ch);
        snd_frame(&tx, &T, smp, &s, NULL);
    }
    uint64_t nbits = s.words * 32u;
    // Copy with damage (bit granular).
    uint64_t o = 0;
    unsigned di = 0;
    memset(in_words, 0, sizeof(in_words));
    for (uint64_t b = 0; b < nbits && o < (uint64_t)UP_WORDS * 32u; b++) {
        while (di < ndm && frame_bit[dm[di].frame] + dm[di].offset == b) {
            if (dm[di].kind == 3)
                for (unsigned k = 0; k < dm[di].n; k++, o++)
                    if (o & 1u) in_words[o >> 5] |= 0x80000000u >> (o & 31u);
            if (dm[di].kind == 2) { b += dm[di].n; }
            di++;
        }
        if (b >= nbits) break;
        uint32_t bit = (up_words[b >> 5] >> (31u - (b & 31u))) & 1u;
        for (unsigned k = 0; k < ndm; k++)
            if (dm[k].kind == 1 && frame_bit[dm[k].frame] + dm[k].offset == b) bit ^= 1u;
        if (bit) in_words[o >> 5] |= 0x80000000u >> (o & 31u);
        o++;
    }
    return o;
}

// ---------------------------------------------------------------------------
// Downstream decode
// ---------------------------------------------------------------------------

typedef struct {
    unsigned frames, bad_samples, flagged, locks, losses, sfs, sf_errors, crc_banks;
    uint64_t first, last, gaps;
    uint8_t  hops, cls, rate_end, c_end;
    bool     seen;
} down_t;

static void down_cb(void *ctx, const alink_rx_event_t *ev) {
    down_t *d = ctx;
    switch (ev->type) {
    case ALINK_RX_EV_LOCK: d->locks++; break;
    case ALINK_RX_EV_LOSS: d->losses++; break;
    case ALINK_RX_EV_SUPERFRAME: d->sfs++; d->hops = ev->sf->hops; d->cls = ev->sf->cls; break;
    case ALINK_RX_EV_SF_ERROR: d->sf_errors++; break;
    case ALINK_RX_EV_FRAME:
        if (d->seen && ev->frame != d->last + 1) d->gaps++;
        if (!d->seen) d->first = ev->frame;
        d->seen = true;
        d->last = ev->frame;
        d->frames++;
        d->c_end = ev->channels;
        if (ev->flags) d->flagged++;
        // Bank failures of frames that arrived in place (a flywheel frame
        // during a loss decodes idles).
        for (unsigned b = 0; b < alink_bank_count(ev->channels) && !ev->flags; b++)
            if (ev->bank_status[b] != ALINK_BANK_OK) d->crc_banks++;
        for (unsigned ch = 0; ch < ev->channels; ch++)
            if (ev->bank_status[ch / 8] == ALINK_BANK_OK && ev->samples[ch] != sample_of(ev->frame, ch))
                d->bad_samples++;
        break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
// The fan-out run
// ---------------------------------------------------------------------------

typedef struct {
    snd_fo_t fo;
    down_t down;
    unsigned acquisitions, starts, losses_fo;
    alink_rx_stats_t st;
    uint32_t out_unfit;
} run_t;

static alink_rx_t scratch, down_rx;

static void run(run_t *R, unsigned in_cls, unsigned out_cls, uint64_t in_bits) {
    memset(R, 0, sizeof(*R));
    snd_fo_init(&R->fo, in_cls, out_cls, &T, &D);
    snd_ring_t ring = { .ring = in_words, .mask = UP_WORDS - 1u };
    snd_sink_t s;
    snd_sink_init(&s, out_words, UP_WORDS, NULL);
    snd_put_idles(&s, 1000);
    uint64_t at = 0;            // where the next acquisition snapshot starts
    uint32_t need = snd_fo_acquire_bits(in_cls);
    while (1) {
        if (R->fo.state != SND_FO_LOCKED) {
            if (at + need > in_bits) break;
            // The snapshot: in_words is the ring itself, word aligned.
            uint64_t w0 = at / 32u;
            if (snd_fo_acquire(&R->fo, in_words + w0, w0 * 32u, need, &scratch)) {
                R->acquisitions++;
            } else {
                at += need / 2u;
                continue;
            }
        }
        if (snd_fo_need(&R->fo) > in_bits) break;
        if (!R->fo.out_on && R->fo.sf_valid && R->fo.frame % ALINK_SF_LEN == 0) {
            snd_put_idles(&s, 37);      // the engine's alignment gap
            snd_fo_start_output(&R->fo);
            R->starts++;
        }
        bool was_on = R->fo.out_on;
        unsigned res = snd_fo_frame(&R->fo, &ring, &s, NULL);
        if (!R->fo.out_on && !was_on) snd_put_idles(&s, 10u * alink_frame_symbols(out_cls, ALINK_RATE_48K, 0));
        if (res & SND_FO_R_LOSS) {
            R->losses_fo++;
            at = R->fo.pos;
            snd_put_idles(&s, 2000);
        }
    }
    R->st = R->fo.st;
    R->out_unfit = R->fo.out_unfit;
    alink_rx_init(&down_rx, out_cls, 0xFF, down_cb, &R->down);
    alink_rx_push(&down_rx, out_words, (size_t)(s.words * 32u));
}

static void test_clean(unsigned in_cls, unsigned out_cls, unsigned rate, unsigned c, const char *what) {
    run_t R;
    uint64_t n = build(in_cls, rate, c, 1000, 3000, 0, 0, 0, NULL, 0);
    run(&R, in_cls, out_cls, n);
    down_t *d = &R.down;
    CHECK(R.acquisitions == 1 && R.starts == 1 && R.losses_fo == 0, "%s: one acquisition and start (%u, %u, %u)",
          what, R.acquisitions, R.starts, R.losses_fo);
    CHECK(d->locks == 1 && d->losses == 0 && d->gaps == 0, "%s: downstream locks once, no loss/gap", what);
    CHECK(d->bad_samples == 0 && d->flagged == 0 && d->crc_banks == 0 && d->sf_errors == 0,
          "%s: clean downstream (samples %u, flagged %u, banks %u, sf errors %u)", what, d->bad_samples,
          d->flagged, d->crc_banks, d->sf_errors);
    CHECK(d->hops == 3 && d->cls == out_cls, "%s: hops 2 -> %u, class %u", what, d->hops, d->cls);
    CHECK(d->frames > 2800 && d->last >= 3900, "%s: %u frames, last F %llu", what, d->frames,
          (unsigned long long)d->last);
    CHECK(R.st.crc8_errors == 0 && R.st.crc16_errors == 0 && R.st.bit_slips == 0 && R.st.superframe_errors == 0,
          "%s: fan-out counters clean", what);
    printf("fan-out %s: %u frames out, F %llu..%llu, hops %u, class L%u, downstream clean\n", what, d->frames,
           (unsigned long long)d->first, (unsigned long long)d->last, d->hops, d->cls);
}

static void test_rate_switch(void) {
    run_t R;
    // L2 48 kHz 16 ch -> RATE to 96 kHz with C 8 at frame 1024 + 32 * 40.
    uint64_t at = 1024 + 32 * 40;
    uint64_t n = build(ALINK_CLASS_L2, ALINK_RATE_48K, 16, 1000, 4000, at, ALINK_RATE_96K, 8, NULL, 0);
    run(&R, ALINK_CLASS_L2, ALINK_CLASS_L2, n);
    down_t *d = &R.down;
    CHECK(d->locks == 1 && d->losses == 0 && d->gaps == 0 && d->bad_samples == 0 && d->sf_errors == 0,
          "RATE through the fan-out: clean (locks %u losses %u gaps %llu bad %u sferr %u)", d->locks, d->losses,
          (unsigned long long)d->gaps, d->bad_samples, d->sf_errors);
    CHECK(down_rx.rate == ALINK_RATE_96K && d->c_end == 8, "downstream ends at 96 kHz C 8 (%u, %u)", down_rx.rate,
          d->c_end);
    printf("fan-out RATE 48k C16 -> 96k C8 at %llu: downstream followed, %u frames, no loss\n",
           (unsigned long long)at, d->frames);
}

static void test_damage(void) {
    run_t R;
    // Frames (index into the stream) and what happens to them.  The stream
    // starts at F 1000 (index 24 is F 1024, a superframe start).
    damage_t dm[] = {
        { 1, 300, 45, 1 },      // a bank bit (bank 0, first symbol): CRC-16 failure
        { 1, 24 + 32 * 12 + 10, 13, 1 },    // a head bit at superframe byte 10: bad head
        { 2, 899, 319, 1 },     // slip -1: the last (idle) bit before frame 900's comma
        { 3, 1200, 0, 2 },      // slip +2
        { 1, 1500, 3, 1 },      // three commas destroyed in a row: loss
        { 1, 1501, 3, 1 },
        { 1, 1502, 3, 1 },
    };
    uint64_t n = build(ALINK_CLASS_L1, ALINK_RATE_48K, 8, 1000, 3000, 0, 0, 0, dm, sizeof(dm) / sizeof(dm[0]));
    run(&R, ALINK_CLASS_L1, ALINK_CLASS_L1, n);
    down_t *d = &R.down;
    CHECK(R.st.crc16_errors == 1, "bank damage seen by the fan-out (%u)", R.st.crc16_errors);
    CHECK(R.st.crc8_errors == 1, "head damage seen by the fan-out (%u)", R.st.crc8_errors);
    CHECK(R.st.bit_slips == 2, "two slips re-aligned (%u)", R.st.bit_slips);
    CHECK(R.losses_fo == 1 && R.st.loss_count == 1 && R.acquisitions == 2 && R.starts == 2,
          "three lost commas: one loss, re-acquired, output restarted (losses %u, acq %u, starts %u)",
          R.losses_fo, R.acquisitions, R.starts);
    CHECK(d->crc_banks >= 1 && d->crc_banks <= 2, "downstream: the damaged bank fails there too (%u)", d->crc_banks);
    CHECK(d->sf_errors == 1, "downstream: the superframe with the bad head is rejected, not laundered (%u)",
          d->sf_errors);
    CHECK(d->losses == 1 && d->locks == 2, "downstream: loss and re-lock with the fan-out (%u, %u)", d->losses,
          d->locks);
    CHECK(d->bad_samples == 0, "downstream: every OK bank carries the right samples (%u)", d->bad_samples);
    printf("fan-out damage: fan-out crc16 %u crc8 %u slips %u loss %u; downstream bank failures %u, "
           "superframes rejected %u, loss %u, re-lock %u, wrong samples %u\n", R.st.crc16_errors,
           R.st.crc8_errors, R.st.bit_slips, R.st.loss_count, d->crc_banks, d->sf_errors, d->losses,
           d->locks - 1, d->bad_samples);
}

static void test_unfit(void) {
    run_t R;
    uint64_t n = build(ALINK_CLASS_L2, ALINK_RATE_48K, 16, 1000, 600, 0, 0, 0, NULL, 0);
    run(&R, ALINK_CLASS_L2, ALINK_CLASS_L1, n);
    CHECK(R.out_unfit > 400 && R.down.locks == 0, "16 channels do not fit L1: idles only (%u unfit, %u locks)",
          R.out_unfit, R.down.locks);
}

static void test_target_bit(void) {
    // Input L1 (20 cycles/bit), output L3 (5): 100 input bits ago = 400 output bits.
    int64_t t = snd_fo_target_bit(9900, 10000, 50000, 20, 5, 2560);
    CHECK(t == 50000 - 400 + 2560, "target bit arithmetic (%lld)", (long long)t);
    t = snd_fo_target_bit(9900, 10000, 50000, 5, 20, 640);
    CHECK(t == 50000 - 25 + 640, "target bit arithmetic, L3 in, L1 out (%lld)", (long long)t);
}

int main(void) {
    snd_tables_init(&T);
    snd_dectab_init(&D);
    test_clean(ALINK_CLASS_L1, ALINK_CLASS_L1, ALINK_RATE_48K, 8, "L1 -> L1 48k C8");
    test_clean(ALINK_CLASS_L3, ALINK_CLASS_L1, ALINK_RATE_48K, 8, "L3 -> L1 48k C8");
    test_clean(ALINK_CLASS_L1, ALINK_CLASS_L2, ALINK_RATE_44K1, 8, "L1 -> L2 44.1k C8");
    test_clean(ALINK_CLASS_L2, ALINK_CLASS_L3, ALINK_RATE_96K, 8, "L2 -> L3 96k C8");
    test_clean(ALINK_CLASS_L3, ALINK_CLASS_L3, ALINK_RATE_44K1, 41, "L3 -> L3 44.1k C41");
    test_rate_switch();
    test_damage();
    test_unfit();
    test_target_bit();
    printf("test_fanout: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
