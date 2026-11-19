/*
 * test_sender.c - host tests for the alink sender core (WP LK4)
 *
 *  1. snd_frame() == LK1's alink_tx_frame(), bit for bit, at every class,
 *     rate and a spread of C, from F = 0, mid-superframe and above 2^40,
 *     with audio and PRBS payloads, flag changes, applies, and RATE ops
 *     that shrink and grow C (the layout switches mid-stream).
 *  2. The sender's own scheduling: a RATE at a multiple of 32, at least
 *     2400 frames ahead, value = the new C, refused while an apply is
 *     pending; a stream with such a switch decoded by LK1's reference
 *     receiver, which follows it with no loss and the right samples.
 *  3. EXT_ALINK_TX and EXT_ALINK_CTRL_MSG byte layouts and refusals
 *     (extension_channel_spec.md 2.8.8, 2.8.9, the fake device's rules).
 *  4. The SYNC ring pulse, the frame start log, the idle-only pattern and
 *     the tone.
 *  5. Encoder speed on this host (an indication only; the firmware measures
 *     its own cycles per frame, EXT_ALINK_DIAG).
 */

#define _POSIX_C_SOURCE 199309L

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "alink_codec.h"
#include "alink_decoder.h"
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

static uint32_t rng_state = 12345u;
static uint32_t rnd(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static int32_t s24(uint32_t u) { return (int32_t)((u & 0xFFFFFFu) ^ 0x800000u) - 0x800000; }

// ---------------------------------------------------------------------------
// 1. Bit identity with the reference encoder
// ---------------------------------------------------------------------------

#define RING_WORDS (1u << 16)
static uint32_t ref_words[RING_WORDS], snd_words[RING_WORDS], sync_words[RING_WORDS];

typedef struct {
    uint64_t at;
    uint8_t  op, arg;
    int16_t  value;
    uint64_t when;      // schedule once tx->frame reaches this
} sched_t;

// Run both encoders over nframes with the same events; true if identical.
static bool run_pair(unsigned cls, unsigned rate, unsigned c, uint64_t first, bool prbs,
                     const sched_t *ev, unsigned nev, unsigned nframes, const char *label) {
    alink_tx_t a, b;
    if (!alink_tx_init(&a, cls, rate, c, first, 0x42) || !alink_tx_init(&b, cls, rate, c, first, 0x42)) {
        CHECK(0, "%s: init", label);
        return false;
    }
    a.flags = b.flags = (uint8_t)((prbs ? ALINK_SF_FLAG_PRBS : 0) | ALINK_SF_FLAG_USB_LIVE);
    alink_bitw_t w;
    alink_bitw_init(&w, ref_words, RING_WORDS);
    snd_sink_t s;
    snd_sink_init(&s, snd_words, RING_WORDS, NULL);
    int32_t samples[ALINK_MAX_CHANNELS];
    for (unsigned i = 0; i < nframes; i++) {
        for (unsigned k = 0; k < nev; k++) {
            if (ev[k].when == a.frame) {
                bool ra, rb;
                if (ev[k].op == 0xFF) {          // a flags change: mute + PRBS toggle
                    a.flags ^= ALINK_SF_FLAG_MUTE | ALINK_SF_FLAG_PRBS;
                    b.flags ^= ALINK_SF_FLAG_MUTE | ALINK_SF_FLAG_PRBS;
                    continue;
                }
                ra = alink_tx_schedule(&a, ev[k].at, ev[k].op, ev[k].arg, ev[k].value);
                rb = alink_tx_schedule(&b, ev[k].at, ev[k].op, ev[k].arg, ev[k].value);
                CHECK(ra == rb, "%s: schedule result differs", label);
            }
        }
        CHECK(a.channels == b.channels && a.rate == b.rate, "%s: layout differs at %llu", label,
              (unsigned long long)a.frame);
        for (unsigned ch = 0; ch < a.channels; ch++) samples[ch] = s24(rnd());
        if (!alink_tx_frame(&a, samples, &w)) {
            CHECK(0, "%s: reference buffer full", label);
            return false;
        }
        snd_frame(&b, &T, samples, &s, NULL);
    }
    // Flush the sink's partial word for the comparison.
    uint64_t nbits = snd_sink_bits(&s);
    if (s.n) snd_words[s.words & s.mask] = s.cur;
    bool same = nbits == w.nbits;
    for (size_t i = 0; same && i < (nbits + 31u) / 32u; i++) {
        uint32_t m = 0xFFFFFFFFu;
        if (i == nbits / 32u && (nbits & 31u)) m = ~(0xFFFFFFFFu >> (nbits & 31u));
        if ((ref_words[i] ^ snd_words[i]) & m) {
            same = false;
            printf("  %s: first difference in word %zu (bit %zu)\n", label, i, i * 32u);
        }
    }
    CHECK(same, "%s: %llu vs %llu bits", label, (unsigned long long)nbits, (unsigned long long)w.nbits);
    CHECK(a.frame == b.frame && a.rd == b.rd && a.apply_op == b.apply_op && a.channels == b.channels &&
          a.rate == b.rate && !memcmp(a.sf, b.sf, ALINK_SF_LEN), "%s: end state differs", label);
    return same;
}

static void test_identity(void) {
    unsigned streams = 0, ok = 0;
    static const uint64_t starts[] = { 0, 17, (1ull << 40) + 5 };
    for (unsigned cls = 1; cls <= 3; cls++) {
        for (unsigned rate = 0; rate <= 2; rate++) {
            unsigned cap = alink_max_channels(cls, rate);
            unsigned cs[] = { 1, 2, 8, 9, 16, cap };
            for (unsigned ci = 0; ci < sizeof(cs) / sizeof(cs[0]); ci++) {
                unsigned c = cs[ci];
                if (c > cap) continue;
                for (unsigned si = 0; si < 3; si++) {
                    for (unsigned prbs = 0; prbs < 2; prbs++) {
                        char label[96];
                        snprintf(label, sizeof(label), "L%u rate %u C %u F0 %llu %s", cls, rate, c,
                                 (unsigned long long)starts[si], prbs ? "PRBS" : "audio");
                        uint64_t f0 = starts[si];
                        uint64_t sf1 = ((f0 + 31u) & ~31ull);
                        // A MASTER_VOLUME, a flags change, then a RATE that
                        // shrinks or grows C (where the new rate allows it).
                        unsigned nrate = (rate + 1u) % 3u;
                        unsigned ncap = alink_max_channels(cls, nrate);
                        unsigned nc = c + 3u <= ncap ? c + 3u : (ncap < c ? ncap : c);
                        sched_t ev[] = {
                            { sf1 + 64u, ALINK_OP_MASTER_VOLUME, 0, -2000, f0 + 1u },
                            { 0, 0xFF, 0, 0, sf1 + 70u },
                            { sf1 + 160u, ALINK_OP_RATE, (uint8_t)nrate, (int16_t)(nc != c ? nc : 0), sf1 + 100u },
                            { sf1 + 300u, ALINK_OP_SYNC_PULSE, 0, 0, sf1 + 200u },
                        };
                        streams++;
                        if (run_pair(cls, rate, c, f0, prbs, ev, 4, 420, label)) ok++;
                    }
                }
            }
        }
    }
    printf("snd_frame == alink_tx_frame: %u of %u streams of 420 frames (applies, flags, RATE)\n", ok, streams);
}

// ---------------------------------------------------------------------------
// 2. Scheduling, and a receiver following the switch
// ---------------------------------------------------------------------------

typedef struct {
    uint64_t frames, bad, first_frame, last_frame;
    uint64_t switch_frame;      // first frame decoded with the new layout
    uint8_t  c0, c1;
    unsigned locks, losses;
    int32_t (*expect)(uint64_t f, unsigned ch);
} rxlog_t;

static int32_t sample_of(uint64_t f, unsigned ch) {
    return s24((uint32_t)(f * 2654435761u) ^ (ch * 0x9E3779B9u));
}

static void rx_cb(void *ctx, const alink_rx_event_t *ev) {
    rxlog_t *l = ctx;
    if (ev->type == ALINK_RX_EV_LOCK) l->locks++;
    if (ev->type == ALINK_RX_EV_LOSS) l->losses++;
    if (ev->type != ALINK_RX_EV_FRAME) return;
    if (!l->frames) { l->first_frame = ev->frame; l->c0 = ev->channels; }
    if (ev->channels != l->c0 && !l->switch_frame) { l->switch_frame = ev->frame; l->c1 = ev->channels; }
    l->frames++;
    l->last_frame = ev->frame;
    bool good = ev->flags == 0;
    for (unsigned ch = 0; ch < ev->channels; ch++)
        if (ev->samples[ch] != sample_of(ev->frame, ch)) good = false;
    if (!good) l->bad++;
}

static void test_schedule(void) {
    alink_tx_t tx;
    uint64_t at;
    // L2 48 kHz, 16 channels, at F = 1000.
    CHECK(alink_tx_init(&tx, ALINK_CLASS_L2, ALINK_RATE_48K, 16, 1000, 1), "init");
    CHECK(snd_schedule_layout(&tx, ALINK_RATE_96K, 8, &at), "48->96 with C 16->8");
    CHECK(at % 32 == 0 && at >= 1000 + SND_RATE_LEAD && at < 1000 + SND_RATE_LEAD + 32,
          "target %llu: first boundary >= 2400 ahead", (unsigned long long)at);
    CHECK(tx.apply_op == ALINK_OP_RATE && tx.apply_arg == ALINK_RATE_96K && tx.apply_value == 8, "RATE value = new C");
    CHECK(!snd_schedule_layout(&tx, ALINK_RATE_48K, 16, &at), "refused while an apply is pending");
    CHECK(!alink_tx_schedule(&tx, at + 640, ALINK_OP_MUTE, 0, 0), "CTRL_MSG refused while pending too");

    CHECK(alink_tx_init(&tx, ALINK_CLASS_L2, ALINK_RATE_48K, 16, 1000, 1), "init");
    CHECK(snd_schedule_layout(&tx, ALINK_RATE_44K1, 16, &at) && tx.apply_value == 0, "rate only: value 0");
    CHECK(alink_tx_init(&tx, ALINK_CLASS_L2, ALINK_RATE_48K, 16, 1000, 1), "init");
    CHECK(snd_schedule_layout(&tx, ALINK_RATE_48K, 12, &at) && tx.apply_value == 12 && tx.apply_arg == ALINK_RATE_48K,
          "C only: same rate code, value 12");
    CHECK(alink_tx_init(&tx, ALINK_CLASS_L1, ALINK_RATE_48K, 8, 1000, 1), "init");
    CHECK(!snd_schedule_layout(&tx, ALINK_RATE_96K, 8, &at) && tx.apply_op == ALINK_OP_NONE,
          "L1 96 kHz cannot carry 8 channels: refused, nothing scheduled");
    CHECK(snd_schedule_layout(&tx, ALINK_RATE_96K, 3, &at), "L1 96 kHz with C 3 fits");
    // Exactly on a boundary: F = 1024 -> 1024 + 2400.
    CHECK(alink_tx_init(&tx, ALINK_CLASS_L1, ALINK_RATE_48K, 8, 1024, 1), "init");
    CHECK(snd_schedule_layout(&tx, ALINK_RATE_44K1, 8, &at) && at == 1024 + SND_RATE_LEAD, "boundary start");

    // CTRL_MSG lead rule (alink_tx_schedule): at least next boundary + 32.
    CHECK(alink_tx_init(&tx, ALINK_CLASS_L1, ALINK_RATE_48K, 8, 1000, 1), "init");
    CHECK(!alink_tx_schedule(&tx, 1024 + 31, ALINK_OP_MUTE, 0, 0), "lead 1024+31 refused");
    CHECK(alink_tx_schedule(&tx, 1024 + 32, ALINK_OP_MUTE, 0, 0), "lead 1024+32 accepted");

    // A stream through the switch, decoded by the reference receiver.
    static const unsigned cases[][5] = {
        // cls, rate, C, new rate, new C
        { ALINK_CLASS_L2, ALINK_RATE_48K, 16, ALINK_RATE_96K, 8 },
        { ALINK_CLASS_L1, ALINK_RATE_96K, 3, ALINK_RATE_48K, 8 },
        { ALINK_CLASS_L3, ALINK_RATE_48K, 32, ALINK_RATE_44K1, 41 },
        { ALINK_CLASS_L1, ALINK_RATE_44K1, 8, ALINK_RATE_48K, 6 },
    };
    for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
        const unsigned *cs = cases[k];
        static alink_rx_t rx;
        rxlog_t log = {0};
        alink_rx_init(&rx, cs[0], 0xFF, rx_cb, &log);
        CHECK(alink_tx_init(&tx, cs[0], cs[1], cs[2], 77, 9), "init");
        snd_sink_t s;
        snd_sink_init(&s, snd_words, RING_WORDS, NULL);
        int32_t smp[ALINK_MAX_CHANNELS];
        uint64_t target = 0;
        uint64_t done_bits = 0;
        for (unsigned i = 0; i < 2900; i++) {
            if (tx.frame == 200) CHECK(snd_schedule_layout(&tx, cs[3], cs[4], &target), "case %u schedule", k);
            for (unsigned ch = 0; ch < tx.channels; ch++) smp[ch] = sample_of(tx.frame, ch);
            snd_frame(&tx, &T, smp, &s, NULL);
            // Feed whole words as they complete (the ring is larger than the
            // window between pushes).
            uint64_t avail = s.words * 32u;
            while (done_bits + 1024u <= avail) {
                uint32_t wi = (uint32_t)(done_bits / 32u) & s.mask;
                if (wi + 32u > RING_WORDS) {
                    uint32_t tmp[32];
                    for (unsigned j = 0; j < 32; j++) tmp[j] = snd_words[(wi + j) & s.mask];
                    alink_rx_push(&rx, tmp, 1024);
                } else {
                    alink_rx_push(&rx, snd_words + wi, 1024);
                }
                done_bits += 1024u;
            }
        }
        CHECK(log.locks == 1 && log.losses == 0, "case %u: one lock, no loss (%u, %u)", k, log.locks, log.losses);
        CHECK(log.bad == 0, "case %u: %llu bad frames", k, (unsigned long long)log.bad);
        CHECK(log.switch_frame == target && log.c1 == cs[4], "case %u: receiver switched at %llu to C %u (target %llu)",
              k, (unsigned long long)log.switch_frame, log.c1, (unsigned long long)target);
        CHECK(rx.rate == cs[3], "case %u: receiver rate %u", k, rx.rate);
        CHECK(log.frames > 2500, "case %u: %llu frames decoded", k, (unsigned long long)log.frames);
        printf("RATE L%u %s C%u -> %s C%u: scheduled at %llu (lead %llu frames), receiver switched at %llu, "
               "%llu frames, 0 bad, %u loss\n", cs[0], cs[1] == 0 ? "44.1" : cs[1] == 1 ? "48" : "96", cs[2],
               cs[3] == 0 ? "44.1" : cs[3] == 1 ? "48" : "96", cs[4], (unsigned long long)target,
               (unsigned long long)(target - 200), (unsigned long long)log.switch_frame,
               (unsigned long long)log.frames, log.losses);
    }
}

// ---------------------------------------------------------------------------
// 3. EXT_ALINK_TX and EXT_ALINK_CTRL_MSG
// ---------------------------------------------------------------------------

static void test_layouts(void) {
    snd_txcfg_t c, d;
    uint8_t p[16], q[16];
    snd_txcfg_default(&c);
    snd_txcfg_pack(&c, 0x0030, 7, p);
    static const uint8_t dflt[16] = { 0, 0, 1, 8, 1, 0, 0, 0xFF, 0xFF, 0, 0x30, 0, 7, 0, 0, 0 };
    CHECK(!memcmp(p, dflt, 16), "default IN bytes");
    CHECK(snd_txcfg_parse(p, 8, &d) == 0 && d.channels == 8 && d.txdis_mask == 0xFF && d.sync_pin == 0xFF,
          "IN form written back is accepted (IN-only fields ignored)");

    struct { int off; uint8_t v; int err; const char *what; } bad[] = {
        { 1, 0x01, 1, "enable bit 8" },
        { 2, 0, 2, "class 0" }, { 2, 4, 2, "class 4" },
        { 4, 3, 3, "rate 3" },
        { 3, 0, 4, "C 0" }, { 3, 9, 4, "C 9 on L1" },
        { 5, 4, 5, "test pattern 4" }, { 6, 1, 6, "slot map 1" },
        { 7, 23, 7, "sync GP23" }, { 7, 29, 7, "sync GP29" },
        { 9, 0x01, 8, "txdis bit 8" }, { 13, 2, 9, "flag bit 1" },
        { 14, 1, 10, "reserved 14" }, { 15, 1, 10, "reserved 15" },
    };
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        memcpy(q, p, 16);
        q[bad[i].off] = bad[i].v;
        int e = snd_txcfg_parse(q, 8, &d);
        CHECK(e == bad[i].err, "EXT_ALINK_TX %s: refused with %d (got %d)", bad[i].what, bad[i].err, e);
    }
    // Capacity from alink_max_channels: L3 96 kHz 18, L2 44.1 kHz 19.
    memcpy(q, p, 16); q[2] = 3; q[4] = 2; q[3] = 18;
    CHECK(snd_txcfg_parse(q, 8, &d) == 0, "L3 96 kHz C 18 accepted");
    q[3] = 19;
    CHECK(snd_txcfg_parse(q, 8, &d) == 4, "L3 96 kHz C 19 refused");
    memcpy(q, p, 16); q[2] = 2; q[4] = 0; q[3] = 19;
    CHECK(snd_txcfg_parse(q, 8, &d) == 0, "L2 44.1 kHz C 19 accepted");
    memcpy(q, p, 16); q[0] = 0xFF; q[7] = 21; q[13] = 1;
    CHECK(snd_txcfg_parse(q, 8, &d) == 0 && d.alink_enable == 0xFF && d.sync_pin == 21 && d.flags == 1,
          "8 alinks, sync GP21, mute");

    // CTRL_MSG OUT.
    snd_ctrl_t m;
    uint8_t o[16] = {0};
    uint64_t f = 1002400;
    for (int i = 0; i < 8; i++) o[i] = (uint8_t)(f >> (8 * i));
    o[8] = ALINK_OP_MASTER_VOLUME; o[10] = 0x30; o[11] = 0xF8;  // -2000
    CHECK(snd_ctrl_parse(o, &m) == 0 && m.frame == f && m.value == -2000, "MASTER_VOLUME -20 dB");
    o[11] = 0xCE; o[10] = 0x00;                                 // -12800
    CHECK(snd_ctrl_parse(o, &m) == 0, "-128 dB accepted");
    o[10] = 0xFF; o[11] = 0xCD;                                 // -12801
    CHECK(snd_ctrl_parse(o, &m) == 5, "-128.01 dB refused");
    memset(o + 8, 0, 8); o[8] = ALINK_OP_RATE; o[9] = 2;
    for (int i = 0; i < 8; i++) o[i] = (uint8_t)(1024 >> (8 * i));
    CHECK(snd_ctrl_parse(o, &m) == 0, "RATE 96 kHz at 1024");
    o[0] = 1;
    CHECK(snd_ctrl_parse(o, &m) == 4, "RATE off a boundary refused");
    o[0] = 0; o[10] = 8;
    CHECK(snd_ctrl_parse(o, &m) == 4, "RATE with value != 0 refused");
    o[10] = 0; o[9] = 3;
    CHECK(snd_ctrl_parse(o, &m) == 4, "RATE code 3 refused");
    // WP FWTB1: ops 9 SIGGEN_START (arg 0) and 10 SIGGEN_STOP (arg 0 fade, 1
    // now) are carried; other arguments and op 11 are refused.
    memset(o + 8, 0, 8); o[8] = 9;
    CHECK(snd_ctrl_parse(o, &m) == 0 && m.op == 9, "op 9 accepted");
    o[9] = 1;
    CHECK(snd_ctrl_parse(o, &m) == 5, "op 9 arg 1 refused");
    o[8] = 10;
    CHECK(snd_ctrl_parse(o, &m) == 0 && m.op == 10 && m.arg == 1, "op 10 now accepted");
    o[9] = 2;
    CHECK(snd_ctrl_parse(o, &m) == 5, "op 10 arg 2 refused");
    o[9] = 0; o[10] = 1;
    CHECK(snd_ctrl_parse(o, &m) == 5, "op 10 value 1 refused");
    o[10] = 0; o[8] = 11;
    CHECK(snd_ctrl_parse(o, &m) == 1, "op 11 refused");
    o[8] = 0; o[12] = 1;
    CHECK(snd_ctrl_parse(o, &m) == 0 && m.op == 0 && m.system_flags == 1, "op 0: system mute only");
    o[12] = 2;
    CHECK(snd_ctrl_parse(o, &m) == 2, "system_flags bit 1 refused");
    o[12] = 0; o[15] = 1;
    CHECK(snd_ctrl_parse(o, &m) == 3, "reserved byte refused");
    o[15] = 0; o[8] = ALINK_OP_OUT_DELAY; o[9] = 9;
    CHECK(snd_ctrl_parse(o, &m) == 5, "OUT_DELAY output 9 refused");
    o[9] = 8; o[10] = 0xFF; o[11] = 0x07;
    CHECK(snd_ctrl_parse(o, &m) == 0, "OUT_DELAY output 8, 2047 samples");
    o[10] = 0; o[11] = 0x08;
    CHECK(snd_ctrl_parse(o, &m) == 5, "OUT_DELAY 2048 refused");
    memset(o + 8, 0, 8); o[8] = ALINK_OP_PRESET_LOAD; o[9] = 10;
    CHECK(snd_ctrl_parse(o, &m) == 5, "PRESET_LOAD slot 10 refused");
    o[8] = ALINK_OP_CLICK; o[9] = 3;
    CHECK(snd_ctrl_parse(o, &m) == 0, "CLICK output 3");
    o[8] = ALINK_OP_SYNC_PULSE; o[9] = 1;
    CHECK(snd_ctrl_parse(o, &m) == 5, "SYNC_PULSE arg 1 refused");

    // CTRL_MSG IN: shows a pending apply at once, 0 after it ends.
    alink_tx_t tx;
    uint8_t in[24];
    CHECK(alink_tx_init(&tx, ALINK_CLASS_L1, ALINK_RATE_48K, 8, 990, 0x5A), "init");
    tx.flags = ALINK_SF_FLAG_USB_LIVE;
    CHECK(alink_tx_schedule(&tx, 1056, ALINK_OP_UNMUTE, 0, 0), "UNMUTE at 1056");
    snd_ctrl_pack_in(&tx, 990, in);
    static const uint8_t want[24] = { 0x0C, 1, 8, 1, 0, 0x5A, 0, 0, 0x20, 0x04, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
                                      0xC0, 0x03, 0, 0 };
    CHECK(!memcmp(in, want, 24), "CTRL_MSG IN with UNMUTE pending");
    snd_sink_t s;
    snd_sink_init(&s, snd_words, RING_WORDS, NULL);
    int32_t z[8] = {0};
    while (tx.frame < 1056) snd_frame(&tx, &T, z, &s, NULL);
    snd_ctrl_pack_in(&tx, 1056, in);
    CHECK(in[0] == 0x08 && !in[16] && !in[8] && !in[9], "apply gone after its frame");

    // WP FWTB1: the sender relays op 9 in its superframe: valid for a
    // receiver with the ops, rejected (code 6) by one built before them.
    CHECK(alink_tx_init(&tx, ALINK_CLASS_L1, ALINK_RATE_48K, 8, 990, 0x5A), "init");
    CHECK(alink_tx_schedule(&tx, 2400, ALINK_OP_SIGGEN_START, 0, 0), "SIGGEN_START at 2400");
    snd_ctrl_pack_in(&tx, 990, in);
    CHECK(in[16] == ALINK_OP_SIGGEN_START && in[8] == 0x60 && in[9] == 0x09, "CTRL_MSG IN shows op 9");
    while (tx.frame < 1024) snd_frame(&tx, &T, z, &s, NULL);
    alink_sf_t sf;
    CHECK(alink_sf_unpack(tx.sf, &sf) == ALINK_SF_OK && sf.apply_op == ALINK_OP_SIGGEN_START &&
          sf.apply_frame == 2400, "the superframe carries op 9 at 2400");
    CHECK(alink_sf_unpack_ops(tx.sf, &sf, ALINK_OP_MAX_PRE_FWTB1) == ALINK_SF_ERR_APPLY,
          "a receiver without the ops rejects it");
}

// ---------------------------------------------------------------------------
// 4. SYNC ring, log, idle pattern, tone
// ---------------------------------------------------------------------------

static void test_sync_log_idle_tone(void) {
    // SYNC: high exactly on bits [from, to) of the absolute stream.
    snd_sink_t s;
    memset(sync_words, 0, sizeof(sync_words));
    snd_sink_init(&s, snd_words, 1024, sync_words);
    alink_tx_t tx;
    CHECK(alink_tx_init(&tx, ALINK_CLASS_L1, ALINK_RATE_44K1, 8, 0, 1), "init");
    int32_t z[8] = {0};
    uint64_t from = 0, to = 0;
    snd_log_t log;
    snd_log_init(&log, 0);
    for (unsigned i = 0; i < 40; i++) {
        if (i == 13) {               // a frame whose comma is not word aligned at 44.1 kHz
            from = snd_sink_bits(&s);
            to = from + 100u;
            s.pulse_from = from;
            s.pulse_to = to;
        }
        snd_frame(&tx, &T, z, &s, &log);
    }
    bool ok = true;
    for (uint64_t b = 0; b < s.words * 32u; b++) {
        bool hi = (sync_words[(b / 32u) & s.mask] >> (31u - (b & 31u))) & 1u;
        if (hi != (b >= from && b < to)) ok = false;
    }
    CHECK(ok && (from & 31u), "SYNC pulse on bits [%llu, %llu) exactly", (unsigned long long)from,
          (unsigned long long)to);
    // The comma of frame 13 starts at `from`.
    uint32_t w = (snd_words[(from / 32u) & s.mask] << (from & 31u)) |
                 (snd_words[(from / 32u + 1u) & s.mask] >> (32u - (from & 31u)));
    CHECK(alink_sym_is_comma((uint16_t)(w >> 22)), "the pulse starts on frame 13's comma");

    // Log: the frame on the wire for a given read position.
    CHECK(snd_log_wire(&log, from) == 13 && snd_log_wire(&log, from + 1) == 13, "wire frame 13 at its start");
    CHECK(snd_log_wire(&log, from + 349) == 13 || snd_log_wire(&log, from + 349) == 14, "inside 13/14");
    CHECK(snd_log_wire(&log, snd_sink_bits(&s)) == 39, "last frame");

    // Idle-only: alternating bits, encoder state as for real frames.
    alink_tx_t a, b;
    alink_tx_init(&a, ALINK_CLASS_L2, ALINK_RATE_48K, 16, 30, 1);
    alink_tx_init(&b, ALINK_CLASS_L2, ALINK_RATE_48K, 16, 30, 1);
    alink_tx_schedule(&a, 128, ALINK_OP_RATE, ALINK_RATE_96K, 8);
    alink_tx_schedule(&b, 128, ALINK_OP_RATE, ALINK_RATE_96K, 8);
    snd_sink_t s2;
    snd_sink_init(&s2, sync_words, RING_WORDS, NULL);
    snd_sink_init(&s, snd_words, RING_WORDS, NULL);
    int32_t zz[16] = {0};
    for (unsigned i = 0; i < 200; i++) {
        snd_frame(&a, &T, zz, &s, NULL);
        snd_idle_frame(&b, &s2, NULL);
    }
    CHECK(snd_sink_bits(&s) == snd_sink_bits(&s2) && a.rate == b.rate && a.channels == b.channels &&
          a.frame == b.frame && !memcmp(a.sf, b.sf, 32), "idle frames advance like real ones (RATE at 128)");
    bool alt = true;
    for (uint64_t i = 0; i < s2.words; i++) if (sync_words[i] != 0xAAAAAAAAu) alt = false;
    CHECK(alt, "idle-only stream is 1010...");

    // Tone: 1 kHz at -20 dBFS.
    snd_tone_t tone = {0};
    for (unsigned r = 0; r < 3; r++) {
        uint32_t fs = alink_rate_hz(r);
        snd_tone_set_rate(&tone, fs);
        tone.phase = 0;
        double sum = 0, peak = 0, ref = 0;
        unsigned n = fs;     // one second = 1000 periods
        for (unsigned i = 0; i < n; i++) {
            double v = snd_tone_next(&tone) / 8388608.0;
            double e = 0.1 * sin(6.283185307179586 * 1000.0 * i / fs);
            sum += (v - e) * (v - e);
            ref += e * e;
            if (fabs(v) > peak) peak = fabs(v);
        }
        double err_db = 10.0 * log10(sum / ref);
        CHECK(fabs(peak - 0.1) < 0.001 && err_db < -40.0, "tone at %u Hz: peak %.4f, error %.1f dB", fs, peak, err_db);
    }
}

// ---------------------------------------------------------------------------
// 5. Speed
// ---------------------------------------------------------------------------

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void test_speed(void) {
    static const unsigned cfg[][3] = {
        { 1, 1, 8 }, { 1, 2, 3 }, { 2, 1, 16 }, { 2, 1, 18 }, { 2, 2, 8 }, { 3, 1, 32 }, { 3, 1, 38 },
        { 3, 2, 18 }, { 3, 0, 41 },
    };
    printf("host encode time per frame (this machine; indicative only):\n");
    for (unsigned k = 0; k < sizeof(cfg) / sizeof(cfg[0]); k++) {
        alink_tx_t a, b;
        alink_tx_init(&a, cfg[k][0], cfg[k][1], cfg[k][2], 0, 1);
        alink_tx_init(&b, cfg[k][0], cfg[k][1], cfg[k][2], 0, 1);
        snd_sink_t s;
        snd_sink_init(&s, snd_words, RING_WORDS, NULL);
        int32_t smp[64];
        for (unsigned ch = 0; ch < 64; ch++) smp[ch] = s24(rnd());
        unsigned n = 200000;
        double t0 = now_s();
        for (unsigned i = 0; i < n; i++) snd_frame(&a, &T, smp, &s, NULL);
        double t1 = now_s();
        alink_bitw_t w;
        unsigned m = 2000;
        double t2 = now_s();
        for (unsigned i = 0; i < m; i++) {
            if (i % 100 == 0) alink_bitw_init(&w, ref_words, RING_WORDS);
            alink_tx_frame(&b, smp, &w);
        }
        double t3 = now_s();
        printf("  L%u %s C%-2u: snd_frame %6.0f ns, alink_tx_frame %7.0f ns (x%.0f)\n", cfg[k][0],
               cfg[k][1] == 0 ? "44.1" : cfg[k][1] == 1 ? "48  " : "96  ", cfg[k][2],
               (t1 - t0) / n * 1e9, (t3 - t2) / m * 1e9, ((t3 - t2) / m) / ((t1 - t0) / n));
    }
}

int main(void) {
    snd_tables_init(&T);
    test_identity();
    test_schedule();
    test_layouts();
    test_sync_log_idle_tone();
    test_speed();
    printf("test_sender: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
