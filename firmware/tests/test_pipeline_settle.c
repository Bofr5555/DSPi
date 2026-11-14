// Host test for pipeline_settle.h (WP IDLE1): the pre-reset settle fades a
// streaming source to silence, returns at once on an idle USB source (no
// packet within the gap threshold although sync_started is still latched),
// and keeps its cap for a producer that is declared live but stalls.
//
// The model: the main-loop spin polls every 100 us; a USB host sends one
// packet per 1 ms while streaming; each processed packet moves the soft-mute
// envelope one step (8 packets from full scale to zero); the ring silence
// mark is reached 42 ms (delay lines + queue) after it is armed.  The USB
// liveness is either the fixed one (pipeline_usb_packets_recent: a packet
// within 50 ms) or the old one (the latched sync_started alone).
#include "../DSPi/pipeline_settle.h"

#include <stdio.h>

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

#define GAP_US        50000u   // AUDIO_GAP_THRESHOLD_US
#define POLL_US         100u
#define PACKET_US      1000u
#define FADE_PACKETS      8
#define DRAIN_US      42000u

typedef struct {
    // USB host / stock decode
    bool     sync_started;
    uint64_t last_packet_us;
    uint64_t next_packet_us;
    uint64_t stream_until_us;   // packets arrive while now < this (0: none)
    // non-USB producer declared live (S/PDIF / ADAT / I2S / alink locked)
    bool     other_live;
    bool     use_usb;
    bool     old_predicate;     // the stock latch: sync_started alone
    // pipeline
    int      envelope;          // FADE_PACKETS = full scale, 0 = silent
    bool     wire_silent;
    uint64_t mark_at_us;        // 0: not armed
    pipeline_settle_t st;
    // results
    int      arms, latches;
} sim_t;

static void sim_packets(sim_t *m, uint64_t now) {
    while (m->stream_until_us && m->next_packet_us <= now && m->next_packet_us < m->stream_until_us) {
        uint64_t t = m->next_packet_us;
        // usb_audio.c process_audio_packet(): the gap check runs on the next packet.
        if (m->sync_started && m->last_packet_us && (t - m->last_packet_us) > GAP_US) m->sync_started = false;
        m->last_packet_us = t;
        m->sync_started = true;
        if (m->envelope > 0) m->envelope--;   // soft mute requested: one step per block
        m->next_packet_us += PACKET_US;
    }
}

static bool sim_streaming(const sim_t *m, uint64_t now) {
    if (m->use_usb)
        return m->old_predicate ? m->sync_started
                                : pipeline_usb_packets_recent(m->sync_started, m->last_packet_us, now, GAP_US);
    return m->other_live;
}

// main.c pipeline_fade_to_silence_poll() with the side effects modelled.
static bool sim_poll(sim_t *m, uint64_t now) {
    if (m->envelope > 0) m->wire_silent = false;
    switch (pipeline_settle_step(&m->st, now, m->wire_silent, sim_streaming(m, now), m->envelope == 0)) {
    case PIPELINE_SETTLE_SILENT:        return true;
    case PIPELINE_SETTLE_WAIT_ENVELOPE: return false;
    case PIPELINE_SETTLE_ARM_DRAIN:     m->mark_at_us = now + DRAIN_US; m->arms++;
        // fall through
    case PIPELINE_SETTLE_WAIT_DRAIN:    if (now < m->mark_at_us) return false; break;
    case PIPELINE_SETTLE_LATCH:         m->envelope = 0; m->latches++; break;
    }
    m->wire_silent = true;
    return true;
}

// main.c pipeline_settle_to_silence(): spin, servicing the producer.  Returns
// the time it took.
static uint64_t sim_settle(sim_t *m, uint64_t t0) {
    uint64_t now = t0;
    while (!sim_poll(m, now)) {
        now += POLL_US;
        sim_packets(m, now);
        if (now - t0 > 10u * PIPELINE_FADE_CAP_US) return ~0ull;   // a hang
    }
    return now - t0;
}

static void sim_init(sim_t *m, bool usb) {
    *m = (sim_t){0};
    m->use_usb = usb;
    m->envelope = FADE_PACKETS;
    pipeline_settle_reset(&m->st);
}

// A USB stream playing since t = 1 s, still playing at the reset.
static uint64_t streaming_usb(sim_t *m, bool old_pred, uint64_t stop_at) {
    sim_init(m, true);
    m->old_predicate = old_pred;
    m->next_packet_us = 1000000u;
    m->stream_until_us = stop_at;
    uint64_t t0 = 2000000u;
    sim_packets(m, t0);
    m->envelope = FADE_PACKETS;               // playing at full scale
    return sim_settle(m, t0);
}

// A stream that played from 1 s to 2 s; the reset comes at 3 s.
static uint64_t idle_usb(sim_t *m, bool old_pred) {
    sim_init(m, true);
    m->old_predicate = old_pred;
    m->next_packet_us = 1000000u;
    m->stream_until_us = 2000000u;
    sim_packets(m, 3000000u);
    m->envelope = FADE_PACKETS;               // not faded by anything (no blocks since)
    return sim_settle(m, 3000000u);
}

int main(void) {
    sim_t m;

    // 1. The liveness helper.
    CHECK(!pipeline_usb_packets_recent(false, 1000, 1000, GAP_US));          // never started
    CHECK(!pipeline_usb_packets_recent(true, 0, 1000, GAP_US));              // no packet yet
    CHECK(pipeline_usb_packets_recent(true, 1000, 1000, GAP_US));
    CHECK(pipeline_usb_packets_recent(true, 1000, 1000 + GAP_US, GAP_US));   // at the threshold
    CHECK(!pipeline_usb_packets_recent(true, 1000, 1001 + GAP_US, GAP_US));  // past it: latched but idle

    // 2. A streaming USB source still fades: the envelope reaches zero on the
    //    packets, the drain is armed and waited, well under the cap.
    uint64_t t = streaming_usb(&m, false, 0xFFFFFFFFu);
    printf("streaming USB settle: %llu us (arms %d, latches %d)\n", (unsigned long long)t, m.arms, m.latches);
    CHECK(m.arms == 1 && m.latches == 0);
    CHECK(t >= DRAIN_US + (FADE_PACKETS - 1) * PACKET_US);
    CHECK(t < DRAIN_US + (FADE_PACKETS + 2) * PACKET_US);
    CHECK(m.wire_silent && m.envelope == 0);
    // Same with the old predicate: streaming behaviour unchanged.
    uint64_t t_old = streaming_usb(&m, true, 0xFFFFFFFFu);
    CHECK(t_old == t && m.arms == 1 && m.latches == 0);

    // 3. Idle USB (sync_started latched, last packet 1 s ago): returns at once.
    t = idle_usb(&m, false);
    printf("idle USB settle: %llu us (old predicate: ", (unsigned long long)t);
    CHECK(m.sync_started);                    // the latch is still set
    CHECK(t == 0 && m.latches == 1 && m.arms == 0 && m.wire_silent);
    // The bug: the latched sync_started kept the settle spinning to the cap.
    t_old = idle_usb(&m, true);
    printf("%llu us)\n", (unsigned long long)t_old);
    CHECK(t_old > PIPELINE_FADE_CAP_US && t_old <= PIPELINE_FADE_CAP_US + POLL_US);
    CHECK(m.latches == 1 && m.arms == 0);

    // 4. The host stops 3 ms into the settle (envelope not yet at zero): the
    //    settle returns once the gap threshold passes, not at the cap.
    t = streaming_usb(&m, false, 2003000u);
    printf("stream stops mid-fade: %llu us\n", (unsigned long long)t);
    CHECK(m.latches == 1 && m.arms == 0);
    CHECK(t > GAP_US && t <= 3000u + GAP_US + POLL_US);
    t_old = streaming_usb(&m, true, 2003000u);
    CHECK(t_old > PIPELINE_FADE_CAP_US);

    // 5. A wire already silent from an earlier settle returns at once.
    sim_init(&m, true);
    m.envelope = 0;
    m.wire_silent = true;
    CHECK(sim_settle(&m, 5000000u) == 0 && m.latches == 0);

    // 6. Non-USB producers are unchanged: not live -> at once; declared live
    //    (locked S/PDIF/ADAT/I2S/alink) but delivering nothing -> the cap.
    sim_init(&m, false);
    m.other_live = false;
    CHECK(sim_settle(&m, 5000000u) == 0 && m.latches == 1);
    sim_init(&m, false);
    m.other_live = true;
    t = sim_settle(&m, 5000000u);
    CHECK(t > PIPELINE_FADE_CAP_US && t <= PIPELINE_FADE_CAP_US + POLL_US && m.latches == 1);

    // 7. A settle abandoned for longer than the stale window starts over.
    pipeline_settle_t s;
    pipeline_settle_reset(&s);
    CHECK(pipeline_settle_step(&s, 1000, false, true, false) == PIPELINE_SETTLE_WAIT_ENVELOPE);
    CHECK(pipeline_settle_step(&s, 2000, false, true, true) == PIPELINE_SETTLE_ARM_DRAIN);
    CHECK(pipeline_settle_step(&s, 3000, false, true, true) == PIPELINE_SETTLE_WAIT_DRAIN);
    CHECK(pipeline_settle_step(&s, 3000 + PIPELINE_FADE_STALE_US + 1, false, true, false)
          == PIPELINE_SETTLE_WAIT_ENVELOPE);  // drain_armed dropped
    CHECK(s.start_us == 3000 + PIPELINE_FADE_STALE_US + 1);
    // The cap counts from the settle's first poll, polled continuously.
    pipeline_settle_reset(&s);
    uint64_t now = 10000;
    pipeline_settle_act_t a;
    do { a = pipeline_settle_step(&s, now, false, true, false); now += 1000; } while (a == PIPELINE_SETTLE_WAIT_ENVELOPE);
    CHECK(a == PIPELINE_SETTLE_LATCH && now - 1000 - 10000 == PIPELINE_FADE_CAP_US + 1000);

    if (failures) { printf("test_pipeline_settle: %d failure(s) of %d\n", failures, checks); return 1; }
    printf("test_pipeline_settle: OK (%d checks)\n", checks);
    return 0;
}
