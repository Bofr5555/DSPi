// Host test for notify.c's BULK_INVALIDATED coalescing and source rules
// (notification_protocol_v2_spec.md, "BULK_INVALIDATED source").  Includes
// notify.c directly; the Pico SDK headers come from tests/stubs.
#include "../DSPi/notify.c"

#include <stdio.h>
#include <stdlib.h>

void bulk_params_collect(WireBulkParams *out) { memset(out, 0, sizeof(*out)); }
volatile uint32_t clip_notify_latch;
volatile SystemStatusPacket global_status;

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

typedef struct { uint8_t evt, src; uint16_t off; } Pkt;

// Drain the USB consumer into pkts; returns the count.
static int drain(Pkt *pkts, int max) {
    uint8_t buf[NOTIFY_MAX_PACKET];
    int n = 0;
    uint16_t len;
    while ((len = notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf)) != 0) {
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);
        if (buf[0] != NOTIFY_V2_VERSION || n >= max) continue;   // skip v1 packets
        pkts[n].evt = buf[1];
        pkts[n].src = (buf[1] == NOTIFY_EVT_PARAM_CHANGED) ? buf[8] : buf[4];
        pkts[n].off = (uint16_t)(buf[4] | (buf[5] << 8));
        n++;
    }
    return n;
}

static void bulk(ParamSource src) { notify_begin_bulk(src); notify_end_bulk(); }

static void param(ParamSource src, uint16_t off, uint8_t v) {
    notify_set_source(src);
    notify_param_write(off, 1, &v);
    notify_set_source(PARAM_SRC_UNKNOWN);
}

int main(void) {
    Pkt p[NOTIFY_RING_SIZE];
    int n;

    // 1. A bulk SET's source rides on its invalidation.
    notify_init();
    bulk(PARAM_SRC_MIDI);
    n = drain(p, NOTIFY_RING_SIZE);
    CHECK(n == 1 && p[0].evt == NOTIFY_EVT_BULK_INVALIDATED && p[0].src == PARAM_SRC_MIDI);

    // 2. Same source back to back: one entry, source kept.
    notify_init();
    bulk(PARAM_SRC_HOST_SET);
    bulk(PARAM_SRC_HOST_SET);
    n = drain(p, NOTIFY_RING_SIZE);
    CHECK(n == 1 && p[0].src == PARAM_SRC_HOST_SET);

    // 3. Different sources back to back: one entry, UNKNOWN (sticky).
    notify_init();
    bulk(PARAM_SRC_FACTORY);
    bulk(PARAM_SRC_MIDI);
    bulk(PARAM_SRC_MIDI);
    n = drain(p, NOTIFY_RING_SIZE);
    CHECK(n == 1 && p[0].src == PARAM_SRC_UNKNOWN);

    // 4. An event queued after a pending invalidation: the next rewrite is
    //    a new entry after it (never refreshes the older one in place).
    notify_init();
    bulk(PARAM_SRC_HOST_SET);
    param(PARAM_SRC_UART, 0x10, 7);
    bulk(PARAM_SRC_HOST_SET);
    n = drain(p, NOTIFY_RING_SIZE);
    CHECK(n == 3);
    CHECK(p[0].evt == NOTIFY_EVT_BULK_INVALIDATED && p[0].src == PARAM_SRC_HOST_SET);
    CHECK(p[1].evt == NOTIFY_EVT_PARAM_CHANGED && p[1].src == PARAM_SRC_UART && p[1].off == 0x10);
    CHECK(p[2].evt == NOTIFY_EVT_BULK_INVALIDATED && p[2].src == PARAM_SRC_HOST_SET);

    // 5. An invalidation already sent is never touched: a later one is new.
    notify_init();
    bulk(PARAM_SRC_I2C);
    n = drain(p, NOTIFY_RING_SIZE);
    CHECK(n == 1 && p[0].src == PARAM_SRC_I2C);
    bulk(PARAM_SRC_MIDI);
    n = drain(p, NOTIFY_RING_SIZE);
    CHECK(n == 1 && p[0].src == PARAM_SRC_MIDI);

    // 6. Per-field writes inside the bracket are suppressed.
    notify_init();
    notify_begin_bulk(PARAM_SRC_MIDI);
    param(PARAM_SRC_MIDI, 0x20, 3);
    notify_end_bulk();
    n = drain(p, NOTIFY_RING_SIZE);
    CHECK(n == 1 && p[0].evt == NOTIFY_EVT_BULK_INVALIDATED && p[0].src == PARAM_SRC_MIDI);

    // 7. Clip event (FW1): latch -> one 12-byte packet with channel, count,
    //    mask; the polled clip flags are not touched.
    {
        uint8_t buf[NOTIFY_MAX_PACKET];
        uint16_t len;
        notify_init();
        global_status.clip_flags = 0;
        clip_notify_latch = 0;
        notify_clip_service(1000);                       // nothing clipped
        CHECK(notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf) == 0);
        CLIP_LATCH(1u << 5);
        CLIP_LATCH(1u << 9);
        notify_clip_service(1000);
        len = notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf);
        CHECK(len == 12 && buf[0] == NOTIFY_V2_VERSION && buf[1] == NOTIFY_EVT_EXT_CLIP);
        CHECK(buf[2] == 0 && buf[4] == 5 && buf[5] == 2 && buf[6] == 0 && buf[7] == 0);
        CHECK(buf[8] == 0x20 && buf[9] == 0x02 && buf[10] == 0 && buf[11] == 0);
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);
        CHECK(clip_notify_latch == 0);
        CHECK(global_status.clip_flags == ((1u << 5) | (1u << 9)));   // still sticky

        // Rate limit: a clip inside the interval waits, then goes out alone.
        CLIP_LATCH(1u << 1);
        notify_clip_service(1000 + NOTIFY_CLIP_MIN_INTERVAL_MS - 1);
        CHECK(notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf) == 0);
        CHECK(clip_notify_latch == (1u << 1));
        notify_clip_service(1000 + NOTIFY_CLIP_MIN_INTERVAL_MS);
        len = notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf);
        CHECK(len == 12 && buf[4] == 1 && buf[5] == 1 && buf[8] == 0x02);
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);

        // A sustained clip nobody drains: the unsent entry absorbs the new
        // channels, so the ring holds one clip event, not one per interval.
        for (uint32_t t = 2000; t < 2000 + 100 * NOTIFY_CLIP_MIN_INTERVAL_MS;
             t += NOTIFY_CLIP_MIN_INTERVAL_MS) {
            CLIP_LATCH(1u << (t / NOTIFY_CLIP_MIN_INTERVAL_MS % 4));
            notify_clip_service(t);
        }
        n = drain(p, NOTIFY_RING_SIZE);
        CHECK(n == 1 && p[0].evt == NOTIFY_EVT_EXT_CLIP);
        CHECK(notify_consumer_drops[NOTIFY_CONSUMER_USB] == 0);

        // An entry queued after the clip event: the next clip is a new entry.
        notify_init();
        CLIP_LATCH(1u << 3);
        notify_clip_service(5000);
        param(PARAM_SRC_UART, 0x10, 9);
        CLIP_LATCH(1u << 4);
        notify_clip_service(5000 + NOTIFY_CLIP_MIN_INTERVAL_MS);
        n = drain(p, NOTIFY_RING_SIZE);
        CHECK(n == 3 && p[0].evt == NOTIFY_EVT_EXT_CLIP && p[2].evt == NOTIFY_EVT_EXT_CLIP);

        // An entry a consumer has peeked (a transfer in flight) is never
        // merged into: the next clip is a new entry.
        notify_init();
        CLIP_LATCH(1u << 2);
        notify_clip_service(9000);
        CHECK(notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf) == 12);
        CLIP_LATCH(1u << 6);
        notify_clip_service(9000 + NOTIFY_CLIP_MIN_INTERVAL_MS);
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);
        CHECK(buf[8] == 0x04);                           // the in-flight packet: channel 2 only
        len = notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf);
        CHECK(len == 12 && buf[4] == 6 && buf[8] == 0x40);
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);

        // The flag that announces the event.
        CHECK((NOTIFY_FEATURES & NOTIFY_FEAT_CLIP_EVENT) != 0);
    }

    // Alink events (WP LK6, review lk6 F7): 0xE1 and 0xE2 share a serialiser;
    // 2.8.12's examples byte for byte (seq aside).
    {
        uint8_t buf[NOTIFY_MAX_PACKET];
        notify_init();
        notify_push_alink_state(2, 1, 1, 1000000u);
        notify_push_apply_result(4, 0, 1002400u);
        CHECK(notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf) == 12);
        static const uint8_t e1[12] = { 2, 0xE1, 0, 0, 2, 1, 1, 0, 0x40, 0x42, 0x0f, 0x00 };
        buf[3] = 0;
        CHECK(memcmp(buf, e1, 12) == 0);
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);
        CHECK(notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf) == 12);
        static const uint8_t e2[12] = { 2, 0xE2, 0, 0, 4, 0, 0, 0, 0xa0, 0x4b, 0x0f, 0x00 };
        buf[3] = 0;
        CHECK(memcmp(buf, e2, 12) == 0);
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);
    }

    // alink USB-direct event 0xE3 (WP USB1, review usb1 F7): 2.9.5's example
    // byte for byte (seq aside): phase-locked, phase state 2, reason LOCKED,
    // S low 32 = 0x04c4b400.
    {
        uint8_t buf[NOTIFY_MAX_PACKET];
        notify_init();
        notify_push_alink_usb(5, 2, 1, 0x04c4b400u);
        CHECK(notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf) == 12);
        static const uint8_t e3[12] = { 2, 0xE3, 0, 0, 5, 2, 1, 0, 0x00, 0xb4, 0xc4, 0x04 };
        buf[3] = 0;
        CHECK(memcmp(buf, e3, 12) == 0);
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);
        CHECK(NOTIFY_FEAT_ALINK_USB == (1u << 4));
    }

    // Time-base event 0xE4 (WP FWTB1): 2.11.3's example byte for byte (seq
    // aside): an S/PDIF follower (time base 3) LOCKED, reason LOCKED, F = 960 000.
    {
        uint8_t buf[NOTIFY_MAX_PACKET];
        notify_init();
        notify_push_time_base(3, 3, 1, 960000u);
        CHECK(notify_peek_next_for(NOTIFY_CONSUMER_USB, buf, sizeof buf) == 12);
        static const uint8_t e4[12] = { 2, 0xE4, 0, 0, 3, 3, 1, 0, 0x00, 0xa6, 0x0e, 0x00 };
        buf[3] = 0;
        CHECK(memcmp(buf, e4, 12) == 0);
        notify_commit_pop_for(NOTIFY_CONSUMER_USB);
        CHECK(NOTIFY_FEAT_TIME_BASE == (1u << 5));
    }

    if (failures) { printf("test_notify: %d failure(s)\n", failures); return 1; }
    printf("test_notify: OK\n");
    return 0;
}
