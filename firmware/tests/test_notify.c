// Host test for notify.c's BULK_INVALIDATED coalescing and source rules
// (notification_protocol_v2_spec.md, "BULK_INVALIDATED source").  Includes
// notify.c directly; the Pico SDK headers come from tests/stubs.
#include "../DSPi/notify.c"

#include <stdio.h>
#include <stdlib.h>

void bulk_params_collect(WireBulkParams *out) { memset(out, 0, sizeof(*out)); }

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

    if (failures) { printf("test_notify: %d failure(s)\n", failures); return 1; }
    printf("test_notify: OK\n");
    return 0;
}
