// Host test for midi_control.c: the TinyUSB MIDI glue, pending slot, bulk
// lock, retry window, notifications and bootloader ordering, against mocks
// of TinyUSB's MIDI FIFOs, the dispatcher, the bulk lock and the notify ring.
// Built by run.sh like every test_*.c (host cc, no Pico SDK).
#define MIDI_CTRL_HOST_TEST 1

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Firmware API stand-ins (same names/values as vendor_commands.h etc.)
// ---------------------------------------------------------------------------

typedef enum {
    CTRL_SOURCE_USB = 0, CTRL_SOURCE_UART = 1, CTRL_SOURCE_I2C = 2,
    CTRL_SOURCE_GPIO = 3, CTRL_SOURCE_MIDI = 4,
} CtrlSource;
typedef enum {
    CTRL_DISPATCH_OK = 0, CTRL_DISPATCH_ERROR = 1, CTRL_DISPATCH_BLOCKED = 2,
    CTRL_DISPATCH_BULK_LOCKED = 3, CTRL_DISPATCH_BUSY = 4,
} CtrlDispatchResult;
#define CTRL_STATUS_OK           0x00
#define CTRL_STATUS_BUSY         0x01
#define CTRL_STATUS_ERROR        0x02
#define CTRL_STATUS_BLOCKED      0x03
#define CTRL_STATUS_BULK_LOCKED  0x04
#define CTRL_STATUS_CRC_ERROR    0x05
#define CTRL_STATUS_OVERSIZE     0x06
#define CTRL_STATUS_FRAME_ERROR  0x07
#define REQ_GET_ALL_PARAMS       0xA0
#define REQ_SET_ALL_PARAMS       0xA1
#define REQ_ENTER_BOOTLOADER     0xF0
#define REQ_GET_ALL_PARAMS_CHUNK 0xA2
#define REQ_SET_ALL_PARAMS_CHUNK 0xA3
#define REQ_RTA_GET_BANDS_ALL    0x0F
#define REQ_SET_UART_CONFIG      0xF5
#define REQ_SET_I2C_CONFIG       0xF7
#define MIDI_IN_ENDPOINT         0x84
#define MIDI_TX_COPY_MAX         132
#define NOTIFY_CONSUMER_MIDI     2
#define BULK_SIZE                6136
#define WIRE_BULK_PARAMS_SIZE    BULK_SIZE

uint8_t bulk_param_buf[BULK_SIZE];

static uint32_t now_us = 1000;
static uint32_t time_us_32(void) { return now_us; }

// --- bulk lock ---
static int bulk_owner = -1;      // CtrlSource or -1
static int bulk_touches = 0;
static bool vendor_bulk_try_acquire(CtrlSource s) {
    if (bulk_owner >= 0) return false;
    bulk_owner = (int)s;
    return true;
}
static void vendor_bulk_release(CtrlSource s) { if (bulk_owner == (int)s) bulk_owner = -1; }
static void vendor_bulk_touch(CtrlSource s) { if (bulk_owner == (int)s) bulk_touches++; }
static bool vendor_is_bulk_set(uint8_t b, uint16_t wl) { return b == REQ_SET_ALL_PARAMS && wl == BULK_SIZE; }
static uint8_t ctrl_status_from_dispatch(CtrlDispatchResult r) {
    switch (r) {
        case CTRL_DISPATCH_OK: return CTRL_STATUS_OK;
        case CTRL_DISPATCH_BLOCKED: return CTRL_STATUS_BLOCKED;
        case CTRL_DISPATCH_BULK_LOCKED: return CTRL_STATUS_BULK_LOCKED;
        case CTRL_DISPATCH_BUSY: return CTRL_STATUS_BUSY;
        default: return CTRL_STATUS_ERROR;
    }
}

// --- dispatcher ---
static int busy_left = 0;              // return BUSY this many times first
static bool busy_forever = false;
static int get_calls = 0, set_calls = 0, boot_calls = 0;
static uint8_t last_set_payload[64];
static uint16_t last_set_len;
static uint8_t last_breq;
static bool bulk_applied = false;
static bool boot_reply_seen_before_dispatch = false;
static uint8_t resp7[7] = { 1, 1, 0x16, 4, 1, 6, 0 };
static uint8_t resp132[200];
static int host_replies_seen(void);

static CtrlDispatchResult vendor_dispatch_get(CtrlSource src, uint8_t bReq, uint16_t wv, uint16_t wi,
                                              uint16_t wl, const uint8_t **rd, uint16_t *rl) {
    (void)wv; (void)wi;
    if (src != CTRL_SOURCE_MIDI) return CTRL_DISPATCH_ERROR;
    get_calls++;
    last_breq = bReq;
    if (busy_forever) return CTRL_DISPATCH_BUSY;
    if (busy_left > 0) { busy_left--; return CTRL_DISPATCH_BUSY; }
    if (bReq == REQ_ENTER_BOOTLOADER) {
        boot_calls++;
        boot_reply_seen_before_dispatch = host_replies_seen() > 0;
        return CTRL_DISPATCH_OK;   // the real one never returns
    }
    if (bReq == REQ_GET_ALL_PARAMS) {
        if (!vendor_bulk_try_acquire(src)) return CTRL_DISPATCH_BULK_LOCKED;
        for (int i = 0; i < BULK_SIZE; i++) bulk_param_buf[i] = (uint8_t)(i * 13 + 5);
        *rd = bulk_param_buf;
        *rl = BULK_SIZE;
        return CTRL_DISPATCH_OK;
    }
    if (bReq == REQ_RTA_GET_BANDS_ALL) {   // CTRL_BULK_GET for MIDI: lock + bulk buffer
        if (!vendor_bulk_try_acquire(src)) return CTRL_DISPATCH_BULK_LOCKED;
        for (int i = 0; i < 300; i++) bulk_param_buf[i] = (uint8_t)(i ^ 0x5A);
        *rd = bulk_param_buf;
        *rl = 300;
        return CTRL_DISPATCH_OK;
    }
    if (bReq == REQ_GET_ALL_PARAMS_CHUNK || bReq == REQ_SET_ALL_PARAMS_CHUNK)
        return CTRL_DISPATCH_BLOCKED;      // must never reach the dispatcher
    if (bReq == 0x7F) { *rd = resp7; *rl = (wl && wl < 7) ? wl : 7; return CTRL_DISPATCH_OK; }
    if (bReq == 0x99) { *rd = resp132; *rl = 200; return CTRL_DISPATCH_OK; }   // over-long
    return CTRL_DISPATCH_ERROR;
}

static CtrlDispatchResult vendor_dispatch_set(CtrlSource src, uint8_t bReq, uint16_t wv, uint16_t wi,
                                              const uint8_t *payload, uint16_t wl) {
    (void)wv; (void)wi;
    if (src != CTRL_SOURCE_MIDI) return CTRL_DISPATCH_ERROR;
    set_calls++;
    last_breq = bReq;
    if (busy_forever) return CTRL_DISPATCH_BUSY;
    if (busy_left > 0) { busy_left--; return CTRL_DISPATCH_BUSY; }
    // Real dispatcher (CTRL_CMD_REFUSED): 0xF5 / 0xF7 allowed for MIDI,
    // chunked forms refused (midi_control.c serves them itself).
    if (bReq == REQ_GET_ALL_PARAMS_CHUNK || bReq == REQ_SET_ALL_PARAMS_CHUNK)
        return CTRL_DISPATCH_BLOCKED;
    if (bReq == REQ_SET_ALL_PARAMS) {
        if (bulk_owner != (int)src) return CTRL_DISPATCH_ERROR;
        bulk_owner = -1;          // handed to the apply
        bulk_applied = true;
        return CTRL_DISPATCH_OK;
    }
    if (wl > 64) return CTRL_DISPATCH_ERROR;
    memcpy(last_set_payload, payload, wl);
    last_set_len = wl;
    return CTRL_DISPATCH_OK;
}

// --- notify ring (one consumer) ---
static bool notify_active = false;
static uint8_t notify_q[8][64];
static uint16_t notify_len[8];
static int notify_n = 0;
static void notify_consumer_set_active(int c, bool a) { (void)c; notify_active = a; }
static uint16_t notify_peek_next_for(int c, uint8_t *out, uint16_t max) {
    (void)c;
    if (!notify_active || !notify_n) return 0;
    uint16_t n = notify_len[0] < max ? notify_len[0] : max;
    memcpy(out, notify_q[0], n);
    return n;
}
static void notify_commit_pop_for(int c) {
    (void)c;
    if (!notify_n) return;
    memmove(notify_q, notify_q[1], sizeof(notify_q[0]) * (size_t)(notify_n - 1));
    memmove(notify_len, notify_len + 1, sizeof(notify_len[0]) * (size_t)(notify_n - 1));
    notify_n--;
}
static void push_notify(const uint8_t *p, uint16_t n) {
    memcpy(notify_q[notify_n], p, n);
    notify_len[notify_n++] = n;
}

// --- TinyUSB MIDI FIFOs ---
static bool mounted = false;
#define RXQ 4096
static uint8_t rxq[RXQ][4];
static int rx_head = 0, rx_tail = 0;
static int rx_fifo_cap = 64;           // packets TinyUSB would buffer (256 B)
#define TXQ 4096
static uint8_t txq[TXQ][4];
static int tx_level = 0;               // packets in the device TX FIFO
static int tx_fifo_cap = 128;          // 512 B
static uint8_t host_pending[20000][4]; // packets the host has not ingested yet
static int host_pkts = 0;

static bool tud_midi_mounted(void) { return mounted; }
static bool tud_midi_packet_read(uint8_t p[4]) {
    if (rx_tail == rx_head) return false;
    memcpy(p, rxq[rx_tail], 4);
    rx_tail = (rx_tail + 1) % RXQ;
    return true;
}
static bool tud_midi_packet_write(const uint8_t p[4]) {
    if (tx_level >= tx_fifo_cap) return false;
    memcpy(txq[tx_level++], p, 4);
    return true;
}
static bool usbd_edpt_busy(uint8_t rhport, uint8_t ep) { (void)rhport; (void)ep; return tx_level > 0; }

#include "midi_frame.c"
#include "midi_control.c"

// ---------------------------------------------------------------------------
// Test harness: host side
// ---------------------------------------------------------------------------

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; \
    fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

static const mf_mfr_id_t ID = { { 0x7D, 0, 0 }, 1 };

// Host -> device: queue a request's packets; returns packet count.  Only as
// many as TinyUSB's RX FIFO holds go in; the rest wait in host_out.
static uint8_t host_out[4096][4];
static int host_out_n = 0, host_out_i = 0;
static void host_send(uint8_t tag, uint8_t type, uint8_t bReq, uint16_t wv, uint16_t wl, const uint8_t *pl) {
    mf_enc_t e;
    mf_enc_begin_request(&e, &ID, tag, type, bReq, wv, 0, wl, pl);
    while (mf_enc_packet(&e, 0, host_out[host_out_n])) host_out_n++;
}
static void host_send_raw(const uint8_t *sysex, size_t n) {
    for (size_t i = 0; i < n; i += 3) {
        size_t k = n - i < 3 ? n - i : 3;
        uint8_t *p = host_out[host_out_n++];
        memset(p, 0, 4);
        memcpy(p + 1, sysex + i, k);
        p[0] = (sysex[i + k - 1] == 0xF7) ? (uint8_t)(0x4 + k) : 0x4;
    }
}
// USB moves host packets into the device RX FIFO while it has room.
static void usb_out(void) {
    while (host_out_i < host_out_n && ((rx_head - rx_tail + RXQ) % RXQ) < rx_fifo_cap) {
        memcpy(rxq[rx_head], host_out[host_out_i++], 4);
        rx_head = (rx_head + 1) % RXQ;
    }
    if (host_out_i == host_out_n) host_out_i = host_out_n = 0;
}
// USB moves up to `n` device TX packets to the host.
static void usb_in(int n) {
    int k = n < tx_level ? n : tx_level;
    memcpy(host_pending[host_pkts], txq, (size_t)k * 4);
    host_pkts += k;
    memmove(txq, txq[k], (size_t)(tx_level - k) * 4);
    tx_level -= k;
}

typedef struct { uint8_t tag, type, status; uint16_t len; uint8_t payload[6200]; bool crc_ok; } reply_t;
static reply_t replies[64];
static int n_replies = 0;
static int bad_messages = 0;
static uint8_t hmsg[8000];
static size_t hlen = 0;
static bool hin = false;

// Parse host_pending into replies.
static void host_ingest(void) {
    for (int i = 0; i < host_pkts; i++) {
        uint8_t b[3];
        uint8_t k = mf_packet_bytes(host_pending[i], 0, b);
        for (uint8_t j = 0; j < k; j++) {
            if (b[j] == 0xF0) { hin = true; hlen = 0; }
            if (!hin) continue;
            hmsg[hlen++] = b[j];
            if (b[j] != 0xF7) continue;
            hin = false;
            static uint8_t frame[7000];
            long fl = (hlen >= 9) ? mf_unpack(hmsg + 7, hlen - 8, frame) : -1;
            if (fl < 6 || hmsg[1] != 0x7D) { bad_messages++; continue; }
            reply_t *r = &replies[n_replies % 64];
            r->tag = hmsg[6];
            r->type = frame[0];
            r->status = frame[1];
            r->len = (uint16_t)(frame[2] | frame[3] << 8);
            memcpy(r->payload, frame + 4, (size_t)fl - 6);
            uint16_t c = mf_crc16(frame, (size_t)fl - 2);
            r->crc_ok = (frame[fl - 2] == (c & 0xFF)) && (frame[fl - 1] == (c >> 8)) &&
                        (r->len == fl - 6);
            if (r->crc_ok) n_replies++;
            else bad_messages++;
        }
    }
    host_pkts = 0;
}
static int host_replies_seen(void) {
    usb_in(tx_level);       // what is in flight reaches the host during the reset wait
    host_ingest();
    return n_replies;
}

// One main-loop iteration: USB OUT, poll, USB IN (drain `in_pkts`), host parse.
static void step(int in_pkts, uint32_t dt_us) {
    usb_out();
    midi_ctrl_poll();
    usb_in(in_pkts);
    host_ingest();
    now_us += dt_us;
}
static void run(int iters, int in_pkts, uint32_t dt_us) { while (iters--) step(in_pkts, dt_us); }

static void reset_world(void) {
    mounted = false;
    midi_ctrl_poll();
    rx_head = rx_tail = 0; tx_level = 0; host_pkts = 0; host_out_n = host_out_i = 0;
    n_replies = 0; bad_messages = 0; hin = false;
    bulk_owner = -1; bulk_touches = 0; busy_left = 0; busy_forever = false;
    get_calls = set_calls = boot_calls = 0; bulk_applied = false; notify_n = 0;
    tx_fifo_cap = 128; rx_fifo_cap = 64;
    mounted = true;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void test_mount(void) {
    mounted = false;
    midi_ctrl_poll();
    CHECK(!notify_active, "consumer inactive while unmounted");
    mounted = true;
    midi_ctrl_poll();
    CHECK(notify_active, "consumer active once mounted");
}

static void test_get_small(void) {
    reset_world();
    host_send(0x01, MF_TYPE_GET, 0x7F, 0, 0, NULL);
    run(5, 64, 100);
    CHECK(n_replies == 1, "one reply (%d)", n_replies);
    reply_t *r = &replies[0];
    CHECK(r->tag == 1 && r->type == MF_RESP_GET && r->status == 0 && r->len == 7 && r->crc_ok &&
          memcmp(r->payload, resp7, 7) == 0, "platform reply");

    // Over-long dispatcher reply is clamped to the copy buffer (132).
    host_send(0x02, MF_TYPE_GET, 0x99, 0, 0, NULL);
    run(5, 64, 100);
    CHECK(n_replies == 2 && replies[1].len == MIDI_TX_COPY_MAX, "clamped to 132");

    // Unknown command -> ERROR with the tag.
    host_send(0x03, MF_TYPE_GET, 0x55, 0, 0, NULL);
    run(5, 64, 100);
    CHECK(n_replies == 3 && replies[2].status == CTRL_STATUS_ERROR && replies[2].tag == 3 &&
          replies[2].type == MF_RESP_GET, "error reply");
}

static void test_set_small(void) {
    reset_world();
    const uint8_t vol[4] = { 0x00, 0x00, 0xA0, 0xC1 };
    host_send(0x05, MF_TYPE_SET, 0xD2, 0, 4, vol);
    run(5, 64, 100);
    CHECK(n_replies == 1 && replies[0].type == MF_RESP_SET && replies[0].status == 0 &&
          replies[0].tag == 5 && replies[0].len == 0, "set reply");
    CHECK(last_set_len == 4 && memcmp(last_set_payload, vol, 4) == 0, "payload reached dispatcher");

    // 0xF5 / 0xF7 reach the dispatcher over MIDI (USB-path parity).
    const uint8_t ucfg[8] = { 0, 16, 17, 0, 0x00, 0xC2, 0x01, 0x00 };
    host_send(0x06, MF_TYPE_SET, REQ_SET_UART_CONFIG, 0, 8, ucfg);
    run(5, 64, 100);
    CHECK(n_replies == 2 && replies[1].status == CTRL_STATUS_OK && last_breq == REQ_SET_UART_CONFIG &&
          last_set_len == 8 && memcmp(last_set_payload, ucfg, 8) == 0, "0xF5 dispatched");

    // Non-bulk payload > 64 -> OVERSIZE, nothing dispatched.
    uint8_t big[100] = { 0 };
    int sc = set_calls;
    host_send(0x07, MF_TYPE_SET, 0x20, 0, 100, big);
    run(10, 64, 100);
    CHECK(n_replies == 3 && replies[2].status == CTRL_STATUS_OVERSIZE && replies[2].tag == 7 &&
          set_calls == sc, "oversize");
}

static void test_rta_bands_all(void) {
    reset_world();
    host_send(0x08, MF_TYPE_GET, REQ_RTA_GET_BANDS_ALL, 0, 0, NULL);
    int it = 0;
    while (n_replies == 0 && it++ < 1000) step(8, 100);
    CHECK(n_replies == 1 && replies[0].status == 0 && replies[0].len == 300 &&
          replies[0].payload[299] == (uint8_t)(299 ^ 0x5A), "0x0F > 132 bytes, zero-copy");
    CHECK(bulk_owner == -1, "0x0F releases the lock");
}

static void test_chunks(void) {
    reset_world();
    static uint8_t img[BULK_SIZE];
    // GET 0xA2 in 2048-byte chunks.
    static uint8_t got[BULK_SIZE];
    size_t gl = 0;
    for (int k = 0; k < 3; k++) {
        int before = n_replies;
        host_send((uint8_t)(0x40 + k), MF_TYPE_GET, REQ_GET_ALL_PARAMS_CHUNK, (uint16_t)(k * 2048), 2048, NULL);
        int it = 0;
        while (n_replies == before && it++ < 5000) step(16, 100);
        CHECK(n_replies == before + 1 && replies[before].status == 0, "chunk %d ok", k);
        memcpy(got + gl, replies[before].payload, replies[before].len);
        gl += replies[before].len;
        if (k < 2) CHECK(bulk_owner == CTRL_SOURCE_MIDI && replies[before].len == 2048, "held between chunks");
    }
    int ok = gl == BULK_SIZE;
    for (int i = 0; ok && i < BULK_SIZE; i++) if (got[i] != (uint8_t)(i * 13 + 5)) ok = 0;
    CHECK(ok, "chunked GET image (%zu bytes)", gl);
    CHECK(bulk_owner == -1, "released after the final chunk");

    // Offset != 0 without a session -> ERROR.
    host_send(0x44, MF_TYPE_GET, REQ_GET_ALL_PARAMS_CHUNK, 2048, 2048, NULL);
    run(20, 64, 100);
    CHECK(replies[n_replies - 1].status == CTRL_STATUS_ERROR, "chunk GET without session");

    // A non-chunk request ends an open GET session.
    host_send(0x45, MF_TYPE_GET, REQ_GET_ALL_PARAMS_CHUNK, 0, 2048, NULL);
    run(200, 64, 100);
    CHECK(bulk_owner == CTRL_SOURCE_MIDI, "session open");
    host_send(0x46, MF_TYPE_GET, 0x7F, 0, 0, NULL);
    run(10, 64, 100);
    CHECK(bulk_owner == -1 && replies[n_replies - 1].tag == 0x46, "non-chunk request closes it");

    // Idle session reaped after 3 s.
    host_send(0x47, MF_TYPE_GET, REQ_GET_ALL_PARAMS_CHUNK, 0, 2048, NULL);
    run(200, 64, 100);
    CHECK(bulk_owner == CTRL_SOURCE_MIDI, "session open again");
    run(40, 64, 100000);
    CHECK(bulk_owner == -1, "idle session reaped");

    // SET 0xA3 in 2048-byte chunks, applied on the last byte.
    for (int i = 0; i < BULK_SIZE; i++) img[i] = (uint8_t)(i * 7 + 1);
    bulk_applied = false;
    for (int k = 0; k < 3; k++) {
        uint16_t off = (uint16_t)(k * 2048), n = (uint16_t)(k < 2 ? 2048 : BULK_SIZE - 4096);
        int before = n_replies;
        host_send((uint8_t)(0x50 + k), MF_TYPE_SET, REQ_SET_ALL_PARAMS_CHUNK, off, n, img + off);
        int it = 0;
        while (n_replies == before && it++ < 5000) step(64, 100);
        CHECK(n_replies == before + 1 && replies[before].status == 0, "set chunk %d", k);
        if (k < 2) CHECK(!bulk_applied && bulk_owner == CTRL_SOURCE_MIDI, "not applied yet");
    }
    CHECK(bulk_applied && memcmp(bulk_param_buf, img, BULK_SIZE) == 0 && bulk_owner == -1,
          "chunked SET applied");

    // Out of order -> ERROR; the session survives a restart at 0.
    host_send(0x60, MF_TYPE_SET, REQ_SET_ALL_PARAMS_CHUNK, 0, 2048, img);
    run(200, 64, 100);
    host_send(0x61, MF_TYPE_SET, REQ_SET_ALL_PARAMS_CHUNK, 4096, 2040, img + 4096);
    run(200, 64, 100);
    CHECK(replies[n_replies - 1].tag == 0x61 && replies[n_replies - 1].status == CTRL_STATUS_ERROR,
          "out-of-order chunk refused");
    host_send(0x62, MF_TYPE_SET, REQ_SET_ALL_PARAMS_CHUNK, 0, 2048, img);
    run(200, 64, 100);
    CHECK(replies[n_replies - 1].status == 0, "restart at 0");
    host_send(0x63, MF_TYPE_GET, 0x7F, 0, 0, NULL);
    run(10, 64, 100);
    CHECK(bulk_owner == -1, "non-chunk request ends SET session");

    // Lock held by another transport -> BULK_LOCKED.
    bulk_owner = CTRL_SOURCE_USB;
    host_send(0x64, MF_TYPE_SET, REQ_SET_ALL_PARAMS_CHUNK, 0, 2048, img);
    run(200, 64, 100);
    CHECK(replies[n_replies - 1].status == CTRL_STATUS_BULK_LOCKED && bulk_owner == CTRL_SOURCE_USB,
          "chunk SET bulk locked");
    host_send(0x65, MF_TYPE_GET, REQ_GET_ALL_PARAMS_CHUNK, 0, 2048, NULL);
    run(200, 64, 1000);
    CHECK(replies[n_replies - 1].status == CTRL_STATUS_BULK_LOCKED, "chunk GET bulk locked");
    bulk_owner = -1;
}

static void test_errors(void) {
    reset_world();
    uint8_t m[64];
    mf_enc_t e;
    mf_enc_begin_request(&e, &ID, 0x09, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL);
    size_t n = mf_enc_read(&e, m, sizeof m);
    m[10] ^= 0x01;
    host_send_raw(m, n);
    run(5, 64, 100);
    CHECK(n_replies == 1 && replies[0].status == CTRL_STATUS_CRC_ERROR && replies[0].tag == 9 &&
          get_calls == 0, "CRC error reply, not dispatched");

    const uint8_t badver[] = { 0xF0, 0x7D, 'D', 'S', 'P', 0x05, 0x01, 0x00, 0xF7 };
    host_send_raw(badver, sizeof badver);
    run(5, 64, 100);
    CHECK(n_replies == 2 && replies[1].status == CTRL_STATUS_FRAME_ERROR && replies[1].tag == 0x7F,
          "bad version -> tag 0x7F");

    const uint8_t foreign[] = { 0xF0, 0x41, 0x10, 0x42, 0x12, 0xF7 };
    host_send_raw(foreign, sizeof foreign);
    run(5, 64, 100);
    CHECK(n_replies == 2, "foreign SysEx ignored");
}

static void test_bulk_set(void) {
    reset_world();
    static uint8_t img[BULK_SIZE];
    for (int i = 0; i < BULK_SIZE; i++) img[i] = (uint8_t)(i * 31 + 7);
    rx_fifo_cap = 16;           // force many polls and back-pressure
    host_send(0x21, MF_TYPE_SET, REQ_SET_ALL_PARAMS, 0, BULK_SIZE, img);
    int it = 0;
    while (n_replies == 0 && it++ < 5000) step(64, 50);
    CHECK(n_replies == 1 && replies[0].status == 0 && replies[0].tag == 0x21, "bulk set ok");
    CHECK(bulk_applied && memcmp(bulk_param_buf, img, BULK_SIZE) == 0, "bulk payload landed");
    CHECK(bulk_touches >= BULK_SIZE / 64 - 2, "lock refreshed (%d touches)", bulk_touches);
    CHECK(bulk_owner == -1, "lock handed over");

    // Lock held elsewhere -> BULK_LOCKED after the message; buffer untouched.
    bulk_owner = CTRL_SOURCE_UART;
    memset(bulk_param_buf, 0xEE, BULK_SIZE);
    host_send(0x22, MF_TYPE_SET, REQ_SET_ALL_PARAMS, 0, BULK_SIZE, img);
    it = 0;
    while (n_replies == 1 && it++ < 5000) step(64, 50);
    CHECK(n_replies == 2 && replies[1].status == CTRL_STATUS_BULK_LOCKED && replies[1].tag == 0x22,
          "bulk locked");
    CHECK(bulk_param_buf[0] == 0xEE && bulk_owner == CTRL_SOURCE_UART, "foreign lock untouched");
    bulk_owner = -1;

    // CRC error on a bulk SET releases the lock.
    static uint8_t m[7100];
    mf_enc_t e;
    mf_enc_begin_request(&e, &ID, 0x23, MF_TYPE_SET, REQ_SET_ALL_PARAMS, 0, 0, BULK_SIZE, img);
    size_t n = mf_enc_read(&e, m, sizeof m);
    m[3000] ^= 0x01;
    host_send_raw(m, n);
    it = 0;
    while (n_replies == 2 && it++ < 5000) step(64, 50);
    CHECK(n_replies == 3 && replies[2].status == CTRL_STATUS_CRC_ERROR && bulk_owner == -1,
          "bulk CRC error releases lock");

    // Host goes silent mid-payload: the 1 s timeout releases the lock.
    host_send_raw(m, 3000);
    run(200, 64, 50);
    CHECK(bulk_owner == CTRL_SOURCE_MIDI, "lock held while receiving");
    run(30, 64, 50000);
    CHECK(bulk_owner == -1 && n_replies == 3, "timeout releases lock silently");

    // Unplug mid-payload releases the lock.
    host_send_raw(m, 3000);
    run(200, 64, 50);
    CHECK(bulk_owner == CTRL_SOURCE_MIDI, "lock held again");
    mounted = false;
    step(64, 50);
    CHECK(bulk_owner == -1, "unmount releases lock");
    mounted = true;
    rx_head = rx_tail = 0;
    step(64, 50);
}

static void test_bulk_get(void) {
    reset_world();
    host_send(0x31, MF_TYPE_GET, REQ_GET_ALL_PARAMS, 0, 0, NULL);
    // Host reads slowly: 8 packets per poll.
    int it = 0, held_polls = 0;
    while (n_replies == 0 && it++ < 20000) {
        step(8, 100);
        if (bulk_owner == CTRL_SOURCE_MIDI) held_polls++;
    }
    CHECK(n_replies == 1 && replies[0].status == 0 && replies[0].len == BULK_SIZE && replies[0].crc_ok,
          "bulk get reply");
    int ok = 1;
    for (int i = 0; i < BULK_SIZE; i++) if (replies[0].payload[i] != (uint8_t)(i * 13 + 5)) ok = 0;
    CHECK(ok, "bulk get payload");
    CHECK(held_polls > 10 && bulk_owner == -1, "lock held while streaming, then released");
    CHECK(bulk_touches > 50, "lock refreshed while streaming (%d)", bulk_touches);

    // Host stops reading mid-reply: lock released after the stall timeout and
    // the SysEx is ended early (the host sees a malformed message).
    host_send(0x32, MF_TYPE_GET, REQ_GET_ALL_PARAMS, 0, 0, NULL);
    run(20, 4, 100);
    CHECK(bulk_owner == CTRL_SOURCE_MIDI, "streaming");
    run(10, 0, 100000);                 // 1 s, no USB IN
    CHECK(bulk_owner == -1, "stall releases lock");
    int bad0 = bad_messages;
    run(200, 64, 100);
    CHECK(n_replies == 1 && bad_messages == bad0 + 1, "truncated reply seen as malformed");

    // The next request still works.
    host_send(0x33, MF_TYPE_GET, 0x7F, 0, 0, NULL);
    run(10, 64, 100);
    CHECK(n_replies == 2 && replies[1].tag == 0x33 && replies[1].status == 0, "recovered");
}

static void test_busy(void) {
    reset_world();
    busy_left = 3;
    host_send(0x41, MF_TYPE_GET, 0x7F, 0, 0, NULL);
    run(10, 64, 1000);
    CHECK(n_replies == 1 && replies[0].status == 0 && get_calls == 4, "BUSY retried within window");

    busy_forever = true;
    host_send(0x42, MF_TYPE_SET, 0xD2, 0, 4, (const uint8_t *)"\0\0\0\0");
    run(100, 64, 1000);
    CHECK(n_replies == 2 && replies[1].status == CTRL_STATUS_BUSY && replies[1].tag == 0x42,
          "BUSY reported after 50 ms");
    busy_forever = false;

    // GET 0xA0 while the lock is elsewhere: retried, then BULK_LOCKED.
    bulk_owner = CTRL_SOURCE_I2C;
    host_send(0x43, MF_TYPE_GET, REQ_GET_ALL_PARAMS, 0, 0, NULL);
    run(100, 64, 1000);
    CHECK(n_replies == 3 && replies[2].status == CTRL_STATUS_BULK_LOCKED, "bulk get locked");
    bulk_owner = -1;
}

static void test_pipelining(void) {
    reset_world();
    // Three requests back to back: answered in order, none dropped.
    host_send(0x51, MF_TYPE_GET, 0x7F, 0, 0, NULL);
    host_send(0x52, MF_TYPE_GET, 0x7F, 0, 0, NULL);
    host_send(0x53, MF_TYPE_SET, 0xD2, 0, 4, (const uint8_t *)"\1\2\3\4");
    run(30, 2, 100);
    CHECK(n_replies == 3 && replies[0].tag == 0x51 && replies[1].tag == 0x52 && replies[2].tag == 0x53,
          "in order (%d replies)", n_replies);
}

static void test_notify(void) {
    reset_world();
    const uint8_t pkt[14] = { 2, 2, 0, 7, 0xD4, 0x0A, 4, 0, 10, 0, 0, 0, 0xAA, 0xBB };
    push_notify(pkt, sizeof pkt);
    run(3, 64, 100);
    CHECK(n_replies == 1 && replies[0].tag == 0 && replies[0].type == MF_TYPE_NOTIFY &&
          replies[0].len == sizeof pkt && memcmp(replies[0].payload, pkt, sizeof pkt) == 0, "notify");

    // Never inside a response: queue a notify while a bulk reply streams.
    host_send(0x61, MF_TYPE_GET, REQ_GET_ALL_PARAMS, 0, 0, NULL);
    run(3, 4, 100);
    push_notify(pkt, sizeof pkt);
    int it = 0;
    while (n_replies < 3 && it++ < 20000) step(4, 100);
    CHECK(n_replies == 3 && replies[1].tag == 0x61 && replies[1].len == BULK_SIZE &&
          replies[2].tag == 0 && bad_messages == 0, "notify after the response");

    // A pending request goes before a queued notification.
    tx_fifo_cap = 128;
    host_send(0x62, MF_TYPE_GET, 0x7F, 0, 0, NULL);
    push_notify(pkt, sizeof pkt);
    run(5, 64, 100);
    CHECK(n_replies == 5 && replies[3].tag == 0x62 && replies[4].tag == 0, "request first");

    // Unmounted: nothing pushed.
    mounted = false;
    step(64, 100);
    CHECK(!notify_active, "inactive when unmounted");
}

static void test_bootloader(void) {
    reset_world();
    // Something still in flight: the bootloader reply waits for an idle EP.
    push_notify((const uint8_t *)"\2\3\0\1\0\0\0\0", 8);
    host_send(0x71, MF_TYPE_GET, REQ_ENTER_BOOTLOADER, 0, 0, NULL);
    run(3, 0, 100);                 // host not reading: EP stays busy
    CHECK(boot_calls == 0, "not dispatched while the EP is busy");
    run(5, 64, 1000);
    CHECK(boot_calls == 1, "dispatched once the reply left");
    CHECK(boot_reply_seen_before_dispatch, "reply delivered before the reboot");
    int found = 0;
    for (int i = 0; i < n_replies; i++)
        if (replies[i].tag == 0x71 && replies[i].status == 0 && replies[i].len == 1 && replies[i].payload[0] == 1)
            found = 1;
    CHECK(found, "0xF0 reply = OK, 1 byte (1)");

    // Host never reads: dispatch anyway after the bounded wait.
    reset_world();
    host_send(0x72, MF_TYPE_GET, REQ_ENTER_BOOTLOADER, 0, 0, NULL);
    run(2, 64, 100);
    run(10, 0, 50000);
    CHECK(boot_calls >= 1, "bounded wait");
}

int main(void) {
    test_mount();
    test_get_small();
    test_set_small();
    test_rta_bands_all();
    test_chunks();
    test_errors();
    test_bulk_set();
    test_bulk_get();
    test_busy();
    test_pipelining();
    test_notify();
    test_bootloader();
    if (failures) {
        fprintf(stderr, "test_midi_control: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_midi_control: OK\n");
    return 0;
}
