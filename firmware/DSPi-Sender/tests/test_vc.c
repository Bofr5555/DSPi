/*
 * test_vc.c - host tests for the sender's vendor front end (WP FWPG4)
 *
 *  1. Page 0, the 1.1.6 envelope: unchanged replies, refusals and stages
 *     (EXT_LIST, the alink ids, platform, serial, build info, save, 0xF0).
 *  2. The paged 1.1.7 envelope: SYS discovery (PROTOCOL_INFO, PAGE_LIST,
 *     PAGE_COMMANDS, COMMAND_INFO, LAST_RESULT) and identity, STORE's
 *     OUTPUT_CONFIG_SAVE, SYS.ENTER_BOOTLOADER after the status stage, the
 *     tunnel 0xE010 with its own checks, upstream's refusal codes and stages.
 *  3. Fail closed: every page and command the sender lacks is VC_ERR_UNKNOWN
 *     and reaches no handler; no ALL_PARAMS; never status 0x02 or 0x03.
 *  4. Both envelopes give the same bytes for every listed id.
 *
 * Expected bytes follow extension_channel_spec.md 1.3 and the console's
 * FakeDevice::sender(true) (crates/dspi-transport/src/fake/boards.rs).
 */

#include <stdio.h>
#include <string.h>

#include "sender_vc.h"

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

// ---------------------------------------------------------------------------
// Hooks: a fake sender behind the front end
// ---------------------------------------------------------------------------

static int hook_calls, save_calls;
static uint8_t set_result;              // what the next OUT handler answers
static uint16_t last_set_id;
static uint8_t last_set[16];

uint8_t snd_hook_ext_get(uint16_t id, uint8_t *o, uint16_t *n) {
    hook_calls++;
    uint16_t len;
    switch (id) {
    case SND_EXT_ALINK_INFO:      len = 32; break;
    case SND_EXT_ALINK_STATUS:    len = 52; break;
    case SND_EXT_ALINK_CONFIG:    len = 8; break;
    case SND_EXT_FRAME_COUNTER:   len = 16; break;
    case SND_EXT_ALINK_TX:        len = 16; break;
    case SND_EXT_ALINK_CTRL_MSG:  len = 24; break;
    case SND_EXT_ALINK_DIAG:      len = 64; break;
    default:                      return SND_VC_ERR_ADDRESS;
    }
    for (uint16_t i = 0; i < len; i++) o[i] = (uint8_t)(id + 3 * i);
    *n = len;
    return SND_VC_OK;
}

uint8_t snd_hook_ext_set(uint16_t id, const uint8_t *p) {
    hook_calls++;
    last_set_id = id;
    memcpy(last_set, p, snd_ext_set_len(id));
    return set_result;
}

uint16_t snd_hook_ident(snd_ident_t which, uint8_t *o) {
    hook_calls++;
    static const uint8_t plat[7] = { 1, 0x01, 0x16, 0, 1, 6, 0 };
    switch (which) {
    case SND_IDENT_PLATFORM:   memcpy(o, plat, 7); return 7;
    case SND_IDENT_SERIAL:     memcpy(o, "7D7E5CC4DEE46848", 16); return 16;
    case SND_IDENT_BUILD_INFO:
        memset(o, 0, 64);
        memcpy(o, "v1.1.6-102-gabcdef0", 19);
        memcpy(o + 48, "2026-11-19", 10);
        return 64;
    }
    return 0;
}

void snd_hook_save(void) { hook_calls++; save_calls++; }

// ---------------------------------------------------------------------------
// A host: one control transfer through the front end
// ---------------------------------------------------------------------------

typedef struct {
    bool ok;                // not STALLed at any stage
    int  stall_stage;       // 0 none, 1 SETUP, 2 status stage
    bool reboot;            // the device rebooted (legacy at once, paged after ACK)
    uint8_t data[64];
    uint16_t n;
} xfer_t;

static snd_vc_req_t req(const uint8_t s[8]) {
    snd_vc_req_t r = {
        .bmRequestType = s[0], .bRequest = s[1],
        .wValue = (uint16_t)(s[2] | s[3] << 8),
        .wIndex = (uint16_t)(s[4] | s[5] << 8),
        .wLength = (uint16_t)(s[6] | s[7] << 8),
    };
    return r;
}

static xfer_t run(const uint8_t setup[8], const uint8_t *payload) {
    xfer_t x;
    memset(&x, 0, sizeof(x));
    snd_vc_req_t r = req(setup);
    const uint8_t *out = 0;
    uint16_t n = 0;
    switch (snd_vc_setup(&r, &out, &n)) {
    case SND_VC_STALL:
        x.stall_stage = 1;
        return x;
    case SND_VC_SEND_REBOOT:
        x.reboot = true;
        /* fall through */
    case SND_VC_SEND:
        if (n > r.wLength) {
            CHECK(0, "reply of %u bytes for wLength %u", n, r.wLength);
            n = r.wLength;
        }
        memcpy(x.data, out, n);
        x.n = n;
        break;
    case SND_VC_RECV:
        if (!snd_vc_data(&r, payload)) {
            x.stall_stage = 2;
            return x;
        }
        break;
    case SND_VC_STATUS:
        break;
    }
    if (snd_vc_ack(&r)) x.reboot = true;
    x.ok = true;
    return x;
}

#define SETUP(...) ((const uint8_t[8]){ __VA_ARGS__ })

static bool eq(const xfer_t *x, const uint8_t *want, uint16_t n) {
    return x->ok && x->n == n && memcmp(x->data, want, n) == 0;
}

// SYS.LAST_RESULT through the paged stack.
static xfer_t last_result(void) { return run(SETUP(0xC1, 0x14, 0, 0, 0x02, 0x01, 8, 0), 0); }

static bool last_is(uint16_t id, bool get, uint8_t result, uint16_t wvalue) {
    xfer_t x = last_result();
    uint8_t want[8] = { (uint8_t)id, (uint8_t)(id >> 8), get ? 1 : 0, result,
                        (uint8_t)wvalue, (uint8_t)(wvalue >> 8), 0, 0 };
    return eq(&x, want, 8);
}

static const uint8_t EXT_LIST_BYTES[18] = {
    0x03, 0x00, 0x00, 0x80, 0x01, 0x80, 0x02, 0x80, 0x05, 0x80,
    0x07, 0x80, 0x08, 0x80, 0x09, 0x80, 0x0A, 0x80,
};

// ---------------------------------------------------------------------------
// 1. Page 0
// ---------------------------------------------------------------------------

static void test_legacy(void) {
    // EXT_LIST, spec 1.3.1's SETUP, and wIndex 0x0000 (UART-style) alike.
    xfer_t x = run(SETUP(0xC1, 0x00, 0x00, 0x00, 0x02, 0x00, 0x40, 0x00), 0);
    CHECK(eq(&x, EXT_LIST_BYTES, 18), "legacy EXT_LIST (n %u)", x.n);
    x = run(SETUP(0xC1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00), 0);
    CHECK(eq(&x, EXT_LIST_BYTES, 18), "legacy EXT_LIST at wIndex 0");
    x = run(SETUP(0xC1, 0x00, 0x00, 0x00, 0x02, 0x00, 4, 0), 0);
    CHECK(eq(&x, EXT_LIST_BYTES, 4), "EXT_LIST truncated to wLength");
    x = run(SETUP(0xC1, 0x00, 0x03, 0x00, 0x02, 0x00, 64, 0), 0);
    CHECK(eq(&x, (const uint8_t[4]){ 0 }, 4), "EXT_NOTIFY_FEATURES reads 0");

    // An alink IN id reaches its handler, truncated to wLength.
    x = run(SETUP(0xC1, 0x00, 0x01, 0x80, 0x02, 0x00, 64, 0), 0);
    CHECK(x.ok && x.n == 52 && x.data[0] == 0x01 && x.data[51] == (uint8_t)(0x01 + 153), "STATUS");
    x = run(SETUP(0xC1, 0x00, 0x0A, 0x80, 0x02, 0x00, 10, 0), 0);
    CHECK(x.ok && x.n == 10, "DIAG truncated");

    // Refused IN ids: STALL at SETUP, no handler.
    const uint16_t bad_in[] = { 0x0001, 0x0002, 0x0004, 0x0006, 0x8003, 0x8004, 0x8006, 0x8009,
                                0x8010, 0x8011, 0x8020, 0x8030, 0x8100, 0xFFFF };
    for (unsigned i = 0; i < sizeof(bad_in) / sizeof(bad_in[0]); i++) {
        int before = hook_calls;
        x = run(SETUP(0xC1, 0x00, (uint8_t)bad_in[i], (uint8_t)(bad_in[i] >> 8), 0x02, 0x00, 64, 0), 0);
        CHECK(x.stall_stage == 1 && hook_calls == before, "legacy IN 0x%04X must STALL", bad_in[i]);
    }

    // Identity, save, bootloader.
    x = run(SETUP(0xC1, 0x7F, 0, 0, 0x02, 0x00, 64, 0), 0);
    CHECK(eq(&x, (const uint8_t[7]){ 1, 0x01, 0x16, 0, 1, 6, 0 }, 7), "platform: 7 bytes, 0 outputs");
    x = run(SETUP(0xC1, 0x7E, 0, 0, 0x02, 0x00, 64, 0), 0);
    CHECK(eq(&x, (const uint8_t *)"7D7E5CC4DEE46848", 16), "serial");
    x = run(SETUP(0xC1, 0x80, 0, 0, 0x02, 0x00, 64, 0), 0);
    CHECK(x.ok && x.n == 64 && memcmp(x.data, "v1.1.6-", 7) == 0 && x.data[48] == '2', "build info");
    int saves = save_calls;
    x = run(SETUP(0xC1, 0x52, 0, 0, 0x02, 0x00, 1, 0), 0);
    CHECK(eq(&x, (const uint8_t[1]){ 0 }, 1) && save_calls == saves + 1, "save: PRESET_OK");
    x = run(SETUP(0xC1, 0xF0, 0, 0, 0x02, 0x00, 1, 0), 0);
    CHECK(eq(&x, (const uint8_t[1]){ 1 }, 1) && x.reboot, "0xF0: [1], then the bootloader");

    // Everything else STALLs: a sender has no DSP.
    for (unsigned op = 0x01; op <= 0xFF; op++) {
        if (op == 0x52 || op == 0x7E || op == 0x7F || op == 0x80 || op == 0xF0) continue;
        int before = hook_calls;
        x = run(SETUP(0xC1, (uint8_t)op, 0, 0, 0x02, 0x00, 64, 0), 0);
        CHECK(x.stall_stage == 1 && hook_calls == before, "legacy IN opcode 0x%02X must STALL", op);
        x = run(SETUP(0x41, (uint8_t)op, 0, 0, 0x02, 0x00, 0, 0), 0);
        CHECK(x.stall_stage == 1 && hook_calls == before, "legacy OUT opcode 0x%02X must STALL", op);
    }

    // OUT extension: an ID and length problem STALLs at SETUP, a refused
    // payload at the status stage; an accepted one reaches the handler.
    uint8_t cfg[8] = { 0, 0xFF, 0, 0xFF, 0xFF, 0xFF, 3, 0 };
    set_result = SND_VC_OK;
    x = run(SETUP(0x41, 0x00, 0x02, 0x80, 0x02, 0x00, 8, 0), cfg);
    CHECK(x.ok && last_set_id == 0x8002 && memcmp(last_set, cfg, 8) == 0, "CONFIG write accepted");
    x = run(SETUP(0x41, 0x00, 0x02, 0x80, 0x02, 0x00, 7, 0), cfg);
    CHECK(x.stall_stage == 1, "CONFIG short: STALL at SETUP");
    x = run(SETUP(0x41, 0x00, 0x00, 0x80, 0x02, 0x00, 8, 0), cfg);
    CHECK(x.stall_stage == 1, "OUT INFO (no OUT form): STALL at SETUP");
    x = run(SETUP(0x41, 0x00, 0x00, 0x00, 0x02, 0x00, 2, 0), cfg);
    CHECK(x.stall_stage == 1, "OUT EXT_LIST: STALL at SETUP");
    x = run(SETUP(0x41, 0x00, 0x02, 0x00, 0x02, 0x00, 2, 0), cfg);
    CHECK(x.stall_stage == 1, "OUT CORE_VREG (not a sender id): STALL at SETUP");
    set_result = SND_VC_ERR_PIN_IN_USE;
    x = run(SETUP(0x41, 0x00, 0x02, 0x80, 0x02, 0x00, 8, 0), cfg);
    CHECK(x.stall_stage == 2, "refused CONFIG: STALL at the status stage");
    set_result = SND_VC_ERR_VALUE;
    x = run(SETUP(0x41, 0x00, 0x09, 0x80, 0x02, 0x00, 1, 0), (const uint8_t[1]){ 2 });
    CHECK(x.stall_stage == 2, "refused STATS_RESET: STALL at the status stage");
    set_result = SND_VC_OK;
    x = run(SETUP(0x41, 0x00, 0x09, 0x80, 0x02, 0x00, 1, 0), (const uint8_t[1]){ 1 });
    CHECK(x.ok && last_set_id == 0x8009, "STATS_RESET");

    // Page 0 records nothing in SYS.LAST_RESULT.
    x = run(SETUP(0xC1, 0x10, 0, 0, 0x02, 0x01, 16, 0), 0);
    CHECK(x.ok, "probe");
    x = run(SETUP(0xC1, 0x00, 0x00, 0x81, 0x02, 0x00, 64, 0), 0);
    CHECK(x.stall_stage == 1, "page-0 refusal");
    CHECK(last_is(0x0110, true, SND_VC_OK, 0), "page 0 left LAST_RESULT alone");
}

// ---------------------------------------------------------------------------
// 2. The paged envelope
// ---------------------------------------------------------------------------

static void test_discovery(void) {
    // The probe, spec 1.3.6's SETUP.
    xfer_t x = run(SETUP(0xC1, 0x10, 0x00, 0x00, 0x02, 0x01, 0x10, 0x00), 0);
    const uint8_t info[16] = { 'D', 'S', 'P', 'i', 1, 0, 3, 0, 64, 0, 64, 0, 0, 0, 0, 0 };
    CHECK(eq(&x, info, 16), "SYS.PROTOCOL_INFO");
    x = run(SETUP(0xC1, 0x10, 0x00, 0x00, 0x02, 0x01, 8, 0), 0);
    CHECK(eq(&x, info, 8), "PROTOCOL_INFO truncated to wLength");
    x = run(SETUP(0xC1, 0x10, 0x00, 0x00, 0x00, 0x01, 16, 0), 0);
    CHECK(eq(&x, info, 16), "the low byte of wIndex is not read");

    x = run(SETUP(0xC1, 0x11, 0, 0, 0x02, 0x01, 64, 0), 0);
    CHECK(eq(&x, (const uint8_t[6]){ 0x01, 1, 0x02, 1, 0xE0, 1 }, 6), "SYS.PAGE_LIST: SYS, STORE, 0xE0");

    // PAGE_COMMANDS: GET bitmap then SET bitmap, bit n = command n.
    uint8_t want[64];
    memset(want, 0, sizeof(want));
    want[0x10 >> 3] = 0x1F;                          // 0x10-0x14
    want[0x20 >> 3] = 0x07;                          // 0x20-0x22
    want[32 + (0x30 >> 3)] = 0x01;                   // SET 0x30
    x = run(SETUP(0xC1, 0x12, 0x01, 0x00, 0x02, 0x01, 64, 0), 0);
    CHECK(eq(&x, want, 64), "PAGE_COMMANDS SYS");
    memset(want, 0, sizeof(want));
    want[32 + (0x21 >> 3)] = 0x02;                   // SET 0x21 only: no ALL_PARAMS, no presets
    x = run(SETUP(0xC1, 0x12, 0x02, 0x00, 0x02, 0x01, 64, 0), 0);
    CHECK(eq(&x, want, 64), "PAGE_COMMANDS STORE");
    memset(want, 0, sizeof(want));
    want[2] = 0x01;
    want[34] = 0x01;                                 // spec 1.3.6: byte 2 bit 0 and byte 34 bit 0
    x = run(SETUP(0xC1, 0x12, 0xE0, 0x00, 0x02, 0x01, 64, 0), 0);
    CHECK(eq(&x, want, 64), "PAGE_COMMANDS 0xE0");
    const uint16_t absent_pages[] = { 0x00, 0x03, 0x04, 0x0F, 0xE1, 0xEF, 0xF0, 0xFF };
    for (unsigned i = 0; i < sizeof(absent_pages) / sizeof(absent_pages[0]); i++) {
        x = run(SETUP(0xC1, 0x12, (uint8_t)absent_pages[i], 0, 0x02, 0x01, 64, 0), 0);
        CHECK(x.stall_stage == 1 && last_is(0x0112, true, SND_VC_ERR_ADDRESS, absent_pages[i]),
              "PAGE_COMMANDS 0x%02X: ADDRESS", absent_pages[i]);
    }
    x = run(SETUP(0xC1, 0x12, 0x01, 0x01, 0x02, 0x01, 64, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0x0112, true, SND_VC_ERR_ADDRESS, 0x0101), "page above 0xFF");

    // COMMAND_INFO: flags, GET length, SET min/max, address kind, forms.
    struct { uint16_t id; uint8_t b[8]; } ci[] = {
        { 0xE010, { 0x00, 0x00, 0xFF, 0xFF, 0x01, 0x40, 0x18, 0x03 } },    // spec 1.3.2
        { 0x0110, { 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01 } },
        { 0x0111, { 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01 } },
        { 0x0112, { 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 21, 0x01 } },
        { 0x0113, { 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 22, 0x01 } },
        { 0x0114, { 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x01 } },
        { 0x0120, { 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01 } },
        { 0x0121, { 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x01 } },
        { 0x0122, { 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01 } },
        { 0x0130, { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 } },    // RESTART | SET_USB_ONLY (1.3.7)
        { 0x0221, { 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 } },    // DEFERRED | FLASH | RESTART
    };
    for (unsigned i = 0; i < sizeof(ci) / sizeof(ci[0]); i++) {
        x = run(SETUP(0xC1, 0x13, (uint8_t)ci[i].id, (uint8_t)(ci[i].id >> 8), 0x02, 0x01, 8, 0), 0);
        CHECK(eq(&x, ci[i].b, 8), "COMMAND_INFO 0x%04X", ci[i].id);
    }
    const uint16_t no_info[] = { 0x0124, 0x0140, 0x0210, 0x0240, 0x0241, 0x0230, 0x0310, 0xE011, 0x0000 };
    for (unsigned i = 0; i < sizeof(no_info) / sizeof(no_info[0]); i++) {
        x = run(SETUP(0xC1, 0x13, (uint8_t)no_info[i], (uint8_t)(no_info[i] >> 8), 0x02, 0x01, 8, 0), 0);
        CHECK(x.stall_stage == 1 && last_is(0x0113, true, SND_VC_ERR_ADDRESS, no_info[i]),
              "COMMAND_INFO 0x%04X: ADDRESS", no_info[i]);
    }

    // Identity: the same bytes as page 0.
    xfer_t l = run(SETUP(0xC1, 0x7F, 0, 0, 0x02, 0x00, 64, 0), 0);
    x = run(SETUP(0xC1, 0x20, 0, 0, 0x02, 0x01, 64, 0), 0);
    CHECK(eq(&x, l.data, l.n) && x.n == 7, "SYS.PLATFORM == REQ_GET_PLATFORM");
    l = run(SETUP(0xC1, 0x7E, 0, 0, 0x02, 0x00, 64, 0), 0);
    x = run(SETUP(0xC1, 0x21, 0, 0, 0x02, 0x01, 64, 0), 0);
    CHECK(eq(&x, l.data, l.n) && x.n == 16, "SYS.SERIAL");
    l = run(SETUP(0xC1, 0x80, 0, 0, 0x02, 0x00, 64, 0), 0);
    x = run(SETUP(0xC1, 0x22, 0, 0, 0x02, 0x01, 64, 0), 0);
    CHECK(eq(&x, l.data, l.n) && x.n == 64, "SYS.BUILD_INFO");
}

static void test_paged_rules(void) {
    // LAST_RESULT: id, direction, result, wValue; a GET of it records nothing.
    xfer_t x = run(SETUP(0xC1, 0x20, 0, 0, 0x02, 0x01, 7, 0), 0);
    CHECK(last_is(0x0120, true, SND_VC_OK, 0), "LAST_RESULT after PLATFORM");
    CHECK(last_is(0x0120, true, SND_VC_OK, 0), "reading LAST_RESULT keeps it");

    // Upstream's checks, in order, at SETUP.
    x = run(SETUP(0x41, 0x10, 0, 0, 0x02, 0x01, 0, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0x0110, false, SND_VC_ERR_DIRECTION, 0), "SET PROTOCOL_INFO");
    x = run(SETUP(0xC1, 0x30, 0, 0, 0x02, 0x01, 1, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0x0130, true, SND_VC_ERR_DIRECTION, 0), "GET ENTER_BOOTLOADER");
    x = run(SETUP(0x41, 0x30, 0, 0, 0x02, 0x01, 1, 0), (const uint8_t[1]){ 0 });
    CHECK(x.stall_stage == 1 && !x.reboot && last_is(0x0130, false, SND_VC_ERR_LENGTH, 0),
          "ENTER_BOOTLOADER with a payload");
    x = run(SETUP(0xC1, 0x10, 1, 0, 0x02, 0x01, 16, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0x0110, true, SND_VC_ERR_ADDRESS, 1), "PROTOCOL_INFO wValue 1");
    x = run(SETUP(0x41, 0x21, 5, 0, 0x02, 0x02, 0, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0x0221, false, SND_VC_ERR_ADDRESS, 5), "OUTPUT_CONFIG_SAVE wValue 5");

    // STORE.OUTPUT_CONFIG_SAVE: a zero-length SET, run at SETUP.
    int saves = save_calls;
    x = run(SETUP(0x41, 0x21, 0, 0, 0x02, 0x02, 0, 0), 0);
    CHECK(x.ok && !x.reboot && save_calls == saves + 1 && last_is(0x0221, false, SND_VC_OK, 0),
          "STORE.OUTPUT_CONFIG_SAVE");

    // SYS.ENTER_BOOTLOADER: accepted, the reboot after the status stage only.
    snd_vc_req_t r = req(SETUP(0x41, 0x30, 0, 0, 0x02, 0x01, 0, 0));
    const uint8_t *out;
    uint16_t n;
    CHECK(snd_vc_setup(&r, &out, &n) == SND_VC_STATUS, "ENTER_BOOTLOADER: status");
    CHECK(snd_vc_ack(&r) && !snd_vc_ack(&r), "reboot once, after the ACK");
    CHECK(snd_vc_setup(&r, &out, &n) == SND_VC_STATUS, "again");
    snd_vc_req_t other = req(SETUP(0xC1, 0x10, 0, 0, 0x02, 0x01, 16, 0));
    snd_vc_setup(&other, &out, &n);
    CHECK(!snd_vc_ack(&r), "a new SETUP drops a reboot whose status stage never completed");
}

static void test_tunnel(void) {
    // EXT_LIST through the tunnel, spec 1.3.1's SETUP: the same bytes.
    xfer_t x = run(SETUP(0xC1, 0x10, 0x00, 0x00, 0x02, 0xE0, 0x40, 0x00), 0);
    CHECK(eq(&x, EXT_LIST_BYTES, 18), "tunnel EXT_LIST");
    CHECK(last_is(0xE010, true, SND_VC_OK, 0x0000), "LAST_RESULT carries the sub-command");

    // Every listed IN id: byte-identical in both envelopes.
    for (uint32_t id = 0; id <= 0xFFFF; id++) {
        if (id != 0 && !snd_ext_listed((uint16_t)id)) continue;
        if (!snd_ext_has_get((uint16_t)id)) continue;
        xfer_t a = run(SETUP(0xC1, 0x00, (uint8_t)id, (uint8_t)(id >> 8), 0x02, 0x00, 64, 0), 0);
        xfer_t b = run(SETUP(0xC1, 0x10, (uint8_t)id, (uint8_t)(id >> 8), 0x02, 0xE0, 64, 0), 0);
        CHECK(a.ok && b.ok && a.n == b.n && memcmp(a.data, b.data, a.n) == 0, "0x%04X same in both", id);
        b = run(SETUP(0xC1, 0x10, (uint8_t)id, (uint8_t)(id >> 8), 0x02, 0xE0, 3, 0), 0);
        CHECK(b.ok && b.n == 3, "0x%04X truncated", id);
    }

    // The tunnel's own GET checks: unlisted -> ADDRESS, no IN form -> DIRECTION.
    const uint16_t unlisted[] = { 0x0001, 0x0002, 0x0004, 0x0005, 0x0006, 0x8003, 0x8004, 0x8006,
                                  0x800B, 0x8010, 0x8011, 0x8012, 0x8020, 0x8030, 0x8031, 0x8100, 0xFFFF };
    for (unsigned i = 0; i < sizeof(unlisted) / sizeof(unlisted[0]); i++) {
        int before = hook_calls;
        uint16_t id = unlisted[i];
        x = run(SETUP(0xC1, 0x10, (uint8_t)id, (uint8_t)(id >> 8), 0x02, 0xE0, 64, 0), 0);
        CHECK(x.stall_stage == 1 && hook_calls == before && last_is(0xE010, true, SND_VC_ERR_ADDRESS, id),
              "tunnel GET 0x%04X: ADDRESS", id);
        x = run(SETUP(0x41, 0x10, (uint8_t)id, (uint8_t)(id >> 8), 0x02, 0xE0, 8, 0),
                (const uint8_t[8]){ 0 });
        CHECK(x.stall_stage == 2 && hook_calls == before && last_is(0xE010, false, SND_VC_ERR_ADDRESS, id),
              "tunnel SET 0x%04X: ADDRESS at the status stage", id);
    }
    x = run(SETUP(0xC1, 0x10, 0x09, 0x80, 0x02, 0xE0, 64, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0xE010, true, SND_VC_ERR_DIRECTION, 0x8009), "GET STATS_RESET");

    // SET: no OUT form -> DIRECTION, wrong length -> LENGTH, both at the
    // status stage; the handler never runs.
    const uint16_t get_only[] = { 0x0000, 0x0003, 0x8000, 0x8001, 0x8005, 0x800A };
    for (unsigned i = 0; i < sizeof(get_only) / sizeof(get_only[0]); i++) {
        int before = hook_calls;
        uint16_t id = get_only[i];
        x = run(SETUP(0x41, 0x10, (uint8_t)id, (uint8_t)(id >> 8), 0x02, 0xE0, 4, 0),
                (const uint8_t[4]){ 0 });
        CHECK(x.stall_stage == 2 && hook_calls == before && last_is(0xE010, false, SND_VC_ERR_DIRECTION, id),
              "tunnel SET 0x%04X: DIRECTION", id);
    }
    uint8_t p16[16] = { 0 };
    int before = hook_calls;
    x = run(SETUP(0x41, 0x10, 0x02, 0x80, 0x02, 0xE0, 16, 0), p16);
    CHECK(x.stall_stage == 2 && hook_calls == before && last_is(0xE010, false, SND_VC_ERR_LENGTH, 0x8002),
          "CONFIG with 16 bytes: LENGTH");
    // The row's 1-64 bytes, checked at SETUP.
    x = run(SETUP(0x41, 0x10, 0x02, 0x80, 0x02, 0xE0, 0, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0xE010, false, SND_VC_ERR_LENGTH, 0x8002), "SET of 0 bytes");
    x = run(SETUP(0x41, 0x10, 0x02, 0x80, 0x02, 0xE0, 65, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0xE010, false, SND_VC_ERR_LENGTH, 0x8002), "SET of 65 bytes");

    // Accepted writes reach the same handler as page 0; refusals carry its code.
    uint8_t cfg[8] = { 1, 0xFF, 0, 0xFF, 0xFF, 0xFF, 3, 0 };
    set_result = SND_VC_OK;
    x = run(SETUP(0x41, 0x10, 0x02, 0x80, 0x02, 0xE0, 8, 0), cfg);
    CHECK(x.ok && last_set_id == 0x8002 && memcmp(last_set, cfg, 8) == 0 &&
          last_is(0xE010, false, SND_VC_OK, 0x8002), "tunnel CONFIG write");
    const uint8_t codes[] = { SND_VC_ERR_VALUE, SND_VC_ERR_UNSUPPORTED, SND_VC_ERR_STATE,
                              SND_VC_ERR_PIN_INVALID, SND_VC_ERR_PIN_IN_USE };
    for (unsigned i = 0; i < sizeof(codes); i++) {
        set_result = codes[i];
        x = run(SETUP(0x41, 0x10, 0x07, 0x80, 0x02, 0xE0, 16, 0), p16);
        CHECK(x.stall_stage == 2 && last_is(0xE010, false, codes[i], 0x8007), "handler code 0x%02X", codes[i]);
    }
    set_result = SND_VC_OK;
    x = run(SETUP(0x41, 0x10, 0x08, 0x80, 0x02, 0xE0, 16, 0), p16);
    CHECK(x.ok && last_set_id == 0x8008, "tunnel CTRL_MSG");
    x = run(SETUP(0x41, 0x10, 0x09, 0x80, 0x02, 0xE0, 1, 0), (const uint8_t[1]){ 1 });
    CHECK(x.ok && last_set_id == 0x8009, "tunnel STATS_RESET");
}

// ---------------------------------------------------------------------------
// 3. Fail closed
// ---------------------------------------------------------------------------

static bool sender_has(uint8_t page, uint8_t cmd) {
    uint16_t id = (uint16_t)(page << 8 | cmd);
    return (id >= 0x0110 && id <= 0x0114) || (id >= 0x0120 && id <= 0x0122) || id == 0x0130 ||
           id == 0x0221 || id == 0xE010;
}

static void test_fail_closed(void) {
    int sweeps = 0;
    for (unsigned page = 1; page <= 0xFF; page++) {
        for (unsigned cmd = 0; cmd <= 0xFF; cmd++) {
            if (sender_has((uint8_t)page, (uint8_t)cmd)) continue;
            int before = hook_calls, saves = save_calls;
            xfer_t g = run(SETUP(0xC1, (uint8_t)cmd, 0, 0, 0x02, (uint8_t)page, 64, 0), 0);
            bool g_ok = g.stall_stage == 1 && last_is((uint16_t)(page << 8 | cmd), true, SND_VC_ERR_UNKNOWN, 0);
            xfer_t s = run(SETUP(0x41, (uint8_t)cmd, 0, 0, 0x02, (uint8_t)page, 0, 0), 0);
            bool s_ok = s.stall_stage == 1 && !s.reboot &&
                        last_is((uint16_t)(page << 8 | cmd), false, SND_VC_ERR_UNKNOWN, 0);
            if (!g_ok || !s_ok || hook_calls != before || save_calls != saves) {
                CHECK(0, "page 0x%02X cmd 0x%02X must be UNKNOWN", page, cmd);
                return;
            }
            sweeps++;
        }
    }
    CHECK(sweeps == 255 * 256 - 11, "swept %d absent commands", sweeps);

    // The old hazards by name: the 1.1.6 opcodes on a page, STORE's bulk and
    // presets, upstream's DSP pages.
    xfer_t x = run(SETUP(0x41, 0xF0, 0, 0, 0x02, 0x01, 0, 0), 0);
    CHECK(x.stall_stage == 1 && !x.reboot, "page 1 opcode 0xF0 is not the bootloader");
    x = run(SETUP(0xC1, 0x40, 0, 0, 0x02, 0x02, 64, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0x0240, true, SND_VC_ERR_UNKNOWN, 0), "no STORE.ALL_PARAMS GET");
    x = run(SETUP(0x41, 0x11, 0, 0, 0x02, 0x02, 0, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0x0211, false, SND_VC_ERR_UNKNOWN, 0), "no STORE.PRESET_LOAD");
    x = run(SETUP(0x41, 0x30, 0, 0, 0x02, 0x02, 0, 0), 0);
    CHECK(x.stall_stage == 1 && last_is(0x0230, false, SND_VC_ERR_UNKNOWN, 0), "no STORE.FACTORY_RESET");
    x = run(SETUP(0x41, 0x10, 0x02, 0x00, 0x02, 0xE1, 4, 0), (const uint8_t[4]){ 0 });
    CHECK(x.stall_stage == 1 && last_is(0xE110, false, SND_VC_ERR_UNKNOWN, 2), "page 0xE1");

    // Every paged answer is VC_OK or a VC_ERR_* code: never 0x02 or 0x03.
    for (unsigned page = 1; page <= 0xFF; page++) {
        for (unsigned cmd = 0; cmd <= 0xFF; cmd += 0x11) {
            run(SETUP(0xC1, (uint8_t)cmd, 0, 0, 0x02, (uint8_t)page, 64, 0), 0);
            xfer_t l = last_result();
            if (!(l.ok && (l.data[3] == 0 || (l.data[3] >= 0x10 && l.data[3] <= 0x1A)))) {
                CHECK(0, "page 0x%02X cmd 0x%02X result 0x%02X", page, cmd, l.data[3]);
                return;
            }
        }
    }
}

int main(void) {
    test_legacy();
    test_discovery();
    test_paged_rules();
    test_tunnel();
    test_fail_closed();
    printf("test_vc: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
