// WP PAGEGUARD: host test of the paged-request guard (vendor_page_guard.h).
//
// 1. The predicate, over every wIndex value: refused exactly when the high
//    byte (the firmware-1.1.7 page) is non-zero; page 0 passes with any low
//    byte (hosts send 0 or the vendor interface 2).
// 2. A static check of the call sites that are not host-buildable (USB EP0,
//    UART, I2C need the Pico SDK; the MIDI path is tested end to end in
//    test_midi_control.c): in each entry, the guard comes before the first
//    opcode test or bulk-lock / session operation.  The sources are found
//    next to this file (run.sh compiles it by absolute path).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vendor_page_guard.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d ", __FILE__, __LINE__); \
    printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static void test_predicate(void) {
    for (unsigned w = 0; w <= 0xFFFFu; w++) {
        int want = (w >> 8) != 0;
        if (vendor_page_refused((uint16_t)w) != want) {
            printf("FAIL wIndex 0x%04X: refused %d, want %d\n", w, !want, want);
            fails++;
        }
    }
    CHECK(!vendor_page_refused(0x0000), "page 0, low 0 allowed");
    CHECK(!vendor_page_refused(0x0002), "page 0, vendor interface allowed");
    CHECK(vendor_page_refused((VENDOR_PAGED_PROBE_PAGE << 8) | 2), "SYS.PROTOCOL_INFO probe refused");
    CHECK(vendor_page_refused(0xE002), "private page 0xE0 refused (not ours yet)");
    CHECK(VENDOR_WINDEX_PAGE(0x0A02) == 0x0A, "page extraction");
    CHECK(VENDOR_PAGED_PROBE_BREQUEST == 0x10, "probe command");
}

static char *slurp(const char *rel) {
    char path[1024];
    const char *f = __FILE__;
    const char *slash = strrchr(f, '/');
    size_t dl = slash ? (size_t)(slash - f + 1) : 0;
    snprintf(path, sizeof path, "%.*s%s", (int)dl, f, rel);
    FILE *fp = fopen(path, "rb");
    if (!fp) { printf("FAIL cannot open %s\n", path); fails++; return NULL; }
    fseek(fp, 0, SEEK_END);
    long n = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    char *b = malloc((size_t)n + 1);
    if (!b || fread(b, 1, (size_t)n, fp) != (size_t)n) { fclose(fp); free(b); fails++; return NULL; }
    b[n] = 0;
    fclose(fp);
    return b;
}

// In `file`, starting at `anchor`, the guard call must appear before every
// token in `later` (each of which must exist after the anchor).
static void order(const char *file, const char *anchor, const char *guard,
                  const char *const *later, int nlater) {
    char *src = slurp(file);
    if (!src) return;
    const char *a = strstr(src, anchor);
    CHECK(a, "%s: anchor '%s' not found", file, anchor);
    if (a) {
        const char *g = strstr(a, guard);
        CHECK(g, "%s: guard '%s' missing after '%s'", file, guard, anchor);
        for (int i = 0; g && i < nlater; i++) {
            const char *t = strstr(a, later[i]);
            CHECK(t, "%s: '%s' not found after '%s'", file, later[i], anchor);
            if (t) CHECK(g < t, "%s: guard after '%s'", file, later[i]);
        }
    }
    free(src);
}

static void test_call_sites(void) {
    // USB EP0: before the bulk-lock cleanup, the MS OS 2.0 opcode and the
    // GET / SET paths.
    static const char *const usb[] = {
        "bulk_buf_owner == BULK_OWNER_USB", "MS_VENDOR_CODE)", "vendor_handle_get(req)",
        "vendor_is_bulk_set(req->bRequest", "tud_control_status(rhport",
    };
    order("../DSPi/vendor_commands.c", "if (stage == CONTROL_STAGE_SETUP) {",
          "vendor_page_refused(req->wIndex)", usb, 5);
    // Dispatchers (every external transport and Control Surfaces).
    static const char *const dget[] = { "usb_set_busy()", "ctrl_cmd_refused(src, bRequest)",
                                        "vendor_bulk_try_acquire(src)", "vendor_handle_get(&req)" };
    order("../DSPi/vendor_commands.c", "CtrlDispatchResult vendor_dispatch_get(",
          "vendor_page_refused(wIndex)", dget, 4);
    static const char *const dset[] = { "usb_set_busy()", "ctrl_cmd_refused(src, bRequest)",
                                        "bulk_params_pending = true", "vendor_handle_set_data(&req)" };
    order("../DSPi/vendor_commands.c", "CtrlDispatchResult vendor_dispatch_set(",
          "vendor_page_refused(wIndex)", dset, 4);
    // UART: in header_complete, before the GET shortcut and the bulk acquire.
    static const char *const uart[] = { "if (rx_is_get)", "vendor_bulk_try_acquire(CTRL_SOURCE_UART)" };
    order("../DSPi/uart_control.c", "static void header_complete(void) {",
          "vendor_page_refused(wIndex)", uart, 2);
    // I2C: at header completion, before the bulk acquire.
    static const char *const i2c[] = { "vendor_bulk_try_acquire(CTRL_SOURCE_I2C)" };
    order("../DSPi/i2c_control.c", "if (rx_count == I2C_CTRL_HEADER_LEN) {",
          "vendor_page_refused(", i2c, 1);
    // MIDI: SET header before the chunk / bulk destinations; request before
    // the pending slot is filled (end to end in test_midi_control.c).
    static const char *const mh[] = { "REQ_SET_ALL_PARAMS_CHUNK", "vendor_bulk_try_acquire(CTRL_SOURCE_MIDI)" };
    order("../DSPi/midi_control.c", "case MF_EV_SET_HEADER:", "vendor_page_refused(dec.wIndex)", mh, 2);
    static const char *const mr[] = { "pending.dispatch    = true" };
    order("../DSPi/midi_control.c", "case MF_EV_REQUEST: {", "vendor_page_refused(dec.wIndex)", mr, 1);
    // alink sender (same VID:PID): before MS OS 2.0 and the GET / SET paths.
    static const char *const snd[] = { "MS_VENDOR_CODE)", "handle_get(rhport, req)", "REQ_EXTENSION)" };
    order("../DSPi-Sender/sender_vendor.c", "if (stage == CONTROL_STAGE_SETUP) {",
          "vendor_page_refused(req->wIndex)", snd, 3);
}

int main(void) {
    test_predicate();
    test_call_sites();
    if (fails) { printf("test_page_guard: %d failure(s)\n", fails); return 1; }
    printf("test_page_guard: OK\n");
    return 0;
}
