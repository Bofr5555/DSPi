// WP FWSEC1: host test of the per-transport command table (transport_gate.h)
// and the core-voltage transport rule (core_vreg.h), the two predicates
// vendor_dispatch_get/set and the EXT_CORE_VREG OUT handler call
// (vendor_commands.c).  Built by run.sh with DSPI_USB_MIDI on (the K340
// build); test_transport_gate_nomidi.c builds it again with it off.
#ifndef TG_NO_MIDI
#define DSPI_USB_MIDI 1
#endif

#include <stdio.h>
#include <stdlib.h>
#include "transport_gate.h"
#include "core_vreg.h"

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, msg); fails++; } } while (0)

static const CtrlSource off_usb[] = { CTRL_SOURCE_UART, CTRL_SOURCE_I2C, CTRL_SOURCE_GPIO, CTRL_SOURCE_MIDI };
#define N_OFF ((int)(sizeof(off_usb) / sizeof(off_usb[0])))

// What vendor_dispatch_get/set answer before running a handler; the USB
// vendor path (TinyUSB EP0) never consults the table.
static int blocked(CtrlSource src, uint8_t b) { return ctrl_cmd_refused(src, b); }

// The EXT_CORE_VREG OUT handler's transport check, as vendor_commands.c calls it.
static int vreg_ok(CtrlSource src, uint8_t op, uint8_t v) {
    return core_vreg_request_transport_ok(src == CTRL_SOURCE_USB, op, v);
}

static void test_bootloader(void) {
    CHECK(!blocked(CTRL_SOURCE_USB, REQ_ENTER_BOOTLOADER), "USB 0xF0 unchanged");
    CHECK(blocked(CTRL_SOURCE_MIDI, REQ_ENTER_BOOTLOADER), "MIDI 0xF0 refused");
    CHECK(blocked(CTRL_SOURCE_UART, REQ_ENTER_BOOTLOADER), "UART 0xF0 refused");
    CHECK(blocked(CTRL_SOURCE_I2C, REQ_ENTER_BOOTLOADER), "I2C 0xF0 refused");
    CHECK(blocked(CTRL_SOURCE_GPIO, REQ_ENTER_BOOTLOADER), "GPIO 0xF0 refused");
}

static void test_table_unchanged(void) {
    // Everything that was allowed stays allowed, everywhere, except 0xF0.
    for (int b = 0; b < 256; b++) {
        CHECK(!blocked(CTRL_SOURCE_USB, (uint8_t)b), "USB never blocked");
        for (int i = 0; i < N_OFF; i++) {
            CtrlSource s = off_usb[i];
            int want = (b == REQ_ENTER_BOOTLOADER) || ctrl_cmd_usb_only((uint8_t)b);
#ifdef DSPI_USB_MIDI
            if (s == CTRL_SOURCE_MIDI && (b == REQ_SET_UART_CONFIG || b == REQ_SET_I2C_CONFIG)) want = 0;
#endif
            if (blocked(s, (uint8_t)b) != want) {
                printf("FAIL src %d bReq 0x%02X: blocked %d, want %d\n", (int)s, b, blocked(s, (uint8_t)b), want);
                fails++;
            }
        }
    }
    // Spot checks of the pre-existing rules.
    CHECK(blocked(CTRL_SOURCE_UART, REQ_SET_UART_CONFIG), "UART 0xF5 refused");
    CHECK(blocked(CTRL_SOURCE_I2C, REQ_SET_I2C_CONFIG), "I2C 0xF7 refused");
    CHECK(blocked(CTRL_SOURCE_UART, REQ_SET_ALL_PARAMS_CHUNK), "UART 0xA3 refused");
    CHECK(!blocked(CTRL_SOURCE_UART, REQ_GET_ALL_PARAMS), "UART 0xA0 allowed");
    CHECK(!blocked(CTRL_SOURCE_MIDI, REQ_EXTENSION), "MIDI 0x00 allowed");
#ifdef DSPI_USB_MIDI
    CHECK(!blocked(CTRL_SOURCE_MIDI, REQ_SET_UART_CONFIG), "MIDI 0xF5 allowed (unchanged)");
    CHECK(!blocked(CTRL_SOURCE_MIDI, REQ_SET_I2C_CONFIG), "MIDI 0xF7 allowed (unchanged)");
#else
    CHECK(blocked(CTRL_SOURCE_MIDI, REQ_SET_UART_CONFIG), "no-MIDI build: 0xF5 refused for an unknown source");
#endif
}

static void test_core_vreg(void) {
    const uint8_t V115 = VREG_VOLTAGE_1_15, V120 = VREG_VOLTAGE_1_20,
                  V125 = VREG_VOLTAGE_1_25, V130 = VREG_VOLTAGE_1_30;
    CHECK(CORE_VREG_OFF_USB_CAP == V120, "cap is 1.20 V");
    // USB vendor: the transport rule never refuses (CAP 1.30 V is
    // core_vreg_request_valid's, unchanged).
    for (int op = 0; op < 8; op++)
        for (int v = 0; v < 32; v++)
            CHECK(vreg_ok(CTRL_SOURCE_USB, (uint8_t)op, (uint8_t)v), "USB never refused by the transport rule");
    CHECK(vreg_ok(CTRL_SOURCE_USB, CORE_VREG_OP_SET, V130), "USB SET 1.30 accepted");
    CHECK(vreg_ok(CTRL_SOURCE_USB, CORE_VREG_OP_TRIAL, V130), "USB TRIAL 1.30 accepted");

    for (int i = 0; i < N_OFF; i++) {
        CtrlSource s = off_usb[i];
        CHECK(!vreg_ok(s, CORE_VREG_OP_SET, V125), "SET 1.25 refused off USB");
        CHECK(!vreg_ok(s, CORE_VREG_OP_SET, V130), "SET 1.30 refused off USB");
        CHECK(!vreg_ok(s, CORE_VREG_OP_TRIAL, V125), "TRIAL 1.25 refused off USB");
        CHECK(!vreg_ok(s, CORE_VREG_OP_TRIAL, V130), "TRIAL 1.30 refused off USB");
        CHECK(vreg_ok(s, CORE_VREG_OP_SET, V120), "SET 1.20 accepted off USB");
        CHECK(vreg_ok(s, CORE_VREG_OP_SET, V115), "SET 1.15 passes the transport rule");
        CHECK(vreg_ok(s, CORE_VREG_OP_SET, 0), "SET 0 (build default) accepted off USB");
        CHECK(vreg_ok(s, CORE_VREG_OP_TRIAL, V120), "TRIAL 1.20 accepted off USB");
        CHECK(vreg_ok(s, CORE_VREG_OP_TRIAL, V115), "TRIAL 1.15 accepted off USB");
        for (int v = 0; v < 256; v++) {
            CHECK(vreg_ok(s, CORE_VREG_OP_TRIAL_END, (uint8_t)v), "TRIAL_END accepted everywhere");
            CHECK(vreg_ok(s, CORE_VREG_OP_AUTO, (uint8_t)v), "AUTO passes the transport rule");
            CHECK(vreg_ok(s, CORE_VREG_OP_CLEAR, (uint8_t)v), "CLEAR accepted everywhere");
            if (v > V120) {
                CHECK(!vreg_ok(s, CORE_VREG_OP_SET, (uint8_t)v), "SET above 1.20 refused");
                CHECK(!vreg_ok(s, CORE_VREG_OP_TRIAL, (uint8_t)v), "TRIAL above 1.20 refused");
            }
        }
    }
    // The MIDI cases the plan names.
    CHECK(!vreg_ok(CTRL_SOURCE_MIDI, CORE_VREG_OP_SET, V125), "MIDI SET 1.25 refused");
    CHECK(vreg_ok(CTRL_SOURCE_MIDI, CORE_VREG_OP_SET, V120), "MIDI SET 1.20 accepted");
    CHECK(vreg_ok(CTRL_SOURCE_MIDI, CORE_VREG_OP_AUTO, 1), "MIDI AUTO accepted");
    CHECK(vreg_ok(CTRL_SOURCE_MIDI, CORE_VREG_OP_TRIAL_END, 0), "MIDI TRIAL_END accepted");
    CHECK(vreg_ok(CTRL_SOURCE_MIDI, CORE_VREG_OP_CLEAR, 0), "MIDI CLEAR accepted");
}

int main(void) {
    test_bootloader();
    test_table_unchanged();
    test_core_vreg();
#ifdef DSPI_USB_MIDI
    const char *name = "test_transport_gate";
#else
    const char *name = "test_transport_gate (DSPI_USB_MIDI off)";
#endif
    if (fails) { printf("%s: %d FAILED\n", name, fails); return 1; }
    printf("%s: OK\n", name);
    return 0;
}
