/*
 * transport_gate.h — which vendor commands each control transport may run.
 *
 * Pure, dependency-free (config.h for the REQ_* codes, stdint/stdbool), so the
 * table is compiled and unit-tested on the host (firmware/tests/
 * test_transport_gate.c).  vendor_dispatch_get/set (vendor_commands.c) are the
 * only callers; the USB vendor path (TinyUSB EP0) never consults it.
 *
 * Refused on a transport = CTRL_DISPATCH_BLOCKED (CTRL_STATUS_BLOCKED on the
 * wire), never applied:
 *
 *  - Interface self-configuration (0xF5 / 0xF7) and the chunked bulk commands
 *    (0xA2 / 0xA3) on UART, I2C and the Control Surfaces engine: an external
 *    controller can never reconfigure (and lock itself out of) its own link,
 *    and the chunked forms exist only for the WinUSB 4 KB control cap.
 *  - USB MIDI (DSPI_USB_MIDI) rides on USB, so 0xF5 / 0xF7 are allowed there;
 *    0xA2 / 0xA3 are served by midi_control.c's own chunk sessions and never
 *    reach the dispatcher.
 *  - Bootloader entry (0xF0) on every transport except the USB vendor path
 *    (OD-SECh, WP FWSEC1): any MIDI application or Web MIDI page, or anything
 *    on the UART / I2C bus, could otherwise take the device off the bus.
 *    Firmware updates are done from a computer over the USB vendor path.
 *
 * The matching core-voltage rule (EXT_CORE_VREG SET / TRIAL capped at 1.20 V
 * off the USB vendor path) is core_vreg_request_transport_ok() in core_vreg.h.
 */

#ifndef TRANSPORT_GATE_H
#define TRANSPORT_GATE_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef enum {
    CTRL_SOURCE_USB  = 0,
    CTRL_SOURCE_UART = 1,
    CTRL_SOURCE_I2C  = 2,
    CTRL_SOURCE_GPIO = 3,   // internal Control Surfaces engine (main loop); not a
                            // wire transport.  The Control Surfaces tick dispatches
                            // through vendor_dispatch_get/set with this source so its
                            // writes are tagged PARAM_SRC_GPIO.
    CTRL_SOURCE_MIDI = 4,   // USB MIDI SysEx tunnel (DSPI_USB_MIDI builds only;
                            // midi_control.c).  Writes are tagged PARAM_SRC_MIDI.
} CtrlSource;

// USB-only on UART / I2C / GPIO (see the header comment).
static inline bool ctrl_cmd_usb_only(uint8_t bRequest) {
    return bRequest == REQ_SET_UART_CONFIG ||
           bRequest == REQ_SET_I2C_CONFIG ||
           bRequest == REQ_GET_ALL_PARAMS_CHUNK ||
           bRequest == REQ_SET_ALL_PARAMS_CHUNK;
}

// True when `src` may not run `bRequest` (the dispatcher answers BLOCKED).
static inline bool ctrl_cmd_refused(CtrlSource src, uint8_t bRequest) {
    if (src == CTRL_SOURCE_USB) return false;
    if (bRequest == REQ_ENTER_BOOTLOADER) return true;   // FWSEC1 (OD-SECh)
#ifdef DSPI_USB_MIDI
    if (src == CTRL_SOURCE_MIDI &&
        (bRequest == REQ_SET_UART_CONFIG || bRequest == REQ_SET_I2C_CONFIG))
        return false;
#endif
    return ctrl_cmd_usb_only(bRequest);
}

#endif
