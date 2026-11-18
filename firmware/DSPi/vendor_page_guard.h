/*
 * vendor_page_guard.h — refuse paged (firmware 1.1.7+) control requests.
 *
 * Weeb Labs firmware 1.1.7 replaced the 1.1.6 vendor protocol with a paged
 * one: page = high byte of wIndex (0x01-0xFF), command = bRequest
 * (0x10-0xFF).  This firmware is 1.1.6-generation: every one of its commands
 * lives on page 0 (hosts send wIndex 0x0000 or 0x0002, the low byte being the
 * vendor interface).  Before this guard the page byte was ignored, so a paged
 * request ran whichever OLD opcode had the same bRequest number — among them
 * factory reset and preset save / load / delete.
 *
 * Rule (WP PAGEGUARD): any control request whose wIndex high byte is non-zero
 * is refused BEFORE any opcode handling, with no side effect (no bulk lock,
 * no chunk-session change, no DATA stage, no handler):
 *   - USB vendor EP0: STALL at SETUP.
 *   - UART, I2C, USB MIDI: status CTRL_STATUS_ERROR (0x02), payload
 *     consumed and discarded.
 *   - vendor_dispatch_get/set (every internal caller): CTRL_DISPATCH_ERROR.
 *
 * The 1.1.7 probe SYS.PROTOCOL_INFO (page 0x01, bRequest 0x10, GET, 16 B;
 * USB SETUP C1 10 00 00 02 01 10 00) is refused the same way.  That is
 * exactly how the 1.1.7 client update guide says a 1.1.6-or-older device
 * appears ("The probe STALLs, or UART and I2C answer status 0x02"), so a
 * paged client classifies this firmware as legacy and falls back to page 0.
 *
 * Pure and dependency-free so the predicate is host-tested
 * (firmware/tests/test_page_guard.c).
 */

#ifndef VENDOR_PAGE_GUARD_H
#define VENDOR_PAGE_GUARD_H

#include <stdbool.h>
#include <stdint.h>

// The paged protocol's page number: the high byte of wIndex.
#define VENDOR_WINDEX_PAGE(wIndex)  ((uint8_t)(((uint16_t)(wIndex)) >> 8))

// Upstream 1.1.7's mandatory probe, for documentation and tests.
#define VENDOR_PAGED_PROBE_PAGE      0x01u   // SYS
#define VENDOR_PAGED_PROBE_BREQUEST  0x10u   // SYS.PROTOCOL_INFO

// True when the request addresses a page other than 0 and must be refused.
static inline bool vendor_page_refused(uint16_t wIndex) {
    return VENDOR_WINDEX_PAGE(wIndex) != 0;
}

#endif
