#ifndef RESET_INFO_H
#define RESET_INFO_H

/*
 * reset_info.h -- why the unit last reset, and for how long it has run
 * (extension channel EXT_RESET_INFO, 0x0005).
 *
 * reset_info_capture() latches the reset-cause registers first thing in
 * main(), before watchdog_enable() rewrites the SDK's scratch[4] marker:
 *   RP2040: VREG_AND_CHIP_RESET.CHIP_RESET (HAD_POR covers power-on AND
 *           brown-out: the chip does not tell them apart), WATCHDOG.REASON.
 *   RP2350: POWMAN.CHIP_RESET (separate HAD_POR / HAD_BOR, RUN pin, glitch
 *           detector, debugger), WATCHDOG.REASON.
 * A watchdog reset is split by the SDK's scratch[4] marker: our own
 * watchdog_enable() timeout is a hang (counted by core_vreg as well), any
 * other watchdog reset was asked for (watchdog_reboot(): a host-requested
 * reboot, the BOOTSEL round trip, the bootrom starting the app after a UF2
 * copy).  The previous run's uptime and the hang count come from core_vreg,
 * which owns WATCHDOG SCRATCH0/1.
 *
 * See Documentation/Features/extension_channel_spec.md section 2.6.
 */

#include <stdint.h>
#include <stdbool.h>

enum {
    RESET_REASON_UNKNOWN  = 0,  // no cause bit the decoder knows
    RESET_REASON_POWER_ON = 1,  // power-on (RP2040: or brown-out)
    RESET_REASON_BROWNOUT = 2,  // brown-out detector (RP2350 only)
    RESET_REASON_RUN_PIN  = 3,  // RUN pin pulled low
    RESET_REASON_WATCHDOG = 4,  // our 8 s watchdog fired: a hang
    RESET_REASON_SOFTWARE = 5,  // a requested reboot (watchdog_reboot, bootrom)
    RESET_REASON_DEBUGGER = 6,  // debug port / rescue reset
    RESET_REASON_GLITCH   = 7,  // RP2350 glitch detector
};

#define RESET_INFO_FLAG_PREV_UPTIME   0x01  // prev_uptime_s is valid (warm reset)

// EXT_RESET_INFO IN reply (20 bytes, little-endian).
typedef struct __attribute__((packed)) {
    uint8_t  reason;           // RESET_REASON_*
    uint8_t  flags;            // RESET_INFO_FLAG_*
    uint8_t  hang_count;       // CoreVregStatus.hang_count
    uint8_t  chip;             // 0 = RP2040, 1 = RP2350
    uint32_t uptime_s;         // this run
    uint32_t prev_uptime_s;    // the run before this reset (1 s steps); 0 if not valid
    uint32_t chip_reset;       // raw CHIP_RESET register at boot (see above)
    uint32_t watchdog_reason;  // raw WATCHDOG.REASON at boot; bit 8 = own-timeout marker
} ResetInfo;

_Static_assert(sizeof(ResetInfo) == 20, "EXT_RESET_INFO IN is 20 bytes");

// First thing in main().
void reset_info_capture(void);

void reset_info_get(ResetInfo *out);

#endif // RESET_INFO_H
