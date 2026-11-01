#ifndef RESET_INFO_H
#define RESET_INFO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Reset-cause capture for field diagnosis of unexplained reboots.
//
// reset_info_capture() must run first thing in main(), before the watchdog
// is armed or anything else touches the reset/watchdog state.  It latches:
//   - the chip reset-cause register (POWMAN CHIP_RESET on RP2350, the
//     VREG_AND_CHIP_RESET CHIP_RESET on RP2040),
//   - WATCHDOG REASON plus whether our own watchdog_enable() timeout fired,
//   - a boot counter and the previous run's uptime, both kept in WATCHDOG
//     SCRATCH0/1.  The scratch registers survive watchdog and soft resets
//     but are cleared by POR/BOR, so after a brownout the counter restarts
//     at 1 and HAD_BOR in the cause register says why.
// SCRATCH2..7 are left alone: the bootrom and SDK use them.
//
// reset_info_tick() stores the current uptime once per second from the main
// loop, so the next boot can report how long the previous run lasted.
//
// Read back via REQ_GET_STATUS wValue 64..68 (see vendor_commands.c).  Stock
// firmware answers unknown wValues with 0, so boot_count >= 1 marks support.
void reset_info_capture(void);
void reset_info_tick(void);

uint32_t reset_info_chip_reset(void);      // raw cause register at boot
uint32_t reset_info_watchdog(void);        // REASON bits | (own timeout << 8)
uint32_t reset_info_boot_count(void);      // boots since last POR/BOR, 1 = first
uint32_t reset_info_prev_uptime_s(void);   // previous run's last stored uptime
uint32_t reset_info_uptime_s(void);        // this run's uptime

#ifdef __cplusplus
}
#endif

#endif
