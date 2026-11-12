#include "reset_info.h"

#include <string.h>

#include "pico/time.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#if PICO_RP2350
#include "hardware/structs/powman.h"
#else
#include "hardware/structs/vreg_and_chip_reset.h"
#endif

#include "core_vreg.h"

#define WDT_OWN_TIMEOUT_BIT  0x100u

static uint32_t chip_reset_at_boot;
static uint32_t watchdog_at_boot;
static uint8_t  reason_at_boot;

static uint8_t decode(uint32_t chip, uint32_t wdt) {
    // WATCHDOG.REASON is zero after a hardware reset, so a set bit means
    // this reset came from the watchdog, whatever CHIP_RESET still shows.
    if (wdt & (WATCHDOG_REASON_TIMER_BITS | WATCHDOG_REASON_FORCE_BITS))
        return (wdt & WDT_OWN_TIMEOUT_BIT) ? RESET_REASON_WATCHDOG : RESET_REASON_SOFTWARE;
#if PICO_RP2350
    if (chip & POWMAN_CHIP_RESET_HAD_BOR_BITS)            return RESET_REASON_BROWNOUT;
    if (chip & POWMAN_CHIP_RESET_HAD_POR_BITS)            return RESET_REASON_POWER_ON;
    if (chip & POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS)        return RESET_REASON_RUN_PIN;
    if (chip & POWMAN_CHIP_RESET_HAD_GLITCH_DETECT_BITS)  return RESET_REASON_GLITCH;
    if (chip & (POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS |
                POWMAN_CHIP_RESET_HAD_HZD_SYS_RESET_REQ_BITS |
                POWMAN_CHIP_RESET_HAD_RESCUE_BITS))       return RESET_REASON_DEBUGGER;
    if (chip & (POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_RSM_BITS |
                POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE_BITS |
                POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_BITS |
                POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_POWMAN_ASYNC_BITS))
        return RESET_REASON_SOFTWARE;
#else
    if (chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_PSM_RESTART_BITS) return RESET_REASON_DEBUGGER;
    if (chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_RUN_BITS)         return RESET_REASON_RUN_PIN;
    if (chip & VREG_AND_CHIP_RESET_CHIP_RESET_HAD_POR_BITS)         return RESET_REASON_POWER_ON;
#endif
    return RESET_REASON_UNKNOWN;
}

void reset_info_capture(void) {
#if PICO_RP2350
    chip_reset_at_boot = powman_hw->chip_reset;
#else
    chip_reset_at_boot = vreg_and_chip_reset_hw->chip_reset;
#endif
    watchdog_at_boot = watchdog_hw->reason |
                       (watchdog_enable_caused_reboot() ? WDT_OWN_TIMEOUT_BIT : 0u);
    reason_at_boot = decode(chip_reset_at_boot, watchdog_at_boot);
}

void reset_info_get(ResetInfo *out) {
    memset(out, 0, sizeof(*out));
    CoreVregStatus st;
    core_vreg_get_status(&st);
    out->reason = reason_at_boot;
    uint32_t prev;
    if (core_vreg_prev_uptime(&prev)) out->flags |= RESET_INFO_FLAG_PREV_UPTIME;
    out->prev_uptime_s = prev;
    out->hang_count = st.hang_count;
#if PICO_RP2350
    out->chip = 1;
#endif
    out->uptime_s = st.uptime_s;
    out->chip_reset = chip_reset_at_boot;
    out->watchdog_reason = watchdog_at_boot;
}
