#include "reset_info.h"

#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "pico/time.h"

#if PICO_RP2350
#include "hardware/structs/powman.h"
#define CHIP_RESET_REG  (powman_hw->chip_reset)
#else
#include "hardware/structs/vreg_and_chip_reset.h"
#define CHIP_RESET_REG  (vreg_and_chip_reset_hw->chip_reset)
#endif

// SCRATCH0 = magic in the top half, boot count in the bottom half.
// SCRATCH1 = uptime in seconds, refreshed by reset_info_tick().
#define RESET_INFO_MAGIC        0xD5B10000u
#define RESET_INFO_MAGIC_MASK   0xFFFF0000u

static uint32_t chip_reset_at_boot;
static uint32_t watchdog_at_boot;
static uint32_t boot_count;
static uint32_t prev_uptime_s;
static uint32_t last_tick_s;

void reset_info_capture(void) {
    chip_reset_at_boot = CHIP_RESET_REG;
    watchdog_at_boot = watchdog_hw->reason |
                       (watchdog_enable_caused_reboot() ? 0x100u : 0u);

    uint32_t s0 = watchdog_hw->scratch[0];
    if ((s0 & RESET_INFO_MAGIC_MASK) == RESET_INFO_MAGIC) {
        boot_count = (s0 & 0xFFFFu) + 1;
        prev_uptime_s = watchdog_hw->scratch[1];
    } else {
        boot_count = 1;
        prev_uptime_s = 0;
    }
    if (boot_count > 0xFFFFu) boot_count = 0xFFFFu;

    watchdog_hw->scratch[0] = RESET_INFO_MAGIC | boot_count;
    watchdog_hw->scratch[1] = 0;
    last_tick_s = 0;
}

void reset_info_tick(void) {
    uint32_t now_s = reset_info_uptime_s();
    if (now_s != last_tick_s) {
        last_tick_s = now_s;
        watchdog_hw->scratch[1] = now_s;
    }
}

uint32_t reset_info_chip_reset(void)    { return chip_reset_at_boot; }
uint32_t reset_info_watchdog(void)      { return watchdog_at_boot; }
uint32_t reset_info_boot_count(void)    { return boot_count; }
uint32_t reset_info_prev_uptime_s(void) { return prev_uptime_s; }
uint32_t reset_info_uptime_s(void)      { return (uint32_t)(time_us_64() / 1000000u); }
