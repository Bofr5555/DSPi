/*
 * core_vreg.h — core voltage selection for the fixed 307.2 MHz system clock.
 *
 * Background: 307.2 MHz is roughly twice the RP2350's rated 150 MHz, and a
 * minority of parts hang at the stock 1.15 V (upstream issue #83; the K340
 * bench unit: ~0.5 hangs/min under USB streaming at 1.15 V, none in 41 min at
 * 1.20 V).  Every observed failure is a hang caught by the 8 s watchdog, never
 * a brown-out, so the firmware can tell its own watchdog resets apart from
 * power events and learn from them.
 *
 * Three layers, all RP2040/RP2350 neutral (same vreg enum):
 *
 *  1. Build default: DSPI_VREG_MV (CMake cache, 1150 by default).  It is
 *     also the floor for the persisted selection: a stored value below it
 *     boots at the build default and REQ_SET_CORE_VREG refuses (STALLs) one.
 *  2. Persisted per-device selection (flash directory; 0 = build default),
 *     set by the host with REQ_SET_CORE_VREG, or raised automatically: two
 *     watchdog hangs within one power session step the voltage up one notch
 *     (+50 mV), up to CORE_VREG_AUTO_CAP, and the step is persisted with the
 *     AUTO_RAISED flag so the host can see the device adapted itself.
 *  3. Trial voltage (RAM only): the host asks for a voltage to be applied
 *     without persisting it.  If the device hangs, the next boot comes up on
 *     the persisted voltage again and reports which trial voltage failed and
 *     how long it survived, so a host-side calibration loop can walk the
 *     voltage down to find the margin without ever being able to brick the
 *     unit.
 *
 * Hard limits, not overridable from the host: never below CORE_VREG_FLOOR
 * (1.15 V, the stock value) and never above CORE_VREG_CAP (1.30 V, the
 * regulator's locked ceiling: this module never touches the POWMAN voltage
 * limit).  Mind that the RP2350 datasheet's recommended DVDD range tops out at
 * 1.16 V and the absolute maximum is 1.21 V, with a +/-3 % regulator
 * tolerance; 1.20 V is one step outside the recommended range and 1.25 V and
 * above exceed the absolute maximum at the regulator's upper tolerance.
 *
 * State across a watchdog reset lives in WATCHDOG SCRATCH0 (bookkeeping) and
 * SCRATCH1 (uptime seconds).  Both clear on power-on reset and survive warm
 * resets.  SCRATCH2..3 (bootrom) and 4..7 (SDK reboot vectors) are untouched.
 *
 * Voltage-only changes at a fixed PLL are glitch-free (the SDK waits for
 * UPDATE_IN_PROGRESS), so no audio teardown is needed for an apply; only the
 * directory write uses the usual flash bracket, which main.c owns.
 */

#ifndef CORE_VREG_H
#define CORE_VREG_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/vreg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CORE_VREG_FLOOR     ((uint8_t)VREG_VOLTAGE_1_15)   // stock; never lower
#define CORE_VREG_CAP       ((uint8_t)VREG_VOLTAGE_1_30)   // regulator lock; never higher
#define CORE_VREG_AUTO_CAP  ((uint8_t)VREG_VOLTAGE_1_25)   // automatic escalation stops here
#define CORE_VREG_HANGS_TO_ESCALATE  2                     // own-watchdog hangs per power session

// Persisted flags byte (flash directory).
#define CORE_VREG_FLAG_AUTO_RAISED    0x01   // the stored selection was raised by the firmware
#define CORE_VREG_FLAG_AUTO_DISABLED  0x02   // user opted out of automatic escalation

// REQ_SET_CORE_VREG payload: {op, value}.
enum {
    CORE_VREG_OP_SET        = 0,  // value: 0 = build default, else vreg enum in [build default, CAP]; persisted
    CORE_VREG_OP_TRIAL      = 1,  // value: vreg enum in [FLOOR, CAP]; RAM only, until TRIAL_END or a hang
    CORE_VREG_OP_TRIAL_END  = 2,  // value ignored; back to the persisted voltage
    CORE_VREG_OP_AUTO       = 3,  // value: 1 = enable automatic escalation (default), 0 = disable
    CORE_VREG_OP_CLEAR      = 4,  // value ignored; forget hang count and last trial failure
};

// REQ_GET_CORE_VREG response (16 bytes, little-endian words).
typedef struct __attribute__((packed)) {
    uint8_t  live_vreg;          // vreg enum currently programmed
    uint8_t  stored_sel;         // persisted selection, 0 = build default
    uint8_t  default_vreg;       // build default (DSPI_VREG_MV)
    uint8_t  flags;              // CORE_VREG_FLAG_*
    uint8_t  hang_count;         // own-watchdog hangs this power session (not yet acted on)
    uint8_t  trial_active;       // 1 while a trial voltage is applied
    uint8_t  trial_vreg;         // the trial voltage (valid when trial_active)
    uint8_t  failed_trial_vreg;  // 0 = none; else the trial voltage whose run ended in a hang
    uint32_t failed_trial_uptime_s;  // how long that trial survived
    uint32_t uptime_s;           // this run's uptime
} CoreVregStatus;

// Boot: decode the scratch bookkeeping, pick the voltage (persisted or
// escalated), program the regulator.  Call in core0_init() where the fixed
// vreg_set_voltage() used to be, before the PLL is raised.
void core_vreg_boot_apply(void);

// Main loop, every iteration: refreshes the uptime scratch once a second.
void core_vreg_tick(void);

// Vendor staging (IRQ/transport context -> main loop).  Validate with
// core_vreg_request_valid() before staging.
bool core_vreg_request_valid(uint8_t op, uint8_t value);
extern volatile bool    core_vreg_req_pending;
extern volatile uint8_t core_vreg_req_op;
extern volatile uint8_t core_vreg_req_value;

// Main loop: performs a staged request.  Returns true when the directory must
// be written; the caller then brackets core_vreg_persist() with the flash
// mute/teardown helpers.
bool core_vreg_apply_request(uint8_t op, uint8_t value);

// True when a boot-time automatic escalation is still waiting to be written
// to flash; the caller brackets core_vreg_persist() the same way.
bool core_vreg_persist_due(void);
void core_vreg_persist(void);

void core_vreg_get_status(CoreVregStatus *out);

#ifdef __cplusplus
}
#endif

#endif
