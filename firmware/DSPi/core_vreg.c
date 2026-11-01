#include "core_vreg.h"

#include "pico/time.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"

#include "flash_storage.h"

// Build default from the DSPI_VREG_MV cache variable (CMakeLists.txt).
#if !defined(DSPI_VREG_MV) || DSPI_VREG_MV == 1150
#define CORE_VREG_DEFAULT ((uint8_t)VREG_VOLTAGE_1_15)
#elif DSPI_VREG_MV == 1200
#define CORE_VREG_DEFAULT ((uint8_t)VREG_VOLTAGE_1_20)
#elif DSPI_VREG_MV == 1250
#define CORE_VREG_DEFAULT ((uint8_t)VREG_VOLTAGE_1_25)
#elif DSPI_VREG_MV == 1300
#define CORE_VREG_DEFAULT ((uint8_t)VREG_VOLTAGE_1_30)
#else
#error "DSPI_VREG_MV must be 1150, 1200, 1250 or 1300"
#endif

// SCRATCH0: [31:16] magic, [3:0] hang count, [8:4] trial vreg (0 = none),
//           [13:9] failed-trial vreg (0 = none).  Bits 14,15 spare.
// SCRATCH1: uptime in seconds, refreshed by core_vreg_tick(); read back on the
//           next boot as "how long did the previous run last".
#define SCRATCH_BOOKKEEPING     (watchdog_hw->scratch[0])
#define SCRATCH_UPTIME          (watchdog_hw->scratch[1])
#define SCR_MAGIC               0xC0DE0000u
#define SCR_MAGIC_MASK          0xFFFF0000u
#define SCR_HANGS(v)            ((v) & 0xFu)
#define SCR_TRIAL(v)            (((v) >> 4) & 0x1Fu)
#define SCR_FAILED(v)           (((v) >> 9) & 0x1Fu)
#define SCR_PACK(h, t, f)       (SCR_MAGIC | ((uint32_t)(h) & 0xFu) | \
                                 (((uint32_t)(t) & 0x1Fu) << 4) | (((uint32_t)(f) & 0x1Fu) << 9))

static uint8_t  stored_sel;          // directory byte: 0 = default
static uint8_t  stored_flags;        // directory flags
static uint8_t  boot_vreg;           // what core_vreg_boot_apply() programmed
static uint8_t  hang_count;
static uint8_t  trial_vreg;          // 0 = no trial running
static uint8_t  failed_trial_vreg;   // 0 = none
static uint32_t failed_trial_uptime_s;
static bool     escalation_unpersisted;
static uint32_t last_tick_s;

volatile bool    core_vreg_req_pending = false;
volatile uint8_t core_vreg_req_op = 0;
volatile uint8_t core_vreg_req_value = 0;

static inline bool vreg_in_range(uint8_t v) {
    return v >= CORE_VREG_FLOOR && v <= CORE_VREG_CAP;
}

// The build default is also a floor for everything persisted: a board whose
// build asks for 1.20 V (K340: -DDSPI_VREG_MV=1200) must never boot lower,
// even on a directory carrying an explicit 1.15 V from an earlier build.  Only
// a RAM-only trial may go below it, and a reboot always ends a trial.
static uint8_t resolve(uint8_t sel) {
    uint8_t v = sel ? sel : CORE_VREG_DEFAULT;
    if (v < CORE_VREG_DEFAULT) v = CORE_VREG_DEFAULT;
    if (v < CORE_VREG_FLOOR) v = CORE_VREG_FLOOR;
    if (v > CORE_VREG_CAP)   v = CORE_VREG_CAP;      // no POWMAN unlock, ever
    return v;
}

static inline uint32_t uptime_s(void) {
    return (uint32_t)(time_us_64() / 1000000u);
}

static void scratch_store(void) {
    SCRATCH_BOOKKEEPING = SCR_PACK(hang_count, trial_vreg, failed_trial_vreg);
}

// The SDK's vreg_set_voltage() clamps anything above VREG_VOLTAGE_MAX while
// the POWMAN limit is in force; resolve() already keeps us at or below it.
static void program(uint8_t v) {
    vreg_set_voltage((enum vreg_voltage)v);
    busy_wait_ms(10);
}

void core_vreg_boot_apply(void) {
    preset_get_core_vreg(&stored_sel, &stored_flags);

    // Reconstruct the previous run's bookkeeping.  Power-on clears scratch,
    // so after a brown-out or power cycle everything starts at zero and a
    // reset cause of HAD_BOR/HAD_POR is the story, not a hang.
    uint32_t scr = SCRATCH_BOOKKEEPING;
    bool have_history = (scr & SCR_MAGIC_MASK) == SCR_MAGIC;
    uint32_t prev_uptime = have_history ? SCRATCH_UPTIME : 0;
    uint8_t prev_trial = have_history ? (uint8_t)SCR_TRIAL(scr) : 0;
    hang_count        = have_history ? (uint8_t)SCR_HANGS(scr) : 0;
    failed_trial_vreg = have_history ? (uint8_t)SCR_FAILED(scr) : 0;
    trial_vreg = 0;

    // Only resets from OUR watchdog_enable() timeout count as hangs: the SDK
    // marks those with its scratch[4] magic, so reboots requested through
    // watchdog_reboot() (USB reset, BOOTSEL round trips) don't.
    if (have_history && watchdog_enable_caused_reboot()) {
        if (prev_trial) {
            failed_trial_vreg = prev_trial;
            failed_trial_uptime_s = prev_uptime;
            // A trial that hung says nothing about the persisted voltage.
        } else if (hang_count < 0xF) {
            hang_count++;
        }
    }

    uint8_t v = resolve(stored_sel);

    bool auto_ok = !(stored_flags & CORE_VREG_FLAG_AUTO_DISABLED);
    if (auto_ok && hang_count >= CORE_VREG_HANGS_TO_ESCALATE && v < CORE_VREG_AUTO_CAP) {
        // Step up one notch.  Applied from RAM now; written to flash from the
        // main loop (core_vreg_persist_due), which also clears hang_count.
        // Until that write lands, every further boot recomputes the same +1
        // from the still-stored selection, so a hang between here and the
        // write cannot ratchet twice.
        v++;
        stored_sel = v;
        stored_flags |= CORE_VREG_FLAG_AUTO_RAISED;
        escalation_unpersisted = true;
    }

    boot_vreg = v;
    program(v);

    scratch_store();
    SCRATCH_UPTIME = 0;
    last_tick_s = 0;
}

void core_vreg_tick(void) {
    uint32_t now = uptime_s();
    if (now != last_tick_s) {
        last_tick_s = now;
        SCRATCH_UPTIME = now;
    }
}

bool core_vreg_request_valid(uint8_t op, uint8_t value) {
    switch (op) {
        case CORE_VREG_OP_SET:       return value == 0 || (vreg_in_range(value) && value >= CORE_VREG_DEFAULT);
        case CORE_VREG_OP_TRIAL:     return vreg_in_range(value);
        case CORE_VREG_OP_TRIAL_END: return true;
        case CORE_VREG_OP_AUTO:      return value <= 1;
        case CORE_VREG_OP_CLEAR:     return true;
        default:                     return false;
    }
}

bool core_vreg_apply_request(uint8_t op, uint8_t value) {
    if (!core_vreg_request_valid(op, value)) return false;
    switch (op) {
        case CORE_VREG_OP_SET:
            // An explicit choice supersedes anything the firmware learned.
            stored_sel = value;
            stored_flags &= (uint8_t)~CORE_VREG_FLAG_AUTO_RAISED;
            hang_count = 0;
            trial_vreg = 0;
            boot_vreg = resolve(stored_sel);
            program(boot_vreg);
            scratch_store();
            return true;

        case CORE_VREG_OP_TRIAL:
            // Record the trial BEFORE lowering the voltage: if the core hangs
            // on the very next instruction the next boot must still know.
            trial_vreg = value;
            scratch_store();
            program(value);
            return false;

        case CORE_VREG_OP_TRIAL_END:
            trial_vreg = 0;
            scratch_store();
            program(boot_vreg);
            return false;

        case CORE_VREG_OP_AUTO:
            if (value) stored_flags &= (uint8_t)~CORE_VREG_FLAG_AUTO_DISABLED;
            else       stored_flags |= CORE_VREG_FLAG_AUTO_DISABLED;
            return true;

        case CORE_VREG_OP_CLEAR:
            hang_count = 0;
            failed_trial_vreg = 0;
            failed_trial_uptime_s = 0;
            scratch_store();
            return false;
    }
    return false;
}

bool core_vreg_persist_due(void) {
    return escalation_unpersisted;
}

void core_vreg_persist(void) {
    preset_set_core_vreg(stored_sel, stored_flags);
    if (escalation_unpersisted) {
        escalation_unpersisted = false;
        hang_count = 0;          // the hangs have been acted on
        scratch_store();
    }
}

void core_vreg_get_status(CoreVregStatus *out) {
    out->live_vreg             = (uint8_t)vreg_get_voltage();
    out->stored_sel            = stored_sel;
    out->default_vreg          = CORE_VREG_DEFAULT;
    out->flags                 = stored_flags;
    out->hang_count            = hang_count;
    out->trial_active          = trial_vreg ? 1 : 0;
    out->trial_vreg            = trial_vreg;
    out->failed_trial_vreg     = failed_trial_vreg;
    out->failed_trial_uptime_s = failed_trial_uptime_s;
    out->uptime_s              = uptime_s();
}
