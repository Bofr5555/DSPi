#include "identify.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/structs/sysinfo.h"
#include "hardware/sync.h"

#include "config.h"
#include "siggen.h"

// Staged by identify_request() (vendor context), taken by identify_service().
static volatile bool    req_pending;
static volatile uint8_t req_seconds;
static volatile uint8_t req_tone;

// Main-loop state.
static bool     active;
static uint32_t end_ms;
static uint8_t  last_tone = IDENTIFY_TONE_NONE;

bool identify_request(const uint8_t *p) {
    if (p[2] != 0 || p[3] != 0) return false;
    if (p[1] != IDENTIFY_TONE_NONE) {
        if (p[1] >= NUM_OUTPUT_CHANNELS) return false;
        if (siggen_wants_output()) return false;   // never take over a measurement (incl. a START waiting for the DAC)
    }
    req_seconds = p[0] > IDENTIFY_MAX_S ? IDENTIFY_MAX_S : p[0];
    req_tone = p[1];
    __dmb();
    req_pending = true;
    return true;
}

static void start_tone(uint8_t out) {
    // Normal mode (no SIGGEN_FLAG_RAW): the whole output chain applies.
    SiggenConfig c = {
        .version      = SIGGEN_CFG_VERSION,
        .signal_type  = SIGGEN_SINE,
        .channel_mask = (uint16_t)(1u << out),
        .invert_mask  = 0,
        .flags        = 0,
        .level_db     = IDENTIFY_TONE_DB,
        .duration_ms  = IDENTIFY_TONE_MS,
        .p1           = IDENTIFY_TONE_HZ,
    };
    // Re-checked here: a host may have started the generator since staging.
    if (siggen_wants_output()) return;
    if (siggen_stage_config(&c, sizeof(c))) siggen_control(SIGGEN_CTL_START);
}

bool identify_service(uint32_t now_ms) {
    if (req_pending) {
        uint32_t f = save_and_disable_interrupts();
        uint8_t secs = req_seconds, tone = req_tone;
        req_pending = false;
        restore_interrupts(f);

        active = secs != 0;
        end_ms = now_ms + (uint32_t)secs * 1000u;
        if (!active) gpio_put(25, 1);
        if (tone != IDENTIFY_TONE_NONE) {
            last_tone = tone;
            start_tone(tone);
        }
    }
    if (!active) return false;
    if ((int32_t)(now_ms - end_ms) >= 0) {
        active = false;
        gpio_put(25, 1);
        return false;
    }
    gpio_put(25, ((now_ms / IDENTIFY_BLINK_HALF_MS) & 1u) == 0);
    return true;
}

static void copy_padded(char *dst, size_t n, const char *src) {
    memset(dst, 0, n);
    size_t len = strlen(src);
    memcpy(dst, src, len < n ? len : n - 1);
}

void identify_get_status(IdentifyStatus *out) {
    memset(out, 0, sizeof(*out));
    if (active) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        int32_t left = (int32_t)(end_ms - now);
        out->remaining_s = left > 0 ? (uint8_t)((left + 999) / 1000) : 0;
    }
    out->tone_output = last_tone;
#if PICO_RP2350
    out->chip = 1;
    out->chip_rev = rp2350_chip_version();
#else
    out->chip = 0;
    out->chip_rev = rp2040_chip_version();
#endif
    out->chip_id = sysinfo_hw->chip_id;
    copy_padded(out->board, sizeof(out->board), DSPI_BOARD_NAME);
    copy_padded(out->hw_rev, sizeof(out->hw_rev), DSPI_HW_REV);
}
