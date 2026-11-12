#ifndef IDENTIFY_H
#define IDENTIFY_H

/*
 * identify.h -- "which box is this?" (extension channel EXT_IDENTIFY, 0x0004).
 *
 * OUT blinks the on-board LED (GPIO25; on the K340 the Pico 2 module's own
 * LED, the board has none) in a fast 4 Hz pattern for N seconds, unlike the
 * loop-rate heartbeat, and can play one short quiet tone on a chosen output.
 * IN reports the blink state plus the chip, the board the build targets and
 * a build-time hardware revision string.
 *
 * The tone is off unless asked for (tone_output = IDENTIFY_TONE_NONE).  It is
 * a 1 kHz sine at IDENTIFY_TONE_DB for IDENTIFY_TONE_MS through the test
 * signal generator in normal (not RAW) mode, so the output's crossover, PEQ,
 * gain, master volume, mute and limiter all apply: it is never louder than a
 * -40 dBFS sine of program material on that output.  It replaces the
 * generator's configuration and is refused while the generator is running.
 *
 * See Documentation/Features/extension_channel_spec.md section 2.5.
 */

#include <stdint.h>
#include <stdbool.h>

#define IDENTIFY_MAX_S          120      // longer requests are shortened
#define IDENTIFY_TONE_NONE      0xFF
#define IDENTIFY_TONE_DB        (-40.0f) // peak dBFS before the output chain
#define IDENTIFY_TONE_HZ        1000.0f
#define IDENTIFY_TONE_MS        400
#define IDENTIFY_BLINK_HALF_MS  125      // 4 Hz

#ifndef DSPI_HW_REV
#define DSPI_HW_REV ""
#endif
#ifndef DSPI_BOARD_NAME
#define DSPI_BOARD_NAME ""
#endif

// EXT_IDENTIFY OUT payload (exactly 4 bytes).
typedef struct __attribute__((packed)) {
    uint8_t seconds;        // 0 = stop blinking; > IDENTIFY_MAX_S is shortened
    uint8_t tone_output;    // output index, or IDENTIFY_TONE_NONE
    uint8_t reserved[2];    // zero
} IdentifyRequest;

// EXT_IDENTIFY IN reply (56 bytes, little-endian).
typedef struct __attribute__((packed)) {
    uint8_t  remaining_s;   // blink time left, rounded up; 0 = idle
    uint8_t  tone_output;   // output of the last tone asked for, IDENTIFY_TONE_NONE if none
    uint8_t  chip;          // 0 = RP2040, 1 = RP2350
    uint8_t  chip_rev;      // SDK rp2040_chip_version() / rp2350_chip_version()
    uint32_t chip_id;       // raw SYSINFO CHIP_ID register
    char     board[16];     // PICO_BOARD of the build, NUL-padded
    char     hw_rev[32];    // DSPI_HW_REV build string, NUL-padded; empty = not set
} IdentifyStatus;

_Static_assert(sizeof(IdentifyRequest) == 4, "EXT_IDENTIFY OUT is 4 bytes");
_Static_assert(sizeof(IdentifyStatus) == 56, "EXT_IDENTIFY IN is 56 bytes");

// Vendor context (USB IRQ or a transport poll): validate and stage.  Refuses
// a tone output out of range, nonzero reserved bytes, or a tone while the
// signal generator is running.
bool identify_request(const uint8_t *payload);

void identify_get_status(IdentifyStatus *out);

// Main loop: applies a staged request, starts the tone, drives the LED.
// Returns true while identify owns the LED (the heartbeat must not toggle it).
bool identify_service(uint32_t now_ms);

#endif // IDENTIFY_H
