/*
 * bench_cmd.h - alink bench: console parsing and report layouts
 *
 * The bench image is driven over its USB serial console, one command per
 * line (Documentation/Bench/alink_gpio_bench.md lists them, and
 * tools/alink/bench.py speaks them). Replies are single lines starting with
 * a tag (OK, ERR, ID, STAT, DIAG, XSTATUS, SCAN, EVT) followed by key=value
 * tokens, so a script can parse them without knowing their order.
 *
 * The 52-byte EXT_ALINK_STATUS and 64-byte EXT_ALINK_DIAG layouts of
 * extension_channel_spec.md 2.8.2 / 2.8.11 are packed here too, so bench
 * tools written for the DSPi receiver (SW4) can read the bench image's
 * XSTATUS and DIAG replies unchanged.
 *
 * Portable C11, no SDK.
 */

#ifndef BENCH_CMD_H
#define BENCH_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bench_trim.h"
#include "alink_decoder.h"

#define BENCH_MAX_ARGS      16
#define BENCH_MAX_TX_PINS   8

typedef struct {
    int   argc;
    char *argv[BENCH_MAX_ARGS];
} bench_args_t;

// Split line in place at spaces/tabs; returns argc (extra tokens dropped).
int bench_split(char *line, bench_args_t *a);

// Value of a "key=value" token, or NULL.
const char *bench_kv(const bench_args_t *a, const char *key);

bool bench_parse_int(const char *s, long lo, long hi, long *out);
bool bench_parse_class(const char *s, unsigned *cls);      // L1 L2 L3 (or 1 2 3)
bool bench_parse_rate(const char *s, unsigned *rate);      // 44k1 48k 96k (or 44.1 48 96)
bool bench_pin_ok(unsigned pin);                           // header GPIOs: 0-22, 26-28
// "2,3,4": distinct header pins, at most max.
bool bench_parse_pins(const char *s, uint8_t *pins, unsigned *n, unsigned max);

const char *bench_class_name(unsigned cls);
const char *bench_rate_name(unsigned rate);

// Channels a bench stream uses by default: whole banks (8 / 16 / 32 at
// 48 kHz), capped at the capacity of the class and rate.
unsigned bench_default_channels(unsigned cls, unsigned rate);

typedef struct {
    uint8_t  cls, rate, channels, pay, sender;
    uint8_t  pins[BENCH_MAX_TX_PINS];
    uint8_t  npins;
    uint64_t f0;
} bench_txcfg_t;

// Parse "<class> <rate> [ch=C] [pay=prbs|vec] [pins=2,3] [f0=N] [sender=N]"
// from argv[first..].  On failure *err names the problem.
bool bench_parse_tx(const bench_args_t *a, int first, const char *pins_key, bench_txcfg_t *c,
                    const char **err);

// ---------------------------------------------------------------------------
// EXT_ALINK_STATUS (52 B) and EXT_ALINK_DIAG (64 B) layouts
// ---------------------------------------------------------------------------

#define BENCH_STATUS_LEN    52u
#define BENCH_DIAG_LEN      64u

typedef struct {
    uint8_t  state;         // 0 off, 1 searching, 2 locked
    uint8_t  cls, rate, channels, hops, sender, sf_flags;
    uint64_t frame;
    alink_rx_stats_t st;
} bench_status_t;

typedef struct {
    uint8_t  comma_offset;
    uint32_t ring_wr_bytes, parser_rd_bytes, tx_words, rx_xfers;
    const bench_scan_t *scan;   // NULL: no scan
    alink_rx_stats_t st;
} bench_diag_t;

void bench_pack_status(uint8_t out[BENCH_STATUS_LEN], const bench_status_t *s);
void bench_pack_diag(uint8_t out[BENCH_DIAG_LEN], const bench_diag_t *d);

// Lower-case hex of n bytes into out (2n + 1 chars).
void bench_hex(char *out, const uint8_t *b, size_t n);

#endif // BENCH_CMD_H
