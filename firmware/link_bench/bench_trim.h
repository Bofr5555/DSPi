/*
 * bench_trim.h - link bench: RX sampling trim and the trim scan
 *
 * The link_rx_N programs (LK2) expose public labels hi_entry and lo_emit.
 * A trim of t moves t cycles of delay from the fall exits (every `jmp
 * lo_emit`) to hi_entry, which shifts the sampling window by t sys cycles
 * (3.26 ns at 307.2 MHz).  This is the patch tools/link_rx_test applies; the
 * bench applies it to the program before loading it into instruction
 * memory, so the .pio files stay as they are.
 *
 * The scan tries every trim the program allows (L1 -9..+9, L2 -4..+4, L3
 * -1..+1), records which decode clean and reports the result in the
 * EXT_LINK_DIAG layout (offsets 32-39, extension_channel_spec.md 2.8.11).
 *
 * Portable C11, no SDK.
 */

#ifndef BENCH_TRIM_H
#define BENCH_TRIM_H

#include <stdbool.h>
#include <stdint.h>

#define BENCH_TRIM_LIMIT    9       // the DIAG range, -9..+9

// Patch ins[0..len) (unrelocated: jmp targets relative to the program's
// start) for trim t.  False, with ins unchanged, if a delay field would
// leave 0..31 or t is outside the DIAG range.
bool bench_trim_patch(uint16_t *ins, unsigned len, unsigned hi_entry, unsigned lo_emit, int t);

// Largest t such that both +t and -t patch (L1 9, L2 4, L3 1).
int bench_trim_max(const uint16_t *ins, unsigned len, unsigned hi_entry, unsigned lo_emit);

// Scan state (scan_state values of EXT_LINK_DIAG offset 35).
#define BENCH_SCAN_NONE     0u
#define BENCH_SCAN_RUNNING  1u
#define BENCH_SCAN_DONE     2u
#define BENCH_SCAN_NOCLEAN  3u

typedef struct {
    int8_t   lo, hi, used;  // the clean range and the trim chosen (its centre)
    uint8_t  state;
    uint32_t clean_mask;    // bit i: trim i - 9 was clean
    int8_t   tmax;          // scanning -tmax..+tmax
    int8_t   cur;           // trim under test
    uint32_t errors[2 * BENCH_TRIM_LIMIT + 1];  // bit errors seen at each trim (bench detail)
} bench_scan_t;

void bench_scan_begin(bench_scan_t *s, int tmax);

// The trim to test next; false when the scan is finished.
bool bench_scan_next(const bench_scan_t *s, int *t);

// Record the result of the trim under test and move on.
void bench_scan_record(bench_scan_t *s, bool clean, uint32_t errors);

// Finish: lo/hi = the longest run of clean trims (the one nearer 0 on a
// tie), used = its centre rounded toward 0; state DONE, or NOCLEAN with
// used 0 when nothing was clean.
void bench_scan_finish(bench_scan_t *s);

#endif // BENCH_TRIM_H
