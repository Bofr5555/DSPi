/*
 * bench_decode.h - alink bench: cycles per frame of LK5's receiver (WP LK5)
 *
 * The `decode` console command (bench_decode.c): runs alink_fast over a canned
 * stream in RAM or the live RX ring and reports DWT cycles per frame.
 */

#ifndef BENCH_DECODE_H
#define BENCH_DECODE_H

#include <stdint.h>

#include "bench_cmd.h"

typedef struct {
    const volatile uint32_t *ring;  // the RX ring, NULL when the rx role is not running
    uint32_t ring_words;
    unsigned cls;
    uint64_t (*writer_bits)(void);  // bits the DMA has written (absolute)
} bench_decode_live_t;

void bench_decode_cmd(const bench_args_t *a, const bench_decode_live_t *live, uint32_t sys_hz);

#endif // BENCH_DECODE_H
