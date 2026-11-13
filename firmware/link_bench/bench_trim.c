/*
 * bench_trim.c - link bench: RX sampling trim and the trim scan
 *
 * See bench_trim.h.  bench_trim_patch() is the apply_trim() of
 * tools/link_rx_test/link_pio_model.c; the host tests check that both give
 * the same instruction words.
 */

#include "bench_trim.h"

#include <string.h>

#define INS_DELAY(w)    (((w) >> 8) & 0x1Fu)
#define IS_JMP_ALWAYS(w) (((w) >> 13) == 0u && (((w) >> 5) & 7u) == 0u)

bool bench_trim_patch(uint16_t *ins, unsigned len, unsigned hi_entry, unsigned lo_emit, int t) {
    if (t < -BENCH_TRIM_LIMIT || t > BENCH_TRIM_LIMIT || hi_entry >= len || lo_emit >= len) return false;
    if (!t) return true;
    uint16_t out[32];
    if (len > 32u) return false;
    memcpy(out, ins, len * sizeof(uint16_t));
    int d = (int)INS_DELAY(out[hi_entry]) + t;
    if (d < 0 || d > 31) return false;
    out[hi_entry] = (uint16_t)((out[hi_entry] & ~0x1F00u) | ((unsigned)d << 8));
    for (unsigned i = 0; i < len; i++) {
        uint16_t w = out[i];
        if (IS_JMP_ALWAYS(w) && (w & 0x1Fu) == lo_emit) {
            int e = (int)INS_DELAY(w) - t;
            if (e < 0 || e > 31) return false;
            out[i] = (uint16_t)((w & ~0x1F00u) | ((unsigned)e << 8));
        }
    }
    memcpy(ins, out, len * sizeof(uint16_t));
    return true;
}

int bench_trim_max(const uint16_t *ins, unsigned len, unsigned hi_entry, unsigned lo_emit) {
    int t = 0;
    uint16_t tmp[32];
    if (len > 32u) return 0;
    while (t < BENCH_TRIM_LIMIT) {
        memcpy(tmp, ins, len * sizeof(uint16_t));
        if (!bench_trim_patch(tmp, len, hi_entry, lo_emit, t + 1)) break;
        memcpy(tmp, ins, len * sizeof(uint16_t));
        if (!bench_trim_patch(tmp, len, hi_entry, lo_emit, -(t + 1))) break;
        t++;
    }
    return t;
}

void bench_scan_begin(bench_scan_t *s, int tmax) {
    memset(s, 0, sizeof(*s));
    if (tmax < 0) tmax = 0;
    if (tmax > BENCH_TRIM_LIMIT) tmax = BENCH_TRIM_LIMIT;
    s->tmax = (int8_t)tmax;
    s->cur = (int8_t)-tmax;
    s->state = BENCH_SCAN_RUNNING;
}

bool bench_scan_next(const bench_scan_t *s, int *t) {
    if (s->state != BENCH_SCAN_RUNNING || s->cur > s->tmax) return false;
    *t = s->cur;
    return true;
}

void bench_scan_record(bench_scan_t *s, bool clean, uint32_t errors) {
    if (s->state != BENCH_SCAN_RUNNING || s->cur > s->tmax) return;
    unsigned i = (unsigned)(s->cur + BENCH_TRIM_LIMIT);
    if (clean) s->clean_mask |= 1u << i;
    s->errors[i] = errors;
    s->cur++;
}

void bench_scan_finish(bench_scan_t *s) {
    int best_lo = 0, best_hi = -1, run_lo = 0;
    bool in_run = false;
    for (int t = -BENCH_TRIM_LIMIT; t <= BENCH_TRIM_LIMIT + 1; t++) {
        bool c = t <= BENCH_TRIM_LIMIT && ((s->clean_mask >> (t + BENCH_TRIM_LIMIT)) & 1u);
        if (c && !in_run) { run_lo = t; in_run = true; }
        if (!c && in_run) {
            int run_hi = t - 1, len = run_hi - run_lo, best = best_hi - best_lo;
            int dist = run_lo + run_hi, best_dist = best_lo + best_hi;
            if (dist < 0) dist = -dist;
            if (best_dist < 0) best_dist = -best_dist;
            if (best_hi < best_lo || len > best || (len == best && dist < best_dist)) {
                best_lo = run_lo;
                best_hi = run_hi;
            }
            in_run = false;
        }
    }
    if (best_hi < best_lo) {
        s->lo = s->hi = s->used = 0;
        s->state = BENCH_SCAN_NOCLEAN;
        return;
    }
    s->lo = (int8_t)best_lo;
    s->hi = (int8_t)best_hi;
    s->used = (int8_t)((best_lo + best_hi) / 2);   // C division rounds toward 0
    s->state = BENCH_SCAN_DONE;
}
