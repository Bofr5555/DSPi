/*
 * link_pio_model.c; cycle-accurate model of the DSPi audio link PIO programs
 * (firmware/DSPi/link_tx_N.pio, link_rx_N.pio; SYS2 2.6/2.7, WP LK2).
 *
 * Unlike tools/adat_rx_test, which transcribes its program into a switch,
 * this model runs the pioasm output itself: a small PIO interpreter executes
 * the assembled instruction words (jmp/in/out/mov/set, delays, wrap,
 * autopush/autopull) one sys cycle at a time, so the .pio files are the only
 * source of truth. run.sh assembles them and builds this file.
 *
 * Signal chain per case:
 *   8b/10b frames (link_8b10b_shim.h: K28.5 + PRBS bytes, plus idle and
 *   all-zero frames) -> link_tx_N on the sender's clock (307.2 MHz scaled by
 *   the clock offset) -> edge list -> channel (pulse-width distortion: rising
 *   edges -PWD/2, falling +PWD/2, so positive PWD widens high pulses; per-edge
 *   jitter) -> link_rx_N on the receiver's 307.2 MHz clock, behind the GPIO's
 *   2-cycle input synchroniser, at a chosen sub-cycle start phase.
 * Oracle: the received bit stream is aligned to the sent one and compared;
 * any divergence is an error event (in this architecture a timing violation
 * is a dropped or duplicated bit, i.e. a slip); and every K28.5 must sit at
 * the frame stride, which is what the firmware's parser relies on.
 *
 * cc -O2 -I<pioasm output dir> -DPICO_NO_HARDWARE=1 -o link_pio_model link_pio_model.c -lm
 *   ./link_pio_model                 regression matrix, exit 1 on any failure
 *   ./link_pio_model sweep           margin tables (offset, PWD, jitter, trim)
 *   ./link_pio_model case N ppm pwd_ns jitter_ns [trim [phase [jmodel]]]
 * jmodel: 0 corner (each edge +-J/2, random sign: hits every worst-case pair),
 * 1 uniform in [-J/2, J/2], 2 gaussian with sigma = J.
 * trim: cycles of entry delay moved from the fall exits to hi_entry (the
 * PWD centring the RX headers describe); phase: RX start, in sys cycles.
 * LINK_MODEL_DUMP=1 prints the first sent and received bits of each case.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link_8b10b_shim.h"
#include "link_rx_10.pio.h"
#include "link_rx_20.pio.h"
#include "link_rx_5.pio.h"
#include "link_tx_10.pio.h"
#include "link_tx_20.pio.h"
#include "link_tx_5.pio.h"

#define SYS_HZ      307200000.0
#define T_SYS_NS    (1e9 / SYS_HZ)
#define SYNC_CYCLES 2           // GPIO input synchroniser (kept enabled)
#define FS_HZ       48000u

// ============================================================================
// PIO INTERPRETER (the subset the link programs use; anything else aborts)
// ============================================================================

typedef struct {
    uint16_t prog[32];
    uint32_t len, wrap_target, wrap;
    uint32_t pc, x, y, isr, osr;
    uint32_t isr_cnt, osr_cnt;      // shift counts, as the hardware keeps them
    int pin_out;                    // last OUT/SET/MOV pins value (TX)
    // I/O hooks
    int (*pin_in)(void *ctx);       // GPIO level seen by this cycle
    bool (*pull)(void *ctx, uint32_t *w);
    void (*push)(void *ctx, uint32_t w);
    void *ctx;
} Sm;

static void die(const char *m, uint32_t v) {
    fprintf(stderr, "model error: %s (0x%x)\n", m, v);
    exit(2);
}

static void sm_load(Sm *s, const uint16_t *ins, uint32_t len, uint32_t wt, uint32_t wr) {
    memset(s, 0, sizeof(*s));
    if (len > 32) die("program too long", len);
    memcpy(s->prog, ins, len * sizeof(uint16_t));
    s->len = len;
    s->wrap_target = wt;
    s->wrap = wr;
    s->osr_cnt = 32;   // pio_sm_init: OSR empty, so the first OUT pulls
}

static uint32_t sm_src(Sm *s, uint32_t src) {
    switch (src) {
    case 0: return (uint32_t)s->pin_in(s->ctx);
    case 1: return s->x;
    case 2: return s->y;
    case 3: return 0;
    case 6: return s->isr;
    case 7: return s->osr;
    default: die("unsupported source", src);
    }
    return 0;
}

// Executes the instruction at pc; returns the cycles it occupies (1 + delay).
// IN shifts left and OUT shifts left (MSB first), threshold 32, as the link
// programs are configured. Returns 0 when a pull finds no data (TX done).
static uint32_t sm_step(Sm *s) {
    uint16_t ins = s->prog[s->pc];
    uint32_t op = ins >> 13, delay = (ins >> 8) & 0x1Fu;
    uint32_t a = (ins >> 5) & 7u, b = ins & 0x1Fu;
    bool jumped = false;
    switch (op) {
    case 0: {   // JMP
        bool take;
        switch (a) {
        case 0: take = true; break;
        case 1: take = s->x == 0; break;
        case 2: take = s->x != 0; s->x--; break;
        case 3: take = s->y == 0; break;
        case 4: take = s->y != 0; s->y--; break;
        case 5: take = s->x != s->y; break;
        case 6: take = s->pin_in(s->ctx) != 0; break;
        default: take = s->osr_cnt < 32; break;
        }
        if (take) { s->pc = b; jumped = true; }
        break;
    }
    case 2: {   // IN
        uint32_t n = b ? b : 32, d = sm_src(s, a);
        s->isr = (n == 32) ? d : ((s->isr << n) | (d & ((1u << n) - 1u)));
        s->isr_cnt += n;
        if (s->isr_cnt >= 32) { s->push(s->ctx, s->isr); s->isr = 0; s->isr_cnt = 0; }
        break;
    }
    case 3: {   // OUT
        uint32_t n = b ? b : 32;
        if (s->osr_cnt >= 32) {
            if (!s->pull(s->ctx, &s->osr)) return 0;
            s->osr_cnt = 0;
        }
        uint32_t d = (n == 32) ? s->osr : (s->osr >> (32 - n));
        s->osr = (n == 32) ? 0 : (s->osr << n);
        s->osr_cnt += n;
        switch (a) {
        case 0: s->pin_out = (int)(d & 1u); break;
        case 1: s->x = d; break;
        case 2: s->y = d; break;
        case 3: break;
        default: die("unsupported OUT destination", a);
        }
        break;
    }
    case 5: {   // MOV (nop = mov y, y)
        uint32_t src = ins & 7u, mop = (ins >> 3) & 3u, d = sm_src(s, src);
        if (mop == 1) d = ~d;
        else if (mop == 2) { uint32_t r = 0; for (int i = 0; i < 32; i++) r |= ((d >> i) & 1u) << (31 - i); d = r; }
        switch (a) {
        case 0: s->pin_out = (int)(d & 1u); break;
        case 1: s->x = d; break;
        case 2: s->y = d; break;
        case 5: s->pc = d & 0x1Fu; jumped = true; break;
        case 6: s->isr = d; s->isr_cnt = 0; break;
        case 7: s->osr = d; s->osr_cnt = 0; break;
        default: die("unsupported MOV destination", a);
        }
        break;
    }
    case 7:     // SET
        switch (a) {
        case 0: s->pin_out = (int)(b & 1u); break;
        case 1: s->x = b; break;
        case 2: s->y = b; break;
        case 4: break;   // pindirs
        default: die("unsupported SET destination", a);
        }
        break;
    default:
        die("unsupported instruction", ins);
    }
    if (!jumped) s->pc = (s->pc == s->wrap) ? s->wrap_target : s->pc + 1;
    return 1 + delay;
}

// ============================================================================
// PROGRAM TABLE
// ============================================================================

typedef struct {
    uint32_t n;                         // sys cycles per bit
    const char *cls;
    const uint16_t *rx; uint32_t rx_len, rx_wt, rx_wr;
    int rx_hi_entry, rx_lo_emit;        // trim anchors (public labels)
    const uint16_t *tx; uint32_t tx_len, tx_wt, tx_wr;
} Prog;

#define RXLEN(p) (uint32_t)(sizeof(p##_program_instructions) / sizeof(uint16_t))
static const Prog progs[3] = {
    { 20, "L1", link_rx_20_program_instructions, RXLEN(link_rx_20), link_rx_20_wrap_target, link_rx_20_wrap,
      link_rx_20_offset_hi_entry, link_rx_20_offset_lo_emit,
      link_tx_20_program_instructions, RXLEN(link_tx_20), link_tx_20_wrap_target, link_tx_20_wrap },
    { 10, "L2", link_rx_10_program_instructions, RXLEN(link_rx_10), link_rx_10_wrap_target, link_rx_10_wrap,
      link_rx_10_offset_hi_entry, link_rx_10_offset_lo_emit,
      link_tx_10_program_instructions, RXLEN(link_tx_10), link_tx_10_wrap_target, link_tx_10_wrap },
    { 5, "L3", link_rx_5_program_instructions, RXLEN(link_rx_5), link_rx_5_wrap_target, link_rx_5_wrap,
      link_rx_5_offset_hi_entry, link_rx_5_offset_lo_emit,
      link_tx_5_program_instructions, RXLEN(link_tx_5), link_tx_5_wrap_target, link_tx_5_wrap },
};

static const Prog *prog_for(uint32_t n) {
    for (int i = 0; i < 3; i++) if (progs[i].n == n) return &progs[i];
    die("no program for N", n);
    return NULL;
}

// Moves `trim` cycles of delay from the fall exits to hi_entry: positive trim
// emits later in high runs and earlier in low runs, centring the window on a
// positive PWD. Returns false if a delay field would leave 0..31.
static bool apply_trim(Sm *s, const Prog *p, int trim) {
    if (!trim) return true;
    uint16_t *h = &s->prog[p->rx_hi_entry];
    int d = (int)((*h >> 8) & 0x1F) + trim;
    if (d < 0 || d > 31) return false;
    *h = (uint16_t)((*h & ~0x1F00u) | ((uint32_t)d << 8));
    for (uint32_t i = 0; i < s->len; i++) {
        uint16_t w = s->prog[i];
        if ((w >> 13) == 0 && ((w >> 5) & 7u) == 0 && (int)(w & 0x1Fu) == p->rx_lo_emit) {
            int e = (int)((w >> 8) & 0x1F) - trim;
            if (e < 0 || e > 31) return false;
            s->prog[i] = (uint16_t)((w & ~0x1F00u) | ((uint32_t)e << 8));
        }
    }
    return true;
}

// ============================================================================
// STREAM GENERATION (TX side)
// ============================================================================

static uint32_t rng_state = 1;
static uint32_t rng(void) {
    uint32_t x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return rng_state = x;
}
static double rng_unit(void) { return (rng() >> 8) * (1.0 / 16777216.0); }
static double rng_gauss(void) {
    double u = rng_unit() + 1e-12, v = rng_unit();
    return sqrt(-2.0 * log(u)) * cos(6.283185307179586 * v);
}

typedef struct {
    uint8_t *bits; size_t n;       // line bits, in wire order
    uint32_t *words; size_t nw, rd; // MSB-first packing, TX pull cursor
    uint32_t frame_bits;
} Stream;

// Frames of 10 x symbols bits: K28.5 then data bytes. Every 8th frame is
// idle (D21.5), every 8th+4 all zero bytes (D0.0, the longest data runs), the
// rest PRBS: every code group and every run length appears.
static void stream_make(Stream *st, uint32_t n_bits_min, uint32_t symbols, uint32_t seed) {
    st->frame_bits = symbols * 10u;
    uint32_t frames = (n_bits_min + st->frame_bits - 1) / st->frame_bits;
    st->n = (size_t)frames * st->frame_bits;
    st->bits = malloc(st->n);
    st->nw = (st->n + 31) / 32;
    st->words = calloc(st->nw, 4);
    if (!st->bits || !st->words) die("out of memory", 0);
    rng_state = seed ? seed : 1;
    bool rd = false;
    size_t k = 0;
    for (uint32_t f = 0; f < frames; f++) {
        for (uint32_t sym = 0; sym < symbols; sym++) {
            uint8_t byte = (f & 7) == 0 ? 0xB5 : (f & 7) == 4 ? 0x00 : (uint8_t)rng();
            uint32_t cg = link_shim_encode(byte, sym == 0, &rd);
            for (int i = 9; i >= 0; i--) st->bits[k++] = (cg >> i) & 1u;
        }
    }
    for (size_t i = 0; i < st->n; i++)
        if (st->bits[i]) st->words[i >> 5] |= 0x80000000u >> (i & 31);
    st->rd = 0;
}

static void stream_free(Stream *st) { free(st->bits); free(st->words); }

// Encoder sanity (so the shim cannot hide a bad stream): run length <= 5,
// digital sum variation <= 6, K28.5 only at frame starts.
static bool stream_check(const Stream *st) {
    int run = 1, maxrun = 1, rds = 0, lo = 0, hi = 0;
    for (size_t i = 0; i < st->n; i++) {
        if (i && st->bits[i] == st->bits[i - 1]) { if (++run > maxrun) maxrun = run; } else run = 1;
        rds += st->bits[i] ? 1 : -1;
        if (rds < lo) lo = rds;
        if (rds > hi) hi = rds;
    }
    uint32_t w = 0;
    size_t bad = 0;
    for (size_t i = 0; i < st->n; i++) {
        w = ((w << 1) | st->bits[i]) & 0x3FFu;
        if (i >= 9 && (w == LINK_K28_5_RDN || w == LINK_K28_5_RDP) && (i - 9) % st->frame_bits) bad++;
    }
    bool ok = maxrun <= 5 && hi - lo <= 6 && bad == 0;
    if (!ok) printf("stream check FAILED: max run %d, DSV %d, misplaced commas %zu\n", maxrun, hi - lo, bad);
    return ok;
}

// ============================================================================
// TX RUN -> EDGES -> CHANNEL
// ============================================================================

typedef struct { double *t; uint8_t *lvl; size_t n; double t_end; } Edges;

static bool tx_pull(void *ctx, uint32_t *w) {
    Stream *st = ctx;
    if (st->rd >= st->nw) return false;
    *w = st->words[st->rd++];
    return true;
}
static int tx_no_pin(void *ctx) { (void)ctx; return 0; }
static void tx_no_push(void *ctx, uint32_t w) { (void)ctx; (void)w; }

// Runs link_tx_N cycle by cycle on a clock offset by `ppm` and records the
// pin's transitions. The line idles low before the first bit.
static void tx_run(const Prog *p, Stream *st, double ppm, Edges *e) {
    Sm s;
    sm_load(&s, p->tx, p->tx_len, p->tx_wt, p->tx_wr);
    s.pin_in = tx_no_pin; s.pull = tx_pull; s.push = tx_no_push; s.ctx = st;
    st->rd = 0;
    double t_cyc = T_SYS_NS / (1.0 + ppm * 1e-6);
    e->t = malloc((st->n + 2) * sizeof(double));
    e->lvl = malloc(st->n + 2);
    e->n = 0;
    int level = 0;
    uint64_t cyc = 0;
    for (;;) {
        uint32_t c = sm_step(&s);
        if (!c) break;
        if (s.pin_out != level) {   // the pin changes at the start of the instruction
            level = s.pin_out;
            e->t[e->n] = (double)cyc * t_cyc;
            e->lvl[e->n++] = (uint8_t)level;
        }
        cyc += c;
    }
    e->t_end = (double)cyc * t_cyc;
}

// Pulse-width distortion and jitter. Edges that cross (a pulse shorter than
// the distortion) are removed in pairs, as a real receiver would swallow it.
static void channel(Edges *e, double pwd, double jit, int jmodel) {
    size_t m = 0;
    for (size_t i = 0; i < e->n; i++) {
        double t = e->t[i] + (e->lvl[i] ? -0.5 * pwd : 0.5 * pwd);
        if (jit > 0) {
            if (jmodel == 0) t += (rng() & 1u) ? 0.5 * jit : -0.5 * jit;
            else if (jmodel == 1) t += (rng_unit() - 0.5) * jit;
            else t += rng_gauss() * jit;
        }
        if (m && t <= e->t[m - 1]) { m--; continue; }
        e->t[m] = t;
        e->lvl[m++] = e->lvl[i];
    }
    e->n = m;
}

// ============================================================================
// RX RUN
// ============================================================================

typedef struct {
    const Edges *e; size_t ei;
    double t_cyc, phase_ns; uint64_t cyc;
    uint8_t *bits; size_t n, cap;
} Rx;

static int rx_pin(void *ctx) {
    Rx *r = ctx;
    double t = r->phase_ns + (double)((int64_t)r->cyc - SYNC_CYCLES) * r->t_cyc;
    while (r->ei < r->e->n && r->e->t[r->ei] <= t) r->ei++;
    return r->ei ? r->e->lvl[r->ei - 1] : 0;
}
static bool rx_no_pull(void *ctx, uint32_t *w) { (void)ctx; (void)w; return false; }
static void rx_push(void *ctx, uint32_t w) {
    Rx *r = ctx;
    for (int i = 31; i >= 0 && r->n < r->cap; i--) r->bits[r->n++] = (w >> i) & 1u;
}

static void rx_run(const Prog *p, const Prog *rxp, int trim, const Edges *e, double phase, Rx *r) {
    Sm s;
    sm_load(&s, rxp->rx, rxp->rx_len, rxp->rx_wt, rxp->rx_wr);
    if (!apply_trim(&s, rxp, trim)) die("trim out of range", (uint32_t)trim);
    s.osr = 0xFFFFFFFFu; s.osr_cnt = 0;   // exec: mov osr, ~null
    s.pin_in = rx_pin; s.pull = rx_no_pull; s.push = rx_push; s.ctx = r;
    r->e = e; r->ei = 0; r->t_cyc = T_SYS_NS; r->phase_ns = phase * T_SYS_NS; r->cyc = 0;
    r->n = 0;
    double t_stop = e->t_end - 64.0 * p->n * T_SYS_NS;
    while (r->phase_ns + (double)r->cyc * r->t_cyc < t_stop) r->cyc += sm_step(&s);
}

// ============================================================================
// ORACLE
// ============================================================================

typedef struct {
    bool aligned;
    uint64_t compared, events, slips;
    uint64_t commas, comma_bad;
} Verdict;

#define WIN 256u

static bool match(const uint8_t *a, const uint8_t *b, size_t n) { return memcmp(a, b, n) == 0; }

static Verdict judge(const Stream *st, const Rx *r) {
    Verdict v = {0};
    // Align on a window inside frame 1 (PRBS; frame 0 is idle and periodic),
    // past the RX start-up, searching a few kbit either side.
    const size_t skip = st->frame_bits + 64;
    if (r->n < skip + 4 * WIN) return v;
    int64_t off = 0;
    bool found = false;
    for (size_t o = 0; o + WIN < st->n && o < skip + 4096 && !found; o++)
        if (match(r->bits + skip, st->bits + o, WIN)) { off = (int64_t)o - (int64_t)skip; found = true; }
    if (!found) return v;
    v.aligned = true;
    size_t i = skip;
    while (i + WIN < r->n && (int64_t)i + off + WIN < (int64_t)st->n) {
        if (r->bits[i] == st->bits[(int64_t)i + off]) { i++; v.compared++; continue; }
        v.events++;
        // re-align on the next window within +-8 bits
        bool fixed = false;
        for (int d = 0; d <= 8 && !fixed; d++)
            for (int sgn = -1; sgn <= 1 && !fixed; sgn += 2) {
                int64_t o2 = off + sgn * d;
                size_t j = i + 16;
                if ((int64_t)j + o2 < 0 || (int64_t)j + o2 + WIN >= (int64_t)st->n || j + WIN >= r->n) continue;
                if (match(r->bits + j, st->bits + (int64_t)j + o2, WIN)) {
                    if (o2 != off) v.slips++;
                    off = o2; i = j; fixed = true;
                }
            }
        if (!fixed) i += WIN;
    }
    // Comma stride: every K28.5 the receiver sees must be a whole number of
    // frames after the first (the parser's constant-offset assumption).
    uint32_t w = 0;
    int64_t first = -1;
    for (size_t k = 0; k < r->n; k++) {
        w = ((w << 1) | r->bits[k]) & 0x3FFu;
        if (k >= 9 && (w == LINK_K28_5_RDN || w == LINK_K28_5_RDP)) {
            v.commas++;
            if (first < 0) first = (int64_t)k;
            else if (((int64_t)k - first) % st->frame_bits) v.comma_bad++;
        }
    }
    return v;
}

static bool clean(const Verdict *v, const Stream *st) {
    uint64_t frames = st->n / st->frame_bits;
    return v->aligned && v->events == 0 && v->comma_bad == 0 && v->commas + 8 >= frames &&
           v->compared + 2048 >= st->n;
}

// ============================================================================
// CASES
// ============================================================================

typedef struct {
    uint32_t n_tx, n_rx;        // TX class and the RX program run against it
    double ppm, pwd, jit, phase;
    int trim, jmodel;
    uint32_t bits;
    uint32_t seed;
} Case;

static bool run(const Case *c, Verdict *out) {
    const Prog *p = prog_for(c->n_tx), *rp = prog_for(c->n_rx);
    uint32_t symbols = (uint32_t)(SYS_HZ / (10.0 * c->n_tx * FS_HZ));   // 32 / 64 / 128
    Stream st;
    stream_make(&st, c->bits, symbols, c->seed);
    if (!stream_check(&st)) die("encoder shim produced a bad stream", 0);
    Edges e;
    tx_run(p, &st, c->ppm, &e);
    rng_state = c->seed * 2654435761u + 7u;
    channel(&e, c->pwd, c->jit, c->jmodel);
    Rx r = {0};
    r.cap = st.n + 4096;
    r.bits = malloc(r.cap);
    rx_run(p, rp, c->trim, &e, c->phase, &r);
    Verdict v = judge(&st, &r);
    const char *dump = getenv("LINK_MODEL_DUMP");
    if (dump && *dump) {
        printf("tx:");
        for (int i = 0; i < 160; i++) putchar('0' + st.bits[i]);
        printf("\nrx:");
        for (int i = 0; i < 160 && i < (int)r.n; i++) putchar('0' + r.bits[i]);
        printf("\nedges %zu rx bits %zu\n", e.n, r.n);
    }
    bool ok = clean(&v, &st);
    if (out) *out = v;
    free(r.bits); free(e.t); free(e.lvl); stream_free(&st);
    return ok;
}

static void print_case(const Case *c, const Verdict *v, bool ok) {
    printf("tx %s rx %s  %+6.0f ppm  PWD %+6.2f ns  jit %5.2f ns  trim %+d  phase %.2f: "
           "bits %llu events %llu slips %llu commas %llu bad-stride %llu  %s\n",
           prog_for(c->n_tx)->cls, prog_for(c->n_rx)->cls, c->ppm, c->pwd, c->jit, c->trim, c->phase,
           (unsigned long long)v->compared, (unsigned long long)v->events, (unsigned long long)v->slips,
           (unsigned long long)v->commas, (unsigned long long)v->comma_bad,
           ok ? "CLEAN" : (v->aligned ? "DIRTY" : "NO LOCK"));
}

// Clean across all start phases (ppm = 0 fixes the phase; elsewhere it walks).
static bool run_phases(Case c) {
    int nph = (c.ppm == 0.0) ? 8 : 1;
    for (int i = 0; i < nph; i++) {
        c.phase = (double)i / nph;
        c.seed = 1000u + (uint32_t)i;
        if (!run(&c, NULL)) return false;
    }
    return true;
}

// Largest |PWD| of one sign that stays clean: 1 ns scan, then 0.1 ns refine.
static double max_pwd(Case c, double sign) {
    double good = -1.0;
    for (double a = 0.0; a <= 60.0; a += 1.0) {
        c.pwd = sign * a;
        if (!run_phases(c)) break;
        good = a;
    }
    if (good < 0) return -1.0;
    for (double a = good + 0.1; a < good + 1.0; a += 0.1) {
        c.pwd = sign * a;
        if (!run_phases(c)) break;
        good = a;
    }
    return good;
}

static int regression(void) {
    int fails = 0;
    Verdict v;
    const double ppms[] = {-1000, -300, -100, 0, 100, 300, 1000};
    printf("==== clean line: every class, +-1000 ppm, 8 start phases at 0 ppm ====\n");
    for (int k = 0; k < 3; k++)
        for (int i = 0; i < 7; i++) {
            Case c = {progs[k].n, progs[k].n, ppms[i], 0, 0, 0, 0, 0, 400000, 1};
            bool ok = run(&c, &v);
            ok = run_phases(c) && ok;
            print_case(&c, &v, ok);
            if (!ok) fails++;
        }
    printf("==== class detection: a wrong-class program must not lock ====\n");
    for (int t = 0; t < 3; t++)
        for (int rx = 0; rx < 3; rx++) {
            Case c = {progs[t].n, progs[rx].n, 37, 0, 0, 0, 0, 0, 200000, 3};
            bool ok = run(&c, &v);
            print_case(&c, &v, ok);
            if (ok != (t == rx)) fails++;
        }
    printf("==== distortion inside the SPEC 2.7 budgets (corner jitter, +-300 ppm) ====\n");
    // L1/L2 Broadcom: TX +-3 + RX +-4 + fibre 0.5 = 7.5 ns PWD, 2 ns jitter
    // allowance; L3: no datasheet figure, 2 ns + 1 ns as a smoke test.
    struct { uint32_t n; double pwd, jit; } bud[] = {
        {20, 7.5, 2.0}, {20, -7.5, 2.0}, {10, 7.5, 2.0}, {10, -7.5, 2.0}, {5, 2.0, 1.0}, {5, -2.0, 1.0},
    };
    for (size_t b = 0; b < sizeof(bud) / sizeof(bud[0]); b++)
        for (int s = -1; s <= 1; s += 2) {
            Case c = {bud[b].n, bud[b].n, 300.0 * s, bud[b].pwd, bud[b].jit, 0, 0, 0, 400000, 5};
            bool ok = run(&c, &v);
            ok = run_phases(c) && ok;
            print_case(&c, &v, ok);
            if (!ok) fails++;
        }
    // At one fixed sub-cycle phase the quantisation of the anchoring edges can
    // act like a trim of up to one poll spacing, so the hard limit is half a
    // bit plus that; 0.75 bit is past it for every class.
    printf("==== simulator fidelity: PWD of 0.75 bit must fail ====\n");
    for (int k = 0; k < 3; k++) {
        Case c = {progs[k].n, progs[k].n, 0, 0.75 * progs[k].n * T_SYS_NS, 0, 0.5, 0, 0, 200000, 9};
        bool ok = run(&c, &v);
        print_case(&c, &v, ok);
        if (ok) fails++;
    }
    printf(fails ? "FAIL: %d\n" : "PASS\n", fails);
    return fails;
}

static void sweep(void) {
    const uint32_t bits = 100000;
    printf("==== PWD tolerance (ns, largest clean |PWD| per sign), corner jitter ====\n");
    printf("class  ppm    jitter  +PWD   -PWD   half-bit\n");
    const double jits[] = {0, 1, 2, 5, 10, 15, 20};
    for (int k = 0; k < 3; k++)
        for (int pi = -1; pi <= 1; pi++)
            for (size_t j = 0; j < sizeof(jits) / sizeof(jits[0]); j++) {
                if (progs[k].n == 5 && jits[j] > 5) continue;
                Case c = {progs[k].n, progs[k].n, 300.0 * pi, 0, jits[j], 0, 0, 0, bits, 11};
                double plus = max_pwd(c, +1), minus = max_pwd(c, -1);
                printf("%s   %+5.0f  %5.1f  %6.1f %6.1f  %5.1f\n", progs[k].cls, c.ppm, jits[j],
                       plus, minus, 0.5 * progs[k].n * T_SYS_NS);
            }
    printf("==== trim: window centre moves by one sys cycle (3.26 ns) per step, 0 ppm, no jitter ====\n");
    printf("class  trim  +PWD   -PWD   centre\n");
    for (int k = 0; k < 3; k++) {
        int lim = progs[k].n == 20 ? 8 : progs[k].n == 10 ? 4 : 1, step = progs[k].n == 20 ? 2 : 1;
        for (int t = -lim; t <= lim; t += step) {
            Case c = {progs[k].n, progs[k].n, 0, 0, 0, 0, t, 0, bits, 13};
            double plus = max_pwd(c, +1), minus = max_pwd(c, -1);
            printf("%s    %+3d   %6.1f %6.1f  %+6.1f\n", progs[k].cls, t, plus, minus, 0.5 * (plus - minus));
        }
    }
    printf("==== PLR237 at L1 (SPEC 2.7: PWD +-25 ns max, jitter 1 typ / 15 / 20 ns max) ====\n");
    printf("pwd     jitter  trim 0   best trim (clean trims)\n");
    const double pj[][2] = {{25, 1}, {-25, 1}, {25, 15}, {-25, 15}, {25, 20}, {-25, 20}, {15, 15}, {-15, 15}};
    for (size_t i = 0; i < sizeof(pj) / sizeof(pj[0]); i++) {
        printf("%+5.1f  %5.1f   ", pj[i][0], pj[i][1]);
        Case c = {20, 20, 300, pj[i][0], pj[i][1], 0, 0, 0, bits, 17};
        printf("%-7s  ", run_phases(c) ? "CLEAN" : "DIRTY");
        int any = 0;
        for (int t = -9; t <= 9; t++) {
            c.trim = t;
            if (run_phases(c)) { printf("%+d ", t); any = 1; }
        }
        printf(any ? "\n" : "none\n");
    }
}

int main(int argc, char **argv) {
    if (argc >= 2 && !strcmp(argv[1], "sweep")) { sweep(); return 0; }
    if (argc >= 6 && !strcmp(argv[1], "case")) {
        Case c = {(uint32_t)atoi(argv[2]), (uint32_t)atoi(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]),
                  argc > 7 ? atof(argv[7]) : 0.0, argc > 6 ? atoi(argv[6]) : 0, argc > 8 ? atoi(argv[8]) : 0,
                  1000000, 21};
        Verdict v;
        bool ok = run(&c, &v);
        print_case(&c, &v, ok);
        return ok ? 0 : 1;
    }
    return regression() ? 1 : 0;
}
