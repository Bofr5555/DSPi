// pio_timing.c - is the phase simulation's ideal line right? (WP LK6)
//
// link_phase_sim.c lets sender bit k reach the receiver's DMA ring at
// k / baud on the sender's clock, plus a constant.  This program checks that
// against LK2's cycle-accurate model of the real PIO programs: LK4's encoder
// output goes through link_tx_N on a clock offset by `ppm`, a line with
// pulse-width distortion and jitter, and link_rx_N on a nominal clock; every
// 32-bit word the RX pushes (the DMA write position the receiver reads) is
// timed and compared with the time its last bit left the sender.  The spread
// of that delay is the error of the ideal line model, in line bits; the phase
// lock measures arrivals in 32-bit DMA words, so anything under a few bits is
// below its resolution.
//
//   tools/link_phase_sim/run.sh pio
#define main lk2_model_main
#include "link_pio_model.c"
#undef main
#include "sender_link.h"

static int fails;
static double *push_t;
static size_t push_n, push_cap;

static void rx_push_timed(void *ctx, uint32_t w) {
    Rx *r = ctx;
    rx_push(ctx, w);
    if (push_n < push_cap) push_t[push_n++] = r->phase_ns + (double)r->cyc * r->t_cyc;
}

static int32_t smp_of(uint64_t f, unsigned ch) { return (int32_t)((f * 2654435761u + ch * 40503u) & 0xFFFFFFu) - 0x800000; }

static void one(unsigned cls, unsigned rate, unsigned c, double ppm, double pwd, double jit, int trim) {
    static snd_tables_t T;
    static bool tinit;
    if (!tinit) { snd_tables_init(&T); tinit = true; }
    link_tx_t tx;
    link_tx_init(&tx, cls, rate, c, 4096, 1);
    uint32_t cap = 1u << 16;
    uint32_t *w = calloc(cap, 4);
    snd_sink_t s;
    snd_sink_init(&s, w, cap, NULL);
    for (unsigned f = 0; f < 160; f++) {
        int32_t v[64];
        for (unsigned ch = 0; ch < tx.channels; ch++) v[ch] = smp_of(tx.frame, ch);
        snd_frame(&tx, &T, v, &s, NULL);
    }
    Stream st;
    memset(&st, 0, sizeof(st));
    st.n = (size_t)(snd_sink_bits(&s) & ~(uint64_t)31u);
    st.nw = st.n / 32u;
    st.words = w;
    st.bits = malloc(st.n);
    for (size_t i = 0; i < st.n; i++) st.bits[i] = (uint8_t)((w[i >> 5] >> (31u - (i & 31u))) & 1u);
    unsigned n = cls == 1 ? 20u : cls == 2 ? 10u : 5u;
    const Prog *p = prog_for(n);
    Edges e;
    tx_run(p, &st, ppm, &e);
    channel(&e, pwd, jit, 0);
    Rx r;
    memset(&r, 0, sizeof(r));
    r.cap = st.n + 4096;
    r.bits = malloc(r.cap);
    push_cap = r.cap / 32u;
    push_t = malloc(push_cap * sizeof(double));
    push_n = 0;
    // rx_run() with the timed push.
    Sm sm;
    sm_load(&sm, p->rx, p->rx_len, p->rx_wt, p->rx_wr);
    if (!apply_trim(&sm, p, trim)) die("trim", 0);
    sm.osr = 0xFFFFFFFFu;
    sm.osr_cnt = 0;
    sm.pin_in = rx_pin; sm.pull = rx_no_pull; sm.push = rx_push_timed; sm.ctx = &r;
    r.e = &e; r.t_cyc = T_SYS_NS; r.phase_ns = 0.37 * T_SYS_NS;
    double t_stop = e.t_end - 64.0 * n * T_SYS_NS;
    while (r.phase_ns + (double)r.cyc * r.t_cyc < t_stop) r.cyc += sm_step(&sm);
    // Received bit j is sent bit j - k0 (the RX starts on the idle line).
    long k0 = 0;
    size_t best = 0;
    for (long k = -8; k <= 256; k++) {
        size_t m = 0;
        for (size_t j = 4096; j < 8192 && j < r.n; j++)
            if ((long)j - k >= 0 && (size_t)((long)j - k) < st.n && r.bits[j] == st.bits[(long)j - k]) m++;
        if (m > best) { best = m; k0 = k; }
    }
    // Delay of each pushed word after its last bit left the sender: the TX
    // emits bit k at k * N cycles of its own clock.
    double t_bit = (double)n * T_SYS_NS / (1.0 + ppm * 1e-6);
    double lo = 1e30, hi = -1e30, sum = 0;
    size_t cnt = 0, bad = 0;
    for (size_t i = 8; i < push_n; i++) {
        long last = (long)(32u * i + 31u) - k0;
        if (last < 0 || (size_t)last >= st.n) continue;
        for (unsigned b = 0; b < 32; b++)
            if (r.bits[32u * i + b] != st.bits[(long)(32u * i + b) - k0]) bad++;
        double d = push_t[i] - (double)(last + 1) * t_bit;
        if (d < lo) lo = d;
        if (d > hi) hi = d;
        sum += d;
        cnt++;
    }
    double bit_ns = t_bit;
    bool ok = bad == 0 && cnt > 100 && (hi - lo) < 2.0 * bit_ns;
    printf("  L%u %5u Hz C %2u, TX %+6.0f ppm, PWD %4.1f ns, jitter %4.1f ns, trim %+d: %zu words, %zu bit errors, "
           "delay %.1f ns, spread %.2f ns = %.3f bit %s\n",
           cls, rate == 0 ? 44100u : rate == 1 ? 48000u : 96000u, c, ppm, pwd, jit, trim, cnt, bad, sum / (double)cnt,
           hi - lo, (hi - lo) / bit_ns, ok ? "OK" : "FAIL");
    if (!ok) fails++;
    free(w); free(st.bits); free(r.bits); free(push_t);
    free(e.t); free(e.lvl);
}

int main(void) {
    printf("pio_timing: the sender's bits through LK2's PIO model; RX word push time minus send time\n");
    one(1, 1, 8, 0.0, 0.0, 0.0, 0);
    one(1, 1, 8, 300.0, 5.0, 4.0, 0);
    one(1, 1, 8, -1000.0, 15.0, 10.0, 5);
    one(1, 0, 8, 45.0, 5.0, 4.0, 0);
    one(2, 1, 16, -38.0, 7.5, 5.0, 0);
    one(3, 1, 32, 120.0, 2.0, 3.0, 0);
    one(3, 2, 16, -400.0, 2.0, 3.0, 0);
    printf("pio_timing: %s\n", fails ? "FAIL" : "OK (the ideal line is right to within the spread shown)");
    return fails ? 1 : 0;
}
