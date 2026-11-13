/*
 * bench_cmd.c - link bench: console parsing and report layouts
 *
 * See bench_cmd.h.
 */

#include "bench_cmd.h"

#include <string.h>

#include "bench_gen.h"

int bench_split(char *line, bench_args_t *a) {
    a->argc = 0;
    char *p = line;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') *p++ = 0;
        if (!*p) break;
        if (a->argc < BENCH_MAX_ARGS) a->argv[a->argc++] = p;
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') p++;
    }
    return a->argc;
}

const char *bench_kv(const bench_args_t *a, const char *key) {
    size_t n = strlen(key);
    for (int i = 0; i < a->argc; i++)
        if (!strncmp(a->argv[i], key, n) && a->argv[i][n] == '=') return a->argv[i] + n + 1;
    return NULL;
}

bool bench_parse_int(const char *s, long lo, long hi, long *out) {
    if (!s || !*s) return false;
    bool neg = false;
    if (*s == '-' || *s == '+') neg = *s++ == '-';
    if (!*s) return false;
    long v = 0;
    for (; *s; s++) {
        if (*s < '0' || *s > '9') return false;
        if (v > 100000000L) return false;
        v = v * 10 + (*s - '0');
    }
    if (neg) v = -v;
    if (v < lo || v > hi) return false;
    *out = v;
    return true;
}

static bool ieq(const char *a, const char *b) {
    for (; *a && *b; a++, b++) {
        char x = *a >= 'A' && *a <= 'Z' ? (char)(*a + 32) : *a;
        if (x != *b) return false;
    }
    return *a == *b;
}

bool bench_parse_class(const char *s, unsigned *cls) {
    if (!s) return false;
    if (ieq(s, "l1") || !strcmp(s, "1")) { *cls = LINK_CLASS_L1; return true; }
    if (ieq(s, "l2") || !strcmp(s, "2")) { *cls = LINK_CLASS_L2; return true; }
    if (ieq(s, "l3") || !strcmp(s, "3")) { *cls = LINK_CLASS_L3; return true; }
    return false;
}

bool bench_parse_rate(const char *s, unsigned *rate) {
    if (!s) return false;
    if (ieq(s, "44k1") || !strcmp(s, "44.1") || !strcmp(s, "44100")) { *rate = LINK_RATE_44K1; return true; }
    if (ieq(s, "48k") || !strcmp(s, "48") || !strcmp(s, "48000")) { *rate = LINK_RATE_48K; return true; }
    if (ieq(s, "96k") || !strcmp(s, "96") || !strcmp(s, "96000")) { *rate = LINK_RATE_96K; return true; }
    return false;
}

bool bench_pin_ok(unsigned pin) {
    return pin <= 22u || (pin >= 26u && pin <= 28u);
}

bool bench_parse_pins(const char *s, uint8_t *pins, unsigned *n, unsigned max) {
    if (!s || !*s) return false;
    unsigned k = 0;
    char buf[64];
    size_t len = strlen(s);
    if (len >= sizeof(buf)) return false;
    memcpy(buf, s, len + 1);
    char *p = buf;
    while (p) {
        char *comma = strchr(p, ',');
        if (comma) *comma = 0;
        long v;
        if (!bench_parse_int(p, 0, 29, &v) || !bench_pin_ok((unsigned)v) || k >= max) return false;
        for (unsigned i = 0; i < k; i++) if (pins[i] == v) return false;
        pins[k++] = (uint8_t)v;
        p = comma ? comma + 1 : NULL;
    }
    *n = k;
    return true;
}

const char *bench_class_name(unsigned cls) {
    switch (cls) {
    case LINK_CLASS_L1: return "L1";
    case LINK_CLASS_L2: return "L2";
    case LINK_CLASS_L3: return "L3";
    default: return "none";
    }
}

const char *bench_rate_name(unsigned rate) {
    switch (rate) {
    case LINK_RATE_44K1: return "44k1";
    case LINK_RATE_48K: return "48k";
    case LINK_RATE_96K: return "96k";
    default: return "none";
    }
}

unsigned bench_default_channels(unsigned cls, unsigned rate) {
    unsigned want = cls == LINK_CLASS_L1 ? 8u : cls == LINK_CLASS_L2 ? 16u : 32u;
    unsigned max = link_max_channels(cls, rate);
    return want < max ? want : max;
}

bool bench_parse_tx(const bench_args_t *a, int first, const char *pins_key, bench_txcfg_t *c,
                    const char **err) {
    memset(c, 0, sizeof(*c));
    unsigned cls, rate;
    if (a->argc <= first || !bench_parse_class(a->argv[first], &cls)) { *err = "class"; return false; }
    if (a->argc <= first + 1 || !bench_parse_rate(a->argv[first + 1], &rate)) { *err = "rate"; return false; }
    c->cls = (uint8_t)cls;
    c->rate = (uint8_t)rate;
    c->channels = (uint8_t)bench_default_channels(cls, rate);
    c->pay = BENCH_PAY_PRBS;
    c->pins[0] = 2;
    c->npins = 1;
    long v;
    const char *s;
    if ((s = bench_kv(a, "ch"))) {
        if (!bench_parse_int(s, 1, (long)link_max_channels(cls, rate), &v)) { *err = "ch"; return false; }
        c->channels = (uint8_t)v;
    }
    if ((s = bench_kv(a, "pay"))) {
        if (!strcmp(s, "prbs")) c->pay = BENCH_PAY_PRBS;
        else if (!strcmp(s, "vec")) c->pay = BENCH_PAY_VECTOR;
        else { *err = "pay"; return false; }
    }
    if ((s = bench_kv(a, pins_key))) {
        unsigned n;
        if (!bench_parse_pins(s, c->pins, &n, BENCH_MAX_TX_PINS)) { *err = pins_key; return false; }
        c->npins = (uint8_t)n;
    }
    if ((s = bench_kv(a, "f0"))) {
        if (!bench_parse_int(s, 0, 1000000000L, &v)) { *err = "f0"; return false; }
        c->f0 = (uint64_t)v;
    }
    if ((s = bench_kv(a, "sender"))) {
        if (!bench_parse_int(s, 0, 255, &v)) { *err = "sender"; return false; }
        c->sender = (uint8_t)v;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Layouts
// ---------------------------------------------------------------------------

static void put_u32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i));
}

void bench_pack_status(uint8_t out[BENCH_STATUS_LEN], const bench_status_t *s) {
    memset(out, 0, BENCH_STATUS_LEN);
    out[0] = s->state;
    out[1] = s->cls;
    out[2] = s->state == 2u ? s->rate : 0xFFu;
    out[3] = s->state == 2u ? s->channels : 0u;
    out[4] = s->hops;
    out[5] = 0xFFu;                 // no LOS pin on the bench
    out[6] = s->sender;
    out[7] = s->sf_flags;
    uint64_t f = s->state == 2u ? s->frame : 0u;
    put_u32(out + 8, (uint32_t)f);
    put_u32(out + 12, (uint32_t)(f >> 32));
    put_u32(out + 16, s->st.lock_count);
    put_u32(out + 20, s->st.loss_count);
    put_u32(out + 24, s->st.crc8_errors);
    put_u32(out + 28, s->st.crc16_errors);
    put_u32(out + 32, s->st.bit_slips);
    put_u32(out + 36, s->st.superframe_errors);
    // 40-51: VCXO ppm, phase error, fill: no servo on the bench, 0.
}

void bench_pack_diag(uint8_t out[BENCH_DIAG_LEN], const bench_diag_t *d) {
    memset(out, 0, BENCH_DIAG_LEN);
    out[0] = 1;                     // diag_version
    out[1] = d->comma_offset & 31u;
    // 4-15: servo terms, none on the bench.
    put_u32(out + 16, d->ring_wr_bytes);
    put_u32(out + 20, d->parser_rd_bytes);
    put_u32(out + 24, d->tx_words);
    put_u32(out + 28, d->rx_xfers);
    if (d->scan) {
        out[32] = (uint8_t)d->scan->lo;
        out[33] = (uint8_t)d->scan->hi;
        out[34] = (uint8_t)d->scan->used;
        out[35] = d->scan->state;
        put_u32(out + 36, d->scan->clean_mask);
    }
    put_u32(out + 40, d->st.comma_errors);
    put_u32(out + 44, d->st.counter_errors);
    put_u32(out + 48, d->st.code_errors);
    put_u32(out + 52, d->st.idle_errors);
}

void bench_hex(char *out, const uint8_t *b, size_t n) {
    static const char hx[] = "0123456789abcdef";
    for (size_t i = 0; i < n; i++) {
        out[2 * i] = hx[b[i] >> 4];
        out[2 * i + 1] = hx[b[i] & 15u];
    }
    out[2 * n] = 0;
}
