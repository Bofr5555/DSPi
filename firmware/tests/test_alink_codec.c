// Host test for the link protocol reference codec (WP LK1):
// firmware/DSPi/link/link_codec.c and link_decoder.c against the shared
// vectors in link_vectors.json (made by tools/link/gen_vectors.py, which the
// Python twin checks too), plus properties the vectors cannot show.
// Built by run.sh: cc -std=c11 -Wall -Wextra -Werror -I ../DSPi test_link_codec.c
#include "link/link_codec.c"
#include "link/link_decoder.c"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; \
    fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

// ---------------------------------------------------------------------------
// Vector file scanning (flat JSON objects, one per line)
// ---------------------------------------------------------------------------

static char line[1 << 20];

static const char *find_key(const char *l, const char *key) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\": ", key);
    const char *p = strstr(l, pat);
    return p ? p + strlen(pat) : NULL;
}


static uint64_t get_u64(const char *l, const char *key) {
    const char *p = find_key(l, key);
    if (!p) { CHECK(0, "missing key %s", key); return 0; }
    return strtoull(p, NULL, 10);
}

static int64_t get_i64(const char *l, const char *key) {
    const char *p = find_key(l, key);
    if (!p) { CHECK(0, "missing key %s", key); return 0; }
    return strtoll(p, NULL, 10);
}

// Copies a string value (no escapes occur in the vectors); returns its length.
static size_t get_str(const char *l, const char *key, char *out, size_t cap) {
    const char *p = find_key(l, key);
    out[0] = 0;
    if (!p || *p != '"') { CHECK(0, "missing string %s", key); return 0; }
    p++;
    const char *e = strchr(p, '"');
    size_t n = (size_t)(e - p);
    if (n >= cap) { CHECK(0, "string %s too long", key); return 0; }
    memcpy(out, p, n);
    out[n] = 0;
    return n;
}

static unsigned hexval(char c) {
    return (unsigned)(c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10);
}

static size_t get_hex(const char *l, const char *key, uint8_t *out, size_t cap) {
    static char s[1 << 20];
    size_t n = get_str(l, key, s, sizeof(s)) / 2;
    if (n > cap) { CHECK(0, "hex %s too long", key); return 0; }
    for (size_t i = 0; i < n; i++) out[i] = (uint8_t)(hexval(s[2 * i]) << 4 | hexval(s[2 * i + 1]));
    return n;
}

// ---------------------------------------------------------------------------
// Stream helpers shared with tools/link/link_codec.py (spec section 9.2)
// ---------------------------------------------------------------------------

static int32_t test_sample(uint64_t frame, unsigned ch, uint32_t seed) {
    uint32_t h = (uint32_t)frame * 0x9E3779B1u;
    h ^= (ch + 1u) * 0x85EBCA77u;
    h ^= seed;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    h *= 0x297A2D39u;
    h ^= h >> 15;
    return link_get_s24((const uint8_t[]){ (uint8_t)(h >> 16), (uint8_t)(h >> 8), (uint8_t)h });
}

static uint32_t fnv1a(uint32_t h, const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) h = (h ^ p[i]) * 0x01000193u;
    return h;
}

#define MAX_BITS (1u << 21)
static uint8_t bits_a[MAX_BITS], bits_b[MAX_BITS];   // one bit per byte

typedef struct {
    char  *log;
    size_t len, cap;
} logbuf_t;

static void log_add(logbuf_t *lb, const char *s) {
    size_t n = strlen(s);
    if (lb->len + n + 2 > lb->cap) { CHECK(0, "log overflow"); return; }
    if (lb->len) lb->log[lb->len++] = ' ';
    memcpy(lb->log + lb->len, s, n + 1);
    lb->len += n;
}

static void log_cb(void *ctx, const link_rx_event_t *ev) {
    logbuf_t *lb = ctx;
    char t[128];
    switch (ev->type) {
    case LINK_RX_EV_ALIGN: snprintf(t, sizeof(t), "A%" PRIu64, ev->bitpos); break;
    case LINK_RX_EV_LOCK: snprintf(t, sizeof(t), "L%" PRIu64, ev->frame); break;
    case LINK_RX_EV_SUPERFRAME: snprintf(t, sizeof(t), "S"); break;
    case LINK_RX_EV_SF_ERROR: snprintf(t, sizeof(t), "E%d", ev->sf_error); break;
    case LINK_RX_EV_LOSS: snprintf(t, sizeof(t), "X"); break;
    case LINK_RX_EV_FRAME: {
        char fl[16] = "", banks[LINK_MAX_BANKS + 1];
        if (ev->flags & LINK_RXF_COMMA_MISSING) strcat(fl, "c");
        if (ev->flags & LINK_RXF_SLIP) snprintf(fl + strlen(fl), 8, "s%+d", ev->slip);
        if (ev->flags & LINK_RXF_HEAD_BAD) strcat(fl, "h");
        if (ev->flags & LINK_RXF_COUNTER) strcat(fl, "n");
        unsigned nb = link_bank_count(ev->channels);
        for (unsigned b = 0; b < nb; b++) banks[b] = ".ohz"[ev->bank_status[b]];
        banks[nb] = 0;
        uint32_t h = 0x811C9DC5u;
        for (unsigned ch = 0; ch < ev->channels; ch++) {
            uint32_t u = (uint32_t)ev->samples[ch];
            uint8_t le[4] = { (uint8_t)u, (uint8_t)(u >> 8), (uint8_t)(u >> 16), (uint8_t)(u >> 24) };
            h = fnv1a(h, le, 4);
        }
        snprintf(t, sizeof(t), "F%" PRIu64 ":%s:%s:%08" PRIx32, ev->frame, fl[0] ? fl : "-", banks, h);
        break;
    }
    }
    log_add(lb, t);
}

static void log_stats(logbuf_t *lb, const link_rx_stats_t *s) {
    char t[256];
    snprintf(t, sizeof(t), "#%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
             ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32,
             s->lock_count, s->loss_count, s->crc8_errors, s->crc16_errors, s->bit_slips,
             s->superframe_errors, s->comma_errors, s->counter_errors, s->code_errors,
             s->idle_errors, s->frames);
    log_add(lb, t);
}

// Decode a bit array (one bit per byte) fed in chunks of `chunk` bits.
static link_rx_t rx;
static void decode_bits(const uint8_t *bits, size_t n, unsigned cls, uint8_t mask, size_t chunk,
                        logbuf_t *lb) {
    lb->len = 0;
    lb->log[0] = 0;
    link_rx_init(&rx, cls, mask, log_cb, lb);
    // Each chunk (at most 4096 bits) is re-packed to start at bit 31 of a
    // word, as link_rx_push() takes it.
    for (size_t at = 0; at < n; at += chunk) {
        size_t k = n - at < chunk ? n - at : chunk;
        uint32_t tmp[128] = { 0 };
        for (size_t i = 0; i < k; i++)
            if (bits[at + i]) tmp[i >> 5] |= 0x80000000u >> (i & 31);
        link_rx_push(&rx, tmp, k);
    }
    log_stats(lb, &rx.stats);
}

// Corruption ops (spec section 9.2): positions in the clean stream, applied
// highest first, equal positions in list order.
typedef enum { OP_FLIP, OP_FILL, OP_DEL, OP_INS } op_kind_t;
typedef struct { size_t pos; op_kind_t kind; unsigned a, b; } op_t;

static size_t apply_ops(const uint8_t *in, size_t n, const char *ops, uint8_t *out) {
    op_t list[512];
    size_t nops = 0;
    for (const char *p = ops; *p;) {
        op_t o = { 0, OP_FLIP, 0, 0 };
        if (strncmp(p, "flip:", 5) == 0) o.kind = OP_FLIP;
        else if (strncmp(p, "fill:", 5) == 0) o.kind = OP_FILL;
        else if (strncmp(p, "del:", 4) == 0) o.kind = OP_DEL;
        else if (strncmp(p, "ins:", 4) == 0) o.kind = OP_INS;
        else { CHECK(0, "bad op %s", p); return 0; }
        p = strchr(p, ':') + 1;
        o.pos = strtoull(p, (char **)&p, 10);
        if (*p == ':') o.a = (unsigned)strtoul(p + 1, (char **)&p, 10);
        if (*p == ':') o.b = (unsigned)strtoul(p + 1, (char **)&p, 10);
        if (*p == ';') p++;
        // stable insertion, descending position
        size_t i = nops++;
        while (i > 0 && list[i - 1].pos < o.pos) { list[i] = list[i - 1]; i--; }
        list[i] = o;
    }
    memcpy(out, in, n);
    for (size_t i = 0; i < nops; i++) {
        op_t *o = &list[i];
        switch (o->kind) {
        case OP_FLIP:
            out[o->pos] ^= 1;
            break;
        case OP_FILL:   // fill:P:N:B
            for (unsigned j = 0; j < o->a && o->pos + j < n; j++) out[o->pos + j] = (uint8_t)o->b;
            break;
        case OP_DEL:
            memmove(out + o->pos, out + o->pos + 1, n - o->pos - 1);
            n--;
            break;
        case OP_INS:    // ins:P:B
            memmove(out + o->pos + 1, out + o->pos, n - o->pos);
            out[o->pos] = (uint8_t)o->a;
            n++;
            break;
        }
    }
    return n;
}

// ---------------------------------------------------------------------------
// Vector checks
// ---------------------------------------------------------------------------

static uint8_t hexbuf[1 << 19];
static char name[128];

static void check_code(const char *l) {
    unsigned rd = (unsigned)get_u64(l, "rd");
    uint8_t byte = (uint8_t)get_u64(l, "byte");
    bool k = get_u64(l, "k") != 0;
    uint16_t sym = link_8b10b_encode(byte, k, &rd);
    CHECK(sym == get_u64(l, "sym") && rd == get_u64(l, "rd_out"),
          "code %c%u.%u rd%" PRIu64 ": got %03x/%u", k ? 'K' : 'D', byte & 31u, byte >> 5,
          get_u64(l, "rd"), sym, rd);
}

static void check_dectab(const char *l) {
    static char s[4 * 1024 + 1];
    CHECK(get_str(l, "table", s, sizeof(s)) == 4096, "dectab length");
    for (unsigned sym = 0; sym < 1024; sym++) {
        unsigned want = 0;
        for (int i = 0; i < 4; i++) want = want << 4 | hexval(s[4 * sym + (unsigned)i]);
        uint8_t byte = 0;
        bool k = false;
        unsigned mask = link_8b10b_decode((uint16_t)sym, &byte, &k);
        unsigned got = mask ? (mask << 9) | ((unsigned)k << 8) | byte : 0;
        CHECK(got == want, "decode %03x: got %04x want %04x", sym, got, want);
    }
}

static void check_layout(const char *l) {
    unsigned cls = (unsigned)get_u64(l, "cls"), rate = (unsigned)get_u64(l, "rate");
    static char s[512];
    get_str(l, "lengths", s, sizeof(s));
    uint64_t sum = 0;
    for (unsigned f = 0; f < 147; f++) {
        unsigned want = hexval(s[2 * f]) << 4 | hexval(s[2 * f + 1]);
        unsigned got = link_frame_symbols(cls, rate, f);
        sum += got;
        CHECK(got == want, "L%u rate %u frame %u: %u symbols, want %u", cls, rate, f, got, want);
    }
    CHECK(sum == get_u64(l, "sum"), "L%u rate %u: sum", cls, rate);
    CHECK(link_frame_symbols_min(cls, rate) == get_u64(l, "min"), "L%u rate %u: min", cls, rate);
    CHECK(link_frame_symbols_max(cls, rate) == get_u64(l, "max"), "L%u rate %u: max", cls, rate);
    CHECK(link_max_channels(cls, rate) == get_u64(l, "max_channels"), "L%u rate %u: capacity", cls, rate);
    CHECK(link_frame_symbols(cls, rate, (1ull << 63) + 12345u) == get_u64(l, "at_big"),
          "L%u rate %u: frame 2^63+12345", cls, rate);
}

static void check_sf(const char *l) {
    get_str(l, "name", name, sizeof(name));
    uint8_t b[LINK_SF_LEN];
    CHECK(get_hex(l, "bytes", b, sizeof(b)) == LINK_SF_LEN, "sf %s: length", name);
    link_sf_t sf;
    memset(&sf, 0, sizeof(sf));
    int res = link_sf_unpack(b, &sf);
    CHECK(res == (int)get_i64(l, "result"), "sf %s: result %d", name, res);
    if (!get_u64(l, "pack")) return;
    link_sf_t want = {
        .flags = (uint8_t)get_u64(l, "flags"), .rate = (uint8_t)get_u64(l, "rate"),
        .channels = (uint8_t)get_u64(l, "channels"), .cls = (uint8_t)get_u64(l, "cls"),
        .hops = (uint8_t)get_u64(l, "hops"), .sender = (uint8_t)get_u64(l, "sender"),
        .frame = get_u64(l, "frame"), .apply_frame = get_u64(l, "apply_frame"),
        .apply_op = (uint8_t)get_u64(l, "apply_op"), .apply_arg = (uint8_t)get_u64(l, "apply_arg"),
        .apply_value = (int16_t)get_i64(l, "apply_value"),
    };
    uint8_t packed[LINK_SF_LEN];
    link_sf_pack(&want, packed);
    CHECK(memcmp(packed, b, LINK_SF_LEN) == 0, "sf %s: pack", name);
    CHECK(sf.flags == want.flags && sf.rate == want.rate && sf.channels == want.channels &&
          sf.cls == want.cls && sf.hops == want.hops && sf.sender == want.sender &&
          sf.frame == want.frame && sf.apply_frame == want.apply_frame &&
          sf.apply_op == want.apply_op && sf.apply_arg == want.apply_arg &&
          sf.apply_value == want.apply_value, "sf %s: unpack fields", name);
}

static void check_sf_repeat(const char *l) {
    uint8_t in[LINK_SF_LEN], want[LINK_SF_LEN], got[LINK_SF_LEN];
    get_hex(l, "bytes_in", in, sizeof(in));
    get_hex(l, "bytes_out", want, sizeof(want));
    link_sf_t a, b;
    CHECK(link_sf_unpack(in, &a) == LINK_SF_OK, "sf_repeat: input");
    link_sf_repeat(&a, (uint8_t)get_u64(l, "cls"), &b);
    link_sf_pack(&b, got);
    CHECK(memcmp(got, want, LINK_SF_LEN) == 0, "sf_repeat %s", get_str(l, "name", name, sizeof(name)) ? name : "");
}

// Encode `frames` frames like tools/link/link_codec.py encode_stream().
static size_t encode_stream(const char *l, uint8_t *bits) {
    static uint32_t words[MAX_BITS / 32];
    link_tx_t tx;
    unsigned cls = (unsigned)get_u64(l, "cls"), rate = (unsigned)get_u64(l, "rate");
    unsigned c = (unsigned)get_u64(l, "channels");
    uint64_t first = get_u64(l, "first"), frames = get_u64(l, "frames");
    uint32_t seed = (uint32_t)get_u64(l, "seed");
    int64_t sched = get_i64(l, "sched_index");
    CHECK(link_tx_init(&tx, cls, rate, c, first, (uint8_t)get_u64(l, "sender")), "tx init");
    tx.flags = (uint8_t)get_u64(l, "flags");
    tx.hops = (uint8_t)get_u64(l, "hops");
    link_bitw_t w;
    link_bitw_init(&w, words, MAX_BITS / 32);
    for (uint64_t i = 0; i < frames; i++) {
        if ((int64_t)i == sched)
            CHECK(link_tx_schedule(&tx, get_u64(l, "sched_at"), (uint8_t)get_u64(l, "sched_op"),
                                   (uint8_t)get_u64(l, "sched_arg"), (int16_t)get_i64(l, "sched_value")),
                  "schedule refused");
        // Exactly tx->channels samples on the heap, so ASan sees any read past them
        // (review lk F1: a RATE that grows C must not read the new C early).
        int32_t *s = malloc(tx.channels * sizeof(int32_t));
        for (unsigned ch = 0; ch < tx.channels; ch++) s[ch] = test_sample(tx.frame, ch, seed);
        CHECK(link_tx_frame(&tx, s, &w), "bit buffer full");
        free(s);
    }
    for (size_t i = 0; i < w.nbits; i++) bits[i] = (uint8_t)((words[i >> 5] >> (31u - (i & 31u))) & 1u);
    return w.nbits;
}

static void check_frame(const char *l) {
    get_str(l, "name", name, sizeof(name));
    static char want[2048];
    get_str(l, "symbols", want, sizeof(want));
    link_tx_t tx;
    unsigned c = (unsigned)get_u64(l, "channels");
    uint64_t f = get_u64(l, "frame");
    CHECK(link_tx_init(&tx, (unsigned)get_u64(l, "cls"), (unsigned)get_u64(l, "rate"), c, f, 0),
          "frame %s: init", name);
    tx.flags = (uint8_t)get_u64(l, "flags");
    int32_t s[LINK_MAX_CHANNELS];
    for (unsigned ch = 0; ch < c; ch++) s[ch] = test_sample(f, ch, (uint32_t)get_u64(l, "seed"));
    uint32_t words[LINK_MAX_FRAME_SYMBOLS * 10 / 32 + 1];
    link_bitw_t w;
    link_bitw_init(&w, words, sizeof(words) / 4);
    CHECK(link_tx_frame(&tx, s, &w), "frame %s: encode", name);
    char got[2048];
    size_t n = w.nbits / 10;
    for (size_t i = 0; i < n; i++) {
        unsigned v = 0;
        for (size_t j = 0; j < 10; j++) v = v << 1 | ((words[(10 * i + j) >> 5] >> (31u - ((10 * i + j) & 31u))) & 1u);
        snprintf(got + 3 * i, 4, "%03x", v);
    }
    got[3 * n] = 0;
    CHECK(strcmp(got, want) == 0, "frame %s: symbols\n got  %s\n want %s", name, got, want);
    CHECK(tx.rd == get_u64(l, "rd_out"), "frame %s: rd_out", name);
}

static void check_stream(const char *l) {
    static char want_log[1 << 18], got_log[1 << 18], ops[1 << 14];
    get_str(l, "name", name, sizeof(name));
    size_t nbits = (size_t)get_u64(l, "bits");
    size_t nbytes = get_hex(l, "stream", hexbuf, sizeof(hexbuf));
    CHECK(nbytes == (nbits + 7) / 8, "stream %s: length", name);
    for (size_t i = 0; i < nbits; i++) bits_a[i] = (uint8_t)((hexbuf[i >> 3] >> (7 - (i & 7))) & 1u);
    if (get_u64(l, "encode")) {
        size_t n = encode_stream(l, bits_b);
        CHECK(n == nbits && memcmp(bits_a, bits_b, nbits) == 0, "stream %s: encoder output differs", name);
    }
    get_str(l, "corrupt", ops, sizeof(ops));
    size_t n = apply_ops(bits_a, nbits, ops, bits_b);
    get_str(l, "log", want_log, sizeof(want_log));
    unsigned cls = (unsigned)get_u64(l, "cls");
    uint8_t mask = (uint8_t)get_u64(l, "bank_mask");
    static const size_t chunks[] = { 32, 1, 7, 320, 4096 };
    for (size_t i = 0; i < sizeof(chunks) / sizeof(chunks[0]); i++) {
        logbuf_t lb = { got_log, 0, sizeof(got_log) };
        decode_bits(bits_b, n, cls, mask, chunks[i], &lb);
        CHECK(strcmp(got_log, want_log) == 0, "stream %s (chunk %zu): log differs\n got  %.600s\n want %.600s",
              name, chunks[i], got_log, want_log);
    }
}

static void test_vectors(void) {
    char path[1024];
    snprintf(path, sizeof(path), "%s", __FILE__);
    char *slash = strrchr(path, '/');
    if (slash) strcpy(slash + 1, "link_vectors.json");
    else strcpy(path, "link_vectors.json");
    FILE *f = fopen(path, "r");
    CHECK(f != NULL, "cannot open %s", path);
    if (!f) return;
    int count[16] = { 0 };
    static const char *types[] = { "crc8", "crc16", "code", "dectab", "layout", "layout_c",
                                   "prbs7", "sf", "sf_repeat", "frame", "stream" };
    while (fgets(line, sizeof(line), f)) {
        char type[32];
        get_str(line, "type", type, sizeof(type));
        size_t t;
        for (t = 0; t < sizeof(types) / sizeof(types[0]); t++)
            if (strcmp(type, types[t]) == 0) break;
        CHECK(t < sizeof(types) / sizeof(types[0]), "unknown vector type %s", type);
        count[t]++;
        uint8_t data[64];
        size_t n;
        switch (t) {
        case 0:
            n = get_hex(line, "data", data, sizeof(data));
            CHECK(link_crc8(data, n) == get_u64(line, "crc"), "crc8 %s", get_str(line, "name", name, sizeof(name)) ? name : "");
            break;
        case 1:
            n = get_hex(line, "data", data, sizeof(data));
            CHECK(link_crc16(data, n) == get_u64(line, "crc"), "crc16 %s", get_str(line, "name", name, sizeof(name)) ? name : "");
            break;
        case 2: check_code(line); break;
        case 3: check_dectab(line); break;
        case 4: check_layout(line); break;
        case 5:
            CHECK(link_layout_symbols((unsigned)get_u64(line, "channels")) == get_u64(line, "symbols"),
                  "layout of %" PRIu64 " channels", get_u64(line, "channels"));
            break;
        case 6: {
            uint8_t want[64], got[64];
            n = get_hex(line, "bytes", want, sizeof(want));
            link_prbs7_fill(got, n);
            CHECK(memcmp(got, want, n) == 0, "prbs7");
            break;
        }
        case 7: check_sf(line); break;
        case 8: check_sf_repeat(line); break;
        case 9: check_frame(line); break;
        case 10: check_stream(line); break;
        }
    }
    fclose(f);
    CHECK(count[2] == 2 * (256 + 12), "code vectors: %d", count[2]);
    CHECK(count[3] == 1 && count[4] == 9 && count[5] == 64, "layout vectors");
    CHECK(count[10] >= 25, "only %d stream vectors", count[10]);
    printf("  vectors: crc8 %d, crc16 %d, code %d, sf %d, frame %d, stream %d\n",
           count[0], count[1], count[2], count[7], count[9], count[10]);
}

// ---------------------------------------------------------------------------
// Properties and anchors from the spec (independent of the vectors)
// ---------------------------------------------------------------------------

static void test_anchors(void) {
    const uint8_t chk[] = "123456789";
    CHECK(link_crc8(chk, 9) == 0xF4, "CRC-8 check value");
    CHECK(link_crc16(chk, 9) == 0x29B1, "CRC-16/CCITT-FALSE check value");
    unsigned rd = 0;
    CHECK(link_8b10b_encode(LINK_BYTE_K28_5, true, &rd) == 0x0FA && rd == 1, "K28.5 RD-");
    CHECK(link_8b10b_encode(LINK_BYTE_K28_5, true, &rd) == 0x305 && rd == 0, "K28.5 RD+");
    rd = 0;
    CHECK(link_8b10b_encode(LINK_BYTE_IDLE, false, &rd) == LINK_SYM_IDLE && rd == 0, "D21.5 RD-");
    rd = 1;
    CHECK(link_8b10b_encode(LINK_BYTE_IDLE, false, &rd) == LINK_SYM_IDLE && rd == 1, "D21.5 RD+");
    rd = 0;
    CHECK(link_8b10b_encode(0x00, false, &rd) == 0x274 && rd == 0, "D0.0 RD- = 100111 0100");
    rd = 0;
    CHECK(link_8b10b_encode(0xF1, false, &rd) == 0x237 && rd == 1, "D17.7 RD- uses A7: 100011 0111");
    rd = 0;
    CHECK(link_8b10b_encode(0x1C, true, &rd) == 0x0F4, "K28.0 RD- = 001111 0100");
    rd = 0;
    CHECK(link_8b10b_encode(0x1D, true, &rd) == LINK_SYM_INVALID && rd == 0, "K29.0 does not exist");
    CHECK(link_frame_symbols(1, 1, 0) == 32 && link_frame_symbols(2, 1, 0) == 64 &&
          link_frame_symbols(3, 1, 0) == 128 && link_frame_symbols(1, 2, 0) == 16,
          "symbols per frame at 48/96 kHz");
    CHECK(link_max_channels(1, 1) == 8 && link_max_channels(2, 1) == 18 && link_max_channels(3, 1) == 38 &&
          link_max_channels(1, 2) == 3 && link_max_channels(2, 2) == 8 && link_max_channels(3, 2) == 18,
          "capacity table");
    unsigned sum = 0;
    for (unsigned f = 0; f < 147; f++) sum += link_frame_symbols(1, 0, f);
    CHECK(sum == 5120, "44.1 kHz L1: 5120 symbols per 147 frames");
    // Every symbol the encoder makes decodes back, and nothing else decodes.
    unsigned valid = 0;
    for (unsigned s = 0; s < 1024; s++) {
        uint8_t b;
        bool k;
        if (link_8b10b_decode((uint16_t)s, &b, &k)) valid++;
    }
    unsigned images = 0;
    for (unsigned k = 0; k < 2; k++)
        for (unsigned v = 0; v < 256; v++)
            for (unsigned r = 0; r < 2; r++) {
                unsigned rr = r;
                uint16_t s = link_8b10b_encode((uint8_t)v, k, &rr);
                if (s == LINK_SYM_INVALID) continue;
                images++;
                uint8_t b;
                bool kk;
                unsigned m = link_8b10b_decode(s, &b, &kk);
                CHECK((m >> r) & 1u && b == v && kk == (k != 0), "roundtrip %u %02x rd%u", k, v, r);
                CHECK(link_8b10b_rd_after(s, r) == rr, "rd_after %u %02x rd%u", k, v, r);
            }
    CHECK(images == 2 * (256 + 12), "encoder images");
    printf("  8b/10b: %u valid code groups of 1024; sizeof(link_rx_t) %zu, sizeof(link_tx_t) %zu\n",
           valid, sizeof(link_rx_t), sizeof(link_tx_t));
}

// A long random stream: run length <= 5, running digital sum bounded, and the
// comma pattern only where a K28.5 was sent.
static void test_stream_properties(void) {
    static uint32_t words[1 << 14];
    link_tx_t tx;
    link_bitw_t w;
    link_bitw_init(&w, words, sizeof(words) / 4);
    CHECK(link_tx_init(&tx, 2, 0, 18, 123456789u, 1), "init");
    uint32_t lcg = 1;
    unsigned frames = 0;
    for (;;) {
        int32_t s[LINK_MAX_CHANNELS];
        for (unsigned ch = 0; ch < 18; ch++) { lcg = lcg * 1103515245u + 12345u; s[ch] = (int32_t)(lcg >> 8) - 0x800000; }
        size_t before = w.nbits;
        if (!link_tx_frame(&tx, s, &w)) { w.nbits = before; break; }
        frames++;
    }
    unsigned run = 0, maxrun = 0, prev = 2, commas = 0, misplaced = 0;
    int rds = 0, rds_min = 0, rds_max = 0;
    uint16_t hist = 0;
    uint64_t frame_start = 0;
    unsigned fidx = 0;
    for (size_t i = 0; i < w.nbits; i++) {
        unsigned b = (words[i >> 5] >> (31u - (i & 31u))) & 1u;
        run = b == prev ? run + 1 : 1;
        prev = b;
        if (run > maxrun) maxrun = run;
        rds += b ? 1 : -1;
        if (rds < rds_min) rds_min = rds;
        if (rds > rds_max) rds_max = rds;
        hist = (uint16_t)(((hist << 1) | b) & 0x3FFu);
        if (i >= 9 && link_sym_is_comma(hist)) {
            commas++;
            if (i + 1 != frame_start + 10u) misplaced++;
        }
        if (i + 1 == frame_start + 10u) {
            frame_start += 10u * link_frame_symbols(2, 0, 123456789u + fidx);
            fidx++;
        }
    }
    CHECK(maxrun <= 5, "run length %u", maxrun);
    CHECK(rds_max - rds_min <= 6, "running digital sum span %d", rds_max - rds_min);
    CHECK(commas == frames && misplaced == 0, "commas %u for %u frames, %u misplaced", commas, frames, misplaced);
    printf("  stream: %u frames, max run %u, RDS span %d, commas only at frame starts\n",
           frames, maxrun, rds_max - rds_min);
}

// Clean streams decode to exactly the samples sent, at every class and rate.
typedef struct { uint32_t seed; unsigned frames, bad; } rt_ctx_t;

static void rt_cb(void *ctx, const link_rx_event_t *ev) {
    rt_ctx_t *c = ctx;
    if (ev->type != LINK_RX_EV_FRAME) return;
    c->frames++;
    if (ev->flags) c->bad++;
    for (unsigned b = 0; b < link_bank_count(ev->channels); b++)
        if (ev->bank_status[b] != LINK_BANK_OK) c->bad++;
    for (unsigned ch = 0; ch < ev->channels; ch++)
        if (ev->samples[ch] != test_sample(ev->frame, ch, c->seed)) c->bad++;
}

static void test_roundtrip(void) {
    static const unsigned cfg[][3] = { { 1, 1, 8 }, { 2, 1, 18 }, { 3, 1, 38 }, { 1, 2, 3 },
                                       { 3, 2, 18 }, { 1, 0, 8 }, { 3, 0, 41 } };
    static uint32_t words[1 << 14];
    for (size_t i = 0; i < sizeof(cfg) / sizeof(cfg[0]); i++) {
        link_tx_t tx;
        link_bitw_t w;
        link_bitw_init(&w, words, sizeof(words) / 4);
        CHECK(link_tx_init(&tx, cfg[i][0], cfg[i][1], cfg[i][2], 1000, 9), "init %zu", i);
        for (unsigned f = 0; f < 120; f++) {
            int32_t s[LINK_MAX_CHANNELS];
            for (unsigned ch = 0; ch < cfg[i][2]; ch++) s[ch] = test_sample(tx.frame, ch, 3);
            CHECK(link_tx_frame(&tx, s, &w), "encode %zu", i);
        }
        rt_ctx_t c = { 3, 0, 0 };
        link_rx_init(&rx, cfg[i][0], 0xFF, rt_cb, &c);
        link_rx_push(&rx, words, w.nbits);
        CHECK(c.frames > 50 && c.bad == 0 && rx.stats.lock_count == 1,
              "roundtrip L%u rate %u C %u: %u frames, %u bad", cfg[i][0], cfg[i][1], cfg[i][2], c.frames, c.bad);
    }
}

static void test_tx_rules(void) {
    link_tx_t tx;
    CHECK(!link_tx_init(&tx, 1, 1, 9, 0, 0), "9 channels on L1 refused");
    CHECK(!link_tx_init(&tx, 4, 1, 8, 0, 0), "class 4 refused");
    CHECK(link_tx_init(&tx, 2, 1, 16, 100, 0), "init");
    CHECK(!link_tx_schedule(&tx, 140, LINK_OP_MUTE, 0, 0), "too close: no superframe completes before it");
    CHECK(!link_tx_schedule(&tx, 168, LINK_OP_RATE, 2, 8), "RATE off a superframe boundary");
    CHECK(!link_tx_schedule(&tx, 160, LINK_OP_RATE, 2, 9), "RATE to 9 channels at 96 kHz on L2");
    CHECK(link_tx_schedule(&tx, 160, LINK_OP_RATE, 2, 8), "RATE at 160");
    CHECK(!link_tx_schedule(&tx, 400, LINK_OP_MUTE, 0, 0), "one apply at a time");
}

int main(void) {
    test_anchors();
    test_tx_rules();
    test_roundtrip();
    test_stream_properties();
    test_vectors();
    if (failures) {
        fprintf(stderr, "test_link_codec: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_link_codec: OK\n");
    return 0;
}
