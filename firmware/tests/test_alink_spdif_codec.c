// Host test for the S/PDIF carriage codec (WP FWSP1, alink_protocol_spec.md 12):
// firmware/DSPi/alink/alink_spdif_codec.c against alink_spdif_vectors.json
// (made by tools/alink/alink_spdif.py, which checks the same file), plus the
// spec's examples and properties the vectors cannot show.
// Built by run.sh: cc -std=c11 -Wall -Wextra -Werror -I ../DSPi test_alink_spdif_codec.c
#include "alink/alink_spdif_codec.c"
#include "alink/alink_codec.c"      // the line's CRC and unpack, for cross-checks only

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

static char line[1 << 16];

static const char *find_key(const char *l, const char *key) {
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\": ", key);
    const char *p = strstr(l, pat);
    return p ? p + strlen(pat) : NULL;
}

static bool has_key(const char *l, const char *key) { return find_key(l, key) != NULL; }

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
    char s[1024];
    size_t n = get_str(l, key, s, sizeof(s)) / 2;
    if (n > cap) { CHECK(0, "hex %s too long", key); return 0; }
    for (size_t i = 0; i < n; i++) out[i] = (uint8_t)(hexval(s[2 * i]) << 4 | hexval(s[2 * i + 1]));
    return n;
}

static char name[128];

static void check_fields(const char *l, const alink_spdif_sf_t *sf) {
#define F(field) CHECK((uint64_t)sf->field == get_u64(l, #field), "%s: " #field, name)
    F(flags); F(rate); F(channels); F(cls); F(hops); F(sender); F(frame);
    F(apply_frame); F(apply_op); F(apply_arg); F(epoch); F(sender2);
#undef F
    CHECK(sf->apply_value == get_i64(l, "apply_value"), "%s: apply_value", name);
}

static void test_vectors(void) {
    char path[1024];
    snprintf(path, sizeof(path), "%s", __FILE__);
    char *slash = strrchr(path, '/');
    if (slash) strcpy(slash + 1, "alink_spdif_vectors.json");
    else strcpy(path, "alink_spdif_vectors.json");
    FILE *f = fopen(path, "r");
    CHECK(f != NULL, "cannot open %s", path);
    if (!f) return;
    int count[5] = { 0 }, results[7] = { 0 };
    static const char *types[] = { "crc16", "build", "sf", "ubits", "lead" };
    while (fgets(line, sizeof(line), f)) {
        char type[32];
        get_str(line, "type", type, sizeof(type));
        if (has_key(line, "name")) get_str(line, "name", name, sizeof(name));
        else name[0] = 0;
        size_t t;
        for (t = 0; t < 5; t++)
            if (strcmp(type, types[t]) == 0) break;
        CHECK(t < 5, "unknown vector type %s", type);
        if (t >= 5) continue;
        count[t]++;
        uint8_t data[64], got[ALINK_SPDIF_SF_LEN];
        size_t n;
        alink_spdif_sf_t sf;
        switch (t) {
        case 0:
            n = get_hex(line, "data", data, sizeof(data));
            CHECK(alink_spdif_crc16(data, n) == get_u64(line, "crc"), "crc16 %s", name);
            break;
        case 1: {
            alink_spdif_src_t src = {
                .flags = (uint8_t)get_u64(line, "flags"),
                .rate = (uint8_t)get_u64(line, "rate"),
                .sender = (uint8_t)get_u64(line, "sender"),
                .sender2 = (uint8_t)get_u64(line, "sender2"),
                .epoch = (uint8_t)get_u64(line, "epoch"),
                .apply_frame = get_u64(line, "apply_frame"),
                .apply_op = (uint8_t)get_u64(line, "apply_op"),
                .apply_arg = (uint8_t)get_u64(line, "apply_arg"),
                .apply_value = (int16_t)get_i64(line, "apply_value"),
            };
            n = get_hex(line, "bytes", data, sizeof(data));
            CHECK(n == ALINK_SPDIF_SF_LEN, "%s: length", name);
            alink_spdif_sf_build(&src, get_u64(line, "frame"), got);
            CHECK(memcmp(got, data, ALINK_SPDIF_SF_LEN) == 0, "build %s", name);
            CHECK(alink_spdif_sf_unpack(got, &sf) == ALINK_SPDIF_OK, "build %s: not valid", name);
            break;
        }
        case 2: {
            n = get_hex(line, "bytes", data, sizeof(data));
            CHECK(n == ALINK_SPDIF_SF_LEN, "%s: length", name);
            memset(&sf, 0, sizeof(sf));
            int r = alink_spdif_sf_unpack(data, &sf);
            CHECK(r == (int)get_u64(line, "result"), "sf %s: result %d, want %d", name, r,
                  (int)get_u64(line, "result"));
            if (r >= 0 && r < 7) results[r]++;
            if (has_key(line, "frame")) check_fields(line, &sf);
            if (r == ALINK_SPDIF_OK) {
                bool pad_zero = true;
                for (unsigned i = 32; i < ALINK_SPDIF_SF_LEN; i++) pad_zero = pad_zero && !data[i];
                alink_spdif_sf_pack(&sf, got);
                CHECK(memcmp(got, data, pad_zero ? ALINK_SPDIF_SF_LEN : 32u) == 0, "repack %s", name);
            }
            break;
        }
        case 3: {
            char bits[ALINK_SPDIF_USER_BITS + 8];
            n = get_hex(line, "bytes", data, sizeof(data));
            get_str(line, "bits", bits, sizeof(bits));
            for (unsigned i = 0; i < ALINK_SPDIF_USER_BITS; i++) {
                CHECK(alink_spdif_ubit(data, i) == (unsigned)(bits[i] - '0'), "ubits %s bit %u", name, i);
                if (i & 1u) continue;
                unsigned fu = alink_spdif_frame_ubits(data, i / 2u);
                CHECK((fu & 1u) == (unsigned)(bits[i] - '0') && (fu >> 1) == (unsigned)(bits[i + 1] - '0'),
                      "frame ubits %s frame %u", name, i / 2u);
            }
            break;
        }
        case 4:
            CHECK(alink_spdif_lead_ok(get_u64(line, "now"), get_u64(line, "target")) ==
                  (get_u64(line, "ok") != 0), "lead %" PRIu64 " -> %" PRIu64,
                  get_u64(line, "now"), get_u64(line, "target"));
            break;
        }
    }
    fclose(f);
    for (int r = 0; r < 7; r++) CHECK(results[r] > 0, "no sf vector with result %d", r);
    CHECK(count[1] >= 15 && count[2] >= 30, "too few vectors: build %d sf %d", count[1], count[2]);
    printf("  vectors: crc16 %d, build %d, sf %d, ubits %d, lead %d\n",
           count[0], count[1], count[2], count[3], count[4]);
}

// ---------------------------------------------------------------------------
// The spec's examples and properties
// ---------------------------------------------------------------------------

static const uint8_t TYPICAL[48] = {
    0xa5, 0x01, 0x04, 0x01, 0x02, 0x10, 0x00, 0x42, 0x00, 0xa6, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x42, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x03, 0x37, 0xa7, 0x80,
};

static void test_spec_examples(void) {
    alink_spdif_src_t src = { .rate = ALINK_SPDIF_RATE_48K, .sender = 0x42, .sender2 = 0x37, .epoch = 3,
                              .apply_frame = 1000000, .apply_op = ALINK_SPDIF_OP_SIGGEN_START };
    uint8_t got[48];
    alink_spdif_sf_build(&src, 960000, got);
    CHECK(memcmp(got, TYPICAL, 48) == 0, "spdif_sf/typical");
    alink_spdif_sf_t sf;
    CHECK(alink_spdif_sf_unpack(got, &sf) == ALINK_SPDIF_OK && alink_spdif_source_id(&sf) == 0x3742,
          "typical source id");
    static const unsigned first16[16] = { 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
    for (unsigned i = 0; i < 16; i++) CHECK(alink_spdif_ubit(got, i) == first16[i], "first bits %u", i);
    CHECK(alink_spdif_frame_ubits(got, 0) == 1u, "frame 0 carries A = 1, B = 0");

    alink_spdif_src_t idle = { .rate = ALINK_SPDIF_RATE_48K, .sender = 0x42, .sender2 = 0x37, .epoch = 3 };
    alink_spdif_sf_build(&idle, 960192, got);
    CHECK(got[2] == 0 && got[8] == 0xc0 && got[30] == 0x90 && got[31] == 0xdd, "spdif_sf/no_apply");
    // Were the op still pending, this block carries it too.
    alink_spdif_sf_build(&src, 960192, got);
    CHECK(got[24] == 9 && got[2] == ALINK_SPDIF_FLAG_APPLY_PEND, "next block carries op 9");
}

static void test_properties(void) {
    // The CRC is the line's.
    uint8_t buf[48];
    uint32_t x = 12345;
    for (unsigned n = 0; n <= 48; n++) {
        for (unsigned i = 0; i < n; i++) { x = x * 1103515245u + 12345u; buf[i] = (uint8_t)(x >> 16); }
        CHECK(alink_spdif_crc16(buf, n) == alink_crc16(buf, n), "crc length %u", n);
    }
    // Own op set: exactly {1, 2, 4, 7, 9, 10}, independent of the line's ALINK_OP_MAX.
    for (unsigned op = 0; op < 300; op++) {
        bool want = op == 1 || op == 2 || op == 4 || op == 7 || op == 9 || op == 10;
        CHECK(alink_spdif_op_allowed(op) == want, "op %u", op);
    }
    // A line receiver never accepts an S/PDIF superframe (class 0x10).
    alink_sf_t lsf;
    CHECK(alink_sf_unpack(TYPICAL, &lsf) == ALINK_SF_ERR_LAYOUT, "line accepts S/PDIF");
    // F mod 192 over the whole u64 (the codec avoids a 64-bit division).
    for (uint64_t k = 0; k < 2000; k++) {
        uint64_t fs[3] = { 192u * k, ((uint64_t)1 << 32) * 3u * k, UINT64_MAX - k };
        for (int j = 0; j < 3; j++) {
            alink_spdif_sf_t s = { .rate = 1, .channels = 2, .cls = ALINK_SPDIF_CLASS, .frame = fs[j] };
            uint8_t b[48];
            alink_spdif_sf_pack(&s, b);
            alink_spdif_sf_t back;
            int r = alink_spdif_sf_unpack(b, &back);
            CHECK((r == ALINK_SPDIF_OK) == (fs[j] % 192u == 0), "frame %" PRIu64 " r %d", fs[j], r);
        }
    }
    // Carriage edge around the top of the u64 and the lead rule.
    CHECK(alink_spdif_carries(960000, 960192) && !alink_spdif_carries(960000, 960191), "edge");
    CHECK(!alink_spdif_carries(UINT64_MAX - 63u, UINT64_MAX), "no overflow at the top");
    CHECK(alink_spdif_lead_ok(0, 896) && !alink_spdif_lead_ok(0, 895) && !alink_spdif_lead_ok(10, 5), "lead");
    // An epoch change changes byte 28 and the CRC only.
    alink_spdif_src_t a = { .rate = 1, .sender = 1, .epoch = 255 }, b2 = a;
    b2.epoch = 0;
    uint8_t ba[48], bb[48];
    alink_spdif_sf_build(&a, 192, ba);
    alink_spdif_sf_build(&b2, 192, bb);
    for (unsigned i = 0; i < 48; i++)
        if (i != 30 && i != 31) CHECK((ba[i] != bb[i]) == (i == 28), "epoch byte %u", i);
    CHECK(memcmp(ba + 30, bb + 30, 2) != 0, "epoch changes the CRC");
}

int main(void) {
    printf("test_alink_spdif_codec\n");
    test_vectors();
    test_spec_examples();
    test_properties();
    if (failures) {
        printf("FAILED: %d\n", failures);
        return 1;
    }
    printf("  all OK\n");
    return 0;
}
