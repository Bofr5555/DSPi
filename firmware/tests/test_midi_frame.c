// Host test for midi_frame.c (USB MIDI control channel codec, contract C1).
// Built by run.sh: cc -std=c11 -Wall -Wextra -Werror -I ../DSPi test_midi_frame.c
#include "midi_frame.c"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; \
    fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
    fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

static const mf_mfr_id_t DEV_ID = { { 0x7D, 0, 0 }, 1 };

static uint32_t lcg = 12345u;
static uint8_t rnd8(void) { lcg = lcg * 1103515245u + 12345u; return (uint8_t)(lcg >> 16); }

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static size_t enc_all(mf_enc_t *e, uint8_t *out, size_t cap) {
    size_t n = 0, k;
    while ((k = mf_enc_read(e, out + n, cap - n > 5 ? 5 : cap - n)) > 0) n += k;
    return n;
}

// Encode a request into SysEx.
static size_t make_request(const mf_mfr_id_t *id, uint8_t tag, uint8_t type, uint8_t bReq,
                           uint16_t wv, uint16_t wi, uint16_t wl, const uint8_t *payload,
                           uint8_t *out, size_t cap) {
    mf_enc_t e;
    mf_enc_begin_request(&e, id, tag, type, bReq, wv, wi, wl, payload);
    return enc_all(&e, out, cap);
}

typedef struct {
    int requests, errors, headers;
    uint8_t last_err_status, last_err_tag, last_err_type;
    uint8_t buf[8192];
    bool discard;
    uint8_t deferred;
} sink_t;

static void feed_bytes(mf_dec_t *d, sink_t *s, const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) {
        mf_event_t ev = mf_dec_feed(d, p[i]);
        if (ev == MF_EV_SET_HEADER) {
            s->headers++;
            d->dst = s->discard ? NULL : s->buf;
            d->deferred_status = s->deferred;
        } else if (ev == MF_EV_REQUEST) {
            s->requests++;
        } else if (ev == MF_EV_ERROR) {
            s->errors++;
            s->last_err_status = d->err_status;
            s->last_err_tag = d->err_tag;
            s->last_err_type = d->err_type;
        }
    }
}

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// Extract "key": "hex" from a line into out; returns byte count or -1.
static long json_hex(const char *line, const char *key, uint8_t *out, size_t cap) {
    char pat[32];
    snprintf(pat, sizeof pat, "\"%s\": \"", key);
    const char *p = strstr(line, pat);
    if (!p) return -1;
    p += strlen(pat);
    size_t n = 0;
    while (p[0] && p[0] != '"') {
        int hi = hexval(p[0]), lo = hexval(p[1]);
        if (hi < 0 || lo < 0 || n >= cap) return -1;
        out[n++] = (uint8_t)(hi << 4 | lo);
        p += 2;
    }
    return (long)n;
}

static long json_int(const char *line, const char *key) {
    char pat[32];
    snprintf(pat, sizeof pat, "\"%s\": ", key);
    const char *p = strstr(line, pat);
    return p ? strtol(p + strlen(pat), NULL, 10) : -1;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void test_crc(void) {
    CHECK(mf_crc16((const uint8_t *)"123456789", 9) == 0x29B1, "CRC check value");
    CHECK(mf_crc16(NULL, 0) == 0xFFFF, "CRC of nothing");
}

static void roundtrip(size_t n) {
    static uint8_t raw[8192], packed[9400], back[8192];
    for (size_t i = 0; i < n; i++) raw[i] = rnd8();
    size_t pl = mf_pack(raw, n, packed);
    CHECK(pl == mf_packed_len(n), "packed len %zu for %zu", pl, n);
    CHECK(pl == (n / 7) * 8 + (n % 7 ? n % 7 + 1 : 0), "formula %zu", n);
    for (size_t i = 0; i < pl; i++) CHECK(packed[i] < 0x80, "7-bit clean at %zu (n=%zu)", i, n);
    long ul = mf_unpack(packed, pl, back);
    CHECK(ul == (long)n && memcmp(raw, back, n) == 0, "round trip n=%zu", n);
}

static void test_pack(void) {
    for (size_t n = 0; n <= 300; n++) roundtrip(n);
    roundtrip(6147);
    roundtrip(6146);
    roundtrip(6143);

    // Worked example (spec): 7 bytes with every MSB pattern bit used once.
    const uint8_t raw[8] = { 0x81, 0x02, 0x83, 0x04, 0x85, 0x06, 0x87, 0xFF };
    uint8_t out[16];
    size_t n = mf_pack(raw, 8, out);
    const uint8_t want[11] = { 0x55, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x01, 0x7F };
    CHECK(n == 10 && memcmp(out, want, 10) == 0, "pack worked example");

    uint8_t back[16];
    const uint8_t lone[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00 };
    CHECK(mf_unpack(lone, sizeof lone, back) == -1, "lone MSB byte rejected");
    const uint8_t hi[] = { 0x00, 0x81 };
    CHECK(mf_unpack(hi, sizeof hi, back) == -1, "data byte >= 0x80 rejected");
    const uint8_t extra[] = { 0x02, 0x11 };     // MSB bit 1 set, only one byte
    CHECK(mf_unpack(extra, sizeof extra, back) == -1, "unused MSB bit rejected");
}

static void test_vectors(void) {
    char path[1024];
    snprintf(path, sizeof path, "%s", __FILE__);
    char *slash = strrchr(path, '/');
    if (slash) strcpy(slash + 1, "midi_vectors.json");
    else strcpy(path, "midi_vectors.json");
    FILE *f = fopen(path, "r");
    CHECK(f != NULL, "open %s", path);
    if (!f) return;

    static char line[65536];
    static uint8_t frame[8192], sysex[9000], got[9000], mfrb[3];
    int count = 0;
    while (fgets(line, sizeof line, f)) {
        if (!strstr(line, "\"frame\"")) continue;
        char name[64] = "?";
        const char *np = strstr(line, "\"name\": \"");
        if (np) sscanf(np + 9, "%63[^\"]", name);
        long fl = json_hex(line, "frame", frame, sizeof frame);
        long sl = json_hex(line, "sysex", sysex, sizeof sysex);
        long ml = json_hex(line, "mfr", mfrb, sizeof mfrb);
        long tag = json_int(line, "tag");
        CHECK(fl > 0 && sl > 0 && (ml == 1 || ml == 3) && tag >= 0, "%s: parse", name);
        if (fl <= 0 || sl <= 0 || (ml != 1 && ml != 3) || tag < 0) continue;
        mf_mfr_id_t id = { { mfrb[0], mfrb[1], mfrb[2] }, (uint8_t)ml };

        // Encoder: frame -> exactly the vector's SysEx.
        // mf_enc_begin appends its own CRC: give it the frame without the CRC
        // (the first <= 9 bytes as the frame head, the rest as payload).
        mf_enc_t e;
        uint8_t hl = (uint8_t)((fl - 2) < 9 ? (fl - 2) : 9);
        mf_enc_begin(&e, &id, (uint8_t)tag, frame, hl,
                     (fl - 2) > 9 ? frame + 9 : NULL,
                     (fl - 2) > 9 ? (uint32_t)(fl - 2 - 9) : 0);
        size_t gl = enc_all(&e, got, sizeof got);
        CHECK(gl == (size_t)sl && memcmp(got, sysex, gl) == 0, "%s: encoder matches vector", name);
        CHECK(gl == mf_sysex_len(&id, (size_t)fl), "%s: sysex length", name);

        // The frame's CRC field matches our CRC.
        uint16_t crc = mf_crc16(frame, (size_t)fl - 2);
        CHECK(frame[fl - 2] == (crc & 0xFF) && frame[fl - 1] == (crc >> 8), "%s: CRC", name);

        // Unpacking the SysEx body gives the frame back.
        size_t hdr = mf_header_len(&id);
        long ul = mf_unpack(sysex + hdr, (size_t)sl - hdr - 1, got);
        CHECK(ul == fl && memcmp(got, frame, (size_t)fl) == 0, "%s: unpack", name);

        if (strstr(line, "\"dir\": \"h2d\"")) {
            mf_dec_t d;
            mf_dec_init(&d, &id);
            sink_t s;
            memset(&s, 0, sizeof s);
            feed_bytes(&d, &s, sysex, (size_t)sl);
            CHECK(s.requests == 1 && s.errors == 0, "%s: decodes as one request", name);
            CHECK(d.tag == tag, "%s: tag", name);
            CHECK(d.is_get == (frame[0] == MF_TYPE_GET) && d.bReq == frame[1], "%s: fields", name);
            uint16_t wl = (uint16_t)(frame[6] | frame[7] << 8);
            CHECK(d.wLength == wl, "%s: wLength", name);
            if (!d.is_get && wl)
                CHECK(memcmp(s.buf, frame + 8, wl) == 0, "%s: payload", name);
        }
        count++;
    }
    fclose(f);
    CHECK(count >= 10, "only %d vectors", count);
}

static void test_worked_example(void) {
    // Spec worked example: GET 0x7F (platform), tag 0x01, wLength 0.
    static const uint8_t want[] = {
        0xF0, 0x7D, 'D', 'S', 'P', 0x01, 0x01,
        0x00, 0x02, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x04, 0x00, 0x10, 0x30,
        0xF7,
    };
    uint8_t out[64];
    size_t n = make_request(&DEV_ID, 0x01, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, out, sizeof out);
    CHECK(n == sizeof want && memcmp(out, want, n) == 0, "GET 0x7F worked example");
}

static void test_sizes(void) {
    // Largest messages: SET 0xA1 (6136 B) and the GET 0xA0 reply.  Frames
    // carry no sync byte: request 8 + 6136 + 2 = 6146 frame bytes -> 7024
    // packed + 7 header + F7 = 7032; reply 4 + 6136 + 2 = 6142 -> 7020 + 8
    // = 7028.  (WORK-PACKAGES.md counted the 0xA5 sync byte: 7034 / 7029.)
    CHECK(mf_sysex_len(&DEV_ID, 8 + 6136 + 2) == 7032, "0xA1 request = 7032 bytes");
    CHECK(mf_sysex_len(&DEV_ID, 4 + 6136 + 2) == 7028, "0xA0 response = 7028 bytes");

    static uint8_t bulk[6136], out[8000];
    for (size_t i = 0; i < sizeof bulk; i++) bulk[i] = rnd8();
    mf_enc_t e;
    mf_enc_begin_response(&e, &DEV_ID, 9, MF_RESP_GET, MF_STATUS_OK, bulk, sizeof bulk);
    size_t n = enc_all(&e, out, sizeof out);
    CHECK(n == 7028, "0xA0 reply encodes to %zu", n);
    size_t hdr = mf_header_len(&DEV_ID);
    static uint8_t frame[6200];
    long fl = mf_unpack(out + hdr, n - hdr - 1, frame);
    CHECK(fl == 6142 && frame[0] == MF_RESP_GET && frame[1] == 0 &&
          frame[2] == (6136 & 0xFF) && frame[3] == (6136 >> 8) &&
          memcmp(frame + 4, bulk, 6136) == 0, "0xA0 reply frame");
    uint16_t crc = mf_crc16(frame, 6140);
    CHECK(frame[6140] == (crc & 0xFF) && frame[6141] == (crc >> 8), "0xA0 reply CRC");

    // A non-OK status carries no payload.
    mf_enc_begin_response(&e, &DEV_ID, 9, MF_RESP_GET, 0x04, bulk, sizeof bulk);
    n = enc_all(&e, out, sizeof out);
    CHECK(n == mf_sysex_len(&DEV_ID, 6), "error reply has no payload");
}

// The largest (7032-byte) request, fed through USB-MIDI packets and through the
// decoder in every piece size 1..64 bytes (pieces never align with packets
// or groups for most sizes).
static void test_streaming(void) {
    static uint8_t bulk[6136], sysex[7100];
    for (size_t i = 0; i < sizeof bulk; i++) bulk[i] = rnd8();
    size_t n = make_request(&DEV_ID, 0x33, MF_TYPE_SET, 0xA1, 0, 0, 6136, bulk, sysex, sizeof sysex);
    CHECK(n == 7032, "bulk request length %zu", n);

    for (size_t piece = 1; piece <= 64; piece++) {
        mf_dec_t d;
        mf_dec_init(&d, &DEV_ID);
        static sink_t s;
        memset(&s, 0, sizeof s);
        for (size_t off = 0; off < n; off += piece) {
            size_t k = (n - off < piece) ? n - off : piece;
            feed_bytes(&d, &s, sysex + off, k);
            if (off + k < n) CHECK(s.requests == 0, "early request at %zu piece %zu", off, piece);
        }
        CHECK(s.headers == 1 && s.requests == 1 && s.errors == 0, "piece %zu: events", piece);
        CHECK(memcmp(s.buf, bulk, 6136) == 0 && d.tag == 0x33 && d.bReq == 0xA1, "piece %zu: payload", piece);
    }

    // Through USB-MIDI packets: encoder -> packets -> mf_packet_bytes -> decoder.
    mf_enc_t e;
    mf_enc_begin_request(&e, &DEV_ID, 0x34, MF_TYPE_SET, 0xA1, 0, 0, 6136, bulk);
    mf_dec_t d;
    mf_dec_init(&d, &DEV_ID);
    static sink_t s;
    memset(&s, 0, sizeof s);
    uint8_t pkt[4], b[3];
    size_t packets = 0, total = 0;
    while (mf_enc_packet(&e, 0, pkt)) {
        uint8_t cin = pkt[0] & 0x0F;
        uint8_t k = mf_packet_bytes(pkt, 0, b);
        total += k;
        packets++;
        if (total < 7032) CHECK(cin == MF_CIN_SYSEX_START && k == 3, "mid packet CIN %u", cin);
        else CHECK(cin == MF_CIN_SYSEX_END1 + (k - 1) && b[k - 1] == MF_SYSEX_END, "end packet");
        for (uint8_t i = k; i < 3; i++) CHECK(pkt[1 + i] == 0, "padding zero");
        feed_bytes(&d, &s, b, k);
    }
    CHECK(total == 7032 && packets == (7032 + 2) / 3, "packets %zu bytes %zu", packets, total);
    CHECK(s.requests == 1 && memcmp(s.buf, bulk, 6136) == 0, "packet path payload");

    // End-packet CIN for every remainder: sysex lengths 3k, 3k+1, 3k+2.
    for (uint16_t pl = 0; pl < 3; pl++) {
        mf_enc_begin_request(&e, &DEV_ID, 1, MF_TYPE_SET, 0x20, 0, 0, pl, bulk);
        uint8_t last = 0;
        size_t cnt = 0;
        while (mf_enc_packet(&e, 0, pkt)) { last = pkt[0]; cnt += mf_packet_bytes(pkt, 0, b); }
        uint8_t want = (uint8_t)(MF_CIN_SYSEX_END1 + (cnt - 1) % 3);
        CHECK((last & 0x0F) == want, "end CIN for len %zu", cnt);
    }

    // Cable filter and non-SysEx CINs.
    const uint8_t note_on[4] = { 0x09, 0x90, 0x40, 0x7F };
    const uint8_t cable1[4] = { 0x14, 0xF0, 0x7D, 0x44 };
    const uint8_t single[4] = { 0x0F, 0xF8, 0x00, 0x00 };
    CHECK(mf_packet_bytes(note_on, 0, b) == 0, "note-on ignored");
    CHECK(mf_packet_bytes(cable1, 0, b) == 0, "cable 1 ignored");
    CHECK(mf_packet_bytes(single, 0, b) == 1 && b[0] == 0xF8, "single byte passed");
}

static void test_header_and_malformed(void) {
    uint8_t m[256];
    size_t n;
    mf_dec_t d;
    sink_t s;
#define RESET() do { mf_dec_init(&d, &DEV_ID); memset(&s, 0, sizeof s); } while (0)

    // Good header.
    RESET();
    n = make_request(&DEV_ID, 0x05, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.requests == 1 && s.errors == 0 && d.tag == 5, "good");

    // Wrong manufacturer: silently ignored, decoder stays usable.
    RESET();
    mf_mfr_id_t other = { { 0x7E, 0, 0 }, 1 };
    n = make_request(&other, 0x05, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.requests == 0 && s.errors == 0 && !mf_dec_busy(&d), "wrong mfr ignored");
    n = make_request(&DEV_ID, 0x06, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.requests == 1 && d.tag == 6, "next message after foreign one");

    // Same mfr, other product magic: ignored.
    RESET();
    const uint8_t other_magic[] = { 0xF0, 0x7D, 'X', 'Y', 'Z', 1, 1, 0, 0, 0xF7 };
    feed_bytes(&d, &s, other_magic, sizeof other_magic);
    CHECK(s.requests == 0 && s.errors == 0, "other magic ignored");

    // 3-byte ID device rejects a 1-byte ID message that shares the first byte.
    RESET();
    mf_mfr_id_t id3 = { { 0x00, 0x21, 0x7E }, 3 };
    mf_dec_init(&d, &id3);
    n = make_request(&id3, 0x07, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.requests == 1, "3-byte id accepted");
    mf_mfr_id_t id3b = { { 0x00, 0x21, 0x7D }, 3 };
    n = make_request(&id3b, 0x07, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.requests == 1 && s.errors == 0, "3-byte id mismatch ignored");

    // Wrong version: FRAME_ERROR, tag 0x7F (tag not read yet), type 0x81.
    RESET();
    n = make_request(&DEV_ID, 0x05, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    m[5] = 0x02;
    feed_bytes(&d, &s, m, n);
    CHECK(s.requests == 0 && s.errors == 1 && s.last_err_status == MF_STATUS_FRAME_ERROR &&
          s.last_err_tag == MF_TAG_UNKNOWN && s.last_err_type == MF_RESP_SET, "wrong version");

    // Host tag 0 is reserved for notifications.
    RESET();
    n = make_request(&DEV_ID, 0x00, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.errors == 1 && s.last_err_tag == MF_TAG_UNKNOWN, "tag 0 refused");

    // Truncated header: F7 right after the magic -> error; inside the mfr -> silent.
    RESET();
    const uint8_t trunc1[] = { 0xF0, 0x7D, 'D', 'S', 'P', 0xF7 };
    feed_bytes(&d, &s, trunc1, sizeof trunc1);
    CHECK(s.errors == 1 && s.last_err_tag == MF_TAG_UNKNOWN, "truncated after magic");
    RESET();
    const uint8_t trunc2[] = { 0xF0, 0x7D, 'D', 0xF7 };
    feed_bytes(&d, &s, trunc2, sizeof trunc2);
    CHECK(s.errors == 0 && s.requests == 0, "truncated in magic: silent");
    RESET();
    const uint8_t trunc3[] = { 0xF0, 0x7D, 'D', 'S', 'P', 0x01, 0x09, 0xF7 };
    feed_bytes(&d, &s, trunc3, sizeof trunc3);
    CHECK(s.errors == 1 && s.last_err_tag == 0x09 && s.last_err_status == MF_STATUS_FRAME_ERROR,
          "no frame at all");

    // Truncated frame (F7 early) -> FRAME_ERROR with the tag.
    RESET();
    n = make_request(&DEV_ID, 0x0A, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    m[n - 3] = 0xF7;
    feed_bytes(&d, &s, m, n - 2);
    CHECK(s.errors == 1 && s.last_err_tag == 0x0A && s.last_err_status == MF_STATUS_FRAME_ERROR &&
          s.last_err_type == MF_RESP_GET, "truncated frame");

    // CRC error -> CRC_ERROR with the tag, type by request type.
    RESET();
    n = make_request(&DEV_ID, 0x0B, MF_TYPE_SET, 0xD2, 0, 0, 4, (const uint8_t *)"\x00\x00\xa0\xc1", m, sizeof m);
    m[12] ^= 0x01;   // a data byte inside the packed frame
    feed_bytes(&d, &s, m, n);
    CHECK(s.errors == 1 && s.last_err_status == MF_STATUS_CRC_ERROR && s.last_err_tag == 0x0B &&
          s.last_err_type == MF_RESP_SET, "CRC error");

    // Bad frame type -> FRAME_ERROR, type 0x81.
    RESET();
    n = make_request(&DEV_ID, 0x0C, 0x40, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.errors == 1 && s.last_err_status == MF_STATUS_FRAME_ERROR && s.last_err_tag == 0x0C &&
          s.last_err_type == MF_RESP_SET, "bad type");

    // Trailing bytes after the CRC -> FRAME_ERROR.
    RESET();
    {
        uint8_t raw[16] = { 0x02, 0x7F, 0, 0, 0, 0, 0, 0 };
        uint16_t c = mf_crc16(raw, 8);
        raw[8] = (uint8_t)c; raw[9] = (uint8_t)(c >> 8); raw[10] = 0x55;
        uint8_t q[64] = { 0xF0, 0x7D, 'D', 'S', 'P', 0x01, 0x0D };
        size_t ql = 7 + mf_pack(raw, 11, q + 7);
        q[ql++] = 0xF7;
        feed_bytes(&d, &s, q, ql);
        CHECK(s.errors == 1 && s.requests == 0 && s.last_err_tag == 0x0D, "trailing byte");
    }

    // Byte >= 0x80 that is not F0/F7 inside a message (system common).
    RESET();
    n = make_request(&DEV_ID, 0x0E, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, 10);
    const uint8_t f6 = 0xF6;
    feed_bytes(&d, &s, &f6, 1);
    CHECK(s.errors == 1 && s.last_err_tag == 0x0E && !mf_dec_busy(&d), "system common aborts");

    // Real-time bytes interleaved anywhere are ignored.
    RESET();
    n = make_request(&DEV_ID, 0x0F, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    for (size_t i = 0; i < n; i++) {
        const uint8_t rt = 0xF8;
        feed_bytes(&d, &s, &rt, 1);
        feed_bytes(&d, &s, &m[i], 1);
    }
    CHECK(s.requests == 1 && s.errors == 0, "real-time interleave");

    // F0 in the middle: error for the interrupted one, the new one decodes.
    RESET();
    n = make_request(&DEV_ID, 0x10, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, 12);
    size_t n2 = make_request(&DEV_ID, 0x11, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n2);
    CHECK(s.errors == 1 && s.last_err_tag == 0x10 && s.requests == 1 && d.tag == 0x11, "F0 restart");

    // Lone MSB byte at the end of the packed data (14-byte frame = 2 full
    // groups, then one extra byte before F7).
    RESET();
    n = make_request(&DEV_ID, 0x12, MF_TYPE_SET, 0xD2, 0, 0, 4, (const uint8_t *)"\x00\x00\xa0\xc1", m, sizeof m);
    CHECK(n == 24, "14-byte frame packs to 16");
    CHECK(mf_dec_feed(&d, 0) == MF_EV_NONE, "idle data byte ignored");
    m[n - 1] = 0x00;
    m[n] = 0xF7;
    feed_bytes(&d, &s, m, n + 1);
    CHECK(s.errors == 1 && s.last_err_tag == 0x12 && s.requests == 0 &&
          s.last_err_status == MF_STATUS_FRAME_ERROR, "lone MSB byte");

    // Unused MSB bit set in a final partial group.
    RESET();
    n = make_request(&DEV_ID, 0x13, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    // last group: index n-5 is its MSB byte (3 data bytes follow)
    m[n - 5] |= 0x40;
    feed_bytes(&d, &s, m, n);
    CHECK(s.errors == 1 && s.last_err_tag == 0x13, "unused MSB bit");

    // SET header: payload discarded on request; deferred status survives F7.
    RESET();
    s.discard = true;
    s.deferred = 0x06;
    uint8_t big[100];
    memset(big, 0xAB, sizeof big);
    n = make_request(&DEV_ID, 0x14, MF_TYPE_SET, 0x20, 0, 0, 100, big, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.headers == 1 && s.requests == 1 && d.deferred_status == 0x06 && s.buf[0] == 0,
          "discarded payload, deferred status");

    // Zero-length SET: no header event, request straight away.
    RESET();
    n = make_request(&DEV_ID, 0x15, MF_TYPE_SET, 0xD4, 0x0102, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.headers == 0 && s.requests == 1 && d.wValue == 0x0102 && !d.is_get, "wValue-only SET");

    // Garbage outside messages and a stray F7 are ignored.
    RESET();
    const uint8_t junk[] = { 0x12, 0x34, 0xF7, 0x7F, 0xF7 };
    feed_bytes(&d, &s, junk, sizeof junk);
    CHECK(s.errors == 0 && s.requests == 0 && !mf_dec_busy(&d), "junk ignored");

    // mf_dec_reset drops a partial message without an event.
    RESET();
    n = make_request(&DEV_ID, 0x16, MF_TYPE_SET, 0x20, 0, 0, 40, big, m, sizeof m);
    feed_bytes(&d, &s, m, 30);
    CHECK(mf_dec_in_payload(&d), "in payload");
    mf_dec_reset(&d);
    CHECK(!mf_dec_busy(&d), "reset idles");
    feed_bytes(&d, &s, m + 30, n - 30);
    CHECK(s.requests == 0 && s.errors == 0, "tail after reset ignored");
#undef RESET
}

static void test_random_garbage(void) {
    // Random bytes must never crash or write past dst, and the decoder must
    // recover for the next good message.
    static uint8_t junk[20000];
    for (size_t i = 0; i < sizeof junk; i++) {
        uint8_t r = rnd8();
        junk[i] = (r & 0x10) ? (uint8_t)(r & 0x7F) : (r > 0xF0 ? r : (uint8_t)(r & 0x7F));
        if (i % 97 == 0) junk[i] = 0xF0;
        if (i % 61 == 0) junk[i] = 0xF7;
    }
    mf_dec_t d;
    mf_dec_init(&d, &DEV_ID);
    static sink_t s;
    memset(&s, 0, sizeof s);
    feed_bytes(&d, &s, junk, sizeof junk);
    uint8_t m[64];
    const uint8_t f7 = 0xF7;
    feed_bytes(&d, &s, &f7, 1);
    int before = s.requests;
    size_t n = make_request(&DEV_ID, 0x20, MF_TYPE_GET, 0x7F, 0, 0, 0, NULL, m, sizeof m);
    feed_bytes(&d, &s, m, n);
    CHECK(s.requests == before + 1 && d.tag == 0x20, "recovers after garbage");
}

int main(void) {
    test_crc();
    test_pack();
    test_worked_example();
    test_vectors();
    test_sizes();
    test_streaming();
    test_header_and_malformed();
    test_random_garbage();
    if (failures) {
        fprintf(stderr, "test_midi_frame: %d failure(s)\n", failures);
        return 1;
    }
    printf("test_midi_frame: OK\n");
    return 0;
}
