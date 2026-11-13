/*
 * sender_fanout.c - DSPi alink sender (WP LK4): fan-out (slaved) mode core
 *
 * See sender_fanout.h.  The input rules are those of LK1's reference
 * receiver (alink_protocol_spec.md 8.4-8.8): the comma is checked where the
 * frame length puts it, then 1 and 2 bits either side (a slip, re-aligned
 * without loss); a frame with no comma is decoded anyway (a flywheel
 * frame); a missing comma, a bad head or an HDR mismatch fails the frame,
 * and three failed frames in a row are a loss.  The host tests run the
 * output through LK1's decoder.
 */

#include "sender_fanout.h"

#include <string.h>

void snd_dectab_init(snd_dectab_t *d) {
    for (unsigned s = 0; s < 1024; s++) {
        uint8_t b;
        bool k;
        unsigned m = alink_8b10b_decode((uint16_t)s, &b, &k);
        d->dec[s] = (m && !k) ? (uint16_t)(SND_DEC_DATA | b) : 0;
    }
}

void snd_fo_init(snd_fo_t *f, unsigned in_cls, unsigned out_cls, const snd_tables_t *t,
                 const snd_dectab_t *d) {
    memset(f, 0, sizeof(*f));
    f->in_cls = (uint8_t)in_cls;
    f->out_cls = (uint8_t)out_cls;
    f->t = t;
    f->d = d;
    f->state = SND_FO_SEARCH;
}

uint32_t snd_fo_acquire_bits(unsigned in_cls) {
    return 10u * alink_frame_symbols_max(in_cls, ALINK_RATE_44K1) * 68u;
}

// ---------------------------------------------------------------------------
// Acquisition with the reference receiver
// ---------------------------------------------------------------------------

typedef struct {
    snd_fo_t *f;
    alink_rx_t *rx;
    uint64_t first_bit;
    bool done;
} acq_t;

static void acq_cb(void *ctx, const alink_rx_event_t *ev) {
    acq_t *a = ctx;
    if (a->done || ev->type != ALINK_RX_EV_FRAME || ev->flags) return;
    const alink_rx_t *rx = a->rx;
    // A RATE in flight: wait until it has happened, the layout is then plain.
    if (rx->apply_pending) return;
    uint64_t at = rx->p - 10u - rx->base;   // this frame's comma, still in the buffer
    uint16_t sym = 0;
    for (unsigned i = 0; i < 10u; i++, at++)
        sym = (uint16_t)((sym << 1) | ((rx->buf[at >> 5] >> (31u - (unsigned)(at & 31u))) & 1u));
    if (!alink_sym_is_comma(sym)) return;
    snd_fo_t *f = a->f;
    f->pos = a->first_bit + rx->p - 10u;
    f->frame = ev->frame;
    f->rate = rx->rate;
    f->channels = ev->channels;
    f->sf = rx->sf;
    f->sf_valid = true;
    alink_sf_pack(&rx->sf, f->last_sf);
    a->done = true;
}

bool snd_fo_acquire(snd_fo_t *f, const uint32_t *words, uint64_t first_bit, uint32_t nbits, alink_rx_t *rx) {
    acq_t a = { .f = f, .rx = rx, .first_bit = first_bit, .done = false };
    alink_rx_init(rx, f->in_cls, 0x01, acq_cb, &a);
    for (uint32_t at = 0; at < nbits && !a.done; at += 1024u) {
        uint32_t n = nbits - at < 1024u ? nbits - at : 1024u;
        alink_rx_push(rx, words + at / 32u, n);
    }
    if (!a.done) return false;
    f->state = SND_FO_LOCKED;
    f->fails = 0;
    f->rate_pending = false;
    f->sf_good = 0;
    f->out_on = false;
    f->st.lock_count++;
    // The superframe in progress started before this frame: its bytes are
    // incomplete until the next boundary, which is not an error.
    f->sf_skip = f->frame % ALINK_SF_LEN != 0;
    return true;
}

bool snd_fo_project(snd_fo_t *f, uint64_t from) {
    while (f->pos < from) {
        if (f->rate_pending) return false;
        f->pos += snd_fo_frame_bits(f->in_cls, f->rate, f->frame);
        f->frame++;
    }
    f->sf_good = 0;
    f->sf_skip = f->frame % ALINK_SF_LEN != 0;
    return true;
}

uint64_t snd_fo_need(const snd_fo_t *f) {
    return f->pos + snd_fo_frame_bits(f->in_cls, f->rate, f->frame) + 16u;
}

void snd_fo_start_output(snd_fo_t *f) {
    f->out_on = true;
}

// ---------------------------------------------------------------------------
// One frame
// ---------------------------------------------------------------------------

#define BANKS_MAX ALINK_MAX_BANKS

static void lose(snd_fo_t *f) {
    f->state = SND_FO_SEARCH;
    f->out_on = false;
    f->rate_pending = false;
    f->st.loss_count++;
}

// The re-sent superframe byte k of the current frame.
static uint8_t out_ctl(snd_fo_t *f, unsigned k) {
    if (k == 0) {
        f->out_crc = 0xFFFFu;
        f->out_taint = false;
    }
    uint8_t b;
    if (f->sf_good & (1u << k)) {
        b = f->sf_bytes[k];
    } else {
        b = f->last_sf[k];
        f->out_taint = true;
    }
    if (k == 5) {
        b = f->out_cls;
    } else if (k == 6) {
        b = b == 0xFFu ? 0xFFu : (uint8_t)(b + 1u);
    } else if (k == 30) {
        b = (uint8_t)f->out_crc;
    } else if (k == 31) {
        // Poison the CRC unless the received superframe was complete and
        // its own CRC matched, so nothing the receivers would have rejected
        // reaches them as valid.
        uint16_t rc = 0xFFFFu;
        for (unsigned i = 0; i < 30u; i++) rc = (uint16_t)((rc << 8) ^ f->t->crc16[(rc >> 8) ^ f->sf_bytes[i]]);
        bool good = f->sf_good == 0xFFFFFFFFu && rc == (uint16_t)(f->sf_bytes[30] | (f->sf_bytes[31] << 8));
        b = (uint8_t)(f->out_crc >> 8);
        if (!good || f->out_taint) b ^= 0xFFu;
    }
    if (k < 30) f->out_crc = (uint16_t)((f->out_crc << 8) ^ f->t->crc16[(f->out_crc >> 8) ^ b]);
    f->out_sf[k] = b;
    return b;
}

#define PUT(v, nb) SND_W_PUT(v, nb)

// The current frame's input words, copied once from the volatile ring: bit
// `o` of the copy is absolute bit base + o.  Largest frame 1400 bits plus
// the slip window and the next comma.
#define FW_WORDS 48u

static inline uint32_t fw_bits32(const uint32_t *fw, uint32_t o) {
    uint32_t wi = o >> 5, sh = o & 31u;
    return sh ? (fw[wi] << sh) | (fw[wi + 1u] >> (32u - sh)) : fw[wi];
}
#define ENC(b) (e = t->enc[rd][(b)], rd = e >> 15, (uint32_t)(e & 0x3FFu))

unsigned snd_fo_frame(snd_fo_t *f, const snd_ring_t *r, snd_sink_t *s, snd_log_t *log) {
    const snd_tables_t *t = f->t;
    const uint16_t *dec = f->d->dec;
    unsigned res = 0;
    if (f->state != SND_FO_LOCKED) return 0;

    // Copy the frame (and 32 bits either side) out of the ring.
    uint32_t fw[FW_WORDS];
    uint64_t base = (f->pos & ~(uint64_t)31u) - 32u;
    {
        uint32_t w0 = (uint32_t)(base >> 5);
        unsigned nw = (unsigned)((f->pos + snd_fo_frame_bits(f->in_cls, f->rate, f->frame) + 64u - base + 31u) >> 5);
        if (nw > FW_WORDS) nw = FW_WORDS;
        for (unsigned i = 0; i < nw; i++) fw[i] = r->ring[(w0 + i) & r->mask];
    }
    // Comma: in place, else 1 or 2 bits off (a slip), else a flywheel frame.
    uint64_t p = f->pos;
    uint32_t po = (uint32_t)(p - base);
    if (!alink_sym_is_comma((uint16_t)(fw_bits32(fw, po) >> 22))) {
        static const int8_t off[4] = { -1, 1, -2, 2 };
        int found = 0;
        for (unsigned i = 0; i < 4 && !found; i++)
            if (alink_sym_is_comma((uint16_t)(fw_bits32(fw, po + (uint32_t)(int32_t)off[i]) >> 22)))
                found = off[i];
        if (found) {
            p += (uint64_t)(int64_t)found;
            po += (uint32_t)(int32_t)found;
            res |= SND_FO_R_SLIP;
            f->st.bit_slips++;
        } else {
            res |= SND_FO_R_COMMA;
            f->st.comma_errors++;
        }
    }

    // Head.
    uint32_t w = fw_bits32(fw, po + 10u);
    uint16_t dh = dec[w >> 22], dc = dec[(w >> 12) & 0x3FFu], dk = dec[(w >> 2) & 0x3FFu];
    uint8_t hdr = (uint8_t)dh, ctl = (uint8_t)dc;
    bool head_ok = (dh & dc & dk & SND_DEC_DATA) && t->crc8[t->crc8[hdr] ^ ctl] == (uint8_t)dk;
    if (!head_ok) {
        res |= SND_FO_R_HEAD;
        f->st.crc8_errors++;
        f->st.code_errors += !(dh & SND_DEC_DATA) + !(dc & SND_DEC_DATA) + !(dk & SND_DEC_DATA);
    } else if (hdr != (uint8_t)f->frame) {
        res |= SND_FO_R_COUNTER;
        f->st.counter_errors++;
    }
    if (res & (SND_FO_R_COMMA | SND_FO_R_HEAD | SND_FO_R_COUNTER)) {
        if (++f->fails >= ALINK_RX_FAIL_LIMIT) {
            lose(f);
            return res | SND_FO_R_LOSS;
        }
    } else {
        f->fails = 0;
    }
    f->st.frames++;

    unsigned k = (unsigned)(f->frame % ALINK_SF_LEN);
    if (k == 0) {
        f->sf_good = 0;
        f->sf_skip = false;
    }
    if (!(res & (SND_FO_R_HEAD | SND_FO_R_COUNTER))) {
        f->sf_bytes[k] = ctl;
        f->sf_good |= 1u << k;
    }

    // Banks: data bytes and their CRC bytes as received.
    unsigned c = f->channels, nb = alink_bank_count(c);
    uint8_t bytes[3u * ALINK_MAX_CHANNELS + 2u * BANKS_MAX];
    uint8_t bad[BANKS_MAX];
    uint32_t q = po + 40u;
    unsigned o = 0;
    for (unsigned b = 0; b < nb; b++) {
        unsigned n = 3u * alink_bank_channels(c, b) + 2u, start = o;
        bad[b] = 0;
        for (unsigned i = 0; i < n;) {
            uint32_t v = fw_bits32(fw, q);
            unsigned m = n - i < 3u ? n - i : 3u;
            for (unsigned j = 0; j < m; j++, i++) {
                uint16_t e = dec[(v >> (22u - 10u * j)) & 0x3FFu];
                if (!(e & SND_DEC_DATA)) {
                    bad[b] = 1;
                    f->st.code_errors++;
                }
                bytes[o++] = (uint8_t)e;
            }
            q += 10u * m;
        }
        unsigned dn = n - 2u;
        uint16_t crc = 0xFFFFu;
        for (unsigned i = 0; i < dn; i++)
            crc = (uint16_t)((crc << 8) ^ t->crc16[(crc >> 8) ^ bytes[start + i]]);
        if (bad[b] || crc != (uint16_t)(bytes[start + dn] | (bytes[start + dn + 1u] << 8))) f->st.crc16_errors++;
        if (bad[b]) {
            // Make sure the downstream CRC check fails too.
            crc = (uint16_t)~crc;
            bytes[start + dn] = (uint8_t)crc;
            bytes[start + dn + 1u] = (uint8_t)(crc >> 8);
        }
    }

    // Superframe completion.
    if (k == ALINK_SF_LEN - 1u && !f->sf_skip) {
        int err = ALINK_RX_SF_INCOMPLETE;
        alink_sf_t v;
        if (f->sf_good == 0xFFFFFFFFu) {
            err = alink_sf_unpack(f->sf_bytes, &v);
            if (err == ALINK_SF_OK && v.cls != f->in_cls) err = ALINK_RX_SF_CLASS;
            if (err == ALINK_SF_OK && (v.rate != f->rate || v.channels != f->channels)) err = ALINK_SF_ERR_LAYOUT;
        }
        if (err == ALINK_SF_OK) {
            if (v.frame != (f->frame & ~(uint64_t)(ALINK_SF_LEN - 1u))) {
                f->st.counter_errors++;
                f->frame = v.frame + (ALINK_SF_LEN - 1u);   // a valid superframe corrects F
            }
            f->sf = v;
            f->sf_valid = true;
            memcpy(f->last_sf, f->sf_bytes, ALINK_SF_LEN);
            f->rate_pending = v.apply_op == ALINK_OP_RATE && v.apply_frame > f->frame;
            if (f->rate_pending) {
                f->rate_at = v.apply_frame;
                f->rate_new = v.apply_arg;
                f->c_new = v.apply_value > 0 ? (uint8_t)v.apply_value : f->channels;
            }
            res |= SND_FO_R_SF;
        } else {
            f->st.superframe_errors++;
            res |= SND_FO_R_SF_ERR;
        }
    }

    // Re-send.
    if (f->out_on) {
        unsigned osym = alink_frame_symbols(f->out_cls, f->rate, f->frame);
        if (log) snd_log_push(log, f->frame, snd_sink_bits(s));
        if (alink_layout_symbols(c) > osym) {
            f->out_unfit++;
            snd_put_idles(s, 10u * osym);
        } else {
            SND_W_BEGIN(s);
            unsigned rd = f->out_rd, e;
            uint8_t ohdr = (uint8_t)f->frame, octl = out_ctl(f, k);
            uint8_t ocrc = t->crc8[t->crc8[ohdr] ^ octl];
            e = t->comma[rd];
            rd = e >> 15;
            PUT(e & 0x3FFu, 10);
            uint32_t h0 = ENC(ohdr), h1 = ENC(octl), h2 = ENC(ocrc);
            PUT((h0 << 20) | (h1 << 10) | h2, 30);
            unsigned i = 0;
            for (; i + 3u <= o; i += 3u) {
                uint32_t s0 = ENC(bytes[i]), s1 = ENC(bytes[i + 1u]), s2 = ENC(bytes[i + 2u]);
                PUT((s0 << 20) | (s1 << 10) | s2, 30);
            }
            for (; i < o; i++) PUT(ENC(bytes[i]), 10);
            SND_W_END(s);
            snd_put_idles(s, 10u * (osym - alink_layout_symbols(c)));
            f->out_rd = rd;
            f->out_frames++;
        }
    }

    // Next frame (a RATE switches the layout after the frame before it).
    f->pos = p + snd_fo_frame_bits(f->in_cls, f->rate, f->frame);
    f->frame++;
    if (f->rate_pending && f->frame == f->rate_at) {
        f->rate = f->rate_new;
        f->channels = f->c_new;
        f->rate_pending = false;
    }
    return res;
}
