/*
 * alink_decoder.c - DSPi alink protocol v1: reference receiver
 *
 * See alink_decoder.h and Documentation/Features/alink_protocol_spec.md
 * section 8.  Every step below runs only once all the bits it reads are
 * present, and reads nothing beyond them, which is what makes the output
 * independent of how the input was chunked.
 */

#include "alink_decoder.h"

#include <string.h>

#define BUF_BITS ((uint64_t)ALINK_RX_BUF_WORDS * 32u)

static void emit(alink_rx_t *rx, alink_rx_event_t *ev) {
    if (rx->cb) rx->cb(rx->ctx, ev);
}

static void emit_simple(alink_rx_t *rx, alink_rx_event_type_t type) {
    alink_rx_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = type;
    ev.frame = rx->frame;
    emit(rx, &ev);
}

// ---------------------------------------------------------------------------
// Bit access
// ---------------------------------------------------------------------------

// The 10 bits [end - 10, end) as a symbol, first bit in bit 9.
static uint16_t sym_at(const alink_rx_t *rx, uint64_t end) {
    uint16_t s = 0;
    for (uint64_t pos = end - 10u; pos < end; pos++) {
        uint64_t i = pos - rx->base;
        s = (uint16_t)((s << 1) | ((rx->buf[i >> 5] >> (31u - (unsigned)(i & 31u))) & 1u));
    }
    return s;
}

// A data code group ending at `end`; counts a code error when asked to.
static bool data_at(alink_rx_t *rx, uint64_t end, uint8_t *byte, bool count) {
    bool k;
    if (alink_8b10b_decode(sym_at(rx, end), byte, &k) && !k) return true;
    if (count) rx->stats.code_errors++;
    return false;
}

// HDR, CTL and CRC8 after the comma ending at p.  Every bad symbol is counted.
static bool head_at(alink_rx_t *rx, uint64_t p, uint8_t *hdr, uint8_t *ctl, bool count) {
    uint8_t b[3];
    bool ok = true;
    for (unsigned i = 0; i < 3; i++)
        if (!data_at(rx, p + 10u * (i + 1u), &b[i], count)) ok = false;
    if (!ok || alink_crc8(b, 2) != b[2]) return false;
    *hdr = b[0];
    *ctl = b[1];
    return true;
}

// ---------------------------------------------------------------------------
// State changes
// ---------------------------------------------------------------------------

static void clear_frame_state(alink_rx_t *rx) {
    rx->fails = 0;
    rx->prev_hdr = -1;
    rx->sf_count = 0;
    rx->apply_pending = false;
    memset(rx->bank_fails, 0, sizeof(rx->bank_fails));
    memset(rx->samples, 0, sizeof(rx->samples));
}

static void enter_search(alink_rx_t *rx, uint64_t q) {
    rx->state = ALINK_RX_SEARCH;
    rx->q = q;
    clear_frame_state(rx);
}

static void loss(alink_rx_t *rx) {
    if (rx->state == ALINK_RX_LOCKED) rx->stats.loss_count++;
    emit_simple(rx, ALINK_RX_EV_LOSS);
    enter_search(rx, rx->p + 1u);
}

static void sf_collect(alink_rx_t *rx, unsigned idx, uint8_t ctl, uint8_t hdr) {
    bool cont = rx->prev_hdr >= 0 && hdr == (uint8_t)(rx->prev_hdr + 1);
    if (idx == 0) {
        rx->sf_bytes[0] = ctl;
        rx->sf_count = 1;
    } else if (cont && rx->sf_count == idx) {
        rx->sf_bytes[idx] = ctl;
        rx->sf_count++;
    } else {
        rx->sf_count = 0;
    }
    rx->prev_hdr = hdr;
}

static void sf_error(alink_rx_t *rx, int err) {
    alink_rx_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = ALINK_RX_EV_SF_ERROR;
    ev.frame = rx->frame;
    ev.sf_error = err;
    rx->stats.superframe_errors++;
    emit(rx, &ev);
}

// Unpack the assembled superframe and check it against this receiver.
static int sf_check(alink_rx_t *rx, alink_sf_t *sf) {
    int err = alink_sf_unpack(rx->sf_bytes, sf);
    if (err == ALINK_SF_OK && sf->cls != rx->cls) err = ALINK_RX_SF_CLASS;
    return err;
}

static void adopt_sf(alink_rx_t *rx, const alink_sf_t *sf) {
    rx->sf = *sf;
    rx->rate = sf->rate;
    rx->channels = sf->channels;
    rx->apply_pending = sf->apply_op == ALINK_OP_RATE && sf->apply_frame > rx->frame;
}

// ---------------------------------------------------------------------------
// Steps
// ---------------------------------------------------------------------------

static bool step_search(alink_rx_t *rx) {
    if (rx->q < 10u) rx->q = 10u;
    while (rx->q + 30u <= rx->avail) {
        uint8_t hdr, ctl;
        if (alink_sym_is_comma(sym_at(rx, rx->q)) && head_at(rx, rx->q, &hdr, &ctl, false)) {
            rx->state = ALINK_RX_ALIGNED;
            rx->p = rx->q;
            rx->body_done = false;
            rx->comma_ok = true;
            rx->slip = 0;
            clear_frame_state(rx);
            alink_rx_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = ALINK_RX_EV_ALIGN;
            ev.bitpos = rx->p;
            emit(rx, &ev);
            return true;
        }
        rx->q++;
    }
    return false;
}

// ALIGNED: the head only; a valid superframe moves to LOCKED.
static bool body_aligned(alink_rx_t *rx) {
    if (rx->p + 30u > rx->avail) return false;
    uint8_t hdr, ctl;
    if (!head_at(rx, rx->p, &hdr, &ctl, true)) {
        rx->stats.crc8_errors++;
        rx->prev_hdr = -1;
        rx->sf_count = 0;
        if (++rx->fails >= ALINK_RX_FAIL_LIMIT) { loss(rx); return true; }
        rx->body_done = true;
        return true;
    }
    rx->fails = 0;
    unsigned idx = hdr & (ALINK_SF_LEN - 1u);
    sf_collect(rx, idx, ctl, hdr);
    if (idx == ALINK_SF_LEN - 1u && rx->sf_count == ALINK_SF_LEN) {
        alink_sf_t sf;
        int err = sf_check(rx, &sf);
        if (err == ALINK_SF_OK && (uint8_t)(sf.frame + ALINK_SF_LEN - 1u) != hdr)
            err = ALINK_RX_SF_COUNTER;
        if (err != ALINK_SF_OK) {
            sf_error(rx, err);
        } else {
            rx->state = ALINK_RX_LOCKED;
            rx->frame = sf.frame + ALINK_SF_LEN - 1u;
            adopt_sf(rx, &sf);
            memset(rx->bank_fails, 0, sizeof(rx->bank_fails));
            memset(rx->samples, 0, sizeof(rx->samples));
            rx->stats.lock_count++;
            alink_rx_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = ALINK_RX_EV_LOCK;
            ev.frame = rx->frame;
            ev.sf = &rx->sf;
            emit(rx, &ev);
        }
    }
    rx->body_done = true;
    return true;
}

static void decode_banks(alink_rx_t *rx, uint8_t status[ALINK_MAX_BANKS]) {
    unsigned c = rx->channels;
    memset(status, ALINK_BANK_SKIPPED, ALINK_MAX_BANKS);
    for (unsigned b = 0; b < alink_bank_count(c); b++) {
        unsigned k = alink_bank_channels(c, b), n = 3u * k;
        int32_t *out = rx->samples + ALINK_BANK_CHANNELS * b;
        if (!((rx->bank_mask >> b) & 1u)) {
            memset(out, 0, k * sizeof(int32_t));
            continue;
        }
        uint8_t bytes[3u * ALINK_BANK_CHANNELS + 2u];
        bool ok = true;
        unsigned s0 = alink_bank_symbol(b);
        for (unsigned i = 0; i < n + 2u; i++)
            if (!data_at(rx, rx->p + 10u * (s0 + i), &bytes[i], true)) ok = false;
        if (ok) ok = (uint16_t)(bytes[n] | (bytes[n + 1u] << 8)) == alink_crc16(bytes, n);
        if (ok) {
            status[b] = ALINK_BANK_OK;
            rx->bank_fails[b] = 0;
            for (unsigned i = 0; i < k; i++) out[i] = alink_get_s24(bytes + 3u * i);
        } else {
            rx->stats.crc16_errors++;
            if (rx->bank_fails[b] < 0xFFu) rx->bank_fails[b]++;
            if (rx->bank_fails[b] == 1u) {
                status[b] = ALINK_BANK_HOLD;    // samples keep the last frame's values
            } else {
                status[b] = ALINK_BANK_ZERO;
                memset(out, 0, k * sizeof(int32_t));
            }
        }
    }
    for (unsigned ch = c; ch < ALINK_MAX_CHANNELS; ch++) rx->samples[ch] = 0;
}

// LOCKED: head, banks, padding, superframe, failure count.
static bool body_locked(alink_rx_t *rx) {
    unsigned s = alink_frame_symbols(rx->cls, rx->rate, rx->frame);
    if (rx->p + 10u * (s - 1u) > rx->avail) return false;

    alink_rx_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = ALINK_RX_EV_FRAME;
    ev.frame = rx->frame;
    ev.slip = rx->slip;
    if (!rx->comma_ok) ev.flags |= ALINK_RXF_COMMA_MISSING;
    if (rx->slip) ev.flags |= ALINK_RXF_SLIP;

    uint8_t hdr, ctl;
    bool good = head_at(rx, rx->p, &hdr, &ctl, true);
    if (!good) {
        rx->stats.crc8_errors++;
        ev.flags |= ALINK_RXF_HEAD_BAD;
    } else if (hdr != (uint8_t)rx->frame) {
        rx->stats.counter_errors++;
        ev.flags |= ALINK_RXF_COUNTER;
        good = false;
    }

    decode_banks(rx, ev.bank_status);
    for (unsigned i = alink_layout_symbols(rx->channels); i < s; i++)
        if (sym_at(rx, rx->p + 10u * i) != ALINK_SYM_IDLE) rx->stats.idle_errors++;

    rx->stats.frames++;
    ev.channels = rx->channels;
    ev.samples = rx->samples;
    emit(rx, &ev);

    unsigned idx = (unsigned)(rx->frame & (ALINK_SF_LEN - 1u));
    if (good) {
        sf_collect(rx, idx, ctl, hdr);
    } else {
        rx->sf_count = 0;
        rx->prev_hdr = -1;
    }
    if (idx == ALINK_SF_LEN - 1u) {
        alink_sf_t sf;
        int err = rx->sf_count == ALINK_SF_LEN ? sf_check(rx, &sf) : ALINK_RX_SF_INCOMPLETE;
        if (err != ALINK_SF_OK) {
            sf_error(rx, err);
        } else {
            if (sf.frame + ALINK_SF_LEN - 1u != rx->frame) {
                rx->stats.counter_errors++;
                rx->frame = sf.frame + ALINK_SF_LEN - 1u;
            }
            adopt_sf(rx, &sf);
            alink_rx_event_t sev;
            memset(&sev, 0, sizeof(sev));
            sev.type = ALINK_RX_EV_SUPERFRAME;
            sev.frame = rx->frame;
            sev.sf = &rx->sf;
            emit(rx, &sev);
        }
    }

    if (ev.flags & (ALINK_RXF_COMMA_MISSING | ALINK_RXF_HEAD_BAD | ALINK_RXF_COUNTER)) {
        if (++rx->fails >= ALINK_RX_FAIL_LIMIT) { loss(rx); return true; }
    } else {
        rx->fails = 0;
    }
    rx->body_done = true;
    return true;
}

// ALIGNED: the next comma at a symbol boundary within the longest frame.
static bool next_aligned(alink_rx_t *rx) {
    unsigned maxgap = alink_frame_symbols_max(rx->cls, ALINK_RATE_44K1);
    for (unsigned i = ALINK_HEAD_SYMBOLS; i <= maxgap; i++) {
        uint64_t end = rx->p + 10u * i;
        if (end > rx->avail) return false;
        if (alink_sym_is_comma(sym_at(rx, end))) {
            rx->p = end;
            rx->body_done = false;
            rx->comma_ok = true;
            rx->slip = 0;
            return true;
        }
    }
    loss(rx);
    return true;
}

// LOCKED: the next comma where the layout puts it, or within the slip
// window, or the flywheel position.
static bool next_locked(alink_rx_t *rx) {
    static const int8_t order[1 + 2 * ALINK_RX_SLIP_MAX] = { 0, -1, 1, -2, 2 };
    uint64_t e = rx->p + 10u * alink_frame_symbols(rx->cls, rx->rate, rx->frame);
    if (e + ALINK_RX_SLIP_MAX > rx->avail) return false;
    bool found = false;
    int d = 0;
    for (unsigned i = 0; i < sizeof(order) && !found; i++) {
        d = order[i];
        found = alink_sym_is_comma(sym_at(rx, (uint64_t)((int64_t)e + d)));
    }
    rx->frame++;
    if (rx->apply_pending && rx->frame == rx->sf.apply_frame) {
        unsigned c = rx->sf.apply_value ? (unsigned)rx->sf.apply_value : rx->channels;
        if (alink_rate_hz(rx->sf.apply_arg) && rx->sf.apply_value >= 0 && c >= 1u &&
            c <= alink_max_channels(rx->cls, rx->sf.apply_arg)) {
            rx->rate = rx->sf.apply_arg;
            rx->channels = (uint8_t)c;
        }
        rx->apply_pending = false;
    }
    if (found) {
        rx->p = (uint64_t)((int64_t)e + d);
        rx->comma_ok = true;
        rx->slip = (int8_t)d;
        if (d) rx->stats.bit_slips++;
    } else {
        rx->p = e;
        rx->comma_ok = false;
        rx->slip = 0;
        rx->stats.comma_errors++;
    }
    rx->body_done = false;
    return true;
}

static bool step(alink_rx_t *rx) {
    switch (rx->state) {
    case ALINK_RX_SEARCH: return step_search(rx);
    case ALINK_RX_ALIGNED: return rx->body_done ? next_aligned(rx) : body_aligned(rx);
    case ALINK_RX_LOCKED: return rx->body_done ? next_locked(rx) : body_locked(rx);
    }
    return false;
}

// ---------------------------------------------------------------------------
// Buffer and entry points
// ---------------------------------------------------------------------------

// Drop whole words below the oldest bit a later step can read.
static void compact(alink_rx_t *rx) {
    uint64_t keep = rx->state == ALINK_RX_SEARCH ? rx->q : rx->p;
    keep = keep >= 10u ? keep - 10u : 0;
    if (keep <= rx->base) return;
    uint64_t words = (keep - rx->base) >> 5;
    uint64_t used = (rx->avail - rx->base + 31u) >> 5;
    if (!words) return;
    memmove(rx->buf, rx->buf + words, (size_t)(used - words) * sizeof(uint32_t));
    rx->base += words * 32u;
}

void alink_rx_init(alink_rx_t *rx, unsigned cls, uint8_t bank_mask, alink_rx_cb_t cb, void *ctx) {
    memset(rx, 0, sizeof(*rx));
    rx->cls = (uint8_t)cls;
    rx->bank_mask = bank_mask;
    rx->cb = cb;
    rx->ctx = ctx;
    enter_search(rx, 10u);
}

size_t alink_rx_push(alink_rx_t *rx, const uint32_t *words, size_t nbits) {
    for (size_t i = 0; i < nbits; i++) {
        if (rx->avail - rx->base >= BUF_BITS) {
            while (step(rx)) {}
            compact(rx);
            if (rx->avail - rx->base >= BUF_BITS) {
                // Cannot happen with ALINK_RX_BUF_WORDS >= 45; recover anyway.
                if (rx->state != ALINK_RX_SEARCH) { rx->p = rx->avail; loss(rx); }
                rx->base = rx->avail;
                rx->q = rx->avail + 10u;
            }
        }
        uint64_t at = rx->avail - rx->base;
        uint32_t bit = (words[i >> 5] >> (31u - (unsigned)(i & 31u))) & 1u;
        if ((at & 31u) == 0) rx->buf[at >> 5] = 0;
        rx->buf[at >> 5] |= bit << (31u - (unsigned)(at & 31u));
        rx->avail++;
    }
    while (step(rx)) {}
    compact(rx);
    return nbits;
}
