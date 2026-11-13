/*
 * link_decoder.c - DSPi link protocol v1: reference receiver
 *
 * See link_decoder.h and Documentation/Features/link_protocol_spec.md
 * section 8.  Every step below runs only once all the bits it reads are
 * present, and reads nothing beyond them, which is what makes the output
 * independent of how the input was chunked.
 */

#include "link_decoder.h"

#include <string.h>

#define BUF_BITS ((uint64_t)LINK_RX_BUF_WORDS * 32u)

static void emit(link_rx_t *rx, link_rx_event_t *ev) {
    if (rx->cb) rx->cb(rx->ctx, ev);
}

static void emit_simple(link_rx_t *rx, link_rx_event_type_t type) {
    link_rx_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = type;
    ev.frame = rx->frame;
    emit(rx, &ev);
}

// ---------------------------------------------------------------------------
// Bit access
// ---------------------------------------------------------------------------

// The 10 bits [end - 10, end) as a symbol, first bit in bit 9.
static uint16_t sym_at(const link_rx_t *rx, uint64_t end) {
    uint16_t s = 0;
    for (uint64_t pos = end - 10u; pos < end; pos++) {
        uint64_t i = pos - rx->base;
        s = (uint16_t)((s << 1) | ((rx->buf[i >> 5] >> (31u - (unsigned)(i & 31u))) & 1u));
    }
    return s;
}

// A data code group ending at `end`; counts a code error when asked to.
static bool data_at(link_rx_t *rx, uint64_t end, uint8_t *byte, bool count) {
    bool k;
    if (link_8b10b_decode(sym_at(rx, end), byte, &k) && !k) return true;
    if (count) rx->stats.code_errors++;
    return false;
}

// HDR, CTL and CRC8 after the comma ending at p.  Every bad symbol is counted.
static bool head_at(link_rx_t *rx, uint64_t p, uint8_t *hdr, uint8_t *ctl, bool count) {
    uint8_t b[3];
    bool ok = true;
    for (unsigned i = 0; i < 3; i++)
        if (!data_at(rx, p + 10u * (i + 1u), &b[i], count)) ok = false;
    if (!ok || link_crc8(b, 2) != b[2]) return false;
    *hdr = b[0];
    *ctl = b[1];
    return true;
}

// ---------------------------------------------------------------------------
// State changes
// ---------------------------------------------------------------------------

static void clear_frame_state(link_rx_t *rx) {
    rx->fails = 0;
    rx->prev_hdr = -1;
    rx->sf_count = 0;
    rx->apply_pending = false;
    memset(rx->bank_fails, 0, sizeof(rx->bank_fails));
    memset(rx->samples, 0, sizeof(rx->samples));
}

static void enter_search(link_rx_t *rx, uint64_t q) {
    rx->state = LINK_RX_SEARCH;
    rx->q = q;
    clear_frame_state(rx);
}

static void loss(link_rx_t *rx) {
    if (rx->state == LINK_RX_LOCKED) rx->stats.loss_count++;
    emit_simple(rx, LINK_RX_EV_LOSS);
    enter_search(rx, rx->p + 1u);
}

static void sf_collect(link_rx_t *rx, unsigned idx, uint8_t ctl, uint8_t hdr) {
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

static void sf_error(link_rx_t *rx, int err) {
    link_rx_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = LINK_RX_EV_SF_ERROR;
    ev.frame = rx->frame;
    ev.sf_error = err;
    rx->stats.superframe_errors++;
    emit(rx, &ev);
}

// Unpack the assembled superframe and check it against this receiver.
static int sf_check(link_rx_t *rx, link_sf_t *sf) {
    int err = link_sf_unpack(rx->sf_bytes, sf);
    if (err == LINK_SF_OK && sf->cls != rx->cls) err = LINK_RX_SF_CLASS;
    return err;
}

static void adopt_sf(link_rx_t *rx, const link_sf_t *sf) {
    rx->sf = *sf;
    rx->rate = sf->rate;
    rx->channels = sf->channels;
    rx->apply_pending = sf->apply_op == LINK_OP_RATE && sf->apply_frame > rx->frame;
}

// ---------------------------------------------------------------------------
// Steps
// ---------------------------------------------------------------------------

static bool step_search(link_rx_t *rx) {
    if (rx->q < 10u) rx->q = 10u;
    while (rx->q + 30u <= rx->avail) {
        uint8_t hdr, ctl;
        if (link_sym_is_comma(sym_at(rx, rx->q)) && head_at(rx, rx->q, &hdr, &ctl, false)) {
            rx->state = LINK_RX_ALIGNED;
            rx->p = rx->q;
            rx->body_done = false;
            rx->comma_ok = true;
            rx->slip = 0;
            clear_frame_state(rx);
            link_rx_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = LINK_RX_EV_ALIGN;
            ev.bitpos = rx->p;
            emit(rx, &ev);
            return true;
        }
        rx->q++;
    }
    return false;
}

// ALIGNED: the head only; a valid superframe moves to LOCKED.
static bool body_aligned(link_rx_t *rx) {
    if (rx->p + 30u > rx->avail) return false;
    uint8_t hdr, ctl;
    if (!head_at(rx, rx->p, &hdr, &ctl, true)) {
        rx->stats.crc8_errors++;
        rx->prev_hdr = -1;
        rx->sf_count = 0;
        if (++rx->fails >= LINK_RX_FAIL_LIMIT) { loss(rx); return true; }
        rx->body_done = true;
        return true;
    }
    rx->fails = 0;
    unsigned idx = hdr & (LINK_SF_LEN - 1u);
    sf_collect(rx, idx, ctl, hdr);
    if (idx == LINK_SF_LEN - 1u && rx->sf_count == LINK_SF_LEN) {
        link_sf_t sf;
        int err = sf_check(rx, &sf);
        if (err == LINK_SF_OK && (uint8_t)(sf.frame + LINK_SF_LEN - 1u) != hdr)
            err = LINK_RX_SF_COUNTER;
        if (err != LINK_SF_OK) {
            sf_error(rx, err);
        } else {
            rx->state = LINK_RX_LOCKED;
            rx->frame = sf.frame + LINK_SF_LEN - 1u;
            adopt_sf(rx, &sf);
            memset(rx->bank_fails, 0, sizeof(rx->bank_fails));
            memset(rx->samples, 0, sizeof(rx->samples));
            rx->stats.lock_count++;
            link_rx_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = LINK_RX_EV_LOCK;
            ev.frame = rx->frame;
            ev.sf = &rx->sf;
            emit(rx, &ev);
        }
    }
    rx->body_done = true;
    return true;
}

static void decode_banks(link_rx_t *rx, uint8_t status[LINK_MAX_BANKS]) {
    unsigned c = rx->channels;
    memset(status, LINK_BANK_SKIPPED, LINK_MAX_BANKS);
    for (unsigned b = 0; b < link_bank_count(c); b++) {
        unsigned k = link_bank_channels(c, b), n = 3u * k;
        int32_t *out = rx->samples + LINK_BANK_CHANNELS * b;
        if (!((rx->bank_mask >> b) & 1u)) {
            memset(out, 0, k * sizeof(int32_t));
            continue;
        }
        uint8_t bytes[3u * LINK_BANK_CHANNELS + 2u];
        bool ok = true;
        unsigned s0 = link_bank_symbol(b);
        for (unsigned i = 0; i < n + 2u; i++)
            if (!data_at(rx, rx->p + 10u * (s0 + i), &bytes[i], true)) ok = false;
        if (ok) ok = (uint16_t)(bytes[n] | (bytes[n + 1u] << 8)) == link_crc16(bytes, n);
        if (ok) {
            status[b] = LINK_BANK_OK;
            rx->bank_fails[b] = 0;
            for (unsigned i = 0; i < k; i++) out[i] = link_get_s24(bytes + 3u * i);
        } else {
            rx->stats.crc16_errors++;
            if (rx->bank_fails[b] < 0xFFu) rx->bank_fails[b]++;
            if (rx->bank_fails[b] == 1u) {
                status[b] = LINK_BANK_HOLD;     // samples keep the last frame's values
            } else {
                status[b] = LINK_BANK_ZERO;
                memset(out, 0, k * sizeof(int32_t));
            }
        }
    }
    for (unsigned ch = c; ch < LINK_MAX_CHANNELS; ch++) rx->samples[ch] = 0;
}

// LOCKED: head, banks, padding, superframe, failure count.
static bool body_locked(link_rx_t *rx) {
    unsigned s = link_frame_symbols(rx->cls, rx->rate, rx->frame);
    if (rx->p + 10u * (s - 1u) > rx->avail) return false;

    link_rx_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = LINK_RX_EV_FRAME;
    ev.frame = rx->frame;
    ev.slip = rx->slip;
    if (!rx->comma_ok) ev.flags |= LINK_RXF_COMMA_MISSING;
    if (rx->slip) ev.flags |= LINK_RXF_SLIP;

    uint8_t hdr, ctl;
    bool good = head_at(rx, rx->p, &hdr, &ctl, true);
    if (!good) {
        rx->stats.crc8_errors++;
        ev.flags |= LINK_RXF_HEAD_BAD;
    } else if (hdr != (uint8_t)rx->frame) {
        rx->stats.counter_errors++;
        ev.flags |= LINK_RXF_COUNTER;
        good = false;
    }

    decode_banks(rx, ev.bank_status);
    for (unsigned i = link_layout_symbols(rx->channels); i < s; i++)
        if (sym_at(rx, rx->p + 10u * i) != LINK_SYM_IDLE) rx->stats.idle_errors++;

    rx->stats.frames++;
    ev.channels = rx->channels;
    ev.samples = rx->samples;
    emit(rx, &ev);

    unsigned idx = (unsigned)(rx->frame & (LINK_SF_LEN - 1u));
    if (good) {
        sf_collect(rx, idx, ctl, hdr);
    } else {
        rx->sf_count = 0;
        rx->prev_hdr = -1;
    }
    if (idx == LINK_SF_LEN - 1u) {
        link_sf_t sf;
        int err = rx->sf_count == LINK_SF_LEN ? sf_check(rx, &sf) : LINK_RX_SF_INCOMPLETE;
        if (err != LINK_SF_OK) {
            sf_error(rx, err);
        } else {
            if (sf.frame + LINK_SF_LEN - 1u != rx->frame) {
                rx->stats.counter_errors++;
                rx->frame = sf.frame + LINK_SF_LEN - 1u;
            }
            adopt_sf(rx, &sf);
            link_rx_event_t sev;
            memset(&sev, 0, sizeof(sev));
            sev.type = LINK_RX_EV_SUPERFRAME;
            sev.frame = rx->frame;
            sev.sf = &rx->sf;
            emit(rx, &sev);
        }
    }

    if (ev.flags & (LINK_RXF_COMMA_MISSING | LINK_RXF_HEAD_BAD | LINK_RXF_COUNTER)) {
        if (++rx->fails >= LINK_RX_FAIL_LIMIT) { loss(rx); return true; }
    } else {
        rx->fails = 0;
    }
    rx->body_done = true;
    return true;
}

// ALIGNED: the next comma at a symbol boundary within the longest frame.
static bool next_aligned(link_rx_t *rx) {
    unsigned maxgap = link_frame_symbols_max(rx->cls, LINK_RATE_44K1);
    for (unsigned i = LINK_HEAD_SYMBOLS; i <= maxgap; i++) {
        uint64_t end = rx->p + 10u * i;
        if (end > rx->avail) return false;
        if (link_sym_is_comma(sym_at(rx, end))) {
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
static bool next_locked(link_rx_t *rx) {
    static const int8_t order[1 + 2 * LINK_RX_SLIP_MAX] = { 0, -1, 1, -2, 2 };
    uint64_t e = rx->p + 10u * link_frame_symbols(rx->cls, rx->rate, rx->frame);
    if (e + LINK_RX_SLIP_MAX > rx->avail) return false;
    bool found = false;
    int d = 0;
    for (unsigned i = 0; i < sizeof(order) && !found; i++) {
        d = order[i];
        found = link_sym_is_comma(sym_at(rx, (uint64_t)((int64_t)e + d)));
    }
    rx->frame++;
    if (rx->apply_pending && rx->frame == rx->sf.apply_frame) {
        unsigned c = rx->sf.apply_value ? (unsigned)rx->sf.apply_value : rx->channels;
        if (link_rate_hz(rx->sf.apply_arg) && rx->sf.apply_value >= 0 && c >= 1u &&
            c <= link_max_channels(rx->cls, rx->sf.apply_arg)) {
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

static bool step(link_rx_t *rx) {
    switch (rx->state) {
    case LINK_RX_SEARCH: return step_search(rx);
    case LINK_RX_ALIGNED: return rx->body_done ? next_aligned(rx) : body_aligned(rx);
    case LINK_RX_LOCKED: return rx->body_done ? next_locked(rx) : body_locked(rx);
    }
    return false;
}

// ---------------------------------------------------------------------------
// Buffer and entry points
// ---------------------------------------------------------------------------

// Drop whole words below the oldest bit a later step can read.
static void compact(link_rx_t *rx) {
    uint64_t keep = rx->state == LINK_RX_SEARCH ? rx->q : rx->p;
    keep = keep >= 10u ? keep - 10u : 0;
    if (keep <= rx->base) return;
    uint64_t words = (keep - rx->base) >> 5;
    uint64_t used = (rx->avail - rx->base + 31u) >> 5;
    if (!words) return;
    memmove(rx->buf, rx->buf + words, (size_t)(used - words) * sizeof(uint32_t));
    rx->base += words * 32u;
}

void link_rx_init(link_rx_t *rx, unsigned cls, uint8_t bank_mask, link_rx_cb_t cb, void *ctx) {
    memset(rx, 0, sizeof(*rx));
    rx->cls = (uint8_t)cls;
    rx->bank_mask = bank_mask;
    rx->cb = cb;
    rx->ctx = ctx;
    enter_search(rx, 10u);
}

size_t link_rx_push(link_rx_t *rx, const uint32_t *words, size_t nbits) {
    for (size_t i = 0; i < nbits; i++) {
        if (rx->avail - rx->base >= BUF_BITS) {
            while (step(rx)) {}
            compact(rx);
            if (rx->avail - rx->base >= BUF_BITS) {
                // Cannot happen with LINK_RX_BUF_WORDS >= 45; recover anyway.
                if (rx->state != LINK_RX_SEARCH) { rx->p = rx->avail; loss(rx); }
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
