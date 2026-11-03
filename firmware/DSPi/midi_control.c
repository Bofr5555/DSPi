/*
 * midi_control.c - USB MIDI control transport for DSPi (DSPI_USB_MIDI)
 *
 * See midi_control.h and Documentation/Features/usb_midi_control_spec.md.
 *
 * Flow (all from midi_ctrl_poll(), main-loop context):
 *   RX : TinyUSB RX FIFO -> 4-byte USB-MIDI packets -> SysEx bytes ->
 *        mf_dec (unpack + UART frame parser) -> one pending slot.
 *        While the slot is full nothing more is read, so USB flow control
 *        (NAK) holds the host back instead of anything being dropped.
 *   DSP: the pending request goes through vendor_dispatch_get/set with
 *        CTRL_SOURCE_MIDI, with the UART transport's BUSY / BULK_LOCKED
 *        retry window, once no response is being transmitted.
 *   TX : one message at a time, mf_enc -> USB-MIDI packets -> TinyUSB TX
 *        FIFO at FIFO pace with a cursor (never waits).  Notifications
 *        (tag 0) start only when no response is transmitting and no request
 *        is pending, so a response is never split.
 */

#include "midi_control.h"
#include "midi_frame.h"

#ifndef MIDI_CTRL_HOST_TEST
#include "vendor_commands.h"
#include "usb_audio.h"          // bulk_param_buf
#include "bulk_params.h"        // WIRE_BULK_PARAMS_SIZE (chunk sessions)
#include "usb_descriptors.h"    // MIDI_IN_ENDPOINT
#include "notify.h"
#include "config.h"             // REQ_* opcodes
#include "control_surfaces.h"   // CsMacro sizes MIDI_TX_COPY_MAX
#include "pico/stdlib.h"
#include "tusb.h"
#include "device/usbd_pvt.h"    // usbd_edpt_busy (bootloader-entry flush)
#endif

#include <string.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Manufacturer SysEx ID.  0x7D is the MIDI non-commercial / development ID;
// a registered ID (1 byte, or 3 bytes starting 0x00) is a build option:
//   -DDSPI_USB_MIDI_MFR_ID=0x00,0x12,0x34
#ifndef DSPI_USB_MIDI_MFR_ID
#define DSPI_USB_MIDI_MFR_ID 0x7D
#endif
#define MF_ID_FIRST_(a, ...) (a)
#define MF_ID_FIRST(...) MF_ID_FIRST_(__VA_ARGS__, 0)
#define MF_ID_LEN (sizeof((const uint8_t[]){ DSPI_USB_MIDI_MFR_ID }))
_Static_assert(MF_ID_LEN == 1 || MF_ID_LEN == 3,
               "DSPI_USB_MIDI_MFR_ID must be 1 or 3 bytes");
_Static_assert((MF_ID_LEN == 1 && MF_ID_FIRST(DSPI_USB_MIDI_MFR_ID) >= 0x01 &&
                MF_ID_FIRST(DSPI_USB_MIDI_MFR_ID) <= 0x7F) ||
               (MF_ID_LEN == 3 && MF_ID_FIRST(DSPI_USB_MIDI_MFR_ID) == 0x00),
               "DSPI_USB_MIDI_MFR_ID: 1 byte 0x01..0x7F, or 3 bytes 0x00 xx yy");

static const mf_mfr_id_t g_id = {
    .bytes = { DSPI_USB_MIDI_MFR_ID },
    .len = (uint8_t)MF_ID_LEN,
};

#define MIDI_CABLE           0u
#define PAYLOAD_MAX          64u          // non-bulk SET payload cap (vendor_rx_buf)
#ifndef MIDI_TX_COPY_MAX
// Same size as the UART/I2C copy buffers and the dispatcher's _ext_resp_copy
// (largest non-bulk GET, the 132-byte CsMacro).
#define MIDI_TX_COPY_MAX     ((uint16_t)sizeof(CsMacro))
#endif
#define RX_BYTE_BUDGET       96           // SysEx bytes parsed per poll
#define TX_PKT_BUDGET        48           // USB-MIDI packets queued per poll
#define RX_TIMEOUT_US        1000000u     // drop a partial message after 1 s of silence
#define DISPATCH_RETRY_US    50000u       // transient-status retry window (as UART)
#define BULK_TOUCH_BYTES     64u          // refresh the bulk lock this often
#define TX_STALL_US          400000u      // host stopped reading a bulk reply
#define BOOT_FLUSH_US        200000u      // max wait for the 0xF0 reply to leave
#define CHUNK_SESSION_US     3000000u     // idle chunk session reaped (USB's stale window)
#define BULK_TOTAL           ((uint32_t)WIRE_BULK_PARAMS_SIZE)

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool      was_mounted = false;
static mf_dec_t  dec;
static uint8_t   rx_bytes[3];
static uint8_t   rx_n = 0, rx_i = 0;
static uint32_t  last_rx_time;
static uint8_t   payload_buf[PAYLOAD_MAX];
static bool      parser_holds_bulk = false;
static uint32_t  parser_touch_mark;

// One pending request (or pre-decided status reply).
static struct {
    bool     active;
    bool     dispatch;      // false: just answer `status`
    bool     is_get;
    bool     holds_bulk;    // bulk SET payload sits in bulk_param_buf under our lock
    uint8_t  tag;
    uint8_t  resp_type;
    uint8_t  status;
    uint8_t  bReq;
    uint16_t wValue, wIndex, wLen;
    uint32_t retry_start;
    uint8_t  boot_phase;    // REQ_ENTER_BOOTLOADER flush sequence
    uint32_t boot_t0;
} pending;

// Chunked bulk sessions (0xA2 / 0xA3), the USB vendor path's semantics
// (bulk_params_chunking.md): a GET session parks one coherent snapshot in
// bulk_param_buf from offset 0 until the final chunk; a SET session takes
// sequential chunks from offset 0 and applies on the final byte.  Either
// holds the bulk lock for its whole life; the first non-chunk request, 3 s
// of inactivity, or unplug closes it.
static struct {
    bool     get_open;
    bool     set_open;
    uint32_t set_received;
    uint32_t last_us;
} chunk;

static mf_enc_t enc;
static struct {
    bool     active;
    bool     bulk;          // payload is bulk_param_buf under our lock
    bool     have_pkt;      // pkt built but not yet accepted by the FIFO
    uint8_t  pkt[4];
    uint32_t touch_mark;
    uint32_t last_progress;
} tx;
static uint8_t tx_copy[MIDI_TX_COPY_MAX];
static uint8_t notify_pkt[64];   // largest v2 notify packet (12 + 52)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void release_parser_bulk(void) {
    if (parser_holds_bulk) {
        vendor_bulk_release(CTRL_SOURCE_MIDI);
        parser_holds_bulk = false;
    }
}

static bool chunk_open(void) { return chunk.get_open || chunk.set_open; }

static void chunk_close(void) {
    if (chunk_open()) vendor_bulk_release(CTRL_SOURCE_MIDI);
    chunk.get_open = chunk.set_open = false;
    chunk.set_received = 0;
}

static bool is_chunk_req(uint8_t bReq) {
    return bReq == REQ_GET_ALL_PARAMS_CHUNK || bReq == REQ_SET_ALL_PARAMS_CHUNK;
}

static void reset_all(void) {
    chunk_close();
    release_parser_bulk();
    if (pending.active && pending.holds_bulk) vendor_bulk_release(CTRL_SOURCE_MIDI);
    if (tx.active && tx.bulk) vendor_bulk_release(CTRL_SOURCE_MIDI);
    memset(&pending, 0, sizeof(pending));
    memset(&tx, 0, sizeof(tx));
    mf_dec_init(&dec, &g_id);
    rx_n = rx_i = 0;
}

static void start_tx(uint8_t tag, uint8_t type, uint8_t status,
                     const uint8_t *payload, uint16_t len, bool bulk) {
    mf_enc_begin_response(&enc, &g_id, tag, type, status, payload, len);
    tx.active = true;
    tx.bulk = bulk && status == CTRL_STATUS_OK;
    tx.have_pkt = false;
    tx.touch_mark = 0;
    tx.last_progress = time_us_32();
}

static void tx_release_bulk(void) {
    if (tx.bulk) {
        vendor_bulk_release(CTRL_SOURCE_MIDI);
        tx.bulk = false;
    }
}

// Queue packets until the FIFO is full, the message ends, or the budget
// runs out.  A packet the FIFO refused is kept and offered again next poll.
static void pump_tx(void) {
    int budget = TX_PKT_BUDGET;
    while (tx.active && budget-- > 0) {
        if (!tx.have_pkt) {
            if (!mf_enc_packet(&enc, MIDI_CABLE, tx.pkt)) {
                tx.active = false;
                tx_release_bulk();
                break;
            }
            tx.have_pkt = true;
        }
        if (!tud_midi_packet_write(tx.pkt)) break;   // FIFO full: resume next poll
        tx.have_pkt = false;
        tx.last_progress = time_us_32();
        if (tx.bulk) {
            uint32_t pos = mf_enc_payload_pos(&enc);
            if (pos >= enc.plen) {
                tx_release_bulk();                    // whole payload already encoded
            } else if (pos - tx.touch_mark >= BULK_TOUCH_BYTES) {
                vendor_bulk_touch(CTRL_SOURCE_MIDI);
                tx.touch_mark = pos;
            }
        }
    }

    // The host stopped reading in the middle of a bulk (or chunk) reply: end the SysEx
    // early (the host sees a malformed message and retries) and give the
    // bulk buffer back instead of pinning it for every other transport.
    if (tx.active && (tx.bulk || chunk.get_open) &&
        (uint32_t)(time_us_32() - tx.last_progress) > TX_STALL_US) {
        tx_release_bulk();
        chunk_close();                      // a GET chunk was streaming from the session
        enc.phase = 3;                      // encoder done
        if (!tx.have_pkt) {
            tx.pkt[0] = (uint8_t)((MIDI_CABLE << 4) | MF_CIN_SYSEX_END1);
            tx.pkt[1] = MF_SYSEX_END;
            tx.pkt[2] = tx.pkt[3] = 0;
            tx.have_pkt = true;
        } else {
            // A queued packet is waiting; terminate it in place.
            uint8_t cin = tx.pkt[0] & 0x0F;
            if (cin == MF_CIN_SYSEX_START) {
                // 3 data bytes, none of them F7: send the F7 separately.
                // Simplest: overwrite the last byte with F7.
                tx.pkt[0] = (uint8_t)((MIDI_CABLE << 4) | MF_CIN_SYSEX_END3);
                tx.pkt[3] = MF_SYSEX_END;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------

static void set_pending_status(uint8_t tag, uint8_t resp_type, uint8_t status) {
    memset(&pending, 0, sizeof(pending));
    pending.active = true;
    pending.dispatch = false;
    pending.tag = tag;
    pending.resp_type = resp_type;
    pending.status = status;
}

static void on_event(mf_event_t ev) {
    switch (ev) {
        case MF_EV_SET_HEADER:
            dec.dst = NULL;
            if (dec.bReq == REQ_SET_ALL_PARAMS_CHUNK) {
                // Payload lands at bulk_param_buf + offset, like USB.
                uint32_t off = dec.wValue, n = dec.wLength;
                if (off + n > BULK_TOTAL) {
                    dec.deferred_status = CTRL_STATUS_ERROR;
                } else if (off == 0) {
                    if (tx.active && chunk.get_open) {
                        // A GET chunk is still being sent out of the buffer.
                        dec.deferred_status = CTRL_STATUS_BULK_LOCKED;
                    } else if (chunk_open() || vendor_bulk_try_acquire(CTRL_SOURCE_MIDI)) {
                        chunk.get_open = false;      // SET supersedes a GET session
                        chunk.set_open = true;
                        chunk.set_received = 0;
                        chunk.last_us = time_us_32();
                        dec.dst = bulk_param_buf;
                    } else {
                        dec.deferred_status = CTRL_STATUS_BULK_LOCKED;
                    }
                } else if (!chunk.set_open || off != chunk.set_received) {
                    dec.deferred_status = CTRL_STATUS_ERROR;   // out of order: restart at 0
                } else {
                    chunk.last_us = time_us_32();
                    dec.dst = bulk_param_buf + off;
                }
                break;
            }
            if (vendor_is_bulk_set(dec.bReq, dec.wLength) && chunk_open() && !tx.active)
                chunk_close();                   // a non-chunk bulk SET ends the session
            if (vendor_is_bulk_set(dec.bReq, dec.wLength)) {
                if (vendor_bulk_try_acquire(CTRL_SOURCE_MIDI)) {
                    parser_holds_bulk = true;
                    parser_touch_mark = 0;
                    dec.dst = bulk_param_buf;
                } else {
                    dec.deferred_status = CTRL_STATUS_BULK_LOCKED;  // discard, answer at F7
                }
            } else if (dec.wLength > PAYLOAD_MAX) {
                dec.deferred_status = CTRL_STATUS_OVERSIZE;
            } else {
                dec.dst = payload_buf;
            }
            break;

        case MF_EV_REQUEST: {
            uint8_t rtype = dec.is_get ? MF_RESP_GET : MF_RESP_SET;
            if (dec.deferred_status != CTRL_STATUS_OK) {
                release_parser_bulk();
                set_pending_status(dec.tag, rtype, dec.deferred_status);
                break;
            }
            memset(&pending, 0, sizeof(pending));
            pending.active      = true;
            pending.dispatch    = true;
            pending.is_get      = dec.is_get;
            pending.tag         = dec.tag;
            pending.resp_type   = rtype;
            pending.bReq        = dec.bReq;
            pending.wValue      = dec.wValue;
            pending.wIndex      = dec.wIndex;
            pending.wLen        = dec.wLength;
            pending.holds_bulk  = parser_holds_bulk;
            pending.retry_start = time_us_32();
            parser_holds_bulk = false;          // ownership moves to the pending slot
            break;
        }

        case MF_EV_ERROR:
            release_parser_bulk();
            set_pending_status(dec.err_tag, dec.err_type, dec.err_status);
            break;

        default:
            break;
    }
}

static void parse_rx(void) {
    int budget = RX_BYTE_BUDGET;
    while (budget > 0 && !pending.active) {
        if (rx_i >= rx_n) {
            uint8_t pkt[4];
            if (!tud_midi_packet_read(pkt)) break;
            rx_n = mf_packet_bytes(pkt, MIDI_CABLE, rx_bytes);
            rx_i = 0;
            last_rx_time = time_us_32();
            continue;
        }
        budget--;
        on_event(mf_dec_feed(&dec, rx_bytes[rx_i++]));
        if (parser_holds_bulk && dec.ppos - parser_touch_mark >= BULK_TOUCH_BYTES) {
            vendor_bulk_touch(CTRL_SOURCE_MIDI);
            parser_touch_mark = dec.ppos;
        }
    }
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------

// REQ_ENTER_BOOTLOADER never returns (reset_usb_boot), so the reply is sent
// first and given time to leave: wait for the IN endpoint to go idle (TX
// FIFO empty), queue the reply, wait for the host to take it (bounded), then
// dispatch.  This is the one deliberate wait-across-polls in the module.
static void dispatch_bootloader(void) {
    switch (pending.boot_phase) {
        case 0:
            if (usbd_edpt_busy(0, MIDI_IN_ENDPOINT)) return;
            tx_copy[0] = 1;   // same payload as the USB/UART handler
            start_tx(pending.tag, MF_RESP_GET, CTRL_STATUS_OK, tx_copy, 1, false);
            pump_tx();
            pending.boot_phase = 1;
            pending.boot_t0 = time_us_32();
            return;
        case 1:
            if (tx.active) pump_tx();
            if ((tx.active || usbd_edpt_busy(0, MIDI_IN_ENDPOINT)) &&
                (uint32_t)(time_us_32() - pending.boot_t0) < BOOT_FLUSH_US)
                return;
            pending.boot_phase = 2;
            /* fall through */
        default: {
            const uint8_t *rd = NULL;
            uint16_t rl = 0;
            // Only returns on BUSY (a USB control SET in flight, reclaimed
            // after 100 ms); keep trying, the reply is already out.
            CtrlDispatchResult r = vendor_dispatch_get(CTRL_SOURCE_MIDI, pending.bReq,
                                                       pending.wValue, pending.wIndex,
                                                       pending.wLen, &rd, &rl);
            if (r != CTRL_DISPATCH_BUSY) pending.active = false;
            return;
        }
    }
}

static bool retry_pending(CtrlDispatchResult res) {
    bool retryable = (res == CTRL_DISPATCH_BUSY) ||
                     (pending.is_get && res == CTRL_DISPATCH_BULK_LOCKED);
    return retryable &&
           (uint32_t)(time_us_32() - pending.retry_start) < DISPATCH_RETRY_US;
}

// GET 0xA2: wValue = offset, wLength = chunk size.  Offset 0 snapshots the
// whole struct under the bulk lock (through the normal 0xA0 dispatch).
static void dispatch_chunk_get(void) {
    uint32_t off = pending.wValue, n = pending.wLen;
    uint8_t status = CTRL_STATUS_OK;
    if (n == 0 || off >= BULK_TOTAL) {
        status = CTRL_STATUS_ERROR;
    } else if (off == 0) {
        chunk_close();
        const uint8_t *rd = NULL;
        uint16_t rl = 0;
        CtrlDispatchResult res = vendor_dispatch_get(CTRL_SOURCE_MIDI, REQ_GET_ALL_PARAMS,
                                                     0, pending.wIndex, 0, &rd, &rl);
        if (retry_pending(res)) return;
        if (res == CTRL_DISPATCH_OK) chunk.get_open = true;   // lock now ours
        else status = ctrl_status_from_dispatch(res);
    } else if (!chunk.get_open) {
        status = CTRL_STATUS_ERROR;      // no snapshot (session lost): restart at 0
    }
    pending.active = false;
    if (status != CTRL_STATUS_OK) {
        start_tx(pending.tag, MF_RESP_GET, status, NULL, 0, false);
        return;
    }
    chunk.last_us = time_us_32();
    if (off + n > BULK_TOTAL) n = BULK_TOTAL - off;
    if (off + n >= BULK_TOTAL) {
        // Final chunk: the reply takes over the lock and releases it once
        // the payload is encoded.
        chunk.get_open = false;
        start_tx(pending.tag, MF_RESP_GET, CTRL_STATUS_OK, bulk_param_buf + off, (uint16_t)n, true);
    } else {
        start_tx(pending.tag, MF_RESP_GET, CTRL_STATUS_OK, bulk_param_buf + off, (uint16_t)n, false);
    }
}

// SET 0xA3: the payload is already at bulk_param_buf + offset (checked at
// the header).  The chunk that completes the struct hands it to the apply.
static void dispatch_chunk_set(void) {
    if (pending.wLen == 0 || !chunk.set_open) {
        pending.active = false;
        start_tx(pending.tag, MF_RESP_SET, CTRL_STATUS_ERROR, NULL, 0, false);
        return;
    }
    chunk.set_received = (uint32_t)pending.wValue + pending.wLen;
    chunk.last_us = time_us_32();
    uint8_t status = CTRL_STATUS_OK;
    if (chunk.set_received >= BULK_TOTAL) {
        CtrlDispatchResult res = vendor_dispatch_set(CTRL_SOURCE_MIDI, REQ_SET_ALL_PARAMS, 0,
                                                     pending.wIndex, NULL, (uint16_t)BULK_TOTAL);
        if (retry_pending(res)) return;
        if (res == CTRL_DISPATCH_OK) {
            chunk.set_open = false;          // lock handed to the apply
            chunk.set_received = 0;
        } else {
            chunk_close();
        }
        status = ctrl_status_from_dispatch(res);
    }
    pending.active = false;
    start_tx(pending.tag, MF_RESP_SET, status, NULL, 0, false);
}

static void try_dispatch(void) {
    if (!pending.dispatch) {
        start_tx(pending.tag, pending.resp_type, pending.status, NULL, 0, false);
        pending.active = false;
        return;
    }
    if (pending.bReq == REQ_GET_ALL_PARAMS_CHUNK && pending.is_get) {
        dispatch_chunk_get();
        return;
    }
    if (pending.bReq == REQ_SET_ALL_PARAMS_CHUNK && !pending.is_get) {
        dispatch_chunk_set();
        return;
    }
    // Like USB: the first non-chunk request ends an open chunk session.
    if (!is_chunk_req(pending.bReq)) chunk_close();
    if (pending.is_get && pending.bReq == REQ_ENTER_BOOTLOADER) {
        dispatch_bootloader();
        return;
    }

    CtrlDispatchResult res;
    const uint8_t *rd = NULL;
    uint16_t rl = 0;
    if (pending.is_get) {
        res = vendor_dispatch_get(CTRL_SOURCE_MIDI, pending.bReq, pending.wValue,
                                  pending.wIndex, pending.wLen, &rd, &rl);
    } else {
        res = vendor_dispatch_set(CTRL_SOURCE_MIDI, pending.bReq, pending.wValue,
                                  pending.wIndex, payload_buf, pending.wLen);
    }

    if (retry_pending(res)) return;   // keep pending; retry next poll

    // Bulk SET: vendor_dispatch_set hands the buffer to the apply on OK;
    // release on anything else (as UART).
    if (!pending.is_get && pending.holds_bulk && res != CTRL_DISPATCH_OK)
        vendor_bulk_release(CTRL_SOURCE_MIDI);
    pending.holds_bulk = false;
    pending.active = false;

    if (pending.is_get && res == CTRL_DISPATCH_OK) {
        if (pending.bReq == REQ_GET_ALL_PARAMS || pending.bReq == REQ_RTA_GET_BANDS_ALL) {
            // Zero-copy from bulk_param_buf; the dispatcher handed us the
            // bulk lock, released once the payload is encoded.
            start_tx(pending.tag, MF_RESP_GET, CTRL_STATUS_OK, rd, rl, true);
        } else {
            uint16_t n = (rl > MIDI_TX_COPY_MAX) ? MIDI_TX_COPY_MAX : rl;
            memcpy(tx_copy, rd, n);
            start_tx(pending.tag, MF_RESP_GET, CTRL_STATUS_OK, tx_copy, n, false);
        }
    } else {
        start_tx(pending.tag, pending.resp_type, ctrl_status_from_dispatch(res),
                 NULL, 0, false);
    }
}

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

void midi_ctrl_poll(void) {
    bool mounted = tud_midi_mounted();
    if (mounted != was_mounted) {
        // (Re)configured or unplugged: TinyUSB cleared its FIFOs, so drop
        // every partial message and any bulk lock we hold.
        was_mounted = mounted;
        reset_all();
        notify_consumer_set_active(NOTIFY_CONSUMER_MIDI, mounted);
    }
    if (!mounted) return;

    // Hold a bulk SET's lock while it waits for dispatch, and an open chunk
    // session's for up to CHUNK_SESSION_US of inactivity.
    if (pending.active && pending.holds_bulk)
        vendor_bulk_touch(CTRL_SOURCE_MIDI);
    if (chunk_open()) {
        if ((uint32_t)(time_us_32() - chunk.last_us) > CHUNK_SESSION_US &&
            !(tx.active && chunk.get_open) &&
            !(pending.active && is_chunk_req(pending.bReq)))
            chunk_close();
        else
            vendor_bulk_touch(CTRL_SOURCE_MIDI);
    }

    // Silence mid-message: drop the partial message, answer nothing (UART rule).
    if (mf_dec_busy(&dec) &&
        (uint32_t)(time_us_32() - last_rx_time) > RX_TIMEOUT_US) {
        mf_dec_reset(&dec);
        release_parser_bulk();
    }

    if (tx.active) pump_tx();
    if (!tx.active && pending.active) try_dispatch();
    parse_rx();
    if (!tx.active && pending.active) try_dispatch();

    // Notifications: only between messages, never ahead of a pending
    // request.  The packet is copied by the peek and committed at once; a
    // host that stops reading backs up the TX FIFO, the ring force-drops for
    // this consumer, and the seq gap tells the client to re-read 0xA0.
    if (!tx.active && !pending.active) {
        uint16_t n = notify_peek_next_for(NOTIFY_CONSUMER_MIDI,
                                          notify_pkt, sizeof(notify_pkt));
        if (n) {
            notify_commit_pop_for(NOTIFY_CONSUMER_MIDI);
            start_tx(MF_TAG_NOTIFY, MF_TYPE_NOTIFY, CTRL_STATUS_OK, notify_pkt, n, false);
        }
    }

    if (tx.active) pump_tx();
}
