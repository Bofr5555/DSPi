/*
 * sender_vc.c - DSPi alink sender (WP FWPG4): the vendor control front end
 *
 * Both envelopes of extension_channel_spec.md 1.3 on one USB vendor
 * interface; sender_vc.h says what each answers.  The paged stack follows
 * upstream 1.1.7's dispatcher (vc_core.c vc_check / vc_run / vc_record) and
 * its EP0 front end (vc_usb.c) for the checks, their order and the stage a
 * refusal lands on:
 *
 *   SETUP   row: unknown page or command (UNKNOWN), no form for the direction
 *           (DIRECTION), SET length outside the row (LENGTH), wValue outside
 *           the address kind (ADDRESS).  A GET then runs and STALLs on a
 *           refusal; a zero-length SET runs now.
 *   DATA    a SET with a payload: the row again, then the handler (the
 *           tunnel's own checks are in its handler); a refusal STALLs the
 *           status stage.
 *
 * Every paged request but a GET of SYS.LAST_RESULT is recorded there.  Page 0
 * keeps the 1.1.6 behaviour byte for byte and records nothing.
 */

#include <string.h>

#include "sender_vc.h"

// 1.1.6 opcodes of page 0 (config.h; sender_vendor.c asserts they match).
#define LEGACY_EXTENSION        0x00u
#define LEGACY_SAVE_OUTPUT_CONFIG 0x52u
#define LEGACY_GET_SERIAL       0x7Eu
#define LEGACY_GET_PLATFORM     0x7Fu
#define LEGACY_GET_BUILD_INFO   0x80u
#define LEGACY_ENTER_BOOTLOADER 0xF0u
#define LEGACY_PRESET_OK        0x00u

// Upstream's address kinds and flags as SYS.COMMAND_INFO reports them.
#define A_NONE                  0u
#define A_PAGE                  21u
#define A_PAGE_CMD              22u
#define A_INDEX                 24u
#define F_SET_USB_ONLY          0x0002u
#define F_DEFERRED              0x0008u
#define F_FLASH                 0x0010u
#define F_RESTART               0x0020u
#define LEN_VAR                 0xFFFFu

#define PROTOCOL_MAJOR          1u
#define PROTOCOL_MINOR          0u

typedef enum {
    H_PROTOCOL_INFO, H_PAGE_LIST, H_PAGE_COMMANDS, H_COMMAND_INFO, H_LAST_RESULT,
    H_PLATFORM, H_SERIAL, H_BUILD_INFO, H_BOOTLOADER, H_SAVE, H_TUNNEL,
} handler_t;

typedef struct {
    uint16_t id;
    uint8_t  addr;          // address kind
    uint8_t  set_min, set_max;
    uint16_t get_len;       // fixed length, LEN_VAR, or 0 with no GET
    uint16_t flags;
    bool     get, set;
    uint8_t  h;             // handler_t
} row_t;

// The sender's paged commands, ascending by id.  The SYS and STORE rows are
// upstream's (pages/sys.def, pages/store.def at 103ba37), except that
// ENTER_BOOTLOADER carries VC_F_SET_USB_ONLY as on the fork's port (1.3.7);
// the tunnel's row is 1.3.2's.
static const row_t rows[] = {
    { SND_SYS_PROTOCOL_INFO,  A_NONE,     0, 0,  16,      0, true,  false, H_PROTOCOL_INFO },
    { SND_SYS_PAGE_LIST,      A_NONE,     0, 0,  LEN_VAR, 0, true,  false, H_PAGE_LIST },
    { SND_SYS_PAGE_COMMANDS,  A_PAGE,     0, 0,  64,      0, true,  false, H_PAGE_COMMANDS },
    { SND_SYS_COMMAND_INFO,   A_PAGE_CMD, 0, 0,  8,       0, true,  false, H_COMMAND_INFO },
    { SND_SYS_LAST_RESULT,    A_NONE,     0, 0,  8,       0, true,  false, H_LAST_RESULT },
    { SND_SYS_PLATFORM,       A_NONE,     0, 0,  7,       0, true,  false, H_PLATFORM },
    { SND_SYS_SERIAL,         A_NONE,     0, 0,  16,      0, true,  false, H_SERIAL },
    { SND_SYS_BUILD_INFO,     A_NONE,     0, 0,  64,      0, true,  false, H_BUILD_INFO },
    { SND_SYS_ENTER_BOOTLOADER, A_NONE,   0, 0,  0, F_RESTART | F_SET_USB_ONLY, false, true, H_BOOTLOADER },
    { SND_STORE_OUTPUT_CONFIG_SAVE, A_NONE, 0, 0, 0, F_DEFERRED | F_FLASH | F_RESTART, false, true, H_SAVE },
    { SND_EXT_TUNNEL,         A_INDEX,    1, SND_VC_MAX, LEN_VAR, 0, true, true, H_TUNNEL },
};
#define NROWS (sizeof(rows) / sizeof(rows[0]))

// The pages, ascending, with their revisions (SYS and STORE are revision 1).
static const uint8_t pages[][2] = {
    { SND_PAGE_SYS, 1 }, { SND_PAGE_STORE, 1 }, { SND_PAGE_EXT, SND_PAGE_EXT_REVISION },
};
#define NPAGES (sizeof(pages) / sizeof(pages[0]))

// EXT_LIST, ascending (2.1; 2.8 for a sender build).
static const uint16_t ext_ids[] = {
    SND_EXT_NOTIFY_FEATURES,
    SND_EXT_ALINK_INFO, SND_EXT_ALINK_STATUS, SND_EXT_ALINK_CONFIG, SND_EXT_FRAME_COUNTER,
    SND_EXT_ALINK_TX, SND_EXT_ALINK_CTRL_MSG, SND_EXT_ALINK_STATS_RESET, SND_EXT_ALINK_DIAG,
};
#define NEXT_IDS (sizeof(ext_ids) / sizeof(ext_ids[0]))

static uint8_t buf[SND_VC_MAX];

// SYS.LAST_RESULT (USB is the sender's only transport).
static struct { uint16_t id, addr; uint8_t is_get, result; } last;
static bool reboot_pending;

// ---------------------------------------------------------------------------
// The extension table
// ---------------------------------------------------------------------------

bool snd_ext_listed(uint16_t id) {
    for (unsigned i = 0; i < NEXT_IDS; i++)
        if (ext_ids[i] == id) return true;
    return false;
}

bool snd_ext_has_get(uint16_t id) {
    return id == SND_EXT_LIST || (snd_ext_listed(id) && id != SND_EXT_ALINK_STATS_RESET);
}

uint16_t snd_ext_set_len(uint16_t id) {
    switch (id) {
    case SND_EXT_ALINK_CONFIG:      return 8;
    case SND_EXT_ALINK_TX:          return 16;
    case SND_EXT_ALINK_CTRL_MSG:    return 16;
    case SND_EXT_ALINK_STATS_RESET: return 1;
    default:                        return 0;
    }
}

// The IN form of any listed id (or EXT_LIST), untruncated.
static uint8_t ext_get(uint16_t id, uint8_t *o, uint16_t *n) {
    switch (id) {
    case SND_EXT_LIST:
        *n = 0;
        for (unsigned i = 0; i < NEXT_IDS; i++) {
            o[(*n)++] = (uint8_t)ext_ids[i];
            o[(*n)++] = (uint8_t)(ext_ids[i] >> 8);
        }
        return SND_VC_OK;
    case SND_EXT_NOTIFY_FEATURES:
        memset(o, 0, 4);                // the sender sends no notifications
        *n = 4;
        return SND_VC_OK;
    default:
        return snd_hook_ext_get(id, o, n);
    }
}

// ---------------------------------------------------------------------------
// Page 0: the 1.1.6 requests
// ---------------------------------------------------------------------------

static snd_vc_action_t legacy_setup(const snd_vc_req_t *r, const uint8_t **out, uint16_t *n) {
    bool in = (r->bmRequestType & 0x80u) != 0;
    *out = buf;
    *n = 0;
    if (!in) {
        // Only an extension write with a listed OUT form and its exact length
        // reaches a data stage; anything else STALLs at SETUP (1.1).
        if (r->bRequest != LEGACY_EXTENSION) return SND_VC_STALL;
        uint16_t len = snd_ext_set_len(r->wValue);
        return len && r->wLength == len ? SND_VC_RECV : SND_VC_STALL;
    }
    snd_vc_action_t act = SND_VC_SEND;
    switch (r->bRequest) {
    case LEGACY_EXTENSION:
        if (!snd_ext_has_get(r->wValue) || ext_get(r->wValue, buf, n) != SND_VC_OK) return SND_VC_STALL;
        break;
    case LEGACY_GET_PLATFORM:   *n = snd_hook_ident(SND_IDENT_PLATFORM, buf); break;
    case LEGACY_GET_SERIAL:     *n = snd_hook_ident(SND_IDENT_SERIAL, buf); break;
    case LEGACY_GET_BUILD_INFO: *n = snd_hook_ident(SND_IDENT_BUILD_INFO, buf); break;
    case LEGACY_SAVE_OUTPUT_CONFIG:
        snd_hook_save();
        buf[0] = LEGACY_PRESET_OK;
        *n = 1;
        break;
    case LEGACY_ENTER_BOOTLOADER:
        buf[0] = 1;
        *n = 1;
        act = SND_VC_SEND_REBOOT;
        break;
    default:
        return SND_VC_STALL;            // a sender has no DSP
    }
    if (*n > r->wLength) *n = r->wLength;
    return act;
}

// ---------------------------------------------------------------------------
// The paged stack
// ---------------------------------------------------------------------------

static const row_t *find_row(uint16_t id) {
    for (unsigned i = 0; i < NROWS; i++)
        if (rows[i].id == id) return &rows[i];
    return 0;
}

static bool addr_ok(uint8_t kind, uint16_t a) {
    switch (kind) {
    case A_NONE: return a == 0;
    case A_PAGE: return a <= 0xFF;
    default:     return true;           // A_PAGE_CMD, A_INDEX
    }
}

static uint8_t check(bool is_get, uint16_t id, uint16_t addr, uint16_t in_len, const row_t **e) {
    const row_t *row = find_row(id);
    if (!row) return SND_VC_ERR_UNKNOWN;
    if (!(is_get ? row->get : row->set)) return SND_VC_ERR_DIRECTION;
    // USB is the sender's only transport, so VC_F_SET_USB_ONLY never refuses.
    if (!is_get && (in_len < row->set_min || in_len > row->set_max)) return SND_VC_ERR_LENGTH;
    if (!addr_ok(row->addr, addr)) return SND_VC_ERR_ADDRESS;
    *e = row;
    return SND_VC_OK;
}

static void record(bool is_get, uint16_t id, uint16_t addr, uint8_t result) {
    if (is_get && id == SND_SYS_LAST_RESULT) return;    // reading it must not overwrite it
    last.id = id;
    last.addr = addr;
    last.is_get = is_get ? 1 : 0;
    last.result = result;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static uint8_t run_get(const row_t *e, uint16_t addr, uint8_t *o, uint16_t *n) {
    if (e->get_len != LEN_VAR) {
        memset(o, 0, e->get_len);
        *n = e->get_len;
    }
    switch (e->h) {
    case H_PROTOCOL_INFO:
        memcpy(o, "DSPi", 4);
        o[4] = PROTOCOL_MAJOR;
        o[5] = PROTOCOL_MINOR;
        o[6] = (uint8_t)NPAGES;
        put16(o + 8, SND_VC_MAX);       // largest SET payload
        put16(o + 10, SND_VC_MAX);      // largest reply
        return SND_VC_OK;
    case H_PAGE_LIST:
        for (unsigned i = 0; i < NPAGES; i++) {
            o[2 * i] = pages[i][0];
            o[2 * i + 1] = pages[i][1];
        }
        *n = (uint16_t)(2 * NPAGES);
        return SND_VC_OK;
    case H_PAGE_COMMANDS: {
        bool found = false;
        for (unsigned i = 0; i < NPAGES; i++) found |= pages[i][0] == addr;
        if (!found) return SND_VC_ERR_ADDRESS;
        for (unsigned i = 0; i < NROWS; i++) {
            if ((rows[i].id >> 8) != addr) continue;
            uint8_t c = (uint8_t)rows[i].id;
            if (rows[i].get) o[c >> 3] |= (uint8_t)(1u << (c & 7));
            if (rows[i].set) o[32 + (c >> 3)] |= (uint8_t)(1u << (c & 7));
        }
        return SND_VC_OK;
    }
    case H_COMMAND_INFO: {
        const row_t *t = find_row(addr);
        if (!t) return SND_VC_ERR_ADDRESS;
        put16(o, t->flags);
        put16(o + 2, t->get ? t->get_len : 0);
        o[4] = t->set_min;
        o[5] = t->set_max;
        o[6] = t->addr;
        o[7] = (uint8_t)((t->get ? 1u : 0u) | (t->set ? 2u : 0u));
        return SND_VC_OK;
    }
    case H_LAST_RESULT:
        put16(o, last.id);
        o[2] = last.is_get;
        o[3] = last.result;
        put16(o + 4, last.addr);
        return SND_VC_OK;
    case H_PLATFORM:   snd_hook_ident(SND_IDENT_PLATFORM, o); return SND_VC_OK;
    case H_SERIAL:     snd_hook_ident(SND_IDENT_SERIAL, o); return SND_VC_OK;
    case H_BUILD_INFO: snd_hook_ident(SND_IDENT_BUILD_INFO, o); return SND_VC_OK;
    case H_TUNNEL:
        // 1.3.2: the id is listed (or EXT_LIST), it has an IN form.
        if (addr != SND_EXT_LIST && !snd_ext_listed(addr)) return SND_VC_ERR_ADDRESS;
        if (!snd_ext_has_get(addr)) return SND_VC_ERR_DIRECTION;
        return ext_get(addr, o, n);
    }
    return SND_VC_ERR_UNKNOWN;
}

static uint8_t run_set(const row_t *e, uint16_t addr, const uint8_t *in, uint16_t len) {
    switch (e->h) {
    case H_BOOTLOADER:
        reboot_pending = true;          // after the status stage (snd_vc_ack)
        return SND_VC_OK;
    case H_SAVE:
        snd_hook_save();
        return SND_VC_OK;
    case H_TUNNEL:
        // 1.3.2: listed, an OUT form, exactly its length; then the handler.
        if (addr != SND_EXT_LIST && !snd_ext_listed(addr)) return SND_VC_ERR_ADDRESS;
        if (!snd_ext_set_len(addr)) return SND_VC_ERR_DIRECTION;
        if (len != snd_ext_set_len(addr)) return SND_VC_ERR_LENGTH;
        return snd_hook_ext_set(addr, in);
    }
    return SND_VC_ERR_UNKNOWN;
}

static snd_vc_action_t paged_setup(const snd_vc_req_t *r, const uint8_t **out, uint16_t *n) {
    bool is_get = (r->bmRequestType & 0x80u) != 0;
    uint16_t id = (uint16_t)((r->wIndex & 0xFF00u) | r->bRequest);
    const row_t *e = 0;
    uint8_t res = check(is_get, id, r->wValue, is_get ? 0 : r->wLength, &e);
    if (res != SND_VC_OK) {
        record(is_get, id, r->wValue, res);
        return SND_VC_STALL;
    }
    if (is_get) {
        uint16_t len = 0;
        res = run_get(e, r->wValue, buf, &len);
        record(true, id, r->wValue, res);
        if (res != SND_VC_OK) return SND_VC_STALL;
        *out = buf;
        *n = len > r->wLength ? r->wLength : len;
        return SND_VC_SEND;
    }
    if (r->wLength == 0) {
        // Zero-length SET: the handler runs now, so a refusal still STALLs.
        res = run_set(e, r->wValue, 0, 0);
        record(false, id, r->wValue, res);
        return res == SND_VC_OK ? SND_VC_STATUS : SND_VC_STALL;
    }
    return SND_VC_RECV;
}

static bool paged_data(const snd_vc_req_t *r, const uint8_t *data) {
    uint16_t id = (uint16_t)((r->wIndex & 0xFF00u) | r->bRequest);
    const row_t *e = 0;
    uint8_t res = check(false, id, r->wValue, r->wLength, &e);
    if (res == SND_VC_OK) res = run_set(e, r->wValue, data, r->wLength);
    record(false, id, r->wValue, res);
    return res == SND_VC_OK;
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

snd_vc_action_t snd_vc_setup(const snd_vc_req_t *r, const uint8_t **out, uint16_t *n) {
    // A reboot whose status stage never completed must not fire later.
    reboot_pending = false;
    *out = buf;
    *n = 0;
    return (r->wIndex >> 8) == 0 ? legacy_setup(r, out, n) : paged_setup(r, out, n);
}

bool snd_vc_data(const snd_vc_req_t *r, const uint8_t *data) {
    if ((r->wIndex >> 8) != 0) return paged_data(r, data);
    if (r->bRequest != LEGACY_EXTENSION || r->wLength != snd_ext_set_len(r->wValue)) return false;
    return snd_hook_ext_set(r->wValue, data) == SND_VC_OK;
}

bool snd_vc_ack(const snd_vc_req_t *r) {
    if ((r->bmRequestType & 0x80u) || !reboot_pending) return false;
    reboot_pending = false;
    return true;
}
