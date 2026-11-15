#!/usr/bin/env python3
"""Fake DSPi for the MIDI control channel (no hardware).

Emulates the firmware's SysEx behaviour (midi_control.c) closely enough to
exercise midi_ctrl.py, measure.py and soak.py without a device:

  * ignores SysEx for another manufacturer / product;
  * FRAME_ERROR with tag 0x7F for a bad version, a host tag of 0 or a
    truncated header; FRAME_ERROR with the tag for malformed packing or a
    frame that is truncated / has trailing bytes; CRC_ERROR with the tag;
  * OVERSIZE for a non-bulk SET payload > 64 bytes;
  * USB-path parity: 0xF5 / 0xF7 accepted (recorded, never applied), chunked
    0xA2 / 0xA3 sessions, 0x0F (RTA bands-all) larger than 132 bytes;
  * a small command set: 0x7F platform, 0xD2/0xD3 master volume (with a
    PARAM_CHANGED notification, src MIDI), 0xA0/0xA1 bulk (BULK_INVALIDATED
    after a SET, src MIDI), 0x50 status, 0xB0/0xB2 stats, 0xF0 bootloader
    (BLOCKED, as the firmware's FWSEC1 gate answers over MIDI), 0x00
    extension channel (EXT_LIST, pair map, core-voltage status, notify
    feature flags);
  * optional alink block 0x8000-0x800A and events 0xE1 / 0xE2
    (FakeDevice(alink="receiver" | "sender"), `--alink` in pty mode); without
    it the IDs are refused like on today's firmware;
  * optional BUSY / BULK_LOCKED injection.

In-process use: FakePort(FakeDevice()) has the RawMidiPort interface.
Cross-process use: `fake_device.py --pty` prints a pty path that
`midi_ctrl.py --port <path>` (and measure.py / soak.py) can open.
"""

from __future__ import annotations

import argparse
import os
import random
import select
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # python -I drops the script dir

import midi_ctrl as mc

WIRE_BULK_SIZE = 6136                   # WireBulkParams V32 (bulk_v32.bin)
MASTER_VOL_OFFSET = 0x0AD4              # illustrative wire offset
PLATFORM = bytes([1, 1, 0x16, 4, 1, 6, 0])   # RP2350, v1.1.6, 4 outputs
NUM_OUTPUTS = 9                         # RP2350: 8 slots + PDM sub (EXT_IDENTIFY tone bound)
PRESET_SLOTS = 10
MAX_DELAY_SAMPLES = 2048                # RP2350 (config.h)

# -- alink block (extension_channel_spec.md 2.8) ------------------------------
ALINK_RX_IDS = (0x8000, 0x8001, 0x8002, 0x8003, 0x8004, 0x8005, 0x8006, 0x8008, 0x8009, 0x800A)
ALINK_TX_IDS = (0x8000, 0x8001, 0x8002, 0x8005, 0x8007, 0x8008, 0x8009, 0x800A)
ALINK_FEAT_RX = 0x23                    # PHASE_LOCK | APPLY_AT_FRAME | LOS_PIN
ALINK_FEAT_TX = 0x38                    # FAN_OUT | TEST_PATTERNS | LOS_PIN
D_LOCAL_MIN, D_LOCAL_MAX = 112, 1024    # 1 ms latency option (SYS2 3.2)
APPLY_QUEUE = 4
# (class, rate code) -> C: a copy of alink_max_channels() (alink_protocol_spec.md 5.7, the only
# capacity table); keep it in step with firmware/DSPi/alink/alink_codec.c.
MAX_CHANNELS = {(1, 0): 8, (2, 0): 19, (3, 0): 41, (1, 1): 8, (2, 1): 18, (3, 1): 38,
                (1, 2): 3, (2, 2): 8, (3, 2): 18}
SF_LEN = 32                             # frames per superframe
RATE_LEAD = 2400                        # frames of lead for a RATE the sender schedules itself
OP_RATE = 6


def alink_pin_ok(p: int) -> bool:
    return p == 0xFF or (p <= 28 and p not in (23, 24, 25))   # Pico 2 header GPIOs


class FakeAlink:
    """Control-plane model of the alink block: validation, state, counters.

    No audio and no timing: tests move the model with lock(), lose(),
    advance() and read the 0xE1 / 0xE2 notifications the device sends.
    The 250 ms rate limit of 0xE1 is not modelled (one event per call).
    """

    def __init__(self, role: str):
        self.sender = role == "sender"
        self.ids = ALINK_TX_IDS if self.sender else ALINK_RX_IDS
        self.config = bytearray([0, 0xFF, 0, 0xFF, 0xFF, 0xFF, 0x03, 0])
        self.slots = bytearray(range(8))
        self.mode, self.d_local, self.hops_offset, self.phase_state = 1, 128, 0, 0
        self.state, self.cls, self.rate, self.chans, self.hops = 0, 0, 0xFF, 0, 0
        self.frame = 0
        self.sf_flags = 0
        self.counters = [0] * 6         # lock, loss, crc8, crc16, bit_slip, superframe
        self.bench = [3, 1, 2, 0]       # DIAG: comma, counter, code, idle errors
        self.vcxo_ppm, self.phase_error, self.fill = 0.0, 0.0, 0
        self.hop_delay = 2              # sender in fan-out: frames from receiving F to re-sending it (LK4: 2-3)
        self.queue = []                 # pending (frame, op, arg, value)
        self.last = (0, 0, 0)           # (op, result, frame)
        self.tx = bytearray(struct.pack("<HBBBBBBHHBBH", 0, 1, 8, 1, 0, 0, 0xFF, 0x00FF, 0, 0, 0, 0))
        self.injected = None            # sender: the pending superframe apply (frame, op, arg, value)
        self.tx_layout = (1, 1, 8)      # sender: (class, rate, C) the stream carries now
        self.tx_pending = None          # sender: the layout its own RATE switches to

    @property
    def role(self) -> int:
        if self.sender:
            return 4 if self.config[0] else 2
        return 3 if self.config[3] != 0xFF else 1

    # -- IN ------------------------------------------------------------------
    def info(self) -> bytes:
        rx = self.config[1] if self.config[1] != 0xFF else (20 if self.sender else 0)
        return (struct.pack("<BBBBBBHBBH", self.role, 1, 0x07,
                            8 if self.sender else 1, 64, 0 if self.sender else 8, 0, rx, self.config[3], 0)
                + struct.pack("<IHH", ALINK_FEAT_TX if self.sender else ALINK_FEAT_RX,
                              0 if self.sender else D_LOCAL_MIN, 0 if self.sender else D_LOCAL_MAX)
                + bytes(12))

    def status(self) -> bytes:
        los = 0xFF if self.config[4] == 0xFF else (1 if self.state == 3 else 0)
        return (struct.pack("<8BQ", self.state, self.cls, self.rate, self.chans, self.hops, los,
                            0x5A if self.state else 0, self.sf_flags, self.frame)
                + struct.pack("<6IffHBB", *self.counters, self.vcxo_ppm, self.phase_error, self.fill,
                              self.hop_delay if self.role == 4 and self.state == 2 else 0, 0))

    def phase(self) -> bytes:
        below = int(self.d_local + self.hops_offset < D_LOCAL_MIN)
        return struct.pack("<BBHhBB", self.mode, 0, self.d_local, self.hops_offset, below, self.phase_state)

    def frame_counter(self) -> bytes:
        f = self.frame if self.state in (2, 3) or self.sender else 0
        return struct.pack("<QII", f, 12_345_678, (self.frame - 1000) & 0xFFFFFFFF)

    def apply_status(self) -> bytes:
        op, res, frame = self.last
        return struct.pack("<BBBBQI", len(self.queue), op, res, 0, frame, 0)

    def ctrl_msg(self) -> bytes:
        # superframe bytes 2-7, 2 reserved, bytes 16-27, u32 F_low32 of its first frame
        if self.state == 0 and not self.sender:
            return bytes(24)
        sf_frame = self.frame & ~(SF_LEN - 1)
        target, op, arg, value = self.injected or (0, 0, 0, 0)
        cls, rate, chans = self.tx_layout if self.sender else (self.cls, self.rate, self.chans)
        return (struct.pack("<BBBBBB", self.sf_flags, rate, chans, cls, self.hops, 0x5A)
                + bytes(2) + struct.pack("<QBBh", target, op, arg, value) + struct.pack("<I", sf_frame & 0xFFFFFFFF))

    def diag(self) -> bytes:
        # v1: offset 32 = trim scan (lo, hi, used, scan_state, clean_mask), 40-55 bench counters
        return (struct.pack("<BBH3f4I", 1, 7, 0, self.vcxo_ppm, 0.0, 0.0, 4096, 3072, 8192, 1 << 20)
                + struct.pack("<bbbBI4I8x", -4, 5, 0, 2, 0x3FF << 5, *self.bench))

    def get(self, sub: int) -> bytes | None:
        return {0x8000: self.info, 0x8001: self.status, 0x8002: lambda: bytes(self.config),
                0x8003: lambda: bytes(self.slots), 0x8004: self.phase, 0x8005: self.frame_counter,
                0x8006: self.apply_status, 0x8007: lambda: bytes(self.tx), 0x8008: self.ctrl_msg,
                0x800A: self.diag}.get(sub, lambda: None)()

    # -- OUT -----------------------------------------------------------------
    SET_LEN = {0x8002: 8, 0x8003: 8, 0x8004: 8, 0x8006: 16, 0x8007: 16, 0x8008: 16, 0x8009: 1}

    def set_len(self, sub: int) -> int | None:
        if sub not in self.ids or (sub == 0x8008 and not self.sender):
            return None
        return self.SET_LEN.get(sub)

    def _apply_ok(self, op: int, arg: int, value: int) -> bool:
        """Arguments of ops 1-5, 7, 8 (2.8.7); RATE is checked by its callers."""
        if op in (1, 2, 8):
            return arg == 0 and value == 0
        if op == 3:
            return arg < PRESET_SLOTS and value == 0
        if op == 4:
            return arg == 0 and -12800 <= value <= 0
        if op == 5:
            return arg < NUM_OUTPUTS and 0 <= value < MAX_DELAY_SAMPLES
        if op == 7:
            return arg < NUM_OUTPUTS and value == 0
        return False

    def set(self, sub: int, p: bytes, dev: "FakeDevice") -> bool:
        if sub == 0x8002:
            enable, rx, cls, tx, los, txdis, flags, res = p
            pins = [x for x in (rx, tx, los, txdis) if x != 0xFF]
            if (enable > 1 or cls > 3 or flags & ~0x03 or res or not all(map(alink_pin_ok, (rx, tx, los, txdis)))
                    or len(set(pins)) != len(pins) or (tx != 0xFF and self.sender)):
                return False
            was = self.config[0]
            self.config[:] = p
            if was and not enable and self.state:
                dev.alink_event(0, 7)
            return True
        if sub == 0x8003:
            if any(x >= 64 and x != 0xFF for x in p):
                return False
            self.slots[:] = p
            return True
        if sub == 0x8004:
            mode, res, d_local, hops_off, flags, _state = struct.unpack("<BBHhBB", p)
            if mode > 1 or res or flags or not -64 <= hops_off <= 64 \
                    or not D_LOCAL_MIN <= d_local + hops_off <= D_LOCAL_MAX:
                return False
            changed = (mode, d_local, hops_off) != (self.mode, self.d_local, self.hops_offset)
            self.mode, self.d_local, self.hops_offset = mode, d_local, hops_off
            if changed and self.state == 2 and self.phase_state == 2:
                dev.alink_event(2, 5)   # re-anchor on the new delay
            return True
        if sub == 0x8006:
            frame, op, arg, value = struct.unpack_from("<QBBh", p)
            if any(p[12:]) or op > 8:
                return False
            if op == 0:
                for f, o, _a, _v in self.queue:
                    self.last = (o, 2, f)
                    dev.notify_apply(o, 2, f)
                self.queue.clear()
                return True
            if not self._apply_ok(op, arg, value) or self.state != 2 or len(self.queue) >= APPLY_QUEUE:
                return False
            self.queue.append((frame, op, arg, value))
            self.advance(0, dev)
            return True
        if sub == 0x8007:
            mask, cls, chans, rate, test, smap, sync, txdis, _los, _alt, flags, res = struct.unpack("<HBBBBBBHHBBH", p)
            if (mask >> 8 or not 1 <= cls <= 3 or rate > 2 or not 1 <= chans <= MAX_CHANNELS[(cls, rate)]
                    or test > 3 or smap or not alink_pin_ok(sync) or txdis >> 8 or flags & ~1 or res):
                return False
            # As sender_vendor.c set_tx(): "a change" is against the last accepted
            # request (self.tx), not the stream's layout, so writing the stream's
            # layout back while its RATE is pending is a change and is refused.
            running = mask != 0 or struct.unpack_from("<H", self.tx)[0] != 0
            req_cls, req_chans, req_rate = self.tx[2], self.tx[3], self.tx[4]
            old_chans = self.tx_layout[2]
            if cls == req_cls and (rate, chans) == (req_rate, req_chans):
                pass                               # no layout change (a pending RATE stays)
            elif running and cls == req_cls:
                if self.injected:
                    return False                   # one apply at a time
                at = (self._next_sf() + RATE_LEAD + SF_LEN - 1) & ~(SF_LEN - 1)
                self.injected = (at, OP_RATE, rate, chans if chans != old_chans else 0)
                self.tx_pending = (cls, rate, chans)
            else:
                # not running, or a class change: the outputs (re)start with the new layout at once
                self.tx_layout = (cls, rate, chans)
            keep = self.tx[10:13]                  # IN-only: los mask, usb_alt
            self.tx[:] = p
            self.tx[10:13] = keep
            return True
        if sub == 0x8008:
            # alink_tx_schedule(): one pending apply, a full superframe of lead,
            # RATE on a superframe boundary with value 0 and a layout that fits.
            frame, op, arg, value, sysflags = struct.unpack_from("<QBBhB", p)
            if op > 8 or sysflags & ~1 or any(p[13:]):
                return False
            if op:
                cls, _rate, chans = self.tx_layout
                if op == OP_RATE:
                    ok = arg <= 2 and value == 0 and frame % SF_LEN == 0 and chans <= MAX_CHANNELS[(cls, arg)]
                else:
                    ok = self._apply_ok(op, arg, value)
                if not ok or self.injected or frame < self._next_sf() + SF_LEN:
                    return False
                self.injected = (frame, op, arg, value)
                if op == OP_RATE:
                    self.tx[4] = arg               # the request follows (set_ctrl())
            self.sf_flags = (self.sf_flags & ~1) | sysflags
            return True
        if sub == 0x8009:
            if p[0] != 1:
                return False
            self.counters = [0] * 6
            self.bench = [0] * 4
            return True
        return False

    def _next_sf(self) -> int:
        return (self.frame + SF_LEN - 1) & ~(SF_LEN - 1)

    # -- model driving (tests) -------------------------------------------------
    def advance(self, frames: int, dev: "FakeDevice") -> None:
        """Move F forward; due ops apply in frame order (late if already past when queued).

        Sender: the pending superframe apply ends when F reaches its frame; a
        RATE the sender scheduled itself switches the layout there.
        """
        late = [q for q in self.queue if q[0] < self.frame]
        self.frame += frames
        if self.injected and self.frame >= self.injected[0]:
            if self.injected[1] == OP_RATE and self.tx_pending:
                self.tx_layout, self.tx_pending = self.tx_pending, None
            elif self.injected[1] == OP_RATE:
                cls, _rate, chans = self.tx_layout
                self.tx_layout = (cls, self.injected[2], chans)
            self.injected = None
        due = sorted((q for q in self.queue if q[0] <= self.frame), key=lambda q: q[0])
        for q in due:
            self.queue.remove(q)
            res = 1 if q in late else 0
            self.last = (q[1], res, q[0])
            dev.notify_apply(q[1], res, q[0])


def default_bulk() -> bytes:
    path = os.environ.get("DSPI_BULK_FIXTURE")
    if path and os.path.exists(path):
        with open(path, "rb") as f:
            return f.read()
    return bytes((i * 7 + 3) & 0xFF for i in range(WIRE_BULK_SIZE))


class FakeDevice:
    def __init__(self, mfr: bytes = mc.MFR_DEV, bulk: bytes | None = None, alink: str | None = None):
        self.mfr = mfr
        self.split = mc.SysexSplitter()
        self.out = bytearray()
        self.bulk = bytearray(bulk if bulk is not None else default_bulk())
        self.master_db = -30.0
        self.seq = 0
        self.inject = []            # statuses returned (once each) before real handling
        self.requests = 0
        self.iface_config = []      # (bReq, payload) of 0xF5 / 0xF7 received
        self.chunk_get = False
        self.chunk_set = None       # bytes received so far, or None
        self.chunk_buf = bytearray()
        self.pair_map = bytes(4)    # EXT_CROSSFEED_PAIR_MAP (normalization not modelled)
        self.identify_s = 0         # EXT_IDENTIFY blink time (no countdown modelled)
        self.identify_tone = mc.IDENTIFY_TONE_NONE
        self.alink = FakeAlink(alink) if alink else None   # None: alink IDs refused (today's firmware)

    # -- output -----------------------------------------------------------
    def _emit(self, tag: int, frame: bytes) -> None:
        self.out += mc.encode(frame, tag, self.mfr)

    def _reply(self, tag: int, is_get: bool, status: int, payload: bytes = b"") -> None:
        self._emit(tag, mc.response_frame(mc.RESP_GET if is_get else mc.RESP_SET, status, payload))

    def notify(self, pkt: bytes) -> None:
        self._emit(mc.TAG_NOTIFY, mc.response_frame(mc.TYPE_NOTIFY, 0, pkt))

    def _notify_param(self, off: int, value: bytes, src: int = 10) -> None:
        pkt = bytes([2, 0x02, 0, self.seq & 0xFF]) + struct.pack("<HH", off, len(value)) + \
            bytes([src, 0, 0, 0]) + value
        self.seq += 1
        self.notify(pkt)

    def _notify_bulk_invalidated(self, src: int = 10) -> None:
        # A bulk SET's invalidation carries the SET's dispatch source (MIDI
        # here; NOTIFY_FEAT_BULK_SOURCE).  Older firmware sent BULK_SET (2).
        self.notify(bytes([2, 0x03, 0, self.seq & 0xFF, src, 0, 0, 0]))
        self.seq += 1

    def alink_event(self, state: int, reason: int) -> None:
        """Set the alink state and send NOTIFY_EVT_ALINK_STATE (0xE1)."""
        ln = self.alink
        if state == 2 and ln.state != 2:
            ln.counters[0] += 1
            ln.cls, ln.rate, ln.chans, ln.hops = ln.cls or 1, 1, 8, 0
            ln.frame = ln.frame or 1_000_000
            ln.sf_flags |= 0x08
        if state in (0, 3) and ln.state == 2:
            ln.counters[1] += 1
        ln.state = state
        ln.phase_state = 2 if state == 2 and ln.mode == 1 else 0
        if state == 0:
            ln.cls, ln.rate, ln.chans = 0, 0xFF, 0
        self.notify(bytes([2, mc.NOTIFY_EVT_ALINK_STATE, 0, self.seq & 0xFF, state, ln.cls, reason, 0])
                    + struct.pack("<I", ln.frame & 0xFFFFFFFF))
        self.seq += 1

    def notify_apply(self, op: int, result: int, frame: int) -> None:
        """NOTIFY_EVT_APPLY_RESULT (0xE2) for one applied or cancelled op."""
        self.notify(bytes([2, mc.NOTIFY_EVT_APPLY_RESULT, 0, self.seq & 0xFF, op, result, 0, 0])
                    + struct.pack("<I", frame & 0xFFFFFFFF))
        self.seq += 1

    # -- input --------------------------------------------------------------
    def feed(self, data: bytes) -> None:
        for msg in self.split.feed(data):
            self._message(msg)

    def _message(self, msg: bytes) -> None:
        body = msg[1:-1]
        ml = len(self.mfr)
        if body[:ml] != self.mfr or body[ml:ml + 3] != mc.MAGIC:
            return                                   # not ours: silent
        if len(body) < ml + 5 or body[ml + 3] != mc.VERSION or body[ml + 4] == 0:
            self._reply(mc.TAG_UNKNOWN, False, mc.ST_FRAME)
            return
        tag = body[ml + 4]
        try:
            frame = mc.unpack7(body[ml + 5:])
        except ValueError:
            self._reply(tag, False, mc.ST_FRAME)
            return
        if not frame or frame[0] not in (mc.TYPE_SET, mc.TYPE_GET):
            self._reply(tag, False, mc.ST_FRAME)
            return
        is_get = frame[0] == mc.TYPE_GET
        if len(frame) < 10:
            self._reply(tag, is_get, mc.ST_FRAME)
            return
        req = mc.parse_request(frame)
        expect = 10 if is_get else 10 + req["wlength"]
        if len(frame) != expect:
            self._reply(tag, is_get, mc.ST_FRAME)
            return
        if not req["crc_ok"]:
            self._reply(tag, is_get, mc.ST_CRC)
            return
        self.requests += 1
        self._dispatch(tag, is_get, req)

    def _dispatch(self, tag: int, is_get: bool, r: dict) -> None:
        breq, wv, payload = r["breq"], r["wvalue"], r["payload"]
        is_bulk_set = (not is_get and breq == mc.REQ_SET_ALL_PARAMS and r["wlength"] == len(self.bulk))
        is_chunk = breq in (0xA2, 0xA3)
        if not is_chunk:
            self.chunk_get, self.chunk_set = False, None     # non-chunk request ends sessions
        if not is_get and len(payload) > mc.NON_BULK_SET_MAX and not is_bulk_set and not is_chunk:
            return self._reply(tag, False, mc.ST_OVERSIZE)
        if self.inject:
            return self._reply(tag, is_get, self.inject.pop(0))
        if not is_get and breq in (0xF5, 0xF7):
            self.iface_config.append((breq, payload))
            return self._reply(tag, False, mc.ST_OK)
        if is_get and breq == 0xA2:
            off, n = wv, r["wlength"]
            if n == 0 or off >= len(self.bulk):
                return self._reply(tag, True, mc.ST_ERROR)
            if off == 0:
                self.chunk_buf = bytearray(self.bulk)
                self.chunk_get = True
            elif not self.chunk_get:
                return self._reply(tag, True, mc.ST_ERROR)
            data = bytes(self.chunk_buf[off:off + n])
            if off + len(data) >= len(self.bulk):
                self.chunk_get = False
            return self._reply(tag, True, mc.ST_OK, data)
        if not is_get and breq == 0xA3:
            off = wv
            if not payload or off + len(payload) > len(self.bulk):
                return self._reply(tag, False, mc.ST_ERROR)
            if off == 0:
                self.chunk_set, self.chunk_get = 0, False
                self.chunk_buf = bytearray(len(self.bulk))
            elif self.chunk_set is None or off != self.chunk_set:
                return self._reply(tag, False, mc.ST_ERROR)
            self.chunk_buf[off:off + len(payload)] = payload
            self.chunk_set = off + len(payload)
            if self.chunk_set >= len(self.bulk):
                self.bulk[:] = self.chunk_buf
                self.chunk_set = None
                self._reply(tag, False, mc.ST_OK)
                self._notify_bulk_invalidated()
                return
            return self._reply(tag, False, mc.ST_OK)

        if breq == mc.REQ_EXTENSION:
            return self._extension(tag, is_get, wv, payload, r["wlength"])

        if is_get:
            cap = r["wlength"]
            if breq == mc.REQ_GET_PLATFORM:
                data = PLATFORM
            elif breq == mc.REQ_GET_MASTER_VOLUME:
                data = struct.pack("<f", self.master_db)
            elif breq == mc.REQ_GET_ALL_PARAMS:
                data = bytes(self.bulk)
            elif breq == mc.REQ_GET_STATUS:
                data = bytes(26) if wv == 9 else bytes(4)
            elif breq == 0x0F:                          # RTA bands-all (4 x 75 B)
                data = bytes(300)
            elif breq == mc.REQ_GET_BUFFER_STATS:
                data = bytes(44)
            elif breq == mc.REQ_GET_USB_ERROR_STATS:
                data = bytes(16)
            elif breq == mc.REQ_ENTER_BOOTLOADER:
                # FWSEC1: bootloader entry is USB-vendor only; MIDI gets BLOCKED.
                return self._reply(tag, True, mc.ST_BLOCKED)
            else:
                return self._reply(tag, True, mc.ST_ERROR)
            if cap:
                data = data[:cap]
            return self._reply(tag, True, mc.ST_OK, data)

        if breq == mc.REQ_SET_MASTER_VOLUME and len(payload) == 4:
            db = struct.unpack("<f", payload)[0]
            if not (-128.0 <= db <= 0.0):
                return self._reply(tag, False, mc.ST_ERROR)
            changed = db != self.master_db
            self.master_db = db
            self._reply(tag, False, mc.ST_OK)
            if changed:
                self._notify_param(MASTER_VOL_OFFSET, payload)
            return
        if is_bulk_set:
            self.bulk[:] = payload
            self._reply(tag, False, mc.ST_OK)
            self._notify_bulk_invalidated()
            return
        return self._reply(tag, False, mc.ST_ERROR)


    def _extension(self, tag: int, is_get: bool, sub: int, payload: bytes, cap: int) -> None:
        # Extension channel (extension_channel_spec.md): unknown sub-command,
        # wrong direction or wrong OUT length -> ERROR (USB: STALL).
        set_len = {mc.EXT_CROSSFEED_PAIR_MAP: 4, mc.EXT_CORE_VREG: 2, mc.EXT_IDENTIFY: 4}
        if sub >= 0x8000:
            return self._alink_ext(tag, is_get, sub, payload, cap)
        if is_get:
            if sub == mc.EXT_LIST:
                ids = (mc.EXT_CROSSFEED_PAIR_MAP, mc.EXT_CORE_VREG, mc.EXT_NOTIFY_FEATURES,
                       mc.EXT_IDENTIFY, mc.EXT_RESET_INFO, mc.EXT_FIRMWARE_FIXES) + \
                    (self.alink.ids if self.alink else ())
                data = struct.pack(f"<{len(ids)}H", *ids)
            elif sub == mc.EXT_CROSSFEED_PAIR_MAP:
                data = self.pair_map
            elif sub == mc.EXT_CORE_VREG:
                data = bytes([0x0D, 0, 0x0D, 0]) + bytes(12)   # live/default 1.20 V
            elif sub == mc.EXT_NOTIFY_FEATURES:
                feats = mc.NOTIFY_FEAT_BULK_SOURCE | mc.NOTIFY_FEAT_CLIP_EVENT
                if self.alink:
                    feats |= mc.NOTIFY_FEAT_ALINK_STATE | mc.NOTIFY_FEAT_APPLY_RESULT
                data = struct.pack("<I", feats)
            elif sub == mc.EXT_IDENTIFY:
                # RP2350 A2 on a pico2 build; the hw_rev string as a board build sets it
                data = (struct.pack("<BBBBI", self.identify_s, self.identify_tone, 1, 2, 0x20004927)
                        + b"pico2".ljust(16, b"\0") + b"Board rev A".ljust(32, b"\0"))
            elif sub == mc.EXT_RESET_INFO:
                # a warm reset after our watchdog fired: hang 1, previous run 3600 s
                data = struct.pack("<BBBBIIII", 4, 1, 1, 1, 12, 3600, 0x01000000, 0x101)
            elif sub == mc.EXT_FIRMWARE_FIXES:
                data = struct.pack("<I", mc.FW_FIX_BULK_DB_POWF)
            else:
                return self._reply(tag, True, mc.ST_ERROR)
            return self._reply(tag, True, mc.ST_OK, data[:cap] if cap else data)
        if set_len.get(sub) != len(payload):
            return self._reply(tag, False, mc.ST_ERROR)
        if sub == mc.EXT_CROSSFEED_PAIR_MAP:
            self.pair_map = bytes(payload)
        elif sub == mc.EXT_CORE_VREG:
            # FWSEC1: over MIDI, SET / TRIAL take at most 1.20 V (0x0D);
            # SET 0 = build default.  Ops 2..4 are allowed.
            op, val = payload[0], payload[1]
            if op > 4 or (op in (0, 1) and val > 0x0D) or (op == 0 and val and val < 0x0D) \
                    or (op == 1 and val < 0x0C) or (op == 3 and val > 1):
                return self._reply(tag, False, mc.ST_ERROR)
        elif sub == mc.EXT_IDENTIFY:
            secs, tone = payload[0], payload[1]
            if payload[2] or payload[3] or (tone != mc.IDENTIFY_TONE_NONE and tone >= 9):
                return self._reply(tag, False, mc.ST_ERROR)
            self.identify_s = min(secs, 120)
            if tone != mc.IDENTIFY_TONE_NONE:
                self.identify_tone = tone
        return self._reply(tag, False, mc.ST_OK)


    def _alink_ext(self, tag: int, is_get: bool, sub: int, payload: bytes, cap: int) -> None:
        # Alink block: an ID the build does not list, the wrong direction or the
        # wrong OUT length is refused like any unknown sub-command.
        ln = self.alink
        if ln is None or sub not in ln.ids:
            return self._reply(tag, is_get, mc.ST_ERROR)
        if is_get:
            data = ln.get(sub)
            if data is None:
                return self._reply(tag, True, mc.ST_ERROR)
            return self._reply(tag, True, mc.ST_OK, data[:cap] if cap else data)
        if ln.set_len(sub) != len(payload) or not ln.set(sub, payload, self):
            return self._reply(tag, False, mc.ST_ERROR)
        return self._reply(tag, False, mc.ST_OK)


class FakePort:
    """RawMidiPort-compatible in-process port.  `chunk` fragments reads."""

    def __init__(self, dev: FakeDevice, chunk: int | None = None, seed: int = 1):
        self.dev = dev
        self.chunk = chunk
        self.rng = random.Random(seed)

    def write(self, data: bytes) -> None:
        self.dev.feed(data)

    def read(self, timeout: float) -> bytes:
        if not self.dev.out:
            time.sleep(min(max(timeout, 0.0), 0.005))
            return b""
        n = len(self.dev.out)
        if self.chunk:
            n = min(n, self.rng.randint(1, self.chunk))
        data = bytes(self.dev.out[:n])
        del self.dev.out[:n]
        return data

    def close(self) -> None:
        pass


def serve_pty(dev: FakeDevice, notify_every: float) -> None:
    import tty
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    print(os.ttyname(slave), flush=True)
    next_notify = time.monotonic() + notify_every if notify_every else None
    while True:
        timeout = 0.05
        r, _, _ = select.select([master], [], [], timeout)
        if r:
            try:
                data = os.read(master, 4096)
            except OSError:
                data = b""
            if data:
                dev.feed(data)
        if next_notify and time.monotonic() >= next_notify:
            dev._notify_param(0x0010, b"\x00\x00\x80\x3f", src=5)
            next_notify += notify_every
        if dev.out:
            n = os.write(master, bytes(dev.out))
            del dev.out[:n]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pty", action="store_true", help="serve on a pty and print its path")
    ap.add_argument("--mfr", default="7d")
    ap.add_argument("--notify-every", type=float, default=0.0,
                    help="push a GPIO PARAM_CHANGED notification every N s")
    ap.add_argument("--alink", choices=("receiver", "sender"),
                    help="add the alink block 0x8000+ (a receiver starts locked)")
    args = ap.parse_args()
    if not args.pty:
        ap.error("only --pty mode is a standalone program; import FakeDevice otherwise")
    dev = FakeDevice(bytes.fromhex(args.mfr), alink=args.alink)
    if args.alink == "receiver":
        dev.alink.config[0] = 1
        dev.alink_event(2, 1)
        dev.out.clear()
    serve_pty(dev, args.notify_every)
    return 0


if __name__ == "__main__":
    sys.exit(main())
