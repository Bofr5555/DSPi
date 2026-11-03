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
    after a SET), 0x50 status, 0xB0/0xB2 stats, 0xF0 bootloader;
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


def default_bulk() -> bytes:
    path = os.environ.get("DSPI_BULK_FIXTURE")
    if path and os.path.exists(path):
        with open(path, "rb") as f:
            return f.read()
    return bytes((i * 7 + 3) & 0xFF for i in range(WIRE_BULK_SIZE))


class FakeDevice:
    def __init__(self, mfr: bytes = mc.MFR_DEV, bulk: bytes | None = None):
        self.mfr = mfr
        self.split = mc.SysexSplitter()
        self.out = bytearray()
        self.bulk = bytearray(bulk if bulk is not None else default_bulk())
        self.master_db = -30.0
        self.seq = 0
        self.inject = []            # statuses returned (once each) before real handling
        self.requests = 0
        self.in_bootloader = False
        self.iface_config = []      # (bReq, payload) of 0xF5 / 0xF7 received
        self.chunk_get = False
        self.chunk_set = None       # bytes received so far, or None
        self.chunk_buf = bytearray()

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

    def _notify_bulk_invalidated(self, src: int = 2) -> None:
        self.notify(bytes([2, 0x03, 0, self.seq & 0xFF, src, 0, 0, 0]))
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
                self.in_bootloader = True
                data = b"\x01"
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
    args = ap.parse_args()
    if not args.pty:
        ap.error("only --pty mode is a standalone program; import FakeDevice otherwise")
    serve_pty(FakeDevice(bytes.fromhex(args.mfr)), args.notify_every)
    return 0


if __name__ == "__main__":
    sys.exit(main())
