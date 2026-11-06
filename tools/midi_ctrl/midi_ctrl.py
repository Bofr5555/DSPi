#!/usr/bin/env python3
"""DSPi USB MIDI control channel: host library and CLI (contract C1).

Standard library only.  Talks to the DSPi's "DSPi Control" USB MIDI port
through the ALSA rawmidi node (/dev/snd/midiC<card>D<dev>), which passes
SysEx through untouched, so no MIDI library is needed.

One SysEx message carries one UART control frame (uart_control.h) from the
type byte through the CRC16, 7-bit packed:

    F0 <mfr> 'D' 'S' 'P' 01 <tag> <packed frame> F7

See Documentation/Features/usb_midi_control_spec.md in the firmware tree.

CLI examples (inside the container, see Containerfile / run.sh):
    midi_ctrl.py list
    midi_ctrl.py get 0x7F                       # platform
    midi_ctrl.py get 0xA0 --out bulk.bin        # bulk read
    midi_ctrl.py set 0xD2 --float -20           # master volume -20 dB
    midi_ctrl.py set 0xA1 --in bulk.bin         # bulk write
    midi_ctrl.py notify --seconds 30
    midi_ctrl.py enter-bootloader               # asks for confirmation
"""

from __future__ import annotations

import argparse
import collections
import errno
import glob
import os
import re
import select
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

SYSEX_START = 0xF0
SYSEX_END = 0xF7
MAGIC = b"DSP"
VERSION = 0x01
MFR_DEV = b"\x7d"          # non-commercial / development ID (build default)
TAG_NOTIFY = 0x00
TAG_UNKNOWN = 0x7F         # error reply to a message whose tag was not read

TYPE_SET = 0x01
TYPE_GET = 0x02
TYPE_NOTIFY = 0x40
RESP_SET = 0x81
RESP_GET = 0x82

STATUS = {
    0x00: "OK",
    0x01: "BUSY",
    0x02: "ERROR",
    0x03: "BLOCKED",
    0x04: "BULK_LOCKED",
    0x05: "CRC_ERROR",
    0x06: "OVERSIZE",
    0x07: "FRAME_ERROR",
}
ST_OK, ST_BUSY, ST_ERROR, ST_BLOCKED, ST_BULK_LOCKED, ST_CRC, ST_OVERSIZE, ST_FRAME = range(8)

PARAM_SRC = {0: "UNKNOWN", 1: "HOST_SET", 2: "BULK_SET", 3: "PRESET", 4: "FACTORY",
             5: "GPIO", 6: "INTERNAL", 7: "UAC1", 8: "UART", 9: "I2C", 10: "MIDI"}

# Opcodes the tools use by name (config.h).
REQ_GET_STATUS = 0x50
REQ_GET_PLATFORM = 0x7F
REQ_GET_ALL_PARAMS = 0xA0
REQ_SET_ALL_PARAMS = 0xA1
REQ_GET_BUFFER_STATS = 0xB0
REQ_GET_USB_ERROR_STATS = 0xB2
REQ_SET_MASTER_VOLUME = 0xD2
REQ_GET_MASTER_VOLUME = 0xD3
REQ_ENTER_BOOTLOADER = 0xF0

# Extension channel (Documentation/Features/extension_channel_spec.md):
# bRequest 0x00, wValue = sub-command, GET/SET frame = IN/OUT.  Stock
# firmware answers ERROR, which means "no extensions".
REQ_EXTENSION = 0x00
EXT_LIST = 0x0000                # GET: supported IDs, uint16 LE ascending
EXT_CROSSFEED_PAIR_MAP = 0x0001  # GET/SET: uint8[4]
EXT_CORE_VREG = 0x0002           # GET: CoreVregStatus (16 B); SET: {op, value}
EXT_NOTIFY_FEATURES = 0x0003     # GET: uint32 LE NOTIFY_FEAT_* flags
NOTIFY_FEAT_BULK_SOURCE = 1 << 0  # BULK_INVALIDATED of a bulk SET carries its dispatch source

# Commands that are reads on the wire although they act ("write-as-read",
# Documentation/commands.md): they go in GET-type frames.  The CLI's `get`
# sends anything; this set only drives the `set` sanity warning.
NON_BULK_SET_MAX = 64

# ---------------------------------------------------------------------------
# Codec (mirror of firmware/DSPi/midi_frame.c)
# ---------------------------------------------------------------------------


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def packed_len(n: int) -> int:
    r = n % 7
    return (n // 7) * 8 + (r + 1 if r else 0)


def pack7(raw: bytes) -> bytes:
    out = bytearray()
    for i in range(0, len(raw), 7):
        grp = raw[i:i + 7]
        msbs = 0
        for k, b in enumerate(grp):
            msbs |= ((b >> 7) & 1) << k
        out.append(msbs)
        out.extend(b & 0x7F for b in grp)
    return bytes(out)


def unpack7(data: bytes) -> bytes:
    out = bytearray()
    for i in range(0, len(data), 8):
        grp = data[i:i + 8]
        if len(grp) < 2:
            raise ValueError("lone MSB byte")
        if any(b & 0x80 for b in grp):
            raise ValueError("byte >= 0x80 in packed data")
        msbs = grp[0]
        if msbs >> (len(grp) - 1):
            raise ValueError("MSB bit set for a missing byte")
        for k, b in enumerate(grp[1:]):
            out.append(b | (((msbs >> k) & 1) << 7))
    return bytes(out)


def with_crc(body: bytes) -> bytes:
    return body + struct.pack("<H", crc16(body))


def request_frame(type_: int, breq: int, wvalue: int = 0, windex: int = 0,
                  wlength: int | None = None, payload: bytes = b"") -> bytes:
    """UART request frame without the 0xA5 sync byte."""
    if type_ == TYPE_GET:
        payload = b""
        wl = 0 if wlength is None else wlength
    else:
        wl = len(payload) if wlength is None else wlength
    body = struct.pack("<BBHHH", type_, breq, wvalue & 0xFFFF, windex & 0xFFFF, wl & 0xFFFF) + payload
    return with_crc(body)


def response_frame(type_: int, status: int, payload: bytes = b"") -> bytes:
    if status != ST_OK:
        payload = b""
    return with_crc(struct.pack("<BBH", type_, status, len(payload)) + payload)


def sysex_header(tag: int, mfr: bytes = MFR_DEV) -> bytes:
    return bytes([SYSEX_START]) + mfr + MAGIC + bytes([VERSION, tag & 0x7F])


def encode(frame: bytes, tag: int, mfr: bytes = MFR_DEV) -> bytes:
    """Wrap one frame (type byte .. CRC) into one SysEx message."""
    return sysex_header(tag, mfr) + pack7(frame) + bytes([SYSEX_END])


def sysex_len(frame_len: int, mfr: bytes = MFR_DEV) -> int:
    return 1 + len(mfr) + 3 + 2 + packed_len(frame_len) + 1


class NotOurs(Exception):
    """A SysEx message for another manufacturer / product (ignore it)."""


def decode(msg: bytes, mfr: bytes = MFR_DEV) -> tuple[int, bytes]:
    """SysEx message -> (tag, frame).  Raises NotOurs or ValueError."""
    if len(msg) < 2 or msg[0] != SYSEX_START or msg[-1] != SYSEX_END:
        raise ValueError("not a complete SysEx message")
    body = msg[1:-1]
    hl = len(mfr) + 3
    if body[:len(mfr)] != mfr or body[len(mfr):hl] != MAGIC:
        raise NotOurs()
    if len(body) < hl + 2:
        raise ValueError("truncated header")
    if body[hl] != VERSION:
        raise ValueError(f"unsupported version {body[hl]:#x}")
    tag = body[hl + 1]
    return tag, unpack7(body[hl + 2:])


class Reply:
    """A device-to-host frame: response or notification."""

    def __init__(self, tag: int, frame: bytes):
        if len(frame) < 6:
            raise ValueError("short device frame")
        self.tag = tag
        self.type, self.status, n = struct.unpack_from("<BBH", frame)
        self.crc_ok = crc16(frame[:-2]) == struct.unpack_from("<H", frame, len(frame) - 2)[0]
        self.payload = frame[4:-2]
        if len(self.payload) != n:
            raise ValueError(f"length field {n} != payload {len(self.payload)}")
        self.t = time.monotonic()

    @property
    def status_name(self) -> str:
        return STATUS.get(self.status, f"0x{self.status:02X}")

    def __repr__(self) -> str:
        return (f"Reply(tag={self.tag:#04x}, type={self.type:#04x}, status={self.status_name}, "
                f"len={len(self.payload)})")


def parse_request(frame: bytes) -> dict:
    """Host-to-device frame -> dict (used by the fake device and tests)."""
    if len(frame) < 10:
        raise ValueError("short request frame")
    type_, breq, wv, wi, wl = struct.unpack_from("<BBHHH", frame)
    payload = frame[8:-2]
    crc_ok = crc16(frame[:-2]) == struct.unpack_from("<H", frame, len(frame) - 2)[0]
    return dict(type=type_, breq=breq, wvalue=wv, windex=wi, wlength=wl,
                payload=payload, crc_ok=crc_ok)


def decode_notify(pkt: bytes) -> str:
    """Human-readable v2 notify packet (notification_protocol_v2_spec.md)."""
    if len(pkt) < 4:
        return f"short packet {pkt.hex()}"
    ver, evt, _flags, seq = pkt[:4]
    names = {0x02: "PARAM_CHANGED", 0x03: "BULK_INVALIDATED", 0x04: "PRESET_LOADED",
             0x05: "INPUT_FORMAT", 0x07: "SIGGEN_STATE", 0x08: "ADAT_STATE",
             0x09: "I2S_SLAVE_STATE", 0x0A: "CS_IR_LEARN", 0x0B: "ADAT_INPUT_STATE",
             0x0C: "CS_AUX", 0x0D: "RTA_CONFIG"}
    s = f"v{ver} seq={seq:3d} {names.get(evt, f'evt 0x{evt:02X}')}"
    if evt == 0x02 and len(pkt) >= 12:
        # [ver, evt, flags, seq, off_LE16, size_LE16, src, 0, 0, 0, value...]
        off, size = struct.unpack_from("<HH", pkt, 4)
        src = pkt[8]
        val = pkt[12:12 + size]
        s += f" off=0x{off:04X} size={size} src={PARAM_SRC.get(src, src)} value={val.hex()}"
        if size == 4:
            s += f" (f32 {struct.unpack('<f', val)[0]:.3f})"
    elif evt == 0x03 and len(pkt) >= 5:
        s += f" src={PARAM_SRC.get(pkt[4], pkt[4])}"
    else:
        s += f" {pkt[4:].hex()}"
    return s


class SysexSplitter:
    """Byte stream -> complete SysEx messages (F0 .. F7).

    Real-time bytes (F8-FF) are dropped, bytes outside a message are dropped,
    a new F0 or a system-common status byte abandons an unfinished message.
    """

    def __init__(self) -> None:
        self.buf: bytearray | None = None
        self.dropped = 0          # bytes discarded outside / with broken messages
        self.aborted = 0          # messages abandoned before F7

    def feed(self, data: bytes) -> list[bytes]:
        out = []
        for b in data:
            if b >= 0xF8:
                continue
            if b == SYSEX_START:
                if self.buf is not None:
                    self.aborted += 1
                    self.dropped += len(self.buf)
                self.buf = bytearray([b])
            elif self.buf is None:
                self.dropped += 1
            elif b == SYSEX_END:
                self.buf.append(b)
                out.append(bytes(self.buf))
                self.buf = None
            elif b >= 0x80:
                self.aborted += 1
                self.dropped += len(self.buf) + 1
                self.buf = None
            else:
                self.buf.append(b)
        return out


# ---------------------------------------------------------------------------
# Transport: ALSA rawmidi node
# ---------------------------------------------------------------------------


def list_ports() -> list[tuple[str, str]]:
    """[(device node, card long name)] for every rawmidi node."""
    names = {}
    try:
        with open("/proc/asound/cards") as f:
            for line in f:
                m = re.match(r"\s*(\d+)\s+\[[^\]]*\]:\s*(.*)$", line)
                if m:
                    names[int(m.group(1))] = m.group(2).strip()
    except OSError:
        pass
    out = []
    for path in sorted(glob.glob("/dev/snd/midiC*D*")):
        m = re.search(r"midiC(\d+)D(\d+)$", path)
        if m:
            out.append((path, names.get(int(m.group(1)), "?")))
    return out


def find_port(name: str = "DSPi") -> str:
    hits = [p for p, n in list_ports() if name.lower() in n.lower()]
    if not hits:
        raise SystemExit(f"no rawmidi port on a card matching {name!r} "
                         f"(found: {list_ports() or 'none'}); pass --port")
    return hits[0]


class RawMidiPort:
    """Unbuffered byte I/O on an ALSA rawmidi node (or any tty/pipe-like fd)."""

    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        if os.isatty(self.fd):         # a pty from fake_device.py --pty
            import tty
            tty.setraw(self.fd)

    def write(self, data: bytes) -> None:
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                select.select([], [self.fd], [], 1.0)

    def read(self, timeout: float) -> bytes:
        r, _, _ = select.select([self.fd], [], [], max(0.0, timeout))
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""
        except OSError as e:
            if e.errno == errno.EIO:     # pty peer closed
                return b""
            raise

    def close(self) -> None:
        os.close(self.fd)


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------


class Timeout(Exception):
    pass


class Client:
    """Request/response over one port; notifications are queued.

    One request in flight (the UART rule).  `port` needs write(bytes) and
    read(timeout) -> bytes.
    """

    def __init__(self, port, mfr: bytes = MFR_DEV, timeout: float = 2.0):
        self.port = port
        self.mfr = mfr
        self.timeout = timeout
        self.split = SysexSplitter()
        self.notifications: collections.deque[Reply] = collections.deque(maxlen=10000)
        self.on_notify = None
        self._tag = 0
        self.stats = collections.Counter()
        self._ready: list[Reply] = []

    def _next_tag(self) -> int:
        self._tag = self._tag % 0x7E + 1        # 1..0x7E; 0 = notify, 0x7F = unknown
        return self._tag

    def _ingest(self, data: bytes) -> None:
        for msg in self.split.feed(data):
            try:
                tag, frame = decode(msg, self.mfr)
                rep = Reply(tag, frame)
            except NotOurs:
                self.stats["foreign_sysex"] += 1
                continue
            except ValueError:
                self.stats["malformed"] += 1
                continue
            if not rep.crc_ok:
                self.stats["crc_bad"] += 1
                continue
            if rep.type == TYPE_NOTIFY:
                self.stats["notify"] += 1
                self.notifications.append(rep)
                if self.on_notify:
                    self.on_notify(rep)
            else:
                self._ready.append(rep)

    def pump(self, timeout: float = 0.0) -> None:
        """Read whatever arrives within `timeout` (notifications included)."""
        end = time.monotonic() + timeout
        while True:
            data = self.port.read(max(0.0, end - time.monotonic()))
            if data:
                self._ingest(data)
            if time.monotonic() >= end:
                break
            if not data and timeout == 0:
                break

    def transact(self, frame: bytes, timeout: float | None = None) -> Reply:
        """Send one frame, wait for the reply with the same tag."""
        timeout = self.timeout if timeout is None else timeout
        tag = self._next_tag()
        self._ready.clear()
        self.port.write(encode(frame, tag, self.mfr))
        end = time.monotonic() + timeout
        while True:
            for i, rep in enumerate(self._ready):
                if rep.tag in (tag, TAG_UNKNOWN):
                    del self._ready[i]
                    self.stats[rep.status_name] += 1
                    return rep
            self._ready = [r for r in self._ready if r.tag not in (tag, TAG_UNKNOWN)]
            if self._ready:
                self.stats["stale_reply"] += len(self._ready)
                self._ready.clear()
            left = end - time.monotonic()
            if left <= 0:
                self.stats["timeout"] += 1
                raise Timeout(f"no reply to tag {tag:#04x} within {timeout:.2f}s")
            data = self.port.read(min(left, 0.5))
            if data:
                self._ingest(data)

    def request(self, type_: int, breq: int, wvalue: int = 0, windex: int = 0,
                wlength: int | None = None, payload: bytes = b"",
                timeout: float | None = None, retries: int = 5) -> Reply:
        """transact() with the transport's retry rule for BUSY / BULK_LOCKED."""
        frame = request_frame(type_, breq, wvalue, windex, wlength, payload)
        for attempt in range(retries + 1):
            rep = self.transact(frame, timeout)
            if rep.status not in (ST_BUSY, ST_BULK_LOCKED) or attempt == retries:
                return rep
            self.stats["retry"] += 1
            time.sleep(0.02 * (attempt + 1))
        return rep  # pragma: no cover

    def get(self, breq: int, wvalue: int = 0, windex: int = 0, wlength: int = 0, **kw) -> Reply:
        return self.request(TYPE_GET, breq, wvalue, windex, wlength, **kw)

    def set(self, breq: int, wvalue: int = 0, windex: int = 0, payload: bytes = b"", **kw) -> Reply:
        return self.request(TYPE_SET, breq, wvalue, windex, None, payload, **kw)

    def wait_notify(self, pred, timeout: float) -> Reply | None:
        """First queued or arriving notification for which pred(rep) is true."""
        end = time.monotonic() + timeout
        while True:
            while self.notifications:
                rep = self.notifications.popleft()
                if pred(rep):
                    return rep
            left = end - time.monotonic()
            if left <= 0:
                return None
            data = self.port.read(min(left, 0.2))
            if data:
                self._ingest(data)


def chunked_read(c: Client, chunk: int = 2048) -> bytes:
    """0xA2 chunked bulk read (the USB vendor path's WinUSB form)."""
    out = bytearray()
    while True:
        rep = c.get(0xA2, len(out), 0, chunk, timeout=10.0)
        if rep.status != ST_OK:
            raise RuntimeError(f"0xA2 at {len(out)}: {rep.status_name}")
        out += rep.payload
        if len(rep.payload) < chunk or not rep.payload:
            return bytes(out)
        if len(out) >= 8 and len(out) >= struct.unpack_from("<H", out, 6)[0]:
            return bytes(out)


def chunked_write(c: Client, image: bytes, chunk: int = 2048) -> int:
    """0xA3 chunked bulk write; returns the last status."""
    status = ST_OK
    for off in range(0, len(image), chunk):
        rep = c.set(0xA3, off, 0, image[off:off + chunk], timeout=10.0)
        status = rep.status
        if status != ST_OK:
            break
    return status


def open_client(args) -> Client:
    if getattr(args, "fake", False):
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import fake_device
        port = fake_device.FakePort(fake_device.FakeDevice())
    else:
        port = RawMidiPort(args.port or find_port(args.card_name))
    return Client(port, bytes.fromhex(args.mfr), timeout=args.timeout)


def add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--port", help="rawmidi node, e.g. /dev/snd/midiC2D0 (default: find by card name)")
    p.add_argument("--card-name", default="DSPi", help="ALSA card name to look for (default DSPi)")
    p.add_argument("--mfr", default="7d", help="manufacturer ID hex, 1 or 3 bytes (default 7d)")
    p.add_argument("--timeout", type=float, default=2.0, help="per-request timeout, s")
    p.add_argument("--fake", action="store_true", help="talk to the in-process fake device")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _int(s: str) -> int:
    return int(s, 0)


def _print_reply(rep: Reply, out: str | None, rtt: float) -> int:
    print(f"status={rep.status_name} len={len(rep.payload)} rtt={rtt * 1e3:.2f} ms")
    if rep.payload:
        if out:
            with open(out, "wb") as f:
                f.write(rep.payload)
            print(f"wrote {len(rep.payload)} bytes to {out}")
        else:
            for i in range(0, min(len(rep.payload), 512), 16):
                print(f"  {i:04x}: {rep.payload[i:i + 16].hex(' ')}")
            if len(rep.payload) > 512:
                print(f"  ... ({len(rep.payload)} bytes; use --out)")
    return 0 if rep.status == ST_OK else 1


def cmd_list(args) -> int:
    for path, name in list_ports():
        print(f"{path}\t{name}")
    return 0


def cmd_get(args) -> int:
    c = open_client(args)
    t0 = time.monotonic()
    rep = c.get(args.req, args.wvalue, args.windex, args.wlength)
    return _print_reply(rep, args.out, time.monotonic() - t0)


def cmd_set(args) -> int:
    if args.payload_in:
        with open(args.payload_in, "rb") as f:
            payload = f.read()
    elif args.float is not None:
        payload = struct.pack("<f", args.float)
    elif args.u8 is not None:
        payload = bytes([args.u8 & 0xFF])
    else:
        payload = bytes.fromhex(args.payload or "")
    c = open_client(args)
    t0 = time.monotonic()
    rep = c.set(args.req, args.wvalue, args.windex, payload)
    return _print_reply(rep, None, time.monotonic() - t0)


def cmd_ext_list(args) -> int:
    c = open_client(args)
    rep = c.get(REQ_EXTENSION, EXT_LIST)
    if rep.status != ST_OK:
        print(f"status={rep.status_name} (no extension channel)")
        return 1
    n = len(rep.payload) // 2
    ids = struct.unpack(f"<{n}H", rep.payload[:2 * n])
    print(" ".join(f"0x{i:04X}" for i in ids) or "(none)")
    if EXT_NOTIFY_FEATURES in ids:
        f = c.get(REQ_EXTENSION, EXT_NOTIFY_FEATURES)
        if f.status == ST_OK and len(f.payload) >= 4:
            flags = struct.unpack("<I", f.payload[:4])[0]
            names = ["BULK_SOURCE"] if flags & NOTIFY_FEAT_BULK_SOURCE else []
            print(f"notify features: 0x{flags:08X} {' '.join(names)}".rstrip())
    return 0


def cmd_notify(args) -> int:
    c = open_client(args)
    c.on_notify = lambda r: print(f"{time.strftime('%H:%M:%S')} {decode_notify(r.payload)}", flush=True)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        c.pump(min(0.5, end - time.monotonic()))
    print(f"stats: {dict(c.stats)}")
    return 0


def cmd_bootloader(args) -> int:
    if not args.yes:
        print("This reboots the DSPi into its USB bootloader (it leaves USB audio and MIDI).")
        if input("Type 'yes' to continue: ").strip() != "yes":
            print("aborted")
            return 1
    c = open_client(args)
    rep = c.get(REQ_ENTER_BOOTLOADER)
    print(f"status={rep.status_name} payload={rep.payload.hex()} (device now reboots)")
    return 0 if rep.status == ST_OK else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list", help="list rawmidi ports")
    p.set_defaults(fn=cmd_list)

    p = sub.add_parser("get", help="GET-type frame (reads and write-as-read commands)")
    add_common_args(p)
    p.add_argument("req", type=_int)
    p.add_argument("--wvalue", type=_int, default=0)
    p.add_argument("--windex", type=_int, default=0)
    p.add_argument("--wlength", type=_int, default=0, help="response cap, 0 = full")
    p.add_argument("--out", help="write the payload to this file")
    p.set_defaults(fn=cmd_get)

    p = sub.add_parser("set", help="SET-type frame")
    add_common_args(p)
    p.add_argument("req", type=_int)
    p.add_argument("--wvalue", type=_int, default=0)
    p.add_argument("--windex", type=_int, default=0)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--payload", help="payload hex")
    g.add_argument("--float", type=float, help="payload = one little-endian float")
    g.add_argument("--u8", type=_int, help="payload = one byte")
    g.add_argument("--in", dest="payload_in", help="payload from file (e.g. 0xA1 bulk image)")
    p.set_defaults(fn=cmd_set)

    p = sub.add_parser("ext-list", help="GET 0x00 EXT_LIST: the extension sub-commands the device supports")
    add_common_args(p)
    p.set_defaults(fn=cmd_ext_list)

    p = sub.add_parser("notify", help="print decoded 0x40 notification frames")
    add_common_args(p)
    p.add_argument("--seconds", type=float, default=30)
    p.set_defaults(fn=cmd_notify)

    p = sub.add_parser("enter-bootloader", help="GET 0xF0 (device reboots to BOOTSEL)")
    add_common_args(p)
    p.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    p.set_defaults(fn=cmd_bootloader)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
