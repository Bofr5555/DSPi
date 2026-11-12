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
EXT_IDENTIFY = 0x0004            # GET: IdentifyStatus (56 B); SET: {seconds, tone_output, 0, 0}
EXT_RESET_INFO = 0x0005          # GET: ResetInfo (20 B)
EXT_FIRMWARE_FIXES = 0x0006      # GET: uint32 LE FW_FIX_* flags
NOTIFY_FEAT_BULK_SOURCE = 1 << 0  # BULK_INVALIDATED of a bulk SET carries its dispatch source
NOTIFY_FEAT_CLIP_EVENT = 1 << 1   # clip event 0xE0 (EXT_CLIP)
FW_FIX_BULK_DB_POWF = 1 << 0      # a bulk SET converts dB with powf (no Taylor series)
NOTIFY_EVT_EXT_CLIP = 0xE0
IDENTIFY_TONE_NONE = 0xFF
RESET_REASONS = {0: "UNKNOWN", 1: "POWER_ON", 2: "BROWNOUT", 3: "RUN_PIN", 4: "WATCHDOG",
                 5: "SOFTWARE", 6: "DEBUGGER", 7: "GLITCH"}

# Link block 0x8000-0x80FF (extension_channel_spec.md 2.8, SYS2 5.3).  Fork
# firmware without the link and stock firmware refuse these IDs (ERROR here,
# STALL over USB); a host checks EXT_LIST for 0x8000 first.
EXT_LINK_INFO = 0x8000           # GET: LinkInfo (32 B)
EXT_LINK_STATUS = 0x8001         # GET: LinkStatus (52 B)
EXT_LINK_CONFIG = 0x8002         # GET/SET: LinkConfig (8 B)
EXT_LINK_SLOTS = 0x8003          # GET/SET: uint8[8] link slot per matrix input, 0xFF = silence
EXT_LINK_PHASE = 0x8004          # GET/SET: LinkPhase (8 B)
EXT_FRAME_COUNTER = 0x8005       # GET: {u64 F, u32 time_us, u32 O} (16 B)
EXT_APPLY_AT_FRAME = 0x8006      # SET: ApplyRequest (16 B); GET: ApplyStatus (16 B)
EXT_LINK_TX = 0x8007             # GET/SET: LinkTx (16 B), sender builds only
EXT_LINK_CTRL_MSG = 0x8008       # GET: superframe snapshot (24 B); SET (sender only): ApplyRequest-like (16 B)
EXT_LINK_STATS_RESET = 0x8009    # SET: 1 byte, 1 = clear the counters
EXT_LINK_DIAG = 0x800A           # GET: bench diagnostics (64 B), layout versioned, not for apps
LINK_INFO_LEN, LINK_STATUS_LEN, LINK_DIAG_LEN, LINK_CTRL_MSG_IN_LEN = 32, 52, 64, 24
NOTIFY_FEAT_LINK_STATE = 1 << 2   # link state event 0xE1
NOTIFY_FEAT_APPLY_RESULT = 1 << 3  # apply-at-frame result event 0xE2
NOTIFY_EVT_LINK_STATE = 0xE1
NOTIFY_EVT_APPLY_RESULT = 0xE2
LINK_ROLES = {0: "none", 1: "receiver", 2: "sender", 3: "repeater", 4: "fan-out"}
LINK_STATES = {0: "off", 1: "searching", 2: "locked", 3: "holdover"}
LINK_PHASE_STATES = {0: "free", 1: "anchoring", 2: "locked", 3: "re-anchor pending"}
LINK_REASONS = {0: "NONE", 1: "LOCKED", 2: "LOS", 3: "HEADER_ERRORS", 4: "COMMA_LOST",
                5: "REANCHOR", 6: "VCXO_LIMIT", 7: "DISABLED"}
LINK_FEATURES = ((1 << 0, "PHASE_LOCK"), (1 << 1, "APPLY_AT_FRAME"), (1 << 2, "REPEATER"),
                 (1 << 3, "FAN_OUT"), (1 << 4, "TEST_PATTERNS"), (1 << 5, "LOS_PIN"))
RATE_CODES = {0: "44.1k", 1: "48k", 2: "96k"}
APPLY_OPS = {0: "CANCEL_ALL", 1: "MUTE", 2: "UNMUTE", 3: "PRESET_LOAD", 4: "MASTER_VOLUME",
             5: "OUT_DELAY", 6: "RATE", 7: "CLICK", 8: "SYNC_PULSE"}
APPLY_RESULTS = {0: "applied", 1: "late", 2: "cancelled", 3: "refused"}

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
             0x0C: "CS_AUX", 0x0D: "RTA_CONFIG", NOTIFY_EVT_EXT_CLIP: "EXT_CLIP",
             NOTIFY_EVT_LINK_STATE: "LINK_STATE", NOTIFY_EVT_APPLY_RESULT: "APPLY_RESULT"}
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
    elif evt == NOTIFY_EVT_EXT_CLIP and len(pkt) >= 12:
        # [ver, evt, flags, seq, channel, count, 0, 0, mask_LE32]
        s += f" channel={pkt[4]} count={pkt[5]} mask=0x{struct.unpack_from('<I', pkt, 8)[0]:08X}"
    elif evt == NOTIFY_EVT_LINK_STATE and len(pkt) >= 12:
        # [ver, evt, flags, seq, state, class, reason, 0, F_low32_LE]
        s += (f" state={LINK_STATES.get(pkt[4], pkt[4])} class=L{pkt[5]} "
              f"reason={LINK_REASONS.get(pkt[6], pkt[6])} F_low32={struct.unpack_from('<I', pkt, 8)[0]}")
    elif evt == NOTIFY_EVT_APPLY_RESULT and len(pkt) >= 12:
        # [ver, evt, flags, seq, op, result, 0, 0, target_frame_low32_LE]
        s += (f" op={APPLY_OPS.get(pkt[4], pkt[4])} result={APPLY_RESULTS.get(pkt[5], pkt[5])} "
              f"frame_low32={struct.unpack_from('<I', pkt, 8)[0]}")
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
    if getattr(args, "fake", False) or getattr(args, "fake_link", None):
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import fake_device
        dev = fake_device.FakeDevice(link=getattr(args, "fake_link", None))
        if dev.link and not dev.link.sender:        # a receiver that has locked
            dev.link.config[0] = 1
            dev.link_event(2, 1)
            dev.out.clear()
        port = fake_device.FakePort(dev)
    else:
        port = RawMidiPort(args.port or find_port(args.card_name))
    return Client(port, bytes.fromhex(args.mfr), timeout=args.timeout)


def add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--port", help="rawmidi node, e.g. /dev/snd/midiC2D0 (default: find by card name)")
    p.add_argument("--card-name", default="DSPi", help="ALSA card name to look for (default DSPi)")
    p.add_argument("--mfr", default="7d", help="manufacturer ID hex, 1 or 3 bytes (default 7d)")
    p.add_argument("--timeout", type=float, default=2.0, help="per-request timeout, s")
    p.add_argument("--fake", action="store_true", help="talk to the in-process fake device")
    p.add_argument("--fake-link", choices=("receiver", "sender"),
                   help="in-process fake device with the link block 0x8000+ (implies --fake)")


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
            names = [n for bit, n in ((NOTIFY_FEAT_BULK_SOURCE, "BULK_SOURCE"),
                                      (NOTIFY_FEAT_CLIP_EVENT, "CLIP_EVENT"),
                                      (NOTIFY_FEAT_LINK_STATE, "LINK_STATE"),
                                      (NOTIFY_FEAT_APPLY_RESULT, "APPLY_RESULT")) if flags & bit]
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


def decode_identify(b: bytes) -> str:
    """IdentifyStatus (extension_channel_spec.md 2.5)."""
    if len(b) < 56:
        return f"short reply {b.hex()}"
    left, tone, chip, rev, chip_id = struct.unpack_from("<BBBBI", b)
    board = b[8:24].split(b"\0")[0].decode("ascii", "replace")
    hw_rev = b[24:56].split(b"\0")[0].decode("ascii", "replace")
    tone_s = "none" if tone == IDENTIFY_TONE_NONE else f"out {tone}"
    return (f"blink_left={left}s last_tone={tone_s} chip={'RP2350' if chip else 'RP2040'} "
            f"rev={rev} chip_id=0x{chip_id:08X} board={board!r} hw_rev={hw_rev!r}")


def decode_reset_info(b: bytes) -> str:
    """ResetInfo (extension_channel_spec.md 2.6)."""
    if len(b) < 20:
        return f"short reply {b.hex()}"
    reason, flags, hangs, chip, up, prev, chip_reset, wdt = struct.unpack_from("<BBBBIIII", b)
    prev_s = f"{prev}s" if flags & 1 else "n/a"
    return (f"reason={RESET_REASONS.get(reason, reason)} uptime={up}s prev_uptime={prev_s} "
            f"hang_count={hangs} chip={'RP2350' if chip else 'RP2040'} "
            f"chip_reset=0x{chip_reset:08X} watchdog_reason=0x{wdt:03X}")


def cmd_identify(args) -> int:
    c = open_client(args)
    if args.seconds is not None:
        tone = IDENTIFY_TONE_NONE if args.tone_output is None else args.tone_output
        rep = c.set(REQ_EXTENSION, EXT_IDENTIFY, payload=bytes([args.seconds, tone, 0, 0]))
        if rep.status != ST_OK:
            print(f"status={rep.status_name} (refused: bad output, generator running, or no EXT_IDENTIFY)")
            return 1
    rep = c.get(REQ_EXTENSION, EXT_IDENTIFY)
    if rep.status != ST_OK:
        print(f"status={rep.status_name} (no EXT_IDENTIFY)")
        return 1
    print(decode_identify(rep.payload))
    return 0


def cmd_reset_info(args) -> int:
    c = open_client(args)
    rep = c.get(REQ_EXTENSION, EXT_RESET_INFO)
    if rep.status != ST_OK:
        print(f"status={rep.status_name} (no EXT_RESET_INFO)")
        return 1
    print(decode_reset_info(rep.payload))
    return 0


def decode_fixes(b: bytes) -> str:
    """EXT_FIRMWARE_FIXES (extension_channel_spec.md 2.7)."""
    if len(b) < 4:
        return f"short reply {b.hex()}"
    flags = struct.unpack_from("<I", b)[0]
    names = [n for bit, n in ((FW_FIX_BULK_DB_POWF, "BULK_DB_POWF"),) if flags & bit]
    return f"firmware fixes: 0x{flags:08X} {' '.join(names)}".rstrip()


def cmd_fixes(args) -> int:
    c = open_client(args)
    rep = c.get(REQ_EXTENSION, EXT_FIRMWARE_FIXES)
    if rep.status != ST_OK:
        # Not listed (stock or older fork firmware): every fix reads as absent.
        print(f"status={rep.status_name} (no EXT_FIRMWARE_FIXES: assume no fixes)")
        return 1
    print(decode_fixes(rep.payload))
    return 0


def _pin(p: int) -> str:
    return "none" if p == 0xFF else f"GP{p}"


def decode_link_info(b: bytes) -> str:
    """LinkInfo (extension_channel_spec.md 2.8.1)."""
    if len(b) < LINK_INFO_LEN:
        return f"short reply {b.hex()}"
    role, ver, cmask, links, chans, slots, _flags, rx, tx = struct.unpack_from("<BBBBBBHBB", b)
    feats, dmin, dmax = struct.unpack_from("<IHH", b, 12)
    classes = ",".join(f"L{i + 1}" for i in range(3) if cmask >> i & 1) or "none"
    names = " ".join(n for bit, n in LINK_FEATURES if feats & bit)
    return (f"role={LINK_ROLES.get(role, role)} protocol=v{ver} classes={classes} max_links={links} "
            f"max_channels={chans} slots={slots} rx_pin={_pin(rx)} tx_pin={_pin(tx)} "
            f"features=0x{feats:08X} {names} d_local={dmin}..{dmax}").replace("  ", " ")


def decode_link_status(b: bytes) -> str:
    """LinkStatus (extension_channel_spec.md 2.8.2)."""
    if len(b) < LINK_STATUS_LEN:
        return f"short reply {b.hex()}"
    state, cls, rate, chans, hops, los, sender, sflags, frame = struct.unpack_from("<8BQ", b)
    locks, losses, crc8, crc16_, slips, sferr, ppm, phase, fill = struct.unpack_from("<6IffH", b, 16)
    los_s = {0: "clear", 1: "ASSERTED", 0xFF: "n/a"}.get(los, los)
    return (f"state={LINK_STATES.get(state, state)} class=L{cls} rate={RATE_CODES.get(rate, rate)} "
            f"C={chans} hops={hops} los={los_s} sender=0x{sender:02X} sf_flags=0x{sflags:02X} F={frame} "
            f"locks={locks} losses={losses} crc8={crc8} crc16={crc16_} slips={slips} sf_errors={sferr} "
            f"vcxo={ppm:+.2f}ppm phase_error={phase:+.3f} fill={fill}")


def decode_link_phase(b: bytes) -> str:
    """LinkPhase (extension_channel_spec.md 2.8.5)."""
    if len(b) < 8:
        return f"short reply {b.hex()}"
    mode, _r, d_local, hops_off, flags, state = struct.unpack_from("<BBHhBB", b)
    return (f"mode={'phase lock' if mode else 'fill servo'} d_local={d_local} hops_offset={hops_off} "
            f"state={LINK_PHASE_STATES.get(state, state)}" + (" D_BELOW_MIN" if flags & 1 else ""))


def cmd_link(args) -> int:
    """Read-only link overview: info, status, config, slots, phase (no SETs)."""
    c = open_client(args)
    rep = c.get(REQ_EXTENSION, EXT_LINK_INFO)
    if rep.status != ST_OK:
        print(f"status={rep.status_name} (no link support: EXT_LINK_INFO not available)")
        return 1
    print("info:   " + decode_link_info(rep.payload))
    role = rep.payload[0]
    for name, sub, dec in (("status", EXT_LINK_STATUS, decode_link_status),
                           ("config", EXT_LINK_CONFIG, lambda b: b.hex(" ")),
                           ("slots", EXT_LINK_SLOTS, lambda b: " ".join("--" if x == 0xFF else str(x) for x in b)),
                           ("phase", EXT_LINK_PHASE, decode_link_phase),
                           ("tx", EXT_LINK_TX, lambda b: b.hex(" "))):
        if sub == EXT_LINK_TX and role not in (2, 4):
            continue
        if sub in (EXT_LINK_SLOTS, EXT_LINK_PHASE) and role in (2, 4):
            continue
        r = c.get(REQ_EXTENSION, sub)
        print(f"{name + ':':8s}" + (dec(r.payload) if r.status == ST_OK else f"status={r.status_name}"))
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

    p = sub.add_parser("identify", help="EXT_IDENTIFY: blink the LED (and optionally a quiet tone), print the board info")
    add_common_args(p)
    p.add_argument("--seconds", type=_int, help="blink for N s (0 stops); omit to only read")
    p.add_argument("--tone-output", type=_int, help="also play the -40 dBFS identify tone on this output")
    p.set_defaults(fn=cmd_identify)

    p = sub.add_parser("reset-info", help="EXT_RESET_INFO: last reset cause, uptime, hang count")
    add_common_args(p)
    p.set_defaults(fn=cmd_reset_info)

    p = sub.add_parser("fixes", help="EXT_FIRMWARE_FIXES: firmware bugs this build has fixed")
    add_common_args(p)
    p.set_defaults(fn=cmd_fixes)

    p = sub.add_parser("link", help="link block 0x8000+: info, status, config, slots, phase (read only)")
    add_common_args(p)
    p.set_defaults(fn=cmd_link)

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
