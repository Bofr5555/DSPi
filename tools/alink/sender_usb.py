#!/usr/bin/env python3
"""sender_usb.py - the link sender's extension block over Linux usbfs (WP LK4).

A DSPi link sender (firmware/DSPi-Sender) answers REQ_EXTENSION (0x00) on its
vendor interface (2) with the sender's part of the link block
(Documentation/Features/extension_channel_spec.md 2.8).  This module reads and
writes those sub-commands with plain control transfers on the device node
(/dev/bus/usb/BBB/DDD), standard library only, so tools/link/bench.py can
configure a sender and read its frame counter in the same container that
talks to the bench boards.  A refused request (STALL) raises SenderRefused.

    dev = SenderUsb("/dev/bus/usb/003/012")
    tx = dev.link_tx()            # dict of EXT_LINK_TX
    dev.set_link_tx(tx | dict(link_enable=1, test_pattern=1))
"""

from __future__ import annotations

import ctypes
import errno
import fcntl
import os
import struct

REQ_EXTENSION = 0x00
ITF_VENDOR = 2

EXT_LIST = 0x0000
EXT_LINK_INFO = 0x8000
EXT_LINK_STATUS = 0x8001
EXT_LINK_CONFIG = 0x8002
EXT_FRAME_COUNTER = 0x8005
EXT_LINK_TX = 0x8007
EXT_LINK_CTRL_MSG = 0x8008
EXT_LINK_STATS_RESET = 0x8009
EXT_LINK_DIAG = 0x800A

RATE_CODE = {"44k1": 0, "48k": 1, "96k": 2}
RATE_NAME = {v: k for k, v in RATE_CODE.items()}
TX_FIELDS = ("link_enable", "cls", "channels", "rate", "test_pattern", "slot_map_id", "sync_pin",
             "txdis_mask", "los_mask", "usb_alt", "flags")
TX_FMT = "<HBBBBBBHHBBH"


class SenderRefused(Exception):
    """The device STALLed the request (refused, extension_channel_spec.md 1.1)."""


class _Ctrl(ctypes.Structure):
    _fields_ = [("bRequestType", ctypes.c_uint8), ("bRequest", ctypes.c_uint8),
                ("wValue", ctypes.c_uint16), ("wIndex", ctypes.c_uint16),
                ("wLength", ctypes.c_uint16), ("timeout", ctypes.c_uint32),
                ("data", ctypes.c_void_p)]


USBDEVFS_CONTROL = (3 << 30) | (ctypes.sizeof(_Ctrl) << 16) | (ord("U") << 8) | 0


def pack_tx(d: dict) -> bytes:
    return struct.pack(TX_FMT, d["link_enable"], d["cls"], d["channels"], d["rate"], d["test_pattern"],
                       d.get("slot_map_id", 0), d["sync_pin"], d["txdis_mask"], d.get("los_mask", 0),
                       d.get("usb_alt", 0), d.get("flags", 0), 0)


def unpack_tx(b: bytes) -> dict:
    v = struct.unpack(TX_FMT, b[:16])
    return dict(zip(TX_FIELDS, v[:11]))


def unpack_ctrl_in(b: bytes) -> dict:
    flags, rate, chans, cls, hops, sender = b[:6]
    frame, op, arg, value = struct.unpack_from("<QBBh", b, 8)
    sf_low = struct.unpack_from("<I", b, 20)[0]
    return dict(flags=flags, rate=rate, channels=chans, cls=cls, hops=hops, sender=sender,
                apply_frame=frame, op=op, arg=arg, value=value, sf_frame_low=sf_low)


def pack_ctrl(frame: int, op: int, arg: int = 0, value: int = 0, system_flags: int = 0) -> bytes:
    return struct.pack("<QBBhB3x", frame, op, arg, value, system_flags)


def unpack_diag(b: bytes) -> dict | None:
    """The sender layout of EXT_LINK_DIAG (byte 0 = 0x81); None for another layout."""
    if len(b) < 64 or b[0] != 0x81:
        return None
    eng, cls, delay = b[1], b[2], b[3]
    (cyc_avg, cyc_max, cyc_frame, lead, late, stall, fill, under, over, fb, pkts, gaps) = \
        struct.unpack_from("<12I", b, 4)
    ppm_ff, = struct.unpack_from("<f", b, 52)
    loop_us, = struct.unpack_from("<I", b, 56)
    realigns, = struct.unpack_from("<H", b, 60)
    return dict(engine=eng, cls=cls, fanout_delay=delay, enc_cycles_avg=cyc_avg, enc_cycles_max=cyc_max,
                cycles_per_frame=cyc_frame, lead_words=lead, tx_late=late, tx_stall=stall, fifo_fill=fill,
                usb_underruns=under, usb_overruns=over, feedback_10_14=fb, usb_packets=pkts, usb_gaps=gaps,
                ppm_ff=ppm_ff, loop_us_max=loop_us, realigns=realigns, usb_playing=b[62])


class SenderUsb:
    """REQ_EXTENSION on a device node; get()/set() raise SenderRefused on a STALL."""

    def __init__(self, node: str):
        self.node = node
        self.fd = os.open(node, os.O_RDWR)

    def _xfer(self, rtype: int, wvalue: int, buf: ctypes.Array, n: int) -> int:
        c = _Ctrl(rtype, REQ_EXTENSION, wvalue, ITF_VENDOR, n, 1000, ctypes.addressof(buf))
        try:
            return fcntl.ioctl(self.fd, USBDEVFS_CONTROL, c)
        except OSError as e:
            if e.errno == errno.EPIPE:
                raise SenderRefused(f"0x{wvalue:04X} refused") from None
            raise

    def get(self, sub: int, length: int = 64) -> bytes:
        buf = ctypes.create_string_buffer(length)
        n = self._xfer(0xC1, sub, buf, length)
        return buf.raw[:n]

    def set(self, sub: int, payload: bytes) -> None:
        buf = ctypes.create_string_buffer(bytes(payload), len(payload))
        self._xfer(0x41, sub, buf, len(payload))

    def close(self) -> None:
        os.close(self.fd)

    # The sub-commands bench.py uses.
    def ext_list(self) -> list[int]:
        b = self.get(EXT_LIST, 64)
        return list(struct.unpack(f"<{len(b) // 2}H", b[:len(b) // 2 * 2]))

    def info(self) -> bytes:
        return self.get(EXT_LINK_INFO, 32)

    def link_tx(self) -> dict:
        return unpack_tx(self.get(EXT_LINK_TX, 16))

    def set_link_tx(self, d: dict) -> None:
        self.set(EXT_LINK_TX, pack_tx(d))

    def frame(self) -> int:
        return struct.unpack_from("<Q", self.get(EXT_FRAME_COUNTER, 16))[0]

    def ctrl_in(self) -> dict:
        return unpack_ctrl_in(self.get(EXT_LINK_CTRL_MSG, 24))

    def ctrl(self, frame: int, op: int, arg: int = 0, value: int = 0, system_flags: int = 0) -> None:
        self.set(EXT_LINK_CTRL_MSG, pack_ctrl(frame, op, arg, value, system_flags))

    def stats_reset(self) -> None:
        self.set(EXT_LINK_STATS_RESET, b"\x01")

    def diag(self) -> dict | None:
        return unpack_diag(self.get(EXT_LINK_DIAG, 64))
