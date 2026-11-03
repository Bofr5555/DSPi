#!/usr/bin/env python3
"""Minimal DSPi vendor-interface access over Linux usbfs (standard library).

Used by measure.py / soak.py / regress.sh to read the device over the
normal vendor interface (interface 2, EP0 vendor requests 0xC1) while the
MIDI channel is busy, so the two transports can be shown to interleave.
Read-only helpers only (GETs); no SET is exposed here on purpose.

The container needs the device node: DSPI_USB_DEV=/dev/bus/usb/BBB/DDD.
"""

from __future__ import annotations

import ctypes
import fcntl
import glob
import os
import struct

VID, PID = 0x2E8B, 0xFEAA
ITF_VENDOR = 2


class _CtrlTransfer(ctypes.Structure):
    _fields_ = [("bRequestType", ctypes.c_uint8), ("bRequest", ctypes.c_uint8),
                ("wValue", ctypes.c_uint16), ("wIndex", ctypes.c_uint16),
                ("wLength", ctypes.c_uint16), ("timeout", ctypes.c_uint32),
                ("data", ctypes.c_void_p)]


def _iowr(t: str, nr: int, size: int) -> int:
    return (3 << 30) | (size << 16) | (ord(t) << 8) | nr


USBDEVFS_CONTROL = _iowr("U", 0, ctypes.sizeof(_CtrlTransfer))


def find_device_node(vid: int = VID, pid: int = PID) -> str | None:
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(d, "idVendor")) as f:
                v = int(f.read(), 16)
            with open(os.path.join(d, "idProduct")) as f:
                p = int(f.read(), 16)
            if (v, p) != (vid, pid):
                continue
            with open(os.path.join(d, "busnum")) as f:
                bus = int(f.read())
            with open(os.path.join(d, "devnum")) as f:
                dev = int(f.read())
            return f"/dev/bus/usb/{bus:03d}/{dev:03d}"
        except OSError:
            continue
    return None


class VendorDevice:
    def __init__(self, node: str | None = None):
        node = node or os.environ.get("DSPI_USB_DEV") or find_device_node()
        if not node:
            raise SystemExit("DSPi (2e8b:feaa) not found; set DSPI_USB_DEV")
        self.node = node
        self.fd = os.open(node, os.O_RDWR)

    def get(self, breq: int, wvalue: int = 0, length: int = 64, timeout_ms: int = 1000) -> bytes:
        """Vendor IN request on interface 2 (bmRequestType 0xC1)."""
        buf = ctypes.create_string_buffer(length)
        ct = _CtrlTransfer(0xC1, breq, wvalue, ITF_VENDOR, length, timeout_ms,
                           ctypes.cast(buf, ctypes.c_void_p))
        n = fcntl.ioctl(self.fd, USBDEVFS_CONTROL, ct)
        return buf.raw[:n]

    def get_all_params(self, chunk: int = 2048) -> bytes:
        """0xA2 chunked read (usbfs caps one control transfer at a page)."""
        out = bytearray()
        total = None
        while total is None or len(out) < total:
            part = self.get(0xA2, len(out), chunk, 2000)
            if not part:
                break
            out += part
            if total is None:
                total = _bulk_total(bytes(out)) or 0
                if not total:
                    break
            if len(part) < chunk:
                break
        return bytes(out)

    def close(self) -> None:
        os.close(self.fd)


def _bulk_total(head: bytes) -> int | None:
    """WireBulkParams total length from its header, if recognisable."""
    # WireHeader (bulk_params.h): 6 x u8 (version, platform, channel counts,
    # bands), then u16 payload_length = total size including the header.
    if len(head) < 8:
        return None
    return struct.unpack_from("<H", head, 6)[0] or None


def underrun_counters(dev) -> dict:
    """REQ_GET_STATUS counters relevant to 'no audio damage' (commands.md 14.1).

    `dev.get(breq, wvalue, length)` works for both VendorDevice and a
    midi_ctrl.Client wrapper.
    """
    names = {3: "pdm_ring_overruns", 4: "pdm_ring_underruns", 5: "pdm_dma_overruns",
             6: "pdm_dma_underruns", 8: "spdif_underruns", 17: "spdif_dma_starvations",
             22: "usb_ring_overruns", 10: "usb_audio_packets"}
    out = {}
    for wv, name in names.items():
        data = dev.get(0x50, wv, 4)
        out[name] = struct.unpack("<I", data[:4])[0] if len(data) >= 4 else None
    return out


if __name__ == "__main__":
    d = VendorDevice()
    print(d.node, d.get(0x7F, 0, 7).hex())
