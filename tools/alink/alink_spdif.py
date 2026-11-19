#!/usr/bin/env python3
"""The alink time base carried on S/PDIF: Python twin of the C codec (WP FWSP1).

Bit-exact with firmware/DSPi/alink/alink_spdif_codec.{c,h}; both check
firmware/tests/alink_spdif_vectors.json. Normative text:
Documentation/Features/alink_protocol_spec.md section 12 ("Carriage on S/PDIF").

The S/PDIF carriage has its own constants and allowed-op set (PLAN-AT R9b): it
does not take them from alink_codec.py. Standard library only.

    python3 -I alink_spdif.py > ../../firmware/tests/alink_spdif_vectors.json
"""

import json
import struct
import sys

VERSION = 1
SF_MAGIC = 0xA5
BLOCK_FRAMES = 192          # SPDIF_BLOCK_FRAMES
USER_BITS = 384             # SPDIF_USER_BITS
SF_LEN = 48                 # SPDIF_SUPERFRAME_LEN
CRC_LEN = 30
CLASS = 0x10                # SPDIF_CARRIER_CLASS
CHANNELS = 2
TX_TABLE_BLOCKS = 4         # SPDIF_TX_TABLE_BLOCKS
TX_RING_FRAMES = 512        # SPDIF_TX_RING_FRAMES
MIN_LEAD_FRAMES = TX_RING_FRAMES + 2 * BLOCK_FRAMES   # SPDIF_CTRL_MIN_LEAD_FRAMES = 896
COAST_BLOCKS = 8            # SPDIF_COAST_BLOCKS

RATE_44K1, RATE_48K, RATE_96K = 0, 1, 2
RATE_HZ = {RATE_44K1: 44100, RATE_48K: 48000, RATE_96K: 96000}

FLAG_MUTE = 0x01
FLAG_APPLY_PEND = 0x04
FLAG_USB_LIVE = 0x08

OP_NONE, OP_MUTE, OP_UNMUTE, OP_MASTER_VOLUME, OP_CLICK = 0, 1, 2, 4, 7
OP_SIGGEN_START, OP_SIGGEN_STOP = 9, 10
OPS_ALLOWED = frozenset((OP_MUTE, OP_UNMUTE, OP_MASTER_VOLUME, OP_CLICK, OP_SIGGEN_START, OP_SIGGEN_STOP))

OK, ERR_CRC, ERR_MAGIC, ERR_VERSION, ERR_FRAME, ERR_LAYOUT, ERR_APPLY = range(7)


def crc16(data):
    """CRC-16/CCITT-FALSE, the line's CRC (alink_codec.crc16)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Superframe:
    FIELDS = ("flags", "rate", "channels", "cls", "hops", "sender", "frame",
              "apply_frame", "apply_op", "apply_arg", "apply_value", "epoch", "sender2")

    def __init__(self, **kw):
        for f in self.FIELDS:
            setattr(self, f, kw.get(f, 0))

    def __eq__(self, other):
        return all(getattr(self, f) == getattr(other, f) for f in self.FIELDS)

    def __repr__(self):
        return "Superframe(%s)" % ", ".join("%s=%r" % (f, getattr(self, f)) for f in self.FIELDS)

    def fields(self):
        return {f: getattr(self, f) for f in self.FIELDS}

    @property
    def source_id(self):
        return self.sender | (self.sender2 << 8)

    def pack(self):
        b = struct.pack("<BBBBBBBBQQBBhBB", SF_MAGIC, VERSION, self.flags, self.rate,
                        self.channels, self.cls, self.hops, self.sender, self.frame,
                        self.apply_frame, self.apply_op, self.apply_arg, self.apply_value,
                        self.epoch, self.sender2)
        return b + struct.pack("<H", crc16(b)) + bytes(SF_LEN - 32)

    @classmethod
    def unpack(cls_, data):
        """(error, Superframe or None) in the order of spec 12.2; the Superframe once CRC, magic, version pass."""
        data = bytes(data)
        if len(data) != SF_LEN:
            raise ValueError("an S/PDIF superframe is 48 bytes")
        if crc16(data[:CRC_LEN]) != struct.unpack_from("<H", data, CRC_LEN)[0]:
            return ERR_CRC, None
        if data[0] != SF_MAGIC:
            return ERR_MAGIC, None
        if data[1] != VERSION:
            return ERR_VERSION, None
        sf = Superframe(**dict(zip(cls_.FIELDS, struct.unpack_from("<BBBBBBQQBBhBB", data, 2))))
        if sf.frame % BLOCK_FRAMES:
            return ERR_FRAME, sf
        if sf.cls != CLASS or sf.rate not in RATE_HZ or sf.channels != CHANNELS:
            return ERR_LAYOUT, sf
        if (sf.apply_op != OP_NONE and sf.apply_op not in OPS_ALLOWED) or \
                (sf.apply_op == OP_NONE) != (sf.apply_frame == 0):
            return ERR_APPLY, sf
        return OK, sf


def carries(frame, apply_frame):
    """A block starting at frame carries the apply at N exactly when frame + 192 <= N."""
    return apply_frame != 0 and frame < apply_frame and apply_frame - frame >= BLOCK_FRAMES


def lead_ok(now_frame, target):
    return target >= now_frame and target - now_frame >= MIN_LEAD_FRAMES


class Source:
    """The source's per-block fields (alink_spdif_src_t)."""

    def __init__(self, rate=RATE_48K, sender=0, sender2=0, epoch=0, flags=0,
                 apply_frame=0, apply_op=0, apply_arg=0, apply_value=0):
        self.rate, self.sender, self.sender2, self.epoch, self.flags = rate, sender, sender2, epoch, flags
        self.apply_frame, self.apply_op, self.apply_arg, self.apply_value = \
            apply_frame, apply_op, apply_arg, apply_value

    def fields(self):
        return dict(rate=self.rate, sender=self.sender, sender2=self.sender2, epoch=self.epoch,
                    flags=self.flags, apply_frame=self.apply_frame, apply_op=self.apply_op,
                    apply_arg=self.apply_arg, apply_value=self.apply_value)

    def block(self, frame):
        """The 48 bytes of the block whose first frame is frame (alink_spdif_sf_build)."""
        sf = Superframe(flags=self.flags & (FLAG_MUTE | FLAG_USB_LIVE), rate=self.rate,
                        channels=CHANNELS, cls=CLASS, sender=self.sender, frame=frame,
                        epoch=self.epoch, sender2=self.sender2)
        if self.apply_op != OP_NONE and carries(frame, self.apply_frame):
            sf.flags |= FLAG_APPLY_PEND
            sf.apply_frame, sf.apply_op = self.apply_frame, self.apply_op
            sf.apply_arg, sf.apply_value = self.apply_arg, self.apply_value
        return sf.pack()


def ubit(sf, i):
    """User bit i of the block: bit i mod 8 of byte i / 8 (subframe 2k = A of frame k)."""
    return (sf[i >> 3] >> (i & 7)) & 1


def ubits(sf):
    return [ubit(sf, i) for i in range(USER_BITS)]


def from_ubits(bits):
    out = bytearray(SF_LEN)
    for i, b in enumerate(bits):
        out[i >> 3] |= (b & 1) << (i & 7)
    return bytes(out)


# ---------------------------------------------------------------------------
# Vectors (one flat JSON object per line; firmware/tests/test_alink_spdif_codec.c)
# ---------------------------------------------------------------------------

def _resign(sf_bytes):
    """Recompute the CRC of tampered bytes, so a later check is the first to fail."""
    b = bytearray(sf_bytes)
    struct.pack_into("<H", b, CRC_LEN, crc16(b[:CRC_LEN]))
    return bytes(b)


def vectors():
    out = []
    for name, data in (("check", b"123456789"), ("empty", b"")):
        out.append({"type": "crc16", "name": name, "data": data.hex(), "crc": crc16(data)})

    src = Source(rate=RATE_48K, sender=0x42, sender2=0x37, epoch=3,
                 apply_frame=1_000_000, apply_op=OP_SIGGEN_START)

    def build(name, s, frame):
        out.append({"type": "build", "name": name, "frame": frame, **s.fields(),
                    "bytes": s.block(frame).hex()})

    def sf_vec(name, data):
        err, sf = Superframe.unpack(data)
        v = {"type": "sf", "name": name, "bytes": bytes(data).hex(), "result": err}
        if sf is not None:
            v.update(sf.fields())
        out.append(v)

    # The spec's two examples (12.2).
    typical = src.block(960_000)
    build("spdif_sf/typical", src, 960_000)
    sf_vec("spdif_sf/typical", typical)
    noapply = Source(rate=RATE_48K, sender=0x42, sender2=0x37, epoch=3)
    build("spdif_sf/no_apply", noapply, 960_192)
    sf_vec("spdif_sf/no_apply", noapply.block(960_192))
    # Apply carriage edge: frame + 192 = N is carried, one more frame is not.
    edge = Source(rate=RATE_48K, sender=0x42, sender2=0x37, epoch=3,
                  apply_frame=960_000 + BLOCK_FRAMES, apply_op=OP_CLICK, apply_arg=1)
    build("spdif_sf/edge_carried", edge, 960_000)
    build("spdif_sf/edge_not_carried", edge, 960_192)
    edge.apply_frame = 960_000 + BLOCK_FRAMES - 1
    build("spdif_sf/edge_one_short", edge, 960_000)
    # Epoch change: same block, next epoch; and the wrap of the u8 epoch.
    build("spdif_sf/epoch4", Source(rate=RATE_48K, sender=0x42, sender2=0x37, epoch=4), 960_000)
    build("spdif_sf/epoch_wrap", Source(rate=RATE_48K, sender=0x42, sender2=0x37, epoch=0), 0)
    # Every rate, every allowed op with arg/value, a large F, flags b0/b3.
    for rate in RATE_HZ:
        build("spdif_sf/rate%d" % rate, Source(rate=rate, sender=1, sender2=2, epoch=9), 192 * 12345)
    F40 = 192 << 34
    for op, arg, val in ((OP_MUTE, 0, 0), (OP_UNMUTE, 0, 0), (OP_MASTER_VOLUME, 0, -1200),
                         (OP_CLICK, 3, 0), (OP_SIGGEN_START, 0, 0), (OP_SIGGEN_STOP, 1, 0)):
        s = Source(rate=RATE_96K, sender=0xFE, sender2=0x01, epoch=255, flags=FLAG_USB_LIVE,
                   apply_frame=F40 + 5000, apply_op=op, apply_arg=arg, apply_value=val)
        build("spdif_sf/op%d" % op, s, F40)
        sf_vec("spdif_sf/op%d" % op, s.block(F40))
    build("spdif_sf/flags_mute_usb", Source(flags=0xFF, sender=7, epoch=1), 192)
    big = 192 * ((1 << 64) // 192)            # the last block start below 2^64
    build("spdif_sf/frame_big", Source(sender=0x11, sender2=0x22, epoch=7,
                                        apply_frame=(1 << 64) - 1, apply_op=OP_MUTE), big)
    build("spdif_sf/frame_big_before", Source(sender=0x11, sender2=0x22, epoch=7,
                                               apply_frame=(1 << 64) - 1, apply_op=OP_MUTE), big - 192)

    # Validity failures in order (12.2): each tampered field re-signed so the
    # named check is the first to fail; two-fault cases show the order.
    base = typical
    t = bytearray(base)
    t[40] ^= 0xFF
    sf_vec("spdif_sf/padding_ignored", bytes(t))
    t = bytearray(base)
    t[8] ^= 1
    sf_vec("spdif_sf/err_crc", bytes(t))
    t = bytearray(base)
    t[30] ^= 0x01
    sf_vec("spdif_sf/err_crc_field", bytes(t))
    sf_vec("spdif_sf/all_zero", bytes(SF_LEN))

    def tamper(name, **edits):
        b = bytearray(base)
        for off, val in edits.values():
            b[off] = val
        sf_vec(name, _resign(b))

    tamper("spdif_sf/err_magic", m=(0, 0x5A))
    tamper("spdif_sf/err_magic_and_frame", m=(0, 0x5A), f=(8, 0x01))
    tamper("spdif_sf/err_version0", v=(1, 0))
    tamper("spdif_sf/err_version2", v=(1, 2))
    tamper("spdif_sf/err_version_and_class", v=(1, 2), c=(5, 0x01))
    for k, off in (("f1", 1), ("f64", 64), ("f128", 128)):
        b = bytearray(base)
        struct.pack_into("<Q", b, 8, 960_000 + off)
        sf_vec("spdif_sf/err_frame_" + k, _resign(b))
    b = bytearray(base)
    struct.pack_into("<Q", b, 8, 1 << 32)       # 2^32 is not a multiple of 3
    sf_vec("spdif_sf/err_frame_2p32", _resign(b))
    b = bytearray(base)
    struct.pack_into("<Q", b, 8, 3 << 32)       # 3 * 2^32 = 192 * 2^26 * 1
    sf_vec("spdif_sf/frame_3x2p32", _resign(b))
    tamper("spdif_sf/err_frame_and_class", f=(8, 0x01), c=(5, 0x01))
    for cls_ in (0x00, 0x01, 0x02, 0x03, 0x11, 0xFF):
        tamper("spdif_sf/err_class_%02x" % cls_, c=(5, cls_))
    tamper("spdif_sf/err_rate3", r=(3, 3))
    for ch in (0, 1, 3, 8):
        tamper("spdif_sf/err_channels%d" % ch, c=(4, ch))
    tamper("spdif_sf/err_layout_and_apply", c=(5, 0x01), o=(24, 3))
    for op in (3, 5, 6, 8, 11, 31, 32, 255):
        tamper("spdif_sf/err_op%d" % op, o=(24, op))
    b = bytearray(base)
    struct.pack_into("<Q", b, 16, 0)            # op 9 without a target
    sf_vec("spdif_sf/err_op_no_target", _resign(b))
    b = bytearray(noapply.block(960_192))
    struct.pack_into("<Q", b, 16, 2_000_000)    # a target without an op
    sf_vec("spdif_sf/err_target_no_op", _resign(b))
    tamper("spdif_sf/arg_not_judged", a=(25, 0xEE), v=(26, 0x34))
    tamper("spdif_sf/hops_flags_ignored", h=(6, 9), f=(2, 0xF2))

    # User-bit order (12.1): the spec's first 16 bits of the typical block.
    out.append({"type": "ubits", "name": "spdif_sf/typical", "bytes": typical.hex(),
                "bits": "".join(str(x) for x in ubits(typical))})
    out.append({"type": "ubits", "name": "spdif_sf/no_apply", "bytes": noapply.block(960_192).hex(),
                "bits": "".join(str(x) for x in ubits(noapply.block(960_192)))})

    # Broadcast lead (12.4 b).
    for now, target in ((1000, 1896), (1000, 1895), (1000, 1000), (1000, 999), (0, 896),
                        ((1 << 40), (1 << 40) + 10_000)):
        out.append({"type": "lead", "now": now, "target": target, "ok": int(lead_ok(now, target))})
    return out


def main():
    for v in vectors():
        sys.stdout.write(json.dumps(v) + "\n")


if __name__ == "__main__":
    main()
