#!/usr/bin/env python3
"""Regenerate firmware/tests/alink_vectors.json (shared C / Python alink vectors).

One flat JSON object per line (strings and integers only) so the C test
(firmware/tests/test_alink_codec.c) scans it without a JSON library. The
format is in Documentation/Features/alink_protocol_spec.md section 9.

    python3 -I gen_vectors.py > ../../firmware/tests/alink_vectors.json
"""

import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import alink_codec as lc # noqa: E402


def sym_hex(symbols):
    return "".join("%03x" % s for s in symbols)


def comma_at(bits, pos):
    """Flip ops that write a RD- K28.5 into bits [pos, pos+10)."""
    want = lc.symbols_to_bits([lc.K28_5_RDN])
    return ";".join("flip:%d" % (pos + i) for i in range(10) if bits[pos + i] != want[i])


class _TamperedEncoder(lc.Encoder):
    """An encoder whose superframe for one first-frame is altered (crafted vectors)."""

    def __init__(self, *a, tamper=None, hdr=None, **kw):
        super().__init__(*a, **kw)
        self.tamper = tamper
        self.hdr_of = hdr

    def _hdr(self, f):
        return self.hdr_of(f) if self.hdr_of else super()._hdr(f)

    def _pack_sf(self, first):
        super()._pack_sf(first)
        if self.tamper:
            self.sf = self.tamper(first, bytearray(self.sf))


def tampered_stream(cls, rate, channels, first, frames, seed, tamper=None, hdr=None):
    enc = _TamperedEncoder(cls, rate, channels, first, tamper=tamper, hdr=hdr)
    syms = []
    for _ in range(frames):
        f = enc.frame
        syms += enc.frame_symbols([lc.test_sample(f, ch, seed) for ch in range(channels)])
    return lc.symbols_to_bits(syms)


def decode_log(bits, cls, bank_mask=0xFF):
    d = lc.Decoder(cls, bank_mask)
    d.push(bits)
    return lc.log_of(d)


STREAM_KEYS = ("cls", "rate", "channels", "first", "frames", "seed", "sender", "flags", "hops")


def stream_vector(name, corrupt_ops="", bank_mask=0xFF, schedule=None, **p):
    args = dict(seed=0, sender=0, flags=0, hops=0)
    args.update(p)
    clean = lc.encode_stream(schedule=schedule, **args)
    ops = corrupt_ops(clean) if callable(corrupt_ops) else corrupt_ops
    bits = lc.corrupt(clean, ops)
    v = dict(type="stream", name=name, encode=1)
    v.update({k: args[k] for k in STREAM_KEYS})
    s = schedule or (-1, 0, 0, 0, 0)
    v.update(sched_index=s[0], sched_at=s[1], sched_op=s[2], sched_arg=s[3], sched_value=s[4])
    v.update(bank_mask=bank_mask, bits=len(clean), stream=lc.bits_to_bytes(clean).hex(),
             corrupt=ops, log=decode_log(bits, args["cls"], bank_mask))
    return v


def crafted_vector(name, cls, bits, note):
    return dict(type="stream", name=name, encode=0, cls=cls, bank_mask=0xFF, bits=len(bits),
                stream=lc.bits_to_bytes(bits).hex(), corrupt="", note=note,
                log=decode_log(bits, cls))


def frame_bit(cls, rate, channels, first, index, symbol):
    """Bit offset of symbol `symbol` of frame `first + index` in a stream from `first`."""
    pos = sum(lc.frame_symbols(cls, rate, first + i) for i in range(index))
    return 10 * (pos + symbol)


def vectors():
    v = []

    # -- CRCs: check values of the catalogue, plus protocol-shaped inputs.
    check = b"123456789"
    for name, data in (("check", check), ("empty", b""), ("head", bytes((0x14, 0xA5))),
                       ("head_ff", bytes((0xFF, 0xFF)))):
        v.append(dict(type="crc8", name=name, data=data.hex(), crc=lc.crc8(data)))
    bank = b"".join(lc.put_s24(lc.test_sample(7, ch, 1)) for ch in range(8))
    for name, data in (("check", check), ("empty", b""), ("zeros24", bytes(24)),
                       ("bank", bank), ("bank2ch", bank[:6])):
        v.append(dict(type="crc16", name=name, data=data.hex(), crc=lc.crc16(data)))

    # -- 8b/10b: every code group the encoder makes, and the full decode table.
    for k in (0, 1):
        for byte in (lc.K_CODES if k else range(256)):
            for rd in (0, 1):
                sym, rd_out = lc.encode_8b10b(byte, bool(k), rd)
                v.append(dict(type="code", k=k, byte=byte, rd=rd, sym=sym, rd_out=rd_out))
    table = ""
    for sym in range(1024):
        mask, byte, k = lc.decode_8b10b(sym)
        table += "%04x" % ((mask << 9) | (int(k) << 8) | byte if mask else 0)
    v.append(dict(type="dectab", table=table))

    # -- Layout: lengths over one 44.1 kHz cycle (147 frames), capacity, layouts.
    for cls in (1, 2, 3):
        for rate in (0, 1, 2):
            lengths = [lc.frame_symbols(cls, rate, f) for f in range(147)]
            v.append(dict(type="layout", cls=cls, rate=rate, min=lc.frame_symbols_min(cls, rate),
                          max=lc.frame_symbols_max(cls, rate),
                          max_channels=lc.max_channels(cls, rate),
                          lengths="".join("%02x" % n for n in lengths), sum=sum(lengths),
                          at_big=lc.frame_symbols(cls, rate, 2**63 + 12345)))
    for c in range(1, 65):
        v.append(dict(type="layout_c", channels=c, symbols=lc.layout_symbols(c)))

    v.append(dict(type="prbs7", bytes=lc.prbs7(48).hex()))

    # -- Superframes.
    sfs = [
        ("typical", lc.Superframe(flags=lc.FLAG_USB_LIVE, rate=1, channels=8, cls=1, hops=0,
                                  sender=0x42, frame=1 << 40)),
        ("apply_volume", lc.Superframe(flags=lc.FLAG_APPLY_PEND, rate=1, channels=16, cls=2,
                                       hops=1, sender=7, frame=1_002_336,
                                       apply_frame=1_002_400, apply_op=lc.OP_MASTER_VOLUME,
                                       apply_value=-2000)),
        ("apply_rate", lc.Superframe(flags=lc.FLAG_APPLY_PEND | lc.FLAG_RATE_PEND, rate=1,
                                     channels=32, cls=3, hops=2, sender=0xFF, frame=2**64 - 8192,
                                     apply_frame=2**64 - 4096, apply_op=lc.OP_RATE,
                                     apply_arg=2, apply_value=16)),
        ("max_441", lc.Superframe(flags=0x3F, rate=0, channels=41, cls=3, hops=255, sender=0,
                                  frame=0)),
        # WP FWTB1: ops 9 SIGGEN_START and 10 SIGGEN_STOP (now) are valid (6.3, 6.4).
        ("apply_siggen_start", lc.Superframe(flags=lc.FLAG_APPLY_PEND, rate=1, channels=8, cls=1,
                                             hops=0, sender=0x42, frame=960_000,
                                             apply_frame=972_000, apply_op=lc.OP_SIGGEN_START)),
        ("apply_siggen_stop", lc.Superframe(flags=lc.FLAG_APPLY_PEND, rate=1, channels=8, cls=1,
                                            hops=0, sender=0x42, frame=960_000,
                                            apply_frame=962_400, apply_op=lc.OP_SIGGEN_STOP,
                                            apply_arg=1)),
    ]
    for name, sf in sfs:
        d = dict(type="sf", name=name, pack=1, result=0, bytes=sf.pack().hex())
        d.update({f: getattr(sf, f) for f in lc.Superframe.FIELDS})
        v.append(d)
    good = sfs[0][1].pack()

    def recrc(b):
        b = bytearray(b)
        b[30:32] = struct.pack("<H", lc.crc16(bytes(b[:30])))
        return bytes(b)

    def edit(i, val):
        b = bytearray(good)
        b[i] = val
        return recrc(b)

    bad = [
        ("bad_crc", bytes(good[:31]) + bytes((good[31] ^ 1,)), lc.SF_ERR_CRC),
        ("bad_magic", edit(0, 0x5A), lc.SF_ERR_MAGIC),
        ("bad_version", edit(1, 2), lc.SF_ERR_VERSION),
        ("bad_frame", edit(8, 0x01), lc.SF_ERR_FRAME),
        ("bad_class", edit(5, 4), lc.SF_ERR_LAYOUT),
        ("bad_rate", edit(3, 3), lc.SF_ERR_LAYOUT),
        ("zero_channels", edit(4, 0), lc.SF_ERR_LAYOUT),
        ("too_many_channels", edit(4, 9), lc.SF_ERR_LAYOUT),
        ("bad_op", recrc(good[:16] + struct.pack("<Q", 5000) + bytes((11,)) + good[25:]),
         lc.SF_ERR_APPLY),
        ("op_without_target", edit(24, 1), lc.SF_ERR_APPLY),
        ("target_without_op", recrc(good[:16] + struct.pack("<Q", 5000) + good[24:]),
         lc.SF_ERR_APPLY),
        ("reserved_ignored", edit(28, 0x77), lc.SF_OK),
    ]
    for name, b, res in bad:
        v.append(dict(type="sf", name=name, pack=0, result=res, bytes=b.hex()))
    # A version-1 receiver built before WP FWTB1 (ops 0..8) rejects ops 9 and 10
    # (code 6) and keeps playing on its counter (6.3, 6.4): the same bytes,
    # judged with op_max 8.
    for name in ("apply_siggen_start", "apply_siggen_stop"):
        b = dict(sfs)[name].pack()
        v.append(dict(type="sf", name=name + "_pre_fwtb1", pack=0, result=lc.SF_ERR_APPLY,
                      op_max=lc.OP_MAX_V1_PRE_FWTB1, bytes=b.hex()))
    for name, sf in sfs[:3]:
        for out_cls in (1, 3):
            v.append(dict(type="sf_repeat", name=name, cls=out_cls, bytes_in=sf.pack().hex(),
                          bytes_out=sf.repeat(out_cls).pack().hex()))

    # -- Single frames (one-frame streams, RD- at the start).
    for name, cls, rate, c, f, seed, flags in (
            ("l1_48_8ch", 1, 1, 8, 1 << 40, 1, 0), ("l1_96_3ch", 1, 2, 3, 77, 2, 0),
            ("l2_48_18ch", 2, 1, 18, 5, 3, 0), ("l3_441_41ch", 3, 0, 41, 146, 4, 0),
            ("l1_48_2ch_prbs", 1, 1, 2, 64, 5, lc.FLAG_PRBS)):
        enc = lc.Encoder(cls, rate, c, f)
        enc.flags = flags
        syms = enc.frame_symbols([lc.test_sample(f, ch, seed) for ch in range(c)])
        v.append(dict(type="frame", name=name, cls=cls, rate=rate, channels=c, frame=f,
                      seed=seed, flags=flags, symbols=sym_hex(syms), rd_out=enc.rd))

    # -- Clean streams.
    big = (1 << 40) + 20
    base = dict(cls=1, rate=1, channels=8, first=big, frames=80, seed=1, sender=0x42,
                flags=lc.FLAG_USB_LIVE)
    v.append(stream_vector("l1_48_clean", **base))
    v.append(stream_vector("l1_48_from0", cls=1, rate=1, channels=2, first=0, frames=40, seed=2))
    v.append(stream_vector("l2_48_18ch", cls=2, rate=1, channels=18, first=1000, frames=70,
                           seed=3, hops=1))
    v.append(stream_vector("l3_48_38ch", cls=3, rate=1, channels=38, first=5_000_000_000,
                           frames=70, seed=4))
    v.append(stream_vector("l1_441_8ch", cls=1, rate=0, channels=8, first=147 * 1000 + 5,
                           frames=200, seed=5))
    v.append(stream_vector("l3_441_41ch", cls=3, rate=0, channels=41, first=2**33 + 100,
                           frames=100, seed=6))
    v.append(stream_vector("l2_96_8ch", cls=2, rate=2, channels=8, first=31, frames=70, seed=7))
    v.append(stream_vector("l1_96_3ch", cls=1, rate=2, channels=3, first=9, frames=70, seed=8))
    v.append(stream_vector("l1_48_prbs", cls=1, rate=1, channels=8, first=3, frames=100,
                           flags=lc.FLAG_PRBS))
    v.append(stream_vector("l2_48_bank1_only", bank_mask=0x02, cls=2, rate=1, channels=18,
                           first=1000, frames=70, seed=3))
    v.append(stream_vector("l2_rate_change_48_to_96", cls=2, rate=1, channels=16, first=640,
                           frames=150, seed=9, schedule=(0, 736, lc.OP_RATE, 2, 8)))
    v.append(stream_vector("l1_rate_change_48_to_441", cls=1, rate=1, channels=8, first=0,
                           frames=120, seed=10, schedule=(3, 96, lc.OP_RATE, 0, 0)))
    v.append(stream_vector("l1_apply_volume", cls=1, rate=1, channels=8, first=200, frames=90,
                           seed=11, schedule=(0, 300, lc.OP_MASTER_VOLUME, 0, -2000)))

    # -- Corrupt and slipped streams (L1 48 kHz, 320 bits per frame, from `big`).
    def fb(index, symbol):
        return frame_bit(1, 1, 8, big, index, symbol)

    v.append(stream_vector("bank_bit_error", "flip:%d" % (fb(50, 10) + 3), **base))
    v.append(stream_vector("bank_hold_then_zero",
                           "flip:%d;flip:%d;flip:%d" % (fb(50, 10), fb(51, 10), fb(52, 10)), **base))
    v.append(stream_vector("head_bit_error", "flip:%d" % (fb(55, 1) + 4), **base))
    v.append(stream_vector("comma_destroyed", "fill:%d:10:0" % fb(60, 0), **base))
    v.append(stream_vector("slip_deleted_bit", "del:%d" % fb(50, 20), **base))
    v.append(stream_vector("slip_inserted_bit", "ins:%d:1" % fb(50, 20), **base))
    v.append(stream_vector("slip_two_bits", "del:%d;del:%d" % (fb(50, 20), fb(50, 22)), **base))
    v.append(stream_vector("slip_three_bits_is_loss",
                           "del:%d;del:%d;del:%d" % (fb(46, 20), fb(46, 22), fb(46, 24)),
                           **dict(base, frames=140)))
    v.append(stream_vector("three_commas_lost",
                           ";".join("fill:%d:10:1" % fb(i, 0) for i in (50, 51, 52)),
                           **dict(base, frames=140)))
    v.append(stream_vector("dropout_2000_bits", "fill:%d:2000:0" % fb(45, 7),
                           **dict(base, frames=150)))
    v.append(stream_vector("stray_comma", lambda bits: comma_at(bits, fb(50, 12) + 4), **base))
    v.append(stream_vector("ctl_error_breaks_superframe", "flip:%d" % fb(70, 2), **base))
    v.append(stream_vector("garbage_before_stream",
                           "ins:0:1;" + ";".join("ins:0:%d" % ((i * 7) % 3 == 0) for i in range(300)),
                           **base))

    # -- RATE that grows C (the encoder switches at the end of the frame before the target).
    v.append(stream_vector("l2_rate_grow_8_to_16", cls=2, rate=1, channels=8, first=10, frames=140,
                           seed=14, schedule=(0, 96, lc.OP_RATE, 1, 16)))
    v.append(stream_vector("l1_rate_grow_96k3_to_441k8", cls=1, rate=2, channels=3, first=0,
                           frames=140, seed=15, schedule=(5, 96, lc.OP_RATE, 0, 8)))

    # -- Damage at 44.1 kHz (L1, frames alternate 34/35 symbols).
    b441 = dict(cls=1, rate=0, channels=8, first=147 * 1000 + 5, frames=150, seed=16)

    def fb441(index, symbol):
        return frame_bit(1, 0, 8, b441["first"], index, symbol)

    v.append(stream_vector("l1_441_bank_and_comma", "flip:%d;fill:%d:10:0"
                           % (fb441(60, 9), fb441(70, 0)), **b441))
    v.append(stream_vector("l1_441_slip_minus1_plus2", "del:%d;ins:%d:0;ins:%d:1"
                           % (fb441(62, 15), fb441(80, 15), fb441(80, 17)), **b441))
    v.append(stream_vector("l1_441_three_commas_lost_is_loss",
                           ";".join("fill:%d:10:1" % fb441(i, 0) for i in (64, 65, 66)),
                           **dict(b441, frames=180)))

    # -- +2-bit slip at 48 kHz.
    v.append(stream_vector("slip_plus_two_bits", "ins:%d:0;ins:%d:1" % (fb(50, 20), fb(50, 22)),
                           **base))

    # -- Fill past the end of the stream is clamped (test-helper parity).
    v.append(stream_vector("fill_past_end", "fill:%d:5000:0" % fb(78, 3), **base))

    # -- Crafted streams the reference encoder cannot make.
    def sf_bad_crc(first, b):
        if first == 32 * 3:
            b[31] ^= 0x40
        return bytes(b)

    def sf_class(first, b):
        b[5] = 2
        b[30:32] = struct.pack("<H", lc.crc16(bytes(b[:30])))
        return bytes(b)

    v.append(crafted_vector("sf_bad_crc", 1, tampered_stream(1, 1, 8, 0, 160, 12, sf_bad_crc),
                            "superframe at F=96 has a bad CRC-16: E1 while locked"))
    v.append(crafted_vector("sf_wrong_class", 1, tampered_stream(1, 1, 8, 0, 100, 13, sf_class),
                            "L1 timing but class byte 2: never locks, E17 per superframe"))

    def hdr_wrong(f):
        return (f + 0x40) & 0xFF if f == 70 else f & 0xFF

    def sf_plus_256(first, b):
        if first == 96:
            frame = struct.unpack_from("<Q", b, 8)[0] + 256
            b[8:16] = struct.pack("<Q", frame)
            b[30:32] = struct.pack("<H", lc.crc16(bytes(b[:30])))
        return bytes(b)

    v.append(crafted_vector("hdr_mismatch", 1, tampered_stream(1, 1, 8, 0, 110, 17, hdr=hdr_wrong),
                            "frame 70 sends a wrong HDR with a good CRC-8: flag n, counter_errors"))
    v.append(crafted_vector("sf_frame_jump_256", 1,
                            tampered_stream(1, 1, 8, 0, 200, 18, tamper=sf_plus_256),
                            "superframe at 96 claims F+256 (HDRs still match): F corrected twice"))
    return v


def main():
    for vec in vectors():
        sys.stdout.write(json.dumps(vec, separators=(", ", ": ")) + "\n")


if __name__ == "__main__":
    main()
