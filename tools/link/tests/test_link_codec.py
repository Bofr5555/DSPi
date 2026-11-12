"""WP LK1: tests for the link protocol Python twin (tools/link/link_codec.py).

    ./run.sh -m unittest discover -s tests -v
"""

import io
import json
import os
import random
import sys
import unittest
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import link_codec as lc  # noqa: E402
import gen_vectors  # noqa: E402

VECTORS = os.path.join(HERE, "..", "..", "..", "firmware", "tests", "link_vectors.json")


def load_vectors():
    with open(VECTORS) as f:
        return [json.loads(line) for line in f]


class TestLK1Anchors(unittest.TestCase):
    """Values fixed by the spec and the standards, independent of the vectors."""

    def test_crc_check_values(self):
        self.assertEqual(lc.crc8(b"123456789"), 0xF4)
        self.assertEqual(lc.crc16(b"123456789"), 0x29B1)

    def test_special_symbols(self):
        self.assertEqual(lc.encode_8b10b(lc.BYTE_K28_5, True, 0), (0b0011111010, 1))
        self.assertEqual(lc.encode_8b10b(lc.BYTE_K28_5, True, 1), (0b1100000101, 0))
        self.assertEqual(lc.encode_8b10b(lc.BYTE_IDLE, False, 0), (lc.SYM_IDLE, 0))
        self.assertEqual(lc.encode_8b10b(lc.BYTE_IDLE, False, 1), (lc.SYM_IDLE, 1))
        self.assertEqual(lc.encode_8b10b(0x00, False, 0), (0b1001110100, 0))
        self.assertEqual(lc.encode_8b10b(0xF1, False, 0), (0b1000110111, 1))   # D17.7 A7
        self.assertEqual(lc.encode_8b10b(0x1D, True, 0), (None, 0))            # no K29.0

    def test_decode_is_exact_inverse(self):
        images = 0
        for k in (False, True):
            for byte in (lc.K_CODES if k else range(256)):
                for rd in (0, 1):
                    sym, rd2 = lc.encode_8b10b(byte, k, rd)
                    images += 1
                    mask, b, kk = lc.decode_8b10b(sym)
                    self.assertTrue(mask >> rd & 1)
                    self.assertEqual((b, kk), (byte, k))
                    self.assertEqual(lc.rd_after(sym, rd), rd2)
        self.assertEqual(images, 2 * (256 + 12))
        self.assertEqual(sum(1 for s in range(1024) if lc.decode_8b10b(s)[0]), 464)

    def test_layout_and_capacity(self):
        self.assertEqual([lc.frame_symbols(c, 1, 0) for c in (1, 2, 3)], [32, 64, 128])
        self.assertEqual([lc.frame_symbols(c, 2, 0) for c in (1, 2, 3)], [16, 32, 64])
        self.assertEqual([lc.max_channels(c, 1) for c in (1, 2, 3)], [8, 18, 38])
        self.assertEqual([lc.max_channels(c, 2) for c in (1, 2, 3)], [3, 8, 18])
        self.assertEqual([lc.max_channels(c, 0) for c in (1, 2, 3)], [8, 19, 41])
        for cls in (1, 2, 3):
            n = sum(lc.frame_symbols(cls, 0, f) for f in range(147))
            self.assertEqual(n * 10 * 147 * 300, lc.CLASS_BAUD[cls] * 147)   # exactly 44.1 kHz
            self.assertEqual(n, lc.CLASS_BAUD[cls] // 3000)
        self.assertEqual(lc.layout_symbols(8), 30)
        self.assertEqual(lc.layout_symbols(18), 4 + 26 + 26 + 8)

    def test_prbs7_period(self):
        bits = lc.bytes_to_bits(lc.prbs7(127 * 2), 127 * 16)
        self.assertEqual(bits[:127], bits[127:254])
        self.assertEqual(sum(bits[:127]), 64)


class TestLK1Stream(unittest.TestCase):
    """Properties of encoded streams and the reference receiver."""

    def test_line_properties(self):
        rng = random.Random(1)
        enc = lc.Encoder(2, 0, 18, 123456789)
        syms, starts = [], []
        for _ in range(300):
            starts.append(len(syms))
            syms += enc.frame_symbols([rng.randrange(-2**23, 2**23) for _ in range(18)])
        bits = lc.symbols_to_bits(syms)
        run = best = 1
        for a, b in zip(bits, bits[1:]):
            run = run + 1 if a == b else 1
            best = max(best, run)
        self.assertLessEqual(best, 5)
        rds, lo, hi = 0, 0, 0
        for b in bits:
            rds += 1 if b else -1
            lo, hi = min(lo, rds), max(hi, rds)
        self.assertLessEqual(hi - lo, 6)
        commas = [i for i in range(len(bits) - 9)
                  if lc.is_comma(int("".join(map(str, bits[i:i + 10])), 2))]
        self.assertEqual(commas, [10 * s for s in starts])

    def _decode(self, bits, cls, mask=0xFF, chunk=None):
        d = lc.Decoder(cls, mask)
        if chunk:
            for i in range(0, len(bits), chunk):
                d.push(bits[i:i + chunk])
        else:
            d.push(bits)
        return d

    def test_samples_round_trip(self):
        for cls, rate, c in ((1, 1, 8), (2, 1, 18), (3, 0, 41), (2, 2, 8)):
            bits = lc.encode_stream(cls, rate, c, 1000, 120, seed=3)
            d = self._decode(bits, cls)
            frames = [e for e in d.events if e[0] == "frame"]
            self.assertGreater(len(frames), 50)
            for _, f, flags, _, status, samples in frames:
                self.assertEqual(flags, 0)
                self.assertTrue(all(s == lc.BANK_OK for s in status))
                self.assertEqual(samples, [lc.test_sample(f, ch, 3) for ch in range(c)])

    def test_prbs_payload(self):
        bits = lc.encode_stream(1, 1, 8, 0, 70, flags=lc.FLAG_PRBS)
        d = self._decode(bits, 1)
        want = [lc.get_s24(lc.prbs7(24)[3 * i:3 * i + 3]) for i in range(8)]
        frames = [e for e in d.events if e[0] == "frame"]
        self.assertTrue(frames)
        for e in frames:
            self.assertEqual(e[5], want)

    def test_chunking_does_not_change_output(self):
        bits = lc.corrupt(lc.encode_stream(1, 0, 8, 7, 160, seed=4),
                          "del:20000;flip:30001;fill:40000:900:0")
        ref = lc.log_of(self._decode(bits, 1))
        for chunk in (1, 7, 32, 333):
            self.assertEqual(lc.log_of(self._decode(bits, 1, chunk=chunk)), ref)

    def test_hold_then_zero(self):
        first = 64
        bits = lc.encode_stream(1, 1, 8, first, 80, seed=5)
        pos = [10 * 32 * i + 10 * 6 for i in (40, 41, 42)]
        d = self._decode(lc.corrupt(bits, ";".join("flip:%d" % p for p in pos)), 1)
        fr = {e[1]: e for e in d.events if e[0] == "frame"}
        self.assertEqual(fr[first + 40][4], [lc.BANK_HOLD])
        self.assertEqual(fr[first + 40][5], fr[first + 39][5])
        self.assertEqual(fr[first + 41][4], [lc.BANK_ZERO])
        self.assertEqual(fr[first + 41][5], [0] * 8)
        self.assertEqual(fr[first + 43][4], [lc.BANK_OK])

    def test_slip_keeps_lock(self):
        bits = lc.encode_stream(2, 1, 16, 0, 100, seed=6)
        d = self._decode(lc.corrupt(bits, "ins:%d:0" % (640 * 60 + 300)), 2)
        self.assertEqual(d.stats["bit_slips"], 1)
        self.assertEqual(d.stats["loss_count"], 0)
        slipped = [e for e in d.events if e[0] == "frame" and e[2] & lc.RXF_SLIP]
        self.assertEqual([(e[1], e[3]) for e in slipped], [(61, 1)])

    def test_inverted_line_aligns_but_never_locks(self):
        bits = lc.encode_stream(1, 1, 8, 0, 100, seed=7)
        d = self._decode([b ^ 1 for b in bits], 1)
        self.assertEqual(d.stats["lock_count"], 0)
        commas = [i for i in range(10, len(bits)) if lc.is_comma(d.sym(i))]
        self.assertEqual(commas[:3], [10, 330, 650])          # the commas survive inversion
        sym = d.sym(20) ^ 0x3FF                                 # complemented HDR decodes again
        self.assertEqual(lc.decode_8b10b(sym)[1], 0)

    def test_rate_change_followed(self):
        bits = lc.encode_stream(2, 1, 16, 640, 150, seed=9, schedule=(0, 736, lc.OP_RATE, 2, 8))
        d = self._decode(bits, 2)
        frames = [e for e in d.events if e[0] == "frame"]
        self.assertEqual(d.stats["loss_count"], 0)
        self.assertEqual({len(e[5]) for e in frames if e[1] < 736}, {16})
        self.assertEqual({len(e[5]) for e in frames if e[1] >= 736}, {8})
        for e in frames:
            self.assertEqual(e[5], [lc.test_sample(e[1], ch, 9) for ch in range(len(e[5]))])

    def test_superframe_repeat_and_apply_carriage(self):
        enc = lc.Encoder(1, 1, 8, 10)
        self.assertFalse(enc.schedule(63, lc.OP_MUTE))       # 32..63 completes at 63, too late
        self.assertTrue(enc.schedule(64, lc.OP_MUTE))
        carried = []
        for _ in range(100):
            f = enc.frame
            enc.frame_symbols([0] * 8)
            if f % 32 == 0 or f == 10:
                carried.append((f - f % 32, lc.Superframe.unpack(enc.sf)[1].apply_op))
        self.assertEqual(carried, [(0, 1), (32, 1), (64, 0), (96, 0)])
        sf = lc.Superframe(rate=1, channels=8, cls=1, hops=254, frame=0)
        self.assertEqual(sf.repeat(3).hops, 255)
        self.assertEqual(sf.repeat(3).repeat(2).hops, 255)
        self.assertEqual(sf.repeat(3).cls, 3)


class TestLK1Vectors(unittest.TestCase):
    """The shared vector file: regenerates identically, and this twin passes it."""

    def test_file_is_current(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            gen_vectors.main()
        with open(VECTORS) as f:
            self.assertEqual(buf.getvalue(), f.read(), "run gen_vectors.py and commit the result")

    def test_vectors(self):
        counts = {}
        for v in load_vectors():
            t = v["type"]
            counts[t] = counts.get(t, 0) + 1
            if t == "crc8":
                self.assertEqual(lc.crc8(bytes.fromhex(v["data"])), v["crc"])
            elif t == "crc16":
                self.assertEqual(lc.crc16(bytes.fromhex(v["data"])), v["crc"])
            elif t == "code":
                self.assertEqual(lc.encode_8b10b(v["byte"], bool(v["k"]), v["rd"]), (v["sym"], v["rd_out"]))
            elif t == "dectab":
                for sym in range(1024):
                    want = int(v["table"][4 * sym:4 * sym + 4], 16)
                    mask, b, k = lc.decode_8b10b(sym)
                    self.assertEqual((mask << 9 | int(k) << 8 | b) if mask else 0, want)
            elif t == "layout":
                self.assertEqual("".join("%02x" % lc.frame_symbols(v["cls"], v["rate"], f)
                                         for f in range(147)), v["lengths"])
                self.assertEqual(lc.max_channels(v["cls"], v["rate"]), v["max_channels"])
            elif t == "layout_c":
                self.assertEqual(lc.layout_symbols(v["channels"]), v["symbols"])
            elif t == "prbs7":
                self.assertEqual(lc.prbs7(48).hex(), v["bytes"])
            elif t == "sf":
                err, sf = lc.Superframe.unpack(bytes.fromhex(v["bytes"]))
                self.assertEqual(err, v["result"], v["name"])
                if v["pack"]:
                    want = lc.Superframe(**{f: v[f] for f in lc.Superframe.FIELDS})
                    self.assertEqual(sf, want)
                    self.assertEqual(want.pack().hex(), v["bytes"])
            elif t == "sf_repeat":
                _, sf = lc.Superframe.unpack(bytes.fromhex(v["bytes_in"]))
                self.assertEqual(sf.repeat(v["cls"]).pack().hex(), v["bytes_out"])
            elif t == "frame":
                enc = lc.Encoder(v["cls"], v["rate"], v["channels"], v["frame"])
                enc.flags = v["flags"]
                syms = enc.frame_symbols([lc.test_sample(v["frame"], ch, v["seed"])
                                          for ch in range(v["channels"])])
                self.assertEqual("".join("%03x" % s for s in syms), v["symbols"], v["name"])
                self.assertEqual(len(syms), lc.frame_symbols(v["cls"], v["rate"], v["frame"]))
            elif t == "stream":
                clean = lc.bytes_to_bits(bytes.fromhex(v["stream"]), v["bits"])
                if v["encode"]:
                    s = v["sched_index"]
                    sched = None if s < 0 else (s, v["sched_at"], v["sched_op"], v["sched_arg"], v["sched_value"])
                    args = {k: v[k] for k in gen_vectors.STREAM_KEYS}
                    self.assertEqual(lc.encode_stream(schedule=sched, **args), clean, v["name"])
                d = lc.Decoder(v["cls"], v["bank_mask"])
                bits = lc.corrupt(clean, v["corrupt"])
                for i in range(0, len(bits), 32):
                    d.push(bits[i:i + 32])
                self.assertEqual(lc.log_of(d), v["log"], v["name"])
            else:
                self.fail("unknown vector type " + t)
        self.assertEqual(counts["code"], 536)
        self.assertGreaterEqual(counts["stream"], 25)


if __name__ == "__main__":
    unittest.main()
