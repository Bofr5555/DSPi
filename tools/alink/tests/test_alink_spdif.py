"""WP FWSP1: the S/PDIF carriage twin (tools/alink/alink_spdif.py), spec section 12.

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
import alink_codec as lc  # noqa: E402
import alink_spdif as sp  # noqa: E402

VECTORS = os.path.join(HERE, "..", "..", "..", "firmware", "tests", "alink_spdif_vectors.json")

# alink_protocol_spec.md 12.2, verbatim.
TYPICAL = bytes.fromhex(
    "a5 01 04 01 02 10 00 42  00 a6 0e 00 00 00 00 00  40 42 0f 00 00 00 00 00  09 00 00 00 03 37 a7 80"
    "00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00".replace(" ", ""))
NO_APPLY = bytes.fromhex("a501000102100042c0a60e0000000000" + "00" * 12 + "033790dd" + "00" * 16)


class TestSpecExamples(unittest.TestCase):
    def test_typical(self):
        src = sp.Source(rate=sp.RATE_48K, sender=0x42, sender2=0x37, epoch=3,
                        apply_frame=1_000_000, apply_op=sp.OP_SIGGEN_START)
        self.assertEqual(src.block(960_000), TYPICAL)
        self.assertEqual(TYPICAL[30:32], b"\xa7\x80")
        err, sf = sp.Superframe.unpack(TYPICAL)
        self.assertEqual(err, sp.OK)
        self.assertEqual(sf.source_id, 0x3742)
        self.assertEqual(sf.frame, 192 * 5000)

    def test_no_apply(self):
        src = sp.Source(rate=sp.RATE_48K, sender=0x42, sender2=0x37, epoch=3)
        self.assertEqual(src.block(960_192), NO_APPLY)
        self.assertEqual(NO_APPLY[30:32], b"\x90\xdd")
        # Were the op still pending, the next block carries it too.
        src.apply_frame, src.apply_op = 1_000_000, sp.OP_SIGGEN_START
        self.assertEqual(sp.Superframe.unpack(src.block(960_192))[1].apply_op, sp.OP_SIGGEN_START)

    def test_first_user_bits(self):
        self.assertEqual(sp.ubits(TYPICAL)[:16], [1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0])
        self.assertEqual(sp.from_ubits(sp.ubits(TYPICAL)), TYPICAL)

    def test_crc_is_the_lines(self):
        rng = random.Random(1)
        for n in (0, 1, 30, 48):
            data = bytes(rng.randrange(256) for _ in range(n))
            self.assertEqual(sp.crc16(data), lc.crc16(data))
        self.assertEqual(sp.crc16(b"123456789"), 0x29B1)


class TestRules(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(sp.MIN_LEAD_FRAMES, 896)
        self.assertEqual(sp.USER_BITS // 8, sp.SF_LEN)
        self.assertEqual(sorted(sp.OPS_ALLOWED), [1, 2, 4, 7, 9, 10])
        self.assertNotIn(sp.CLASS, lc.CLASS_BAUD)

    def test_carriage_edge(self):
        self.assertTrue(sp.carries(960_000, 960_192))
        self.assertFalse(sp.carries(960_000, 960_191))
        self.assertFalse(sp.carries(960_000, 0))
        self.assertFalse(sp.carries(960_192, 960_192))

    def test_lead(self):
        self.assertTrue(sp.lead_ok(100, 996))
        self.assertFalse(sp.lead_ok(100, 995))
        self.assertFalse(sp.lead_ok(100, 50))

    def test_line_receiver_never_accepts(self):
        # Same CRC over bytes 0-29, so a line receiver gets as far as the class: 0x10 is no line class.
        self.assertEqual(lc.Superframe.unpack(TYPICAL[:32])[0], lc.SF_ERR_LAYOUT)

    def test_roundtrip_random(self):
        rng = random.Random(7)
        for _ in range(500):
            sf = sp.Superframe(flags=rng.randrange(256), rate=rng.randrange(3), channels=2,
                               cls=sp.CLASS, hops=rng.randrange(256), sender=rng.randrange(256),
                               frame=192 * rng.randrange(1 << 50), epoch=rng.randrange(256),
                               sender2=rng.randrange(256))
            if rng.random() < 0.5:
                sf.apply_op = rng.choice(sorted(sp.OPS_ALLOWED))
                sf.apply_frame = rng.randrange(1, 1 << 60)
                sf.apply_arg, sf.apply_value = rng.randrange(256), rng.randrange(-32768, 32768)
            err, back = sp.Superframe.unpack(sf.pack())
            self.assertEqual(err, sp.OK)
            self.assertEqual(back, sf)


class TestVectors(unittest.TestCase):
    def test_vector_file_is_current(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            sp.main()
        with open(VECTORS) as f:
            self.assertEqual(f.read(), buf.getvalue(), "regenerate alink_spdif_vectors.json")

    def test_vectors_pass(self):
        with open(VECTORS) as f:
            vecs = [json.loads(line) for line in f]
        seen = set()
        for v in vecs:
            seen.add(v["type"])
            if v["type"] == "crc16":
                self.assertEqual(sp.crc16(bytes.fromhex(v["data"])), v["crc"])
            elif v["type"] == "build":
                src = sp.Source(**{k: v[k] for k in sp.Source().fields()})
                self.assertEqual(src.block(v["frame"]).hex(), v["bytes"], v["name"])
            elif v["type"] == "sf":
                err, sf = sp.Superframe.unpack(bytes.fromhex(v["bytes"]))
                self.assertEqual(err, v["result"], v["name"])
                if sf is not None:
                    for k in sp.Superframe.FIELDS:
                        self.assertEqual(getattr(sf, k), v[k], v["name"] + " " + k)
            elif v["type"] == "ubits":
                self.assertEqual("".join(map(str, sp.ubits(bytes.fromhex(v["bytes"])))), v["bits"])
            elif v["type"] == "lead":
                self.assertEqual(int(sp.lead_ok(v["now"], v["target"])), v["ok"])
        self.assertEqual(seen, {"crc16", "build", "sf", "ubits", "lead"})
        results = {v["result"] for v in vecs if v["type"] == "sf"}
        self.assertEqual(results, set(range(7)), "every validity result has a vector")


if __name__ == "__main__":
    unittest.main()
