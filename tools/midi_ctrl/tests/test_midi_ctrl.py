"""Device-free tests for the MIDI control host tool.

    ./run.sh -m unittest discover -s tests -v
    (pytest also collects these: python -I -m pytest tests)
"""

import io
import json
import os
import struct
import sys
import unittest
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import fake_device as fd  # noqa: E402
import midi_ctrl as mc  # noqa: E402

VECTORS = os.path.join(HERE, "..", "..", "..", "firmware", "tests", "midi_vectors.json")


def lcg_bytes(n, seed=1):
    out = bytearray()
    x = seed
    for _ in range(n):
        x = (x * 1103515245 + 12345) & 0xFFFFFFFF
        out.append((x >> 16) & 0xFF)
    return bytes(out)


class Codec(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(mc.crc16(b"123456789"), 0x29B1)

    def test_pack_roundtrip(self):
        for n in list(range(0, 301)) + [6142, 6146, 6147]:
            raw = lcg_bytes(n, n + 1)
            p = mc.pack7(raw)
            self.assertEqual(len(p), mc.packed_len(n))
            self.assertTrue(all(b < 0x80 for b in p))
            self.assertEqual(mc.unpack7(p), raw)

    def test_pack_worked_example(self):
        raw = bytes([0x81, 0x02, 0x83, 0x04, 0x85, 0x06, 0x87, 0xFF])
        self.assertEqual(mc.pack7(raw), bytes([0x55, 1, 2, 3, 4, 5, 6, 7, 0x01, 0x7F]))

    def test_unpack_malformed(self):
        for bad in (bytes([0, 1, 2, 3, 4, 5, 6, 7, 0]), bytes([0, 0x81]), bytes([0x02, 0x11])):
            with self.assertRaises(ValueError):
                mc.unpack7(bad)

    def test_sizes(self):
        self.assertEqual(mc.sysex_len(8 + 6136 + 2), 7032)
        self.assertEqual(mc.sysex_len(4 + 6136 + 2), 7028)
        self.assertEqual(len(mc.encode(mc.request_frame(mc.TYPE_SET, 0xA1, payload=bytes(6136)), 1)), 7032)

    def test_worked_example(self):
        self.assertEqual(mc.encode(mc.request_frame(mc.TYPE_GET, 0x7F), 1).hex(),
                         "f07d445350010100027f000000000004001030f7")

    def test_vectors(self):
        with open(VECTORS) as f:
            vecs = [json.loads(line) for line in f if line.strip()]
        self.assertGreaterEqual(len(vecs), 10)
        for v in vecs:
            mfr = bytes.fromhex(v["mfr"])
            frame = bytes.fromhex(v["frame"])
            sysex = bytes.fromhex(v["sysex"])
            self.assertEqual(mc.encode(frame, v["tag"], mfr), sysex, v["name"])
            self.assertEqual(mc.decode(sysex, mfr), (v["tag"], frame), v["name"])
            if v["dir"] == "h2d":
                self.assertTrue(mc.parse_request(frame)["crc_ok"], v["name"])
            else:
                self.assertTrue(mc.Reply(v["tag"], frame).crc_ok, v["name"])

    def test_decode_errors(self):
        good = mc.encode(mc.request_frame(mc.TYPE_GET, 0x7F), 1)
        with self.assertRaises(mc.NotOurs):
            mc.decode(good, b"\x7e")
        with self.assertRaises(ValueError):
            mc.decode(good[:5] + b"\x02" + good[6:])
        with self.assertRaises(ValueError):
            mc.decode(good[:-1])

    def test_splitter(self):
        s = mc.SysexSplitter()
        a = mc.encode(mc.request_frame(mc.TYPE_GET, 0x7F), 1)
        b = mc.encode(mc.request_frame(mc.TYPE_GET, 0x7F), 2)
        stream = b"\x12\x34" + a[:5] + b"\xf8" + a[5:] + a[:7] + b      # junk, real-time, abort
        out = []
        for i in range(0, len(stream), 3):
            out += s.feed(stream[i:i + 3])
        self.assertEqual(out, [a, b])
        self.assertEqual(s.aborted, 1)
        out = s.feed(a[:6] + b"\xf6" + a)                               # system common aborts
        self.assertEqual(out, [a])


class ClientAgainstFake(unittest.TestCase):
    def setUp(self):
        self.dev = fd.FakeDevice()
        self.c = mc.Client(fd.FakePort(self.dev, chunk=7), timeout=0.5)

    def test_get_platform(self):
        rep = self.c.get(mc.REQ_GET_PLATFORM)
        self.assertEqual((rep.status, rep.payload), (mc.ST_OK, fd.PLATFORM))
        self.assertEqual(rep.type, mc.RESP_GET)
        rep = self.c.get(mc.REQ_GET_PLATFORM, wlength=4)
        self.assertEqual(rep.payload, fd.PLATFORM[:4])

    def test_set_volume_and_notify(self):
        rep = self.c.set(mc.REQ_SET_MASTER_VOLUME, payload=struct.pack("<f", -20.0))
        self.assertEqual((rep.status, rep.type, rep.payload), (mc.ST_OK, mc.RESP_SET, b""))
        n = self.c.wait_notify(lambda r: r.payload[1] == 0x02, 0.5)
        self.assertIsNotNone(n)
        self.assertEqual(n.tag, 0)
        self.assertEqual(n.payload[8], 10)          # PARAM_SRC_MIDI
        self.assertIn("src=MIDI", mc.decode_notify(n.payload))
        rep = self.c.get(mc.REQ_GET_MASTER_VOLUME)
        self.assertAlmostEqual(struct.unpack("<f", rep.payload)[0], -20.0)

    def test_bulk_roundtrip(self):
        img = self.c.get(mc.REQ_GET_ALL_PARAMS).payload
        self.assertEqual(len(img), 6136)
        new = bytes(b ^ 0x55 for b in img)
        rep = self.c.set(mc.REQ_SET_ALL_PARAMS, payload=new)
        self.assertEqual(rep.status, mc.ST_OK)
        self.assertEqual(self.c.get(mc.REQ_GET_ALL_PARAMS).payload, new)
        n = self.c.wait_notify(lambda r: r.payload[1] == 0x03, 0.5)
        self.assertIsNotNone(n)

    def test_status_codes(self):
        self.assertEqual(self.c.set(0x20, payload=bytes(65)).status, mc.ST_OVERSIZE)
        self.assertEqual(self.c.get(0x9F).status, mc.ST_ERROR)   # unassigned in v1.1.6

    def test_extension_channel(self):
        rep = self.c.get(mc.REQ_EXTENSION, mc.EXT_LIST)
        self.assertEqual(rep.status, mc.ST_OK)
        self.assertEqual(struct.unpack("<2H", rep.payload), (1, 2))
        self.assertEqual(self.c.get(mc.REQ_EXTENSION, mc.EXT_LIST, wlength=2).payload, b"\x01\x00")
        k340 = bytes([0x02, 0x13, 0, 0])
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_CROSSFEED_PAIR_MAP, payload=k340).status, mc.ST_OK)
        self.assertEqual(self.c.get(mc.REQ_EXTENSION, mc.EXT_CROSSFEED_PAIR_MAP).payload, k340)
        self.assertEqual(len(self.c.get(mc.REQ_EXTENSION, mc.EXT_CORE_VREG).payload), 16)
        # Refusals: wrong OUT length, OUT on the IN-only list, unknown sub-command.
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_CROSSFEED_PAIR_MAP, payload=k340[:3]).status, mc.ST_ERROR)
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_LIST, payload=b"\x00\x00").status, mc.ST_ERROR)
        self.assertEqual(self.c.get(mc.REQ_EXTENSION, 0x7FFF).status, mc.ST_ERROR)

    def test_usb_parity_opcodes(self):
        # 0xF5 / 0xF7 are accepted over MIDI (fake only: never sent to a unit).
        self.assertEqual(self.c.set(0xF5, payload=bytes(8)).status, mc.ST_OK)
        self.assertEqual(self.dev.iface_config, [(0xF5, bytes(8))])
        # RTA bands-all is not capped at 132 bytes.
        self.assertEqual(len(self.c.get(0x0F).payload), 300)
        # Chunked bulk read / write, 2048-byte chunks.
        img = self.c.get(mc.REQ_GET_ALL_PARAMS).payload
        self.assertEqual(mc.chunked_read(self.c), img)
        new = bytes(b ^ 0xA5 for b in img)
        self.assertEqual(mc.chunked_write(self.c, new), mc.ST_OK)
        self.assertEqual(self.c.get(mc.REQ_GET_ALL_PARAMS).payload, new)
        self.assertEqual(self.c.get(0xA2, 2048, 0, 2048).status, mc.ST_ERROR)   # no session

    def test_busy_retry(self):
        self.dev.inject = [mc.ST_BUSY, mc.ST_BULK_LOCKED]
        rep = self.c.get(mc.REQ_GET_PLATFORM)
        self.assertEqual(rep.status, mc.ST_OK)
        self.assertEqual(self.c.stats["retry"], 2)
        self.dev.inject = [mc.ST_BUSY] * 3
        rep = self.c.get(mc.REQ_GET_PLATFORM, retries=1)
        self.assertEqual(rep.status, mc.ST_BUSY)

    def test_malformed_requests(self):
        port = self.c.port
        bad = bytearray(mc.encode(mc.request_frame(mc.TYPE_GET, 0x7F), 5))
        bad[10] ^= 1
        port.write(bytes(bad))
        self.c.pump(0.05)
        rep = self.c._ready.pop()
        self.assertEqual((rep.tag, rep.status), (5, mc.ST_CRC))
        port.write(bytes([0xF0, 0x7D, 0x44, 0x53, 0x50, 0x09, 0x01, 0xF7]))
        self.c.pump(0.05)
        rep = self.c._ready.pop()
        self.assertEqual((rep.tag, rep.status), (mc.TAG_UNKNOWN, mc.ST_FRAME))
        port.write(bytes([0xF0, 0x41, 0x10, 0xF7]))     # foreign: ignored
        self.c.pump(0.05)
        self.assertEqual(self.c._ready, [])

    def test_foreign_and_garbage_from_device(self):
        self.dev.out += bytes([0xF0, 0x41, 0x10, 0x42, 0xF7, 0x33, 0xF8])
        self.assertEqual(self.c.get(mc.REQ_GET_PLATFORM).status, mc.ST_OK)
        self.assertEqual(self.c.stats["foreign_sysex"], 1)

    def test_timeout(self):
        self.dev.feed = lambda data: None               # device goes deaf
        with self.assertRaises(mc.Timeout):
            self.c.get(mc.REQ_GET_PLATFORM, timeout=0.1)


class Scripts(unittest.TestCase):
    def run_main(self, mod, argv):
        buf = io.StringIO()
        with redirect_stdout(buf):
            rc = mod.main(argv)
        return rc, buf.getvalue()

    def test_cli(self):
        rc, out = self.run_main(mc, ["get", "0x7F", "--fake"])
        self.assertEqual(rc, 0)
        self.assertIn("status=OK len=7", out)
        rc, out = self.run_main(mc, ["set", "0xD2", "--float", "-12", "--fake"])
        self.assertEqual(rc, 0)
        rc, out = self.run_main(mc, ["enter-bootloader", "--yes", "--fake"])
        self.assertEqual(rc, 0)

    def test_measure(self):
        import measure
        rc, out = self.run_main(measure, ["--fake", "--rtt", "50", "--bulk-get", "3",
                                          "--allow-bulk-set", "--bulk-set", "2", "--notify", "4"])
        self.assertEqual(rc, 0, out)
        self.assertIn("| GET 0x7F round trip |", out)
        self.assertIn("SET 0xA1 bulk write", out)
        self.assertIn("PARAM_CHANGED", out)
        self.assertIn("missing 0", out)

    def test_soak(self):
        import soak
        rc, out = self.run_main(soak, ["--fake", "--minutes", "0.05", "--vol-min", "-40",
                                       "--vol-max", "-30", "--bulk-s", "0.5", "--vol-s", "0.3"])
        self.assertEqual(rc, 0, out)
        self.assertIn("SOAK PASS", out)


if __name__ == "__main__":
    unittest.main()


class PtyLoopback(unittest.TestCase):
    """The real RawMidiPort code path against fake_device.py --pty in another process."""

    def test_pty(self):
        import subprocess
        proc = subprocess.Popen([sys.executable, "-I", os.path.join(HERE, "..", "fake_device.py"),
                                 "--pty", "--notify-every", "0.2"],
                                stdout=subprocess.PIPE, text=True)
        try:
            path = proc.stdout.readline().strip()
            self.assertTrue(path.startswith("/dev/pts/"), path)
            port = mc.RawMidiPort(path)
            c = mc.Client(port, timeout=2.0)
            self.assertEqual(c.get(mc.REQ_GET_PLATFORM).payload, fd.PLATFORM)
            self.assertEqual(len(c.get(mc.REQ_GET_ALL_PARAMS).payload), 6136)
            n = c.wait_notify(lambda r: r.payload[8] == 5, 2.0)     # GPIO-sourced push
            self.assertIsNotNone(n)
            import measure
            buf = io.StringIO()
            with redirect_stdout(buf):
                rc = measure.main(["--port", path, "--rtt", "100", "--bulk-get", "3", "--notify", "4"])
            self.assertEqual(rc, 0, buf.getvalue())
            port.close()
        finally:
            proc.kill()
            proc.wait()
