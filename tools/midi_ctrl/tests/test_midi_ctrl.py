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
        self.assertEqual(n.payload[4], 10)          # PARAM_SRC_MIDI (NOTIFY_FEAT_BULK_SOURCE)
        self.assertIn("src=MIDI", mc.decode_notify(n.payload))

    def test_status_codes(self):
        self.assertEqual(self.c.set(0x20, payload=bytes(65)).status, mc.ST_OVERSIZE)
        self.assertEqual(self.c.get(0x9F).status, mc.ST_ERROR)   # unassigned in v1.1.6

    def test_extension_channel(self):
        rep = self.c.get(mc.REQ_EXTENSION, mc.EXT_LIST)
        self.assertEqual(rep.status, mc.ST_OK)
        self.assertEqual(struct.unpack("<6H", rep.payload), (1, 2, 3, 4, 5, 6))
        self.assertEqual(self.c.get(mc.REQ_EXTENSION, mc.EXT_LIST, wlength=2).payload, b"\x01\x00")
        k340 = bytes([0x02, 0x13, 0, 0])
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_CROSSFEED_PAIR_MAP, payload=k340).status, mc.ST_OK)
        self.assertEqual(self.c.get(mc.REQ_EXTENSION, mc.EXT_CROSSFEED_PAIR_MAP).payload, k340)
        self.assertEqual(len(self.c.get(mc.REQ_EXTENSION, mc.EXT_CORE_VREG).payload), 16)
        feats = struct.unpack("<I", self.c.get(mc.REQ_EXTENSION, mc.EXT_NOTIFY_FEATURES).payload)[0]
        self.assertTrue(feats & mc.NOTIFY_FEAT_BULK_SOURCE)
        self.assertTrue(feats & mc.NOTIFY_FEAT_CLIP_EVENT)
        # EXT_IDENTIFY (FWX): 4-byte OUT, 56-byte IN; bad tone output refused.
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_IDENTIFY, payload=bytes([10, 0xFF, 0, 0])).status, mc.ST_OK)
        ident = self.c.get(mc.REQ_EXTENSION, mc.EXT_IDENTIFY).payload
        self.assertEqual(len(ident), 56)
        self.assertIn("blink_left=10s", mc.decode_identify(ident))
        self.assertIn("hw_rev='Board rev A'", mc.decode_identify(ident))
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_IDENTIFY, payload=bytes([5, 99, 0, 0])).status, mc.ST_ERROR)
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_IDENTIFY, payload=bytes(3)).status, mc.ST_ERROR)
        # EXT_RESET_INFO (FWX): IN only, 20 bytes.
        ri = self.c.get(mc.REQ_EXTENSION, mc.EXT_RESET_INFO).payload
        self.assertEqual(len(ri), 20)
        self.assertIn("reason=WATCHDOG", mc.decode_reset_info(ri))
        self.assertIn("prev_uptime=3600s", mc.decode_reset_info(ri))
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_RESET_INFO, payload=bytes(1)).status, mc.ST_ERROR)
        # EXT_FIRMWARE_FIXES (FWFIX): IN only, uint32 LE, bit 0 = bulk dB via powf.
        fx = self.c.get(mc.REQ_EXTENSION, mc.EXT_FIRMWARE_FIXES).payload
        self.assertEqual(fx, b"\x01\x00\x00\x00")
        self.assertEqual(mc.decode_fixes(fx), "firmware fixes: 0x00000001 BULK_DB_POWF")
        self.assertEqual(mc.decode_fixes(bytes(4)), "firmware fixes: 0x00000000")
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_FIRMWARE_FIXES, payload=bytes(4)).status, mc.ST_ERROR)
        # The clip event decodes (FWX).
        clip = bytes([2, 0xE0, 0, 7, 9, 2, 0, 0]) + struct.pack("<I", (1 << 9) | (1 << 12))
        self.assertIn("EXT_CLIP", mc.decode_notify(clip))
        self.assertIn("channel=9 count=2 mask=0x00001200", mc.decode_notify(clip))
        self.assertEqual(self.c.set(mc.REQ_EXTENSION, mc.EXT_NOTIFY_FEATURES, payload=bytes(4)).status, mc.ST_ERROR)
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
        rc, out = self.run_main(mc, ["fixes", "--fake"])   # FWFIX
        self.assertEqual(rc, 0)
        self.assertIn("firmware fixes: 0x00000001 BULK_DB_POWF", out)
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


class LinkBlock(unittest.TestCase):
    """LK7: link block 0x8000-0x800A and events 0xE1 / 0xE2 (extension_channel_spec.md 2.8)."""

    RX_IDS = (1, 2, 3, 4, 5, 6, 0x8000, 0x8001, 0x8002, 0x8003, 0x8004, 0x8005, 0x8006,
              0x8008, 0x8009, 0x800A)
    TX_IDS = (1, 2, 3, 4, 5, 6, 0x8000, 0x8001, 0x8002, 0x8005, 0x8007, 0x8008, 0x8009, 0x800A)

    def client(self, link):
        self.dev = fd.FakeDevice(link=link)
        self.c = mc.Client(fd.FakePort(self.dev, chunk=7), timeout=0.5)
        return self.c

    def ext(self, sub, wlength=0):
        return self.c.get(mc.REQ_EXTENSION, sub, wlength=wlength)

    def put(self, sub, payload):
        return self.c.set(mc.REQ_EXTENSION, sub, payload=payload).status

    def lock(self):
        self.assertEqual(self.put(mc.EXT_LINK_CONFIG, bytes([1, 0xFF, 0, 0xFF, 0xFF, 0xFF, 3, 0])), mc.ST_OK)
        self.dev.link_event(2, 1)
        return self.c.wait_notify(lambda r: r.payload[1] == mc.NOTIFY_EVT_LINK_STATE, 0.5)

    def test_lk7_no_link_refuses_ids(self):
        # Today's firmware (and the fake by default): not listed, every ID refused, bits 2/3 clear.
        self.client(None)
        self.assertEqual(struct.unpack("<6H", self.ext(mc.EXT_LIST).payload), (1, 2, 3, 4, 5, 6))
        for sub in range(0x8000, 0x800B):
            self.assertEqual(self.ext(sub).status, mc.ST_ERROR, hex(sub))
        self.assertEqual(self.put(mc.EXT_LINK_CONFIG, bytes(8)), mc.ST_ERROR)
        self.assertEqual(self.put(mc.EXT_LINK_STATS_RESET, b"\x01"), mc.ST_ERROR)
        feats = struct.unpack("<I", self.ext(mc.EXT_NOTIFY_FEATURES).payload)[0]
        self.assertFalse(feats & (mc.NOTIFY_FEAT_LINK_STATE | mc.NOTIFY_FEAT_APPLY_RESULT))

    def test_lk7_receiver_discovery_and_vectors(self):
        self.client("receiver")
        rep = self.ext(mc.EXT_LIST)
        self.assertEqual(struct.unpack(f"<{len(rep.payload) // 2}H", rep.payload), self.RX_IDS)
        self.assertEqual(self.ext(mc.EXT_LIST, wlength=64).payload, rep.payload)   # 32 B fit in 64
        feats = struct.unpack("<I", self.ext(mc.EXT_NOTIFY_FEATURES).payload)[0]
        self.assertEqual(feats, 0x0F)
        # Byte-exact vectors from the spec (2.8.1, 2.8.3, 2.8.4, 2.8.5).
        self.assertEqual(self.ext(mc.EXT_LINK_INFO).payload.hex(),
                         "010107014008000000ff00002300000070000004000000000000000000000000")
        self.assertEqual(self.ext(mc.EXT_LINK_CONFIG).payload.hex(), "00ff00ffffff0300")
        self.assertEqual(self.ext(mc.EXT_LINK_SLOTS).payload.hex(), "0001020304050607")
        self.assertEqual(self.ext(mc.EXT_LINK_PHASE).payload.hex(), "0100800000000000")
        self.assertIn("role=receiver protocol=v1 classes=L1,L2,L3 max_links=1 max_channels=64 slots=8 "
                      "rx_pin=GP0 tx_pin=none features=0x00000023 PHASE_LOCK APPLY_AT_FRAME LOS_PIN "
                      "d_local=112..1024", mc.decode_link_info(self.ext(mc.EXT_LINK_INFO).payload))
        # Sender-only forms are refused on a receiver; wrong direction / length too.
        self.assertEqual(self.ext(mc.EXT_LINK_TX).status, mc.ST_ERROR)
        self.assertEqual(self.put(mc.EXT_LINK_TX, bytes(16)), mc.ST_ERROR)
        self.assertEqual(self.put(mc.EXT_LINK_CTRL_MSG, bytes(16)), mc.ST_ERROR)
        self.assertEqual(self.put(mc.EXT_LINK_INFO, bytes(32)), mc.ST_ERROR)
        self.assertEqual(self.ext(mc.EXT_LINK_STATS_RESET).status, mc.ST_ERROR)
        self.assertEqual(self.ext(0x800B).status, mc.ST_ERROR)
        self.assertEqual(self.ext(mc.EXT_LINK_STATUS, wlength=8).payload, bytes([0, 0, 0xFF, 0, 0, 0xFF, 0, 0]))

    def test_lk7_receiver_config_slots_phase(self):
        self.client("receiver")
        cfg = bytes([1, 0, 1, 22, 21, 0xFF, 1, 0])          # GP0 in, L1, repeater GP22, LOS GP21
        self.assertEqual(self.put(mc.EXT_LINK_CONFIG, cfg), mc.ST_OK)
        self.assertEqual(self.ext(mc.EXT_LINK_CONFIG).payload, cfg)
        self.assertEqual(self.ext(mc.EXT_LINK_INFO).payload[0], 3)      # repeater while tx_pin is set
        for bad in (bytes([2, 0, 0, 0xFF, 0xFF, 0xFF, 0, 0]),           # enable > 1
                    bytes([1, 0, 4, 0xFF, 0xFF, 0xFF, 0, 0]),           # class > 3
                    bytes([1, 23, 0, 0xFF, 0xFF, 0xFF, 0, 0]),          # internal pin
                    bytes([1, 29, 0, 0xFF, 0xFF, 0xFF, 0, 0]),          # VSYS sense, not on the header
                    bytes([1, 0, 0, 0, 0xFF, 0xFF, 0, 0]),              # rx == tx
                    bytes([1, 0, 0, 0xFF, 0xFF, 0xFF, 4, 0]),           # undefined flag
                    bytes([1, 0, 0, 0xFF, 0xFF, 0xFF, 0, 1])):          # reserved
            self.assertEqual(self.put(mc.EXT_LINK_CONFIG, bad), mc.ST_ERROR, bad.hex())
        self.assertEqual(self.put(mc.EXT_LINK_CONFIG, cfg[:7]), mc.ST_ERROR)
        self.assertEqual(self.ext(mc.EXT_LINK_CONFIG).payload, cfg)     # refusals change nothing
        slots = bytes([4, 5, 0xFF, 0xFF, 11, 0xFF, 63, 0xFF])
        self.assertEqual(self.put(mc.EXT_LINK_SLOTS, slots), mc.ST_OK)
        self.assertEqual(self.ext(mc.EXT_LINK_SLOTS).payload, slots)
        self.assertEqual(self.put(mc.EXT_LINK_SLOTS, bytes([64] + [0xFF] * 7)), mc.ST_ERROR)
        phase = struct.pack("<BBHhBB", 1, 0, 176, -1, 0, 0)
        self.assertEqual(self.put(mc.EXT_LINK_PHASE, phase), mc.ST_OK)
        self.assertIn("mode=phase lock d_local=176 hops_offset=-1", mc.decode_link_phase(self.ext(mc.EXT_LINK_PHASE).payload))
        for bad in (struct.pack("<BBHhBB", 2, 0, 128, 0, 0, 0), struct.pack("<BBHhBB", 1, 1, 128, 0, 0, 0),
                    struct.pack("<BBHhBB", 1, 0, 111, 0, 0, 0), struct.pack("<BBHhBB", 1, 0, 1025, 0, 0, 0),
                    struct.pack("<BBHhBB", 1, 0, 128, 65, 0, 0), struct.pack("<BBHhBB", 1, 0, 128, 0, 1, 0)):
            self.assertEqual(self.put(mc.EXT_LINK_PHASE, bad), mc.ST_ERROR, bad.hex())
        # state (byte 7) is ignored on OUT, so a read-modify-write works.
        self.assertEqual(self.put(mc.EXT_LINK_PHASE, struct.pack("<BBHhBB", 0, 0, 128, 0, 0, 2)), mc.ST_OK)

    def test_lk7_receiver_lock_loss_events(self):
        self.client("receiver")
        n = self.lock()
        self.assertIsNotNone(n)
        self.assertEqual(n.payload.hex(), "02e100000201010040420f00")
        self.assertEqual(mc.decode_notify(n.payload),
                         "v2 seq=  0 LINK_STATE state=locked class=L1 reason=LOCKED F_low32=1000000")
        st = self.ext(mc.EXT_LINK_STATUS).payload
        self.assertEqual(len(st), mc.LINK_STATUS_LEN)
        self.assertIn("state=locked class=L1 rate=48k C=8 hops=0 los=n/a sender=0x5A sf_flags=0x08 F=1000000 "
                      "locks=1 losses=0", mc.decode_link_status(st))
        fc = self.ext(mc.EXT_FRAME_COUNTER).payload
        self.assertEqual(struct.unpack("<QII", fc)[0], 1_000_000)
        self.assertEqual(mc.decode_link_phase(self.ext(mc.EXT_LINK_PHASE).payload),
                         "mode=phase lock d_local=128 hops_offset=0 state=locked")
        # A new delay while locked re-anchors (0xE1 reason REANCHOR, state stays locked).
        self.assertEqual(self.put(mc.EXT_LINK_PHASE, struct.pack("<BBHhBB", 1, 0, 176, 0, 0, 0)), mc.ST_OK)
        n = self.c.wait_notify(lambda r: r.payload[1] == mc.NOTIFY_EVT_LINK_STATE, 0.5)
        self.assertIn("state=locked class=L1 reason=REANCHOR", mc.decode_notify(n.payload))
        # Long loss -> holdover; counters; reset.
        self.dev.link_event(3, 2)
        n = self.c.wait_notify(lambda r: r.payload[1] == mc.NOTIFY_EVT_LINK_STATE, 0.5)
        self.assertIn("state=holdover class=L1 reason=LOS", mc.decode_notify(n.payload))
        self.assertEqual(struct.unpack_from("<II", self.ext(mc.EXT_LINK_STATUS).payload, 16), (1, 1))
        self.assertEqual(self.put(mc.EXT_LINK_STATS_RESET, b"\x02"), mc.ST_ERROR)
        self.assertEqual(self.put(mc.EXT_LINK_STATS_RESET, b"\x01"), mc.ST_OK)
        self.assertEqual(struct.unpack_from("<6I", self.ext(mc.EXT_LINK_STATUS).payload, 16), (0,) * 6)
        # Disabling the link turns it off with reason DISABLED.
        self.assertEqual(self.put(mc.EXT_LINK_CONFIG, bytes([0, 0xFF, 0, 0xFF, 0xFF, 0xFF, 3, 0])), mc.ST_OK)
        n = self.c.wait_notify(lambda r: r.payload[1] == mc.NOTIFY_EVT_LINK_STATE, 0.5)
        self.assertIn("state=off class=L0 reason=DISABLED", mc.decode_notify(n.payload))
        self.assertEqual(struct.unpack("<QII", self.ext(mc.EXT_FRAME_COUNTER).payload)[0], 0)
        diag = self.ext(mc.EXT_LINK_DIAG).payload
        self.assertEqual((len(diag), diag[0]), (mc.LINK_DIAG_LEN, 1))
        self.assertEqual(self.ext(mc.EXT_LINK_CTRL_MSG).payload, bytes(24))   # nothing received

    def test_lk7_apply_at_frame(self):
        self.client("receiver")
        req = lambda frame, op, arg=0, value=0: struct.pack("<QBBh4x", frame, op, arg, value)
        self.assertEqual(self.put(mc.EXT_APPLY_AT_FRAME, req(1_002_400, 1)), mc.ST_ERROR)   # not locked
        self.lock()
        self.assertEqual(req(1_002_400, 4, 0, -2000).hex(), "a04b0f0000000000040030f800000000")
        self.assertEqual(self.put(mc.EXT_APPLY_AT_FRAME, req(1_002_400, 4, 0, -2000)), mc.ST_OK)
        self.assertEqual(self.ext(mc.EXT_APPLY_AT_FRAME).payload.hex(), "01000000" + "00" * 12)
        self.dev.link.advance(2400, self.dev)
        n = self.c.wait_notify(lambda r: r.payload[1] == mc.NOTIFY_EVT_APPLY_RESULT, 0.5)
        self.assertEqual(n.payload.hex(), "02e2000104000000a04b0f00")
        self.assertIn("APPLY_RESULT op=MASTER_VOLUME result=applied frame_low32=1002400", mc.decode_notify(n.payload))
        self.assertEqual(self.ext(mc.EXT_APPLY_AT_FRAME).payload.hex(), "00040000a04b0f000000000000000000")
        # A frame already passed is applied at once and reported late.
        self.assertEqual(self.put(mc.EXT_APPLY_AT_FRAME, req(1_000_000, 7, 2)), mc.ST_OK)
        n = self.c.wait_notify(lambda r: r.payload[1] == mc.NOTIFY_EVT_APPLY_RESULT, 0.5)
        self.assertIn("op=CLICK result=late frame_low32=1000000", mc.decode_notify(n.payload))
        # Queue of 4; a fifth is refused; CANCEL_ALL cancels each with an event.
        for i in range(4):
            self.assertEqual(self.put(mc.EXT_APPLY_AT_FRAME, req(1_010_000 + i, 1 + i % 2)), mc.ST_OK)
        self.assertEqual(self.put(mc.EXT_APPLY_AT_FRAME, req(1_020_000, 8)), mc.ST_ERROR)
        self.assertEqual(self.ext(mc.EXT_APPLY_AT_FRAME).payload[0], 4)
        self.assertEqual(self.put(mc.EXT_APPLY_AT_FRAME, req(0, 0)), mc.ST_OK)
        got = []
        while (n := self.c.wait_notify(lambda r: r.payload[1] == mc.NOTIFY_EVT_APPLY_RESULT, 0.2)):
            got.append(n.payload[5])
        self.assertEqual(got, [2, 2, 2, 2])
        self.assertEqual(self.ext(mc.EXT_APPLY_AT_FRAME).payload[:3], bytes([0, 2, 2]))
        # Invalid arguments and reserved bytes are refused.
        for bad in (req(1_100_000, 9), req(1_100_000, 1, 1), req(1_100_000, 3, 10), req(1_100_000, 4, 0, 1),
                    req(1_100_000, 5, 9, 10), req(1_100_000, 5, 0, 2048), req(1_100_000, 6, 3),
                    req(1_100_000, 1)[:12] + b"\x00\x00\x00\x01"):
            self.assertEqual(self.put(mc.EXT_APPLY_AT_FRAME, bad), mc.ST_ERROR, bad.hex())

    def test_lk7_sender(self):
        self.client("sender")
        rep = self.ext(mc.EXT_LIST)
        self.assertEqual(struct.unpack(f"<{len(rep.payload) // 2}H", rep.payload), self.TX_IDS)
        self.assertEqual(self.ext(mc.EXT_LINK_INFO).payload.hex(),
                         "020107084000000014ff00003800000000000000000000000000000000000000")
        for sub in (mc.EXT_LINK_SLOTS, mc.EXT_LINK_PHASE, mc.EXT_APPLY_AT_FRAME):
            self.assertEqual(self.ext(sub).status, mc.ST_ERROR, hex(sub))
        self.assertEqual(self.ext(mc.EXT_LINK_TX).payload.hex(), "00000108010000ffff00000000000000")
        tx = struct.pack("<HBBBBBBHHBBH", 0x0003, 2, 16, 1, 1, 0, 21, 0x00FC, 0xFFFF, 9, 0, 0)
        self.assertEqual(self.put(mc.EXT_LINK_TX, tx), mc.ST_OK)
        got = self.ext(mc.EXT_LINK_TX).payload
        self.assertEqual(got[:10], tx[:10])
        self.assertEqual(got[10:13], bytes(3))          # los mask / usb_alt are IN-only
        for bad in (struct.pack("<HBBBBBBHHBBH", 0x0100, 1, 8, 1, 0, 0, 0xFF, 0, 0, 0, 0, 0),   # link 8 of 8
                    struct.pack("<HBBBBBBHHBBH", 1, 1, 9, 1, 0, 0, 0xFF, 0, 0, 0, 0, 0),        # 9 ch on L1
                    struct.pack("<HBBBBBBHHBBH", 1, 3, 17, 2, 0, 0, 0xFF, 0, 0, 0, 0, 0),       # 17 ch on L3 96k
                    struct.pack("<HBBBBBBHHBBH", 1, 1, 8, 1, 4, 0, 0xFF, 0, 0, 0, 0, 0),        # test pattern 4
                    struct.pack("<HBBBBBBHHBBH", 1, 1, 8, 1, 0, 1, 0xFF, 0, 0, 0, 0, 0)):       # slot map 1
            self.assertEqual(self.put(mc.EXT_LINK_TX, bad), mc.ST_ERROR, bad.hex())
        msg = struct.pack("<QBBhB3x", 5_000_000, 1, 0, 0, 1)
        self.assertEqual(self.put(mc.EXT_LINK_CTRL_MSG, msg), mc.ST_OK)
        snap = self.ext(mc.EXT_LINK_CTRL_MSG).payload
        self.assertEqual(len(snap), mc.LINK_CTRL_MSG_IN_LEN)
        self.assertEqual((snap[0] & 1, struct.unpack_from("<QB", snap, 8)), (1, (5_000_000, 1)))
        self.assertEqual(self.put(mc.EXT_LINK_CTRL_MSG, struct.pack("<QBBhB3x", 0, 9, 0, 0, 0)), mc.ST_ERROR)
        self.assertEqual(self.put(mc.EXT_LINK_CONFIG, bytes([1, 0xFF, 0, 22, 0xFF, 0xFF, 3, 0])), mc.ST_ERROR)
        self.assertEqual(self.put(mc.EXT_LINK_CONFIG, bytes([1, 0xFF, 0, 0xFF, 0xFF, 0xFF, 3, 0])), mc.ST_OK)
        self.assertEqual(self.ext(mc.EXT_LINK_INFO).payload[0], 4)      # fan-out while its input is enabled

    def test_lk7_cli(self):
        rc, out = Scripts.run_main(None, mc, ["link", "--fake-link", "receiver"])
        self.assertEqual(rc, 0, out)
        self.assertIn("info:   role=receiver", out)
        self.assertIn("status: state=locked class=L1 rate=48k C=8", out)
        self.assertIn("slots:  0 1 2 3 4 5 6 7", out)
        self.assertIn("phase:  mode=phase lock d_local=128 hops_offset=0 state=locked", out)
        self.assertNotIn("tx:", out)
        rc, out = Scripts.run_main(None, mc, ["link", "--fake-link", "sender"])
        self.assertEqual(rc, 0, out)
        self.assertIn("role=sender", out)
        self.assertIn("tx:     00 00 01 08", out)
        rc, out = Scripts.run_main(None, mc, ["link", "--fake"])
        self.assertEqual(rc, 1)
        self.assertIn("no link support", out)
        rc, out = Scripts.run_main(None, mc, ["ext-list", "--fake-link", "receiver"])
        self.assertIn("0x8000 0x8001", out)
        self.assertIn("notify features: 0x0000000F BULK_SOURCE CLIP_EVENT LINK_STATE APPLY_RESULT", out)


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
