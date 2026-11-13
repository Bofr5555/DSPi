"""Tests for bench.py's sender scenario (WP LK4) against simulated hardware.

The RX board is LK3's simulated bench board (test_bench_tool.FakeBoard) with
class auto-detection added; the sender is LK7's control-plane model of a
sender build (tools/midi_ctrl/fake_device.FakeLink("sender"), which follows
extension_channel_spec.md 2.8.8-2.8.9) behind sender_usb's interface, its
frame counter driven by the same virtual clock as the wire.  Checked: a
healthy sender passes every step (BER, frame counter, encoder budget, RATE
scheduling, one apply at a time, the apply's end, a class change); the bench
image without --usb passes with the frame counter N/A; a sender that
switches the layout at once instead of scheduling RATE fails.
"""

import os
import struct
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "midi_ctrl"))
import bench  # noqa: E402
import sender_usb  # noqa: E402
from test_bench_tool import FS, FakeBoard, FakeTime, Wire  # noqa: E402
import fake_device  # noqa: E402

CLS_NAME = {1: "L1", 2: "L2", 3: "L3"}


class AutoRxBoard(FakeBoard):
    """An RX bench board that also understands cls=auto."""

    def handle(self, line):
        super().handle(line)
        if line.startswith("rx"):
            self.auto = "cls=auto" in line

    def _follow(self):
        getattr(self.wire, "tick", lambda: None)()
        if getattr(self, "auto", False) and self.wire.stream and self.cls != self.wire.stream[0]:
            self.cls = self.wire.stream[0]
            self.t_start = self.clock.t         # re-detection takes a moment

    def synced(self):
        self._follow()
        return super().synced()

    def stat(self):
        self._follow()
        return super().stat()


class _Dev:
    def link_event(self, *a):
        pass

    def notify_apply(self, *a):
        pass


class FakeSender:
    """sender_usb.SenderUsb's interface over FakeLink("sender") and the wire."""

    def __init__(self, clock, wire, immediate_layout=False):
        self.clock, self.wire = clock, wire
        self.link = fake_device.FakeLink("sender")
        self.dev = _Dev()
        self.t_last = clock.t
        self.immediate = immediate_layout
        self.layout = None
        wire.tick = self._sync              # the sender runs while the tool polls the RX board
        self._wire_update()

    def _fs(self):
        return FS[sender_usb.RATE_NAME[self.link.tx_layout[1]]]

    def _sync(self):
        fs = self._fs()
        n = int((self.clock.t - self.t_last) * fs)
        if n > 0:
            self.link.advance(n, self.dev)
            self.t_last += n / fs
        self._wire_update()

    def _wire_update(self):
        enabled = struct.unpack_from("<H", self.link.tx)[0] != 0
        layout = self.link.tx_layout if enabled else None
        if layout != self.layout:
            self.layout = layout
            w = self.wire
            if layout:
                cls, rate, ch = layout
                w.stream = (CLS_NAME[cls], sender_usb.RATE_NAME[rate], ch)
                w.t0 = self.clock.t
                w.f0 = self.link.frame
            else:
                w.stream = None

    def get(self, sub, length=64):
        self._sync()
        if sub == 0x0000:
            return struct.pack("<9H", 0x0003, *self.link.ids)[:length]
        if sub == 0x800A:
            d = bytearray(64)
            d[0], d[1], d[2] = 0x81, 1, self.link.tx_layout[0]
            struct.pack_into("<12I", d, 4, 410, 640, 6400, 240, 0, 0, 0, 0, 0, 786432, 0, 0)
            return bytes(d)
        b = self.link.get(sub)
        if b is None:
            raise sender_usb.SenderRefused(hex(sub))
        return b[:length]

    def set(self, sub, payload):
        self._sync()
        if self.immediate and sub == 0x8007:
            # A broken sender: switches the layout at once, no RATE.
            mask, cls, chans, rate = struct.unpack_from("<HBBB", payload)
            self.link.tx[:] = payload
            self.link.tx_layout = (cls, rate, chans)
            self._wire_update()
            return
        if not self.link.set(sub, bytes(payload), self.dev):
            raise sender_usb.SenderRefused(hex(sub))
        self._wire_update()

    # The SenderUsb helpers, on top of get/set.
    ext_list = sender_usb.SenderUsb.ext_list
    info = sender_usb.SenderUsb.info
    link_tx = sender_usb.SenderUsb.link_tx
    set_link_tx = sender_usb.SenderUsb.set_link_tx
    frame = sender_usb.SenderUsb.frame
    ctrl_in = sender_usb.SenderUsb.ctrl_in
    ctrl = sender_usb.SenderUsb.ctrl
    stats_reset = sender_usb.SenderUsb.stats_reset
    diag = sender_usb.SenderUsb.diag


class SenderToolTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeTime()
        self.saved = bench.time
        bench.time = self.clock
        self.lines = []

    def tearDown(self):
        bench.time = self.saved

    def run_tool(self, argv, boards, sender=None):
        def opener(path, name, verbose):
            return bench.Bench(boards[path], name=name)
        return bench.main(argv, opener=opener, out=self.lines.append, usb_opener=lambda path: sender)

    def text(self):
        return "\n".join(self.lines)

    def test_lk4_sender_over_usb_passes_every_step(self):
        w = Wire(self.clock)
        rx = AutoRxBoard(self.clock, w, "R")
        snd = FakeSender(self.clock, w)
        rc = self.run_tool(["sender", "--rx", "R", "--usb", "/dev/bus/usb/001/002", "--minutes", "0.2"],
                           {"R": rx}, snd)
        t = self.text()
        self.assertEqual(rc, 0, t)
        for step in ("sender: link sender firmware", "EXT_LINK_TX written", "L1 48k: BER",
                     "frame counter tracks TX", "encoder within budget", "RATE scheduled",
                     "a second layout change is refused while RATE is pending", "receiver follows to 96k C=3",
                     "CTRL_MSG apply carried, a second refused", "the apply ends at its frame",
                     "class change L1 -> L2 restarts the links"):
            self.assertRegex(t, r"PASS  .*" + step.replace("(", r"\(").replace(")", r"\)"), step)
        self.assertNotIn("FAIL", t)
        # Left as configured: link 0, PRBS, L1 48 kHz 8 channels.
        tx = snd.link_tx()
        self.assertEqual((tx["link_enable"], tx["cls"], tx["rate"], tx["channels"], tx["test_pattern"]),
                         (1, 1, 1, 8, 1))

    def test_lk4_sender_l2_over_usb(self):
        w = Wire(self.clock)
        rc = self.run_tool(["sender", "--rx", "R", "--usb", "U", "--cls", "L2", "--minutes", "0.1"],
                           {"R": AutoRxBoard(self.clock, w, "R")}, FakeSender(self.clock, w))
        self.assertEqual(rc, 0, self.text())
        self.assertIn("receiver follows to 96k C=8", self.text())
        self.assertIn("class change L2 -> L1", self.text())

    def test_lk4_bench_image_without_usb(self):
        w = Wire(self.clock)
        w.stream, w.t0, w.f0 = ("L1", "48k", 8), self.clock.t, 0      # dspi_sender_bench from boot
        rc = self.run_tool(["sender", "--rx", "R", "--minutes", "0.1"], {"R": AutoRxBoard(self.clock, w, "R")})
        t = self.text()
        self.assertEqual(rc, 0, t)
        self.assertIn("N/A   L1 48k: frame counter tracks TX", t)
        self.assertNotIn("RATE scheduled", t)

    def test_lk4_bench_image_needs_its_layout(self):
        w = Wire(self.clock)
        rc = self.run_tool(["sender", "--rx", "R", "--cls", "L2"], {"R": AutoRxBoard(self.clock, w, "R")})
        self.assertEqual(rc, 1)
        self.assertIn("without --usb the sender must be the bench image", self.text())

    def test_lk4_immediate_layout_change_fails(self):
        w = Wire(self.clock)
        rc = self.run_tool(["sender", "--rx", "R", "--usb", "U", "--minutes", "0.1"],
                           {"R": AutoRxBoard(self.clock, w, "R")}, FakeSender(self.clock, w, immediate_layout=True))
        self.assertEqual(rc, 1)
        self.assertRegex(self.text(), r"FAIL  L1 48k: RATE scheduled")

    def test_lk4_sender_diag_prints(self):
        w = Wire(self.clock)
        snd = FakeSender(self.clock, w)
        rc = self.run_tool(["sender-diag", "--usb", "U", "--pattern", "tone"], {}, snd)
        self.assertEqual(rc, 0)
        self.assertEqual(snd.link_tx()["test_pattern"], 3)
        t = self.text()
        self.assertIn("'enc_cycles_avg': 410", t)
        self.assertIn("'cls': 1", t)

    def test_lk4_sender_usb_layouts(self):
        tx = dict(link_enable=0x81, cls=3, channels=18, rate=2, test_pattern=1, slot_map_id=0, sync_pin=21,
                  txdis_mask=0x00FF, los_mask=0x30, usb_alt=7, flags=1)
        b = sender_usb.pack_tx(tx)
        self.assertEqual(b.hex(" "), "81 00 03 12 02 01 00 15 ff 00 30 00 07 01 00 00")
        self.assertEqual(sender_usb.unpack_tx(b), tx)
        self.assertEqual(sender_usb.pack_ctrl(1002400, 4, 0, -2000).hex(" "),
                         "a0 4b 0f 00 00 00 00 00 04 00 30 f8 00 00 00 00")
        c = sender_usb.unpack_ctrl_in(bytes([0x0C, 1, 8, 1, 0, 0x5A, 0, 0, 0x20, 0x04, 0, 0, 0, 0, 0, 0,
                                            2, 0, 0, 0, 0xC0, 0x03, 0, 0]))
        self.assertEqual((c["flags"], c["apply_frame"], c["op"], c["sf_frame_low"]), (0x0C, 1056, 2, 960))


if __name__ == "__main__":
    unittest.main()
