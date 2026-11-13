"""WP LK6: bench.py clicks (test (e)): one CLICK apply per period through the sender, waiting out a pending one."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import bench  # noqa: E402
import sender_usb  # noqa: E402


class StubSender:
    """Frame counter plus the one-apply-at-a-time rule of 2.8.9."""

    def __init__(self):
        self.f = 1_000_000
        self.pending = None
        self.sent = []

    def frame(self):
        self.f += 480                   # 10 ms of 48 kHz frames per read
        if self.pending is not None and self.f >= self.pending:
            self.pending = None
        return self.f

    def ctrl(self, frame, op, arg=0, value=0, system_flags=0):
        if self.pending is not None:
            raise sender_usb.SenderRefused("apply pending")
        self.pending = frame
        self.sent.append((frame, op, arg, value))


class Clicks(unittest.TestCase):
    def test_lk6_click_train(self):
        dev, lines = StubSender(), []
        targets = bench.run_clicks(dev, 4, 0.0, 3, 960, lines.append, sleep=lambda s: None)
        self.assertEqual(len(targets), 4)
        self.assertTrue(all(op == bench.OP_CLICK and arg == 3 and v == 0 for _f, op, arg, v in dev.sent))
        self.assertEqual(sorted(targets), targets)
        self.assertEqual(len(set(targets)), 4)
        self.assertIn("CLICK on output 3", lines[0])

    def test_lk6_cli_refuses_bad_arguments(self):
        with self.assertRaises(SystemExit):
            bench.main(["clicks", "--usb", "/dev/null", "--output", "9"], usb_opener=lambda p: StubSender())


if __name__ == "__main__":
    unittest.main()
