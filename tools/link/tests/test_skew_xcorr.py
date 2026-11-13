"""WP LK6: skew_xcorr.py on synthetic recordings (selftest, WAV round trip, pass/fail)."""

import math
import os
import random
import struct
import sys
import tempfile
import unittest
import wave

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import skew_xcorr  # noqa: E402


def write_wav(path, rate, left, right, width=3):
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(width)
        w.setframerate(rate)
        full = (1 << (8 * width - 1)) - 1
        out = bytearray()
        for a, b in zip(left, right):
            for v in (a, b):
                out += int(round(v * full)).to_bytes(width, "little", signed=True)
        w.writeframes(bytes(out))


class SkewXcorr(unittest.TestCase):
    def test_lk6_selftest(self):
        self.assertEqual(skew_xcorr.selftest(), 0)

    def _programme(self, rate, seconds, d, seed=3):
        rnd = random.Random(seed)
        comps = [(rnd.uniform(40.0, rate / 4), rnd.uniform(0, 6.28), rnd.uniform(0.2, 1.0)) for _ in range(40)]
        n = int(rate * seconds)
        f = lambda t: 0.02 * sum(a * math.sin(2 * math.pi * fr * t + p) for fr, p, a in comps)
        return [f(i / rate) for i in range(n)], [f((i - d) / rate) for i in range(n)]

    def test_lk6_wav_noise_pass_and_calibration(self):
        rate = 48000
        with tempfile.TemporaryDirectory(dir=os.environ.get("TMPDIR_TESTS")) as td:
            rec, cal = os.path.join(td, "rec.wav"), os.path.join(td, "cal.wav")
            left, right = self._programme(rate, 3.0, 0.6)
            write_wav(rec, rate, left, right)
            l2, r2 = self._programme(rate, 3.0, 0.25, seed=4)
            write_wav(cal, rate, l2, r2, width=2)
            r, rows = skew_xcorr.measure(rec, "noise", 1.0, 0.25, 8, 0.5)
            self.assertEqual(r, rate)
            self.assertGreaterEqual(len(rows), 2)
            for _t, lag, _p in rows:
                self.assertAlmostEqual(lag, 0.6, delta=0.05)
            # 0.6 sample is under one link sample: pass; with the interface's
            # 0.25 subtracted the mean is 0.35.
            self.assertEqual(skew_xcorr.main([rec, "--every", "1", "--span", "0.25", "--max-lag", "8"]), 0)
            self.assertEqual(skew_xcorr.main([rec, "--every", "1", "--span", "0.25", "--max-lag", "8",
                                              "--calib", cal]), 0)

    def test_lk6_wav_noise_fail_beyond_one_sample(self):
        rate = 48000
        with tempfile.TemporaryDirectory(dir=os.environ.get("TMPDIR_TESTS")) as td:
            rec = os.path.join(td, "rec.wav")
            left, right = self._programme(rate, 2.0, 1.7)
            write_wav(rec, rate, left, right)
            self.assertEqual(skew_xcorr.main([rec, "--every", "1", "--span", "0.25", "--max-lag", "8"]), 1)

    def test_lk6_wav_clicks_minus_20_dbfs(self):
        # Review lk6 F5: CLICK plays at -20 dBFS; a bench TX's vector payload
        # adds DC.  The automatic threshold (half each channel's peak) finds
        # the clicks, the skew is right to 0.02 sample.
        rate, d = 48000, 0.3
        with tempfile.TemporaryDirectory(dir=os.environ.get("TMPDIR_TESTS")) as td:
            rec = os.path.join(td, "clicks.wav")
            n = rate
            left, right = [0.05] * n, [-0.04] * n
            for c in range(1, 6):
                p = c * 8000 + 0.25
                for k in range(int(p) - 32, int(p) + 33):
                    for x, q in ((left, p), (right, p + d)):
                        u = k - q
                        sv = 1.0 if u == 0 else math.sin(math.pi * u * 0.9) / (math.pi * u * 0.9)
                        x[k] += 0.1 * sv * (0.5 + 0.5 * math.cos(math.pi * u / 33.0))
            write_wav(rec, rate, left, right)
            r, rows = skew_xcorr.measure(rec, "click", 60.0, 1.0, 8, 0.0)
            self.assertEqual(len(rows), 5)
            for _t, lag, _p in rows:
                self.assertAlmostEqual(lag, d, delta=0.02)
            self.assertEqual(skew_xcorr.main([rec, "--mode", "click", "--max-lag", "8"]), 0)

    def test_lk6_mono_refused(self):
        with tempfile.TemporaryDirectory(dir=os.environ.get("TMPDIR_TESTS")) as td:
            p = os.path.join(td, "mono.wav")
            with wave.open(p, "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(48000)
                w.writeframes(struct.pack("<4h", 0, 1, 2, 3))
            with self.assertRaises(SystemExit):
                skew_xcorr.read_wav(p)


if __name__ == "__main__":
    unittest.main()
