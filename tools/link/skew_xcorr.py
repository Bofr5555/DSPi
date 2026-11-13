#!/usr/bin/env python3
"""skew_xcorr.py - skew between two link receivers from one stereo recording (WP LK6).

Bench test (e) of the SYS2 link spec (section 8): two receivers play the same
link stream, their outputs are recorded on the left and right inputs of one
audio interface, and this script measures how far apart they are.

  noise  (default) programme material (pink noise or music from the PC through
         the sender): every --every seconds, the cross-correlation of --span
         seconds of left and right, over lags of +-max-lag samples, with a
         parabolic fit through the peak for the sub-sample part.
  click  receivers fed the apply op CLICK through the sender's superframe
         (both play a full-scale one-sample impulse on the same link frame):
         each impulse is located on both channels to a sub-sample position
         and the differences are reported.

Positive skew means the right channel lags the left one.  A --calib recording
of ONE output split into both inputs measures the interface's own channel skew,
which is subtracted.  Pass (SYS2 3.5 and 8e): |mean skew| below one sample of
the link rate (20.8 us at 48 kHz), target below 10 us, and the per-window
values constant to within half a sample (no drift).

Standard library only (wave + array), so it runs in the dspi-link-tools
container; the recording must be integer PCM (16, 24 or 32 bit), any rate.

  tools/link/run.sh skew_xcorr.py /work/rec.wav
  tools/link/run.sh skew_xcorr.py /work/rec.wav --mode click --calib /work/calib.wav
  tools/link/run.sh skew_xcorr.py --selftest
"""

import argparse
import array
import math
import random
import sys
import wave


def read_wav(path):
    """(rate, [left samples], [right samples]) as floats in -1..1."""
    with wave.open(path, "rb") as w:
        nch, width, rate, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if nch < 2:
        raise SystemExit(f"{path}: {nch} channel(s); need a stereo recording (receiver A left, B right)")
    if width == 2:
        a = array.array("h", raw)
        scale = 32768.0
    elif width == 3:
        a = array.array("i", (int.from_bytes(raw[i:i + 3], "little", signed=True) for i in range(0, len(raw), 3)))
        scale = 8388608.0
    elif width == 4:
        a = array.array("i", raw)
        scale = 2147483648.0
    else:
        raise SystemExit(f"{path}: {8 * width}-bit samples are not supported (use 16/24/32-bit PCM)")
    if sys.byteorder != "little" and width != 3:
        a.byteswap()
    left = [a[i] / scale for i in range(0, len(a), nch)]
    right = [a[i] / scale for i in range(1, len(a), nch)]
    return rate, left, right


def parabolic(ym, y0, yp):
    """Offset (-0.5..0.5) of the vertex of the parabola through three points."""
    d = ym - 2.0 * y0 + yp
    return 0.0 if d == 0 else 0.5 * (ym - yp) / d


def xcorr_lag(a, b, start, n, max_lag):
    """Lag (samples, fractional) of b against a, and the normalised peak."""
    lo, hi = start + max_lag, start + n - max_lag
    if lo >= hi:
        return None
    sa = a[lo:hi]
    ea = math.sqrt(sum(x * x for x in sa)) or 1.0
    best, best_k, cs = 0.0, 0, {}
    for k in range(-max_lag, max_lag + 1):
        sb = b[lo + k:hi + k]
        c = sum(x * y for x, y in zip(sa, sb))
        eb = math.sqrt(sum(y * y for y in sb)) or 1.0
        cs[k] = c / (ea * eb)
        if abs(cs[k]) > abs(best):
            best, best_k = cs[k], k
    if abs(best_k) == max_lag:
        return best_k, best     # at the edge: no fit, widen --max-lag
    sign = 1.0 if best > 0 else -1.0
    off = parabolic(sign * cs[best_k - 1], sign * cs[best_k], sign * cs[best_k + 1])
    return best_k + off, best


def find_clicks(x, rate, thresh, min_gap_s=0.05):
    """Sub-sample positions of impulses (|x| above thresh, one per min_gap)."""
    out, i, gap = [], 0, int(min_gap_s * rate)
    n = len(x)
    while i < n:
        if abs(x[i]) >= thresh:
            j_end = min(n, i + gap // 4)
            j = max(range(i, j_end), key=lambda k: abs(x[k]))
            if 0 < j < n - 1:
                s = 1.0 if x[j] > 0 else -1.0
                out.append(j + parabolic(s * x[j - 1], s * x[j], s * x[j + 1]))
            i = j + gap
        else:
            i += 1
    return out


def frac_delay(a, b, ca, cb, half=32):
    """Delay of the click around cb in b against the one around ca in a: the
    whole part from the peaks, the fraction from the slope of the
    cross-spectrum's phase over the lower 40 % of the band (exact for a
    band-limited pulse, where a parabola through the peak is biased)."""
    k0 = int(round(cb - ca))
    i0 = int(round(ca)) - half
    n = 2 * half
    if i0 < 0 or i0 + k0 < 0 or i0 + n + k0 > len(b) or i0 + n > len(a):
        return None
    wa = a[i0:i0 + n]
    wb = b[i0 + k0:i0 + k0 + n]
    num = den = 0.0
    for k in range(1, int(0.4 * n) + 1):
        xr = xi = yr = yi = 0.0
        for j in range(n):
            c, s_ = math.cos(2 * math.pi * k * j / n), math.sin(2 * math.pi * k * j / n)
            xr += wa[j] * c
            xi -= wa[j] * s_
            yr += wb[j] * c
            yi -= wb[j] * s_
        # B times conj(A): phase -2 pi k d / n for a delay d.
        pr, pi_ = yr * xr + yi * xi, yi * xr - yr * xi
        w = math.hypot(pr, pi_)
        if w == 0:
            continue
        phi = math.atan2(pi_, pr)
        num += w * k * phi
        den += w * k * k
    if den == 0:
        return None
    return k0 - num / den * n / (2 * math.pi)


def measure(path, mode, every, span, max_lag, thresh):
    rate, left, right = read_wav(path)
    rows = []
    if mode == "noise":
        n = int(span * rate)
        step = int(every * rate)
        for start in range(0, len(left) - n, max(step, 1)):
            r = xcorr_lag(left, right, start, n, max_lag)
            if r and abs(r[1]) > 0.3:
                rows.append((start / rate, r[0], r[1]))
    else:
        cl, cr = find_clicks(left, rate, thresh), find_clicks(right, rate, thresh)
        j = 0
        for t in cl:
            while j < len(cr) and cr[j] < t - max_lag:
                j += 1
            if j < len(cr) and abs(cr[j] - t) <= max_lag:
                d = frac_delay(left, right, t, cr[j])
                if d is not None:
                    rows.append((t / rate, d, 1.0))
    return rate, rows


def summary(rows):
    lags = [r[1] for r in rows]
    n = len(lags)
    mean = sum(lags) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in lags) / n) if n > 1 else 0.0
    ts = [r[0] for r in rows]
    tm = sum(ts) / n
    den = sum((t - tm) ** 2 for t in ts)
    slope = sum((t - tm) * (x - mean) for t, x in zip(ts, lags)) / den if den else 0.0
    return mean, sd, min(lags), max(lags), slope * 3600.0


def selftest():
    """Synthetic recordings with known fractional skews; exits non-zero on a miss."""
    rnd = random.Random(6)
    rate = 48000
    ok = True
    # Band-limited programme: 60 sines below rate/4, B delayed by d exactly.
    comps = [(rnd.uniform(40.0, rate / 4), rnd.uniform(0, 2 * math.pi), rnd.uniform(0.2, 1.0)) for _ in range(60)]

    def prog(t):
        return 0.02 * sum(a * math.sin(2 * math.pi * f * t + p) for f, p, a in comps)

    for d in (0.0, 0.37, -0.81, 3.25):
        n = rate // 2
        left = [prog(i / rate) for i in range(n)]
        right = [prog((i - d) / rate) for i in range(n)]
        lag, peak = xcorr_lag(left, right, 0, n, 8)
        good = abs(lag - d) < 0.05
        ok &= good
        print(f"  noise: skew {d:+.2f} sample -> {lag:+.3f} (peak {peak:.3f}) {'OK' if good else 'MISS'}")
    # Clicks: a windowed-sinc impulse (a DAC's reconstruction) at fractional
    # positions, B later by d.
    def click_at(x, pos, amp=0.9):
        for k in range(int(pos) - 32, int(pos) + 33):
            u = k - pos
            s = 1.0 if u == 0 else math.sin(math.pi * u * 0.9) / (math.pi * u * 0.9)
            win = 0.5 + 0.5 * math.cos(math.pi * u / 33.0)
            x[k] += amp * s * win
    for d in (0.0, 0.42, -0.65):
        n = rate
        left, right = [0.0] * n, [0.0] * n
        for c in range(1, 8):
            p = c * 5000 + 0.3 * c
            click_at(left, p)
            click_at(right, p + d)
        cl, cr = find_clicks(left, rate, 0.5), find_clicks(right, rate, 0.5)
        diffs = [frac_delay(left, right, a, b) for a, b in zip(cl, cr)]
        good = len(diffs) == 7 and all(x is not None and abs(x - d) < 0.02 for x in diffs)
        ok &= good
        print(f"  click: skew {d:+.2f} sample -> {', '.join(f'{x:+.3f}' for x in diffs)} {'OK' if good else 'MISS'}")
    print("selftest:", "OK" if ok else "FAIL")
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("wav", nargs="?", help="stereo recording: receiver A on the left, B on the right")
    ap.add_argument("--mode", choices=("noise", "click"), default="noise")
    ap.add_argument("--every", type=float, default=60.0, help="noise: seconds between windows (default 60)")
    ap.add_argument("--span", type=float, default=1.0, help="noise: window length in seconds (default 1)")
    ap.add_argument("--max-lag", type=int, default=48, help="largest skew searched, in samples (default 48)")
    ap.add_argument("--thresh", type=float, default=0.5, help="click: detection level, full scale 1 (default 0.5)")
    ap.add_argument("--calib", help="recording of one output split into both inputs: the interface's own skew")
    ap.add_argument("--link-rate", type=float, default=48000.0, help="the link's sample rate for the pass limits")
    ap.add_argument("--selftest", action="store_true", help="check the estimators on synthetic data and exit")
    a = ap.parse_args(argv)
    if a.selftest:
        return selftest()
    if not a.wav:
        ap.error("a recording is needed (or --selftest)")
    calib = 0.0
    if a.calib:
        crate, crows = measure(a.calib, a.mode, a.every, a.span, a.max_lag, a.thresh)
        if not crows:
            raise SystemExit(f"{a.calib}: nothing to correlate")
        calib = summary(crows)[0]
        print(f"interface channel skew (calibration): {calib:+.3f} samples = {calib / crate * 1e6:+.2f} us")
    rate, rows = measure(a.wav, a.mode, a.every, a.span, a.max_lag, a.thresh)
    if not rows:
        raise SystemExit("nothing to correlate: check the levels, --thresh, --max-lag and the channel order")
    print(f"{'t (s)':>9} {'skew (samples)':>15} {'skew (us)':>10} {'peak':>6}")
    for t, lag, peak in rows:
        lag -= calib
        print(f"{t:9.1f} {lag:+15.3f} {lag / rate * 1e6:+10.2f} {peak:6.3f}")
    mean, sd, lo, hi, drift = summary(rows)
    mean -= calib
    lo -= calib
    hi -= calib
    us = 1e6 / rate
    one = rate / a.link_rate          # one link sample in recording samples
    span_s = rows[-1][0] - rows[0][0]
    drift_txt = f"{drift:+.3f} samples/h" if span_s >= 600.0 else "n/a (under 10 min)"
    print(f"windows {len(rows)}: mean {mean:+.3f} samples ({mean * us:+.2f} us), sd {sd:.3f}, "
          f"range {lo:+.3f}..{hi:+.3f} ({(hi - lo) * us:.2f} us), drift {drift_txt}")
    passed = abs(mean) < one and (hi - lo) < 0.5 * one
    target = abs(mean * us) < 10.0
    print(f"pass (|mean| < 1 link sample = {1e6 / a.link_rate:.1f} us, constant to < half a sample): "
          f"{'PASS' if passed else 'FAIL'}; target < 10 us: {'met' if target else 'not met'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
