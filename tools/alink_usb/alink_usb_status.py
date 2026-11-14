#!/usr/bin/env python3
"""alink_usb_status.py - decode the alink USB-direct replies (WP USB1).

Decodes EXT_ALINK_USB_STATUS (0x8011, 64 bytes), EXT_ALINK_USB (0x8010, 8 bytes)
and EXT_FRAME_COUNTER (0x8005, 20 bytes) as saved by
`midi_ctrl.py get 0x00 --wvalue 0x80NN --out FILE` (extension_channel_spec.md
2.9).  The layout is picked by the file length.  With two status files it also
prints the start-offset correction of 2.9.3 (U6).  Standard library only.

  python3 tools/alink_usb/alink_usb_status.py st_a.bin
  python3 tools/alink_usb/alink_usb_status.py st_a.bin st_b.bin
  python3 tools/alink_usb/alink_usb_status.py --hex 0201c00000000005
  python3 tools/alink_usb/alink_usb_status.py --diag diag.bin    (EXT_ALINK_USB_DIAG, 0x8012)
"""
import struct
import sys

STATES = ["off", "no SOF", "pull-in", "rate-locked", "anchoring", "phase-locked", "rejected (host range)"]
PHASE = ["free", "anchoring", "locked", "re-anchor pending"]
RATES = {0: "44.1 kHz", 1: "48 kHz", 2: "96 kHz"}


def name(table, i):
    return table[i] if i < len(table) else f"? ({i})"


def status(b):
    (st, ph, rc, fl, hppm, vppm, e, s, bus, dmin, dmax, fill, first, gaps, age, shift, nsamp, anch, rean, miss,
     late) = struct.unpack("<BBBBfffQHHHHHHIiIIIII", b[:64])
    print(f"state {name(STATES, st)}, phase {name(PHASE, ph)}, rate {RATES.get(rc, 'other')}")
    print(f"flags: hw-timestamp {fl & 1}, stream {fl >> 1 & 1}, nominal-feedback {fl >> 2 & 1}, time-base {fl >> 3 & 1}, "
          f"holdover {fl >> 4 & 1}, seeded {fl >> 5 & 1}")
    print(f"host {hppm:+.3f} ppm, vcxo {vppm:+.3f} ppm, phase error {e:+.4f} samples")
    print(f"S now {s}, bus frame {bus}, d {dmin}..{dmax}, ring fill {fill}")
    if first == 0xFFFF:
        print("stream: none placed")
    else:
        print(f"stream: first bus frame {first}, age {age} frames, shift {shift:+d} samples, {nsamp} samples placed")
    print(f"anchors {anch}, re-anchors {rean}, gaps {gaps}, missed {miss} samples, late {late} samples")
    return dict(first=first, age=age, shift=shift, bus=bus, rate=rc)


def config(b):
    mode, flags, d, _r, sf, st = struct.unpack("<BBHHBB", b[:8])
    print(f"mode {mode} ({['off', 'rate lock', 'phase lock'][mode] if mode < 3 else '?'}), auto re-anchor {flags & 1}, "
          f"D {d}, status flags 0x{sf:02x}, state {name(STATES, st)}")


ENDS = ["SOF lost", "gap > 1 s", "discontinuity", "host rejected", "rate/config", "re-anchor: phase",
        "re-anchor: output reset", "re-anchor: other", "stop"]


def diag(b):
    ver, last, rides, holds = struct.unpack("<BBHH", b[:6])
    ends = struct.unpack("<9H", b[6:24])
    sus, res, rst, unp = struct.unpack("<4H", b[24:32])
    lf, lus, ldev, lride, mpass, mquiet, mlate, hold = struct.unpack("<IIiIIIII", b[32:64])
    print(f"diag v{ver}: last end {ENDS[last] if last < len(ENDS) else 'none'}; holdovers {holds}, ridden {rides}")
    print("timeline ends: " + ", ".join(f"{n} {c}" for n, c in zip(ENDS, ends) if c) if any(ends) else "timeline ends: none")
    print(f"USB events: suspend {sus}, resume {res}, bus reset {rst}, unplug {unp}")
    print(f"last absence: {lf} frames, {lus} us, clock - frames {ldev:+d} us; longest ridden {lride} us; running holdover {hold} us")
    print(f"max main-loop pass {mpass} us, max time without a SOF interrupt {mquiet} us, max SOF interrupt latency {mlate} us")


def frame_counter(b):
    f, t, o, tb, _r, bus = struct.unpack("<QIIBBH", b[:20])
    base = {0: "alink F", 1: "USB S", 0xFF: "none"}.get(tb, f"? {tb}")
    print(f"time base {base}: frame {f}, time_us {t}, output samples {o}, frame - output {f - o}, bus frame {bus}")


def offset(a, b):
    """Samples by which unit A's content plays earlier than unit B's (2.9.3)."""
    spf = 96 if a["rate"] == 2 else 48
    fa = (a["bus"] - a["age"]) % 2048
    fb = (b["bus"] - b["age"]) % 2048
    d = (fb - fa) % 2048
    if d >= 1024:
        d -= 2048
    return d * spf + (b["shift"] - a["shift"])


def main(argv):
    if len(argv) >= 2 and argv[0] == "--diag":
        for p in argv[1:]:
            diag(open(p, "rb").read())
        return 0
    if len(argv) >= 2 and argv[0] == "--hex":
        blobs = [bytes.fromhex(x) for x in argv[1:]]
    else:
        blobs = [open(p, "rb").read() for p in argv]
    if not blobs:
        print(__doc__)
        return 2
    st = []
    for b in blobs:
        if len(b) >= 64:
            st.append(status(b))
        elif len(b) >= 20:
            frame_counter(b)
        elif len(b) >= 8:
            config(b)
        else:
            print(f"{len(b)} bytes: unknown layout")
        print()
    if len(st) == 2 and st[0]["first"] != 0xFFFF and st[1]["first"] != 0xFFFF:
        n = offset(st[0], st[1])
        if n >= 0:
            print(f"start offset: A plays the content {n} samples before B: delay A by {n} samples")
        else:
            print(f"start offset: B plays the content {-n} samples before A: delay B by {-n} samples")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
