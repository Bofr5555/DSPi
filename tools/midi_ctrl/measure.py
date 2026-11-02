#!/usr/bin/env python3
"""Measure the DSPi USB MIDI control channel; prints a Markdown table.

  * round-trip time of GET 0x7F (platform), N times: median, p99, max;
  * GET 0xA0 (bulk read) wall time and effective bytes/s;
  * SET 0xA1 (bulk write) wall time and bytes/s, only with --allow-bulk-set
    (writes back the image just read unless --bulk-in is given; live state
    only, nothing is saved to flash);
  * notification latency: SET master volume over MIDI, time to the echoed
    PARAM_CHANGED (src MIDI).  The volume only ever goes DOWN from the
    current value (current, current - step, current, ...) and is restored;
  * with --vendor: a thread reads REQ_GET_STATUS over the vendor interface
    (usbfs, interface 2) at 10 Hz the whole time, to show both transports
    interleave; BUSY / BULK_LOCKED counts are reported.

    ./run.sh measure.py --fake                 # no hardware
    DSPI_MIDI_DEV=/dev/snd/midiC2D0 ./run.sh measure.py --port /dev/snd/midiC2D0
"""

from __future__ import annotations

import argparse
import os
import statistics
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # python -I drops the script dir

import midi_ctrl as mc


def pct(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))]


class VendorPoller(threading.Thread):
    def __init__(self, node: str | None, hz: float = 10.0):
        super().__init__(daemon=True)
        import usb_vendor
        self.dev = usb_vendor.VendorDevice(node)
        self.period = 1.0 / hz
        self.stop = threading.Event()
        self.ok = 0
        self.errors = 0
        self.lat = []

    def run(self):
        while not self.stop.is_set():
            t0 = time.monotonic()
            try:
                data = self.dev.get(0x50, 9, 26)
                self.ok += 1 if data else 0
            except OSError:
                self.errors += 1
            self.lat.append(time.monotonic() - t0)
            self.stop.wait(max(0.0, self.period - (time.monotonic() - t0)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mc.add_common_args(ap)
    ap.add_argument("--rtt", type=int, default=1000, help="GET 0x7F round trips")
    ap.add_argument("--bulk-get", type=int, default=20, help="GET 0xA0 repetitions")
    ap.add_argument("--bulk-set", type=int, default=5, help="SET 0xA1 repetitions")
    ap.add_argument("--allow-bulk-set", action="store_true", help="enable the SET 0xA1 test")
    ap.add_argument("--bulk-in", help="image for SET 0xA1 (default: the image just read)")
    ap.add_argument("--notify", type=int, default=20, help="notification latency samples (0 = skip)")
    ap.add_argument("--vol-step", type=float, default=0.5, help="dB below current for the notify test")
    ap.add_argument("--vendor", action="store_true", help="poll the vendor interface at 10 Hz meanwhile")
    ap.add_argument("--usb-node", help="/dev/bus/usb/BBB/DDD for --vendor (default: find / DSPI_USB_DEV)")
    args = ap.parse_args(argv)

    c = mc.open_client(args)
    rows = []
    poller = None
    if args.vendor:
        poller = VendorPoller(args.usb_node)
        poller.start()

    try:
        # --- RTT ---------------------------------------------------------
        rtts = []
        for _ in range(args.rtt):
            t0 = time.monotonic()
            rep = c.get(mc.REQ_GET_PLATFORM)
            rtts.append(time.monotonic() - t0)
            if rep.status != mc.ST_OK:
                raise SystemExit(f"GET 0x7F failed: {rep.status_name}")
        if rtts:
            rows.append(("GET 0x7F round trip", f"n={len(rtts)}",
                         f"median {statistics.median(rtts) * 1e3:.2f} ms, p99 {pct(rtts, 99) * 1e3:.2f} ms, "
                         f"max {max(rtts) * 1e3:.2f} ms"))

        # --- bulk GET ------------------------------------------------------
        image = None
        times = []
        for _ in range(args.bulk_get):
            t0 = time.monotonic()
            rep = c.get(mc.REQ_GET_ALL_PARAMS, timeout=10.0)
            times.append(time.monotonic() - t0)
            if rep.status != mc.ST_OK:
                raise SystemExit(f"GET 0xA0 failed: {rep.status_name}")
            image = rep.payload
        if times:
            n = len(image)
            wire = mc.sysex_len(4 + n + 2, c.mfr)
            med = statistics.median(times)
            rows.append(("GET 0xA0 bulk read", f"n={len(times)}, {n} B payload, {wire} B SysEx",
                         f"median {med * 1e3:.1f} ms, max {max(times) * 1e3:.1f} ms, "
                         f"{n / med:.0f} payload B/s ({wire / med:.0f} wire B/s)"))

        # --- bulk SET ------------------------------------------------------
        if args.allow_bulk_set and args.bulk_set:
            if args.bulk_in:
                with open(args.bulk_in, "rb") as f:
                    image = f.read()
            if image is None:
                image = c.get(mc.REQ_GET_ALL_PARAMS, timeout=10.0).payload
            times = []
            for _ in range(args.bulk_set):
                t0 = time.monotonic()
                rep = c.set(mc.REQ_SET_ALL_PARAMS, payload=image, timeout=10.0)
                times.append(time.monotonic() - t0)
                if rep.status != mc.ST_OK:
                    raise SystemExit(f"SET 0xA1 failed: {rep.status_name}")
                time.sleep(0.1)          # let the main-loop apply run
            n = len(image)
            wire = mc.sysex_len(8 + n + 2, c.mfr)
            med = statistics.median(times)
            rows.append(("SET 0xA1 bulk write", f"n={len(times)}, {n} B payload, {wire} B SysEx",
                         f"median {med * 1e3:.1f} ms, max {max(times) * 1e3:.1f} ms, "
                         f"{n / med:.0f} payload B/s"))

        # --- notification latency ----------------------------------------
        if args.notify:
            rep = c.get(mc.REQ_GET_MASTER_VOLUME)
            if rep.status != mc.ST_OK or len(rep.payload) < 4:
                raise SystemExit("GET 0xD3 failed")
            current = struct.unpack("<f", rep.payload[:4])[0]
            lower = max(-127.0, current - abs(args.vol_step))
            c.pump(0.2)
            c.notifications.clear()
            lats, missing = [], 0
            try:
                for i in range(args.notify):
                    target = lower if i % 2 == 0 else current
                    val = struct.pack("<f", target)
                    t0 = time.monotonic()
                    rep = c.set(mc.REQ_SET_MASTER_VOLUME, payload=val)
                    if rep.status != mc.ST_OK:
                        raise SystemExit(f"SET 0xD2 failed: {rep.status_name}")

                    def is_echo(r, val=val):
                        p = r.payload
                        return len(p) >= 16 and p[1] == 0x02 and p[8] == 10 and p[12:16] == val
                    n = c.wait_notify(is_echo, 1.0)
                    if n is None:
                        missing += 1
                    else:
                        lats.append(n.t - t0)
            finally:
                c.set(mc.REQ_SET_MASTER_VOLUME, payload=struct.pack("<f", current))
            if lats:
                rows.append(("SET 0xD2 -> PARAM_CHANGED (src MIDI)", f"n={len(lats)}, missing {missing}",
                             f"median {statistics.median(lats) * 1e3:.2f} ms, max {max(lats) * 1e3:.2f} ms "
                             f"(volume {current:.1f} / {lower:.1f} dB, restored)"))
            else:
                rows.append(("SET 0xD2 -> PARAM_CHANGED (src MIDI)", f"missing {missing}", "no echo seen"))
    finally:
        if poller:
            poller.stop.set()
            poller.join(2)

    st = c.stats
    rows.append(("MIDI status counts", "", ", ".join(f"{k}={v}" for k, v in sorted(st.items()))))
    if poller:
        rows.append(("Vendor interface at 10 Hz meanwhile", f"{poller.ok} ok, {poller.errors} errors",
                     f"median {statistics.median(poller.lat) * 1e3:.2f} ms" if poller.lat else ""))

    print("| Test | Samples | Result |")
    print("|---|---|---|")
    for r in rows:
        print(f"| {r[0]} | {r[1]} | {r[2]} |")
    bad = st.get("timeout", 0) + st.get("crc_bad", 0) + st.get("malformed", 0)
    return 1 if bad or (poller and poller.errors) else 0


if __name__ == "__main__":
    sys.exit(main())
