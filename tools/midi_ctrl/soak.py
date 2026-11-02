#!/usr/bin/env python3
"""Soak the DSPi USB MIDI control channel (default 1 hour).

  * meter read (GET 0x50 wValue 9) every 100 ms;
  * bulk read (GET 0xA0) every 10 s;
  * a random master-volume SET every few seconds, strictly inside
    [--vol-min, --vol-max] (both required; the coordinator chooses them),
    original volume restored at the end;
  * every notification logged; sequence gaps counted (a gap is only
    acceptable right after a flash write, which this script never issues);
  * underrun / overrun counters (REQ_GET_STATUS) read before and after, over
    the vendor interface with --vendor, otherwise over MIDI.

Audio is NOT played by this script.  Run the coordinator's guarded player
(<= 80 % volume, no-fallback, watchdog) alongside it for the audio part.

Pass criteria (exit 0): zero timeouts, zero CRC / frame / malformed replies,
zero notification seq gaps, no growth in the underrun counters.

    ./run.sh soak.py --fake --minutes 1 --vol-min -40 --vol-max -30
"""

from __future__ import annotations

import argparse
import os
import random
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # python -I drops the script dir

import midi_ctrl as mc
import usb_vendor


class MidiGetter:
    """usb_vendor.underrun_counters() adapter over MIDI."""

    def __init__(self, c: mc.Client):
        self.c = c

    def get(self, breq: int, wvalue: int, length: int) -> bytes:
        rep = self.c.get(breq, wvalue, 0, length)
        return rep.payload if rep.status == mc.ST_OK else b""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mc.add_common_args(ap)
    ap.add_argument("--minutes", type=float, default=60.0)
    ap.add_argument("--vol-min", type=float, required=True, help="lowest master volume to set, dB")
    ap.add_argument("--vol-max", type=float, required=True, help="highest master volume to set, dB")
    ap.add_argument("--meter-ms", type=float, default=100.0)
    ap.add_argument("--bulk-s", type=float, default=10.0)
    ap.add_argument("--vol-s", type=float, default=3.0, help="mean interval between volume SETs")
    ap.add_argument("--vendor", action="store_true", help="read counters over the vendor interface")
    ap.add_argument("--usb-node")
    ap.add_argument("--log", help="append notification / event log here")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args(argv)
    if not (-127.0 <= args.vol_min <= args.vol_max <= 0.0):
        ap.error("need -127 <= vol-min <= vol-max <= 0")

    rng = random.Random(args.seed)
    c = mc.open_client(args)
    log = open(args.log, "a") if args.log else None

    def note(s: str) -> None:
        line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {s}"
        if log:
            log.write(line + "\n")
            log.flush()

    last_seq = [None]
    gaps = [0]

    def on_notify(rep: mc.Reply) -> None:
        p = rep.payload
        if len(p) >= 4 and p[0] == 2:
            seq = p[3]
            if last_seq[0] is not None and seq != (last_seq[0] + 1) & 0xFF:
                gaps[0] += 1
                note(f"SEQ GAP {last_seq[0]} -> {seq}")
            last_seq[0] = seq
        note("notify " + mc.decode_notify(p))

    c.on_notify = on_notify

    counter_dev = usb_vendor.VendorDevice(args.usb_node) if args.vendor else MidiGetter(c)
    before = usb_vendor.underrun_counters(counter_dev)
    rep = c.get(mc.REQ_GET_MASTER_VOLUME)
    if rep.status != mc.ST_OK:
        raise SystemExit("GET 0xD3 failed")
    original = struct.unpack("<f", rep.payload[:4])[0]
    note(f"start: volume {original:.2f} dB, counters {before}")

    t_end = time.monotonic() + args.minutes * 60.0
    next_meter = next_bulk = time.monotonic()
    next_vol = time.monotonic() + rng.expovariate(1.0 / args.vol_s)
    counts = dict(meter=0, bulk=0, vol=0, errors=0)
    bulk_ms = []
    try:
        while time.monotonic() < t_end:
            now = time.monotonic()
            try:
                if now >= next_meter:
                    r = c.get(mc.REQ_GET_STATUS, 9)
                    counts["meter"] += 1
                    if r.status != mc.ST_OK:
                        counts["errors"] += 1
                        note(f"meter status {r.status_name}")
                    next_meter += args.meter_ms / 1000.0
                if now >= next_bulk:
                    t0 = time.monotonic()
                    r = c.get(mc.REQ_GET_ALL_PARAMS, timeout=10.0)
                    bulk_ms.append((time.monotonic() - t0) * 1e3)
                    counts["bulk"] += 1
                    if r.status != mc.ST_OK:
                        counts["errors"] += 1
                        note(f"bulk status {r.status_name}")
                    next_bulk += args.bulk_s
                if now >= next_vol:
                    db = round(rng.uniform(args.vol_min, args.vol_max), 1)
                    r = c.set(mc.REQ_SET_MASTER_VOLUME, payload=struct.pack("<f", db))
                    counts["vol"] += 1
                    if r.status != mc.ST_OK:
                        counts["errors"] += 1
                        note(f"volume status {r.status_name}")
                    next_vol = now + rng.expovariate(1.0 / args.vol_s)
            except mc.Timeout as e:
                counts["errors"] += 1
                note(f"TIMEOUT {e}")
            c.pump(max(0.0, min(next_meter, next_bulk, next_vol) - time.monotonic()))
    finally:
        c.set(mc.REQ_SET_MASTER_VOLUME, payload=struct.pack("<f", original))
        c.pump(0.3)

    after = usb_vendor.underrun_counters(counter_dev)
    grew = {k: (before[k], after[k]) for k in before
            if k != "usb_audio_packets" and before[k] is not None and after[k] is not None
            and after[k] > before[k]}
    note(f"end: counters {after}")
    st = c.stats
    bad = st.get("timeout", 0) + st.get("crc_bad", 0) + st.get("malformed", 0) + \
        st.get("CRC_ERROR", 0) + st.get("FRAME_ERROR", 0)

    print("| Soak item | Result |")
    print("|---|---|")
    print(f"| duration | {args.minutes:.1f} min |")
    print(f"| meter reads / bulk reads / volume sets | {counts['meter']} / {counts['bulk']} / {counts['vol']} |")
    if bulk_ms:
        bulk_ms.sort()
        print(f"| bulk read ms (median / max) | {bulk_ms[len(bulk_ms) // 2]:.1f} / {bulk_ms[-1]:.1f} |")
    print(f"| notifications / seq gaps | {st.get('notify', 0)} / {gaps[0]} |")
    print(f"| non-OK replies, timeouts, CRC/frame/malformed | {counts['errors']}, {st.get('timeout', 0)}, {bad} |")
    print(f"| retries (BUSY/BULK_LOCKED) | {st.get('retry', 0)} |")
    print(f"| underrun counters that grew | {grew or 'none'} |")
    print(f"| volume restored to | {original:.2f} dB |")
    ok = counts["errors"] == 0 and bad == 0 and gaps[0] == 0 and not grew
    print(f"\nSOAK {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
