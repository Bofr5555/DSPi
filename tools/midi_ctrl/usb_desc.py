#!/usr/bin/env python3
"""Snapshot / compare the DSPi's USB descriptors and ALSA view (no usbutils).

Reads the raw descriptors the kernel caches in sysfs
(/sys/bus/usb/devices/<dev>/descriptors), so it needs no device node and
sends nothing to the device.

    usb_desc.py snapshot out/before.json      # also lists ALSA cards / MIDI
    usb_desc.py compare out/before.json out/after.json
"""

from __future__ import annotations

import glob
import json
import os
import re
import struct
import sys

VID, PID = 0x2E8B, 0xFEAA


def find_sysfs(vid=VID, pid=PID):
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(d, "idVendor")) as f:
                v = int(f.read(), 16)
            with open(os.path.join(d, "idProduct")) as f:
                p = int(f.read(), 16)
        except OSError:
            continue
        if (v, p) == (vid, pid):
            return d
    return None


def split_descriptors(raw: bytes):
    out, i = [], 0
    while i + 2 <= len(raw):
        n = raw[i]
        if n < 2:
            break
        out.append(raw[i:i + n])
        i += n
    return out


def parse(raw: bytes) -> dict:
    descs = split_descriptors(raw)
    dev = descs[0]
    res = {"device": dev.hex(), "bcdDevice": f"{struct.unpack_from('<H', dev, 12)[0]:04x}",
           "config": None, "interfaces": {}, "iads": [], "summary": []}
    cur = None
    for d in descs[1:]:
        t = d[1]
        if t == 2:
            res["config"] = d.hex()
            res["summary"].append(f"config wTotalLength={struct.unpack_from('<H', d, 2)[0]} "
                                  f"bNumInterfaces={d[4]}")
            cur = None
        elif t == 11:   # IAD
            res["iads"].append(d.hex())
            res["summary"].append(f"IAD first={d[2]} count={d[3]} class={d[4]:02x}/{d[5]:02x}/{d[6]:02x}"
                                  f" iFunction={d[7]}")
            cur = None
        elif t == 4:
            cur = str(d[2])
            res["interfaces"].setdefault(cur, "")
            res["interfaces"][cur] += d.hex()
            res["summary"].append(f"  itf {d[2]} alt {d[3]} eps={d[4]} class={d[5]:02x}/{d[6]:02x}/{d[7]:02x}"
                                  f" iInterface={d[8]}")
        else:
            if cur is not None:
                res["interfaces"][cur] += d.hex()
            if t == 5:
                res["summary"].append(f"    ep {d[2]:#04x} attr={d[3]:#04x} maxpkt={struct.unpack_from('<H', d, 4)[0]}")
    return res


def alsa_view() -> dict:
    out = {"cards": "", "midi": [], "pcm": []}
    try:
        with open("/proc/asound/cards") as f:
            out["cards"] = f.read()
    except OSError:
        pass
    out["midi"] = sorted(glob.glob("/dev/snd/midiC*D*"))
    for p in sorted(glob.glob("/proc/asound/card*/pcm*p/info")):
        try:
            with open(p) as f:
                txt = f.read()
            m = re.search(r"^name: (.*)$", txt, re.M)
            out["pcm"].append(f"{p.split('/')[3]} {p.split('/')[4]} {m.group(1) if m else '?'}")
        except OSError:
            pass
    return out


def snapshot(path: str) -> int:
    d = find_sysfs()
    if not d:
        print("DSPi (2e8b:feaa) not present", file=sys.stderr)
        return 1
    with open(os.path.join(d, "descriptors"), "rb") as f:
        raw = f.read()
    res = parse(raw)
    res["sysfs"] = d
    res["alsa"] = alsa_view()
    with open(path, "w") as f:
        json.dump(res, f, indent=1)
    print("\n".join(res["summary"]))
    print("ALSA rawmidi:", res["alsa"]["midi"])
    print("ALSA playback:", res["alsa"]["pcm"])
    return 0


def compare(a_path: str, b_path: str) -> int:
    with open(a_path) as f:
        a = json.load(f)
    with open(b_path) as f:
        b = json.load(f)
    ok = True
    for itf in ("0", "1", "2"):
        same = a["interfaces"].get(itf) == b["interfaces"].get(itf)
        print(f"interface {itf}: {'identical' if same else 'DIFFERENT'}")
        ok &= same
    ia = [x for x in a["iads"] if int(x[4:6], 16) <= 2]
    ib = [x for x in b["iads"] if int(x[4:6], 16) <= 2]
    print(f"IAD for interfaces 0-1: {'identical' if ia == ib else 'DIFFERENT'}")
    ok &= ia == ib
    print(f"bcdDevice: {a['bcdDevice']} -> {b['bcdDevice']}")
    extra = sorted(set(b["interfaces"]) - set(a["interfaces"]), key=int)
    print(f"interfaces added: {extra or 'none'}")
    print(f"rawmidi nodes: {a['alsa']['midi']} -> {b['alsa']['midi']}")
    print(f"playback PCMs: {a['alsa']['pcm']} -> {b['alsa']['pcm']}")
    print("RESULT:", "interfaces 0-2 unchanged" if ok else "INTERFACES 0-2 CHANGED")
    return 0 if ok else 1


def main(argv) -> int:
    if len(argv) == 3 and argv[1] == "snapshot":
        return snapshot(argv[2])
    if len(argv) == 4 and argv[1] == "compare":
        return compare(argv[2], argv[3])
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
