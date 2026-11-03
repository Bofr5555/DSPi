#!/usr/bin/env python3
"""Simulate TinyUSB's SET_CONFIGURATION driver binding on a built DSPi ELF.

TinyUSB 0.18 (usbd.c process_set_config) walks the configuration descriptor
interface by interface (an IAD in front sets the interface count), offers each
interface to every class driver in order (DSPi's app drivers first: the UAC1
driver from usb_audio.c, then the loopback driver; then the built-in MIDI
driver), binds it to the first driver whose open() returns a length between 9
and the remaining bytes, and skips that many bytes.  An interface no driver
accepts makes SET_CONFIGURATION fail (the device STALLs it and stays
unconfigured), which is exactly what a DSPI_USB_MIDI build did when the UAC1
driver claimed the MIDI AudioControl interface.

This script:
  * reads `device_descriptor` and `usb_config_descriptor` out of the ELF
    (stdlib ELF32 reader, no toolchain needed);
  * detects the build variant from symbols (midid_open -> DSPI_USB_MIDI,
    lb_driver_open -> DSPI_LOOPBACK);
  * models each driver's open() after its source, taking the UAC1 driver's
    interface-number scoping and its preprocessor guard from usb_audio.c, so
    the check follows the code rather than restating it;
  * runs the walk and checks: every interface bound exactly once, interfaces
    0/1 and 2 on the UAC1 driver, MIDI on the MIDI driver, no endpoint
    address claimed twice, the vendor interface is still number 2.

Usage: check_usb_config.py [--source firmware/DSPi] <DSPi.elf> [...]
Exit status 1 on any failure.
"""

from __future__ import annotations

import argparse
import os
import re
import struct
import sys

T_DEVICE, T_CONFIG, T_INTERFACE, T_ENDPOINT, T_IAD, T_CS_INTERFACE = 1, 2, 4, 5, 11, 0x24
AUDIO, SUB_CONTROL, SUB_STREAMING, SUB_MIDI = 0x01, 0x01, 0x02, 0x03


# ---------------------------------------------------------------------------
# ELF32 little-endian: symbol bytes
# ---------------------------------------------------------------------------

class Elf:
    def __init__(self, path: str):
        with open(path, "rb") as f:
            self.data = f.read()
        d = self.data
        if d[:4] != b"\x7fELF" or d[4] != 1 or d[5] != 1:
            raise SystemExit(f"{path}: not an ELF32 little-endian file")
        shoff, = struct.unpack_from("<I", d, 0x20)
        shentsize, shnum, shstrndx = struct.unpack_from("<HHH", d, 0x2E)
        self.sections = []
        for i in range(shnum):
            name, typ, flags, addr, off, size, link, info, align, entsize = \
                struct.unpack_from("<IIIIIIIIII", d, shoff + i * shentsize)
            self.sections.append(dict(name=name, type=typ, addr=addr, off=off, size=size,
                                      link=link, entsize=entsize))
        self.symbols = {}
        for s in self.sections:
            if s["type"] != 2:          # SHT_SYMTAB
                continue
            strtab = self.sections[s["link"]]
            for j in range(s["size"] // 16):
                st_name, st_value, st_size, st_info, st_other, st_shndx = \
                    struct.unpack_from("<IIIBBH", d, s["off"] + j * 16)
                end = d.index(b"\0", strtab["off"] + st_name)
                nm = d[strtab["off"] + st_name:end].decode("ascii", "replace")
                if nm and nm not in self.symbols:
                    self.symbols[nm] = (st_value, st_size)

    def read_symbol(self, name: str) -> bytes:
        if name not in self.symbols:
            raise SystemExit(f"symbol {name} not found")
        addr, size = self.symbols[name]
        for s in self.sections:
            if s["type"] == 1 and s["addr"] <= addr < s["addr"] + s["size"]:   # PROGBITS
                o = s["off"] + addr - s["addr"]
                return self.data[o:o + size]
        raise SystemExit(f"symbol {name} at {addr:#x} is not in a loaded section")


# ---------------------------------------------------------------------------
# Source-derived facts
# ---------------------------------------------------------------------------

def eval_guard(cond: str, defines: set[str]) -> bool:
    """Evaluate the simple #if forms the firmware uses."""
    cond = cond.strip()
    m = re.match(r"#\s*ifdef\s+(\w+)", cond)
    if m:
        return m.group(1) in defines
    m = re.match(r"#\s*ifndef\s+(\w+)", cond)
    if m:
        return m.group(1) not in defines
    m = re.match(r"#\s*if\s+(.*)$", cond)
    if not m:
        raise SystemExit(f"cannot evaluate guard: {cond}")
    expr = re.sub(r"defined\s*\(\s*(\w+)\s*\)", lambda x: "True" if x.group(1) in defines else "False",
                  m.group(1))
    expr = expr.replace("||", " or ").replace("&&", " and ").replace("!", " not ")
    if not re.fullmatch(r"[\sA-Za-z()]*", expr) or re.search(r"\b(?!True|False|and|or|not)\w+", expr):
        raise SystemExit(f"cannot evaluate guard: {cond}")
    return bool(eval(expr))  # noqa: S307 - restricted to True/False/and/or/not


def uac1_scoped(source_dir: str, defines: set[str]) -> bool:
    """Does uac1_driver_open refuse audio-control interfaces other than 0?"""
    with open(os.path.join(source_dir, "usb_audio.c")) as f:
        lines = f.read().splitlines()
    # The definition (not the forward declaration): signature line ending in "{".
    start = next(i for i, l in enumerate(lines)
                 if re.match(r"static uint16_t uac1_driver_open\(", l) and l.rstrip().endswith("{"))
    end = next(i for i in range(start + 1, len(lines)) if lines[i].startswith("}"))
    stack = []        # enclosing preprocessor conditions
    for i in range(start, end):
        l = lines[i].strip()
        if re.match(r"#\s*if", l):
            stack.append(l)
        elif re.match(r"#\s*endif", l):
            stack.pop()
        elif re.match(r"#\s*(else|elif)", l):
            raise SystemExit("usb_audio.c: #else/#elif in uac1_driver_open; extend this checker")
        elif "TU_VERIFY(itf_desc->bInterfaceNumber == ITF_NUM_AUDIO_CONTROL)" in l:
            return all(eval_guard(c, defines) for c in stack)
    return False


# ---------------------------------------------------------------------------
# Driver models (TinyUSB 0.18 usbd.c / midi_device.c, DSPi usb_audio.c / loopback.c)
# ---------------------------------------------------------------------------

def dlen(cfg, p):
    return cfg[p]


def dtype(cfg, p):
    return cfg[p + 1]


def uac1_open(cfg, p, max_len, scoped, ctx):
    """usb_audio.c uac1_driver_open."""
    cls, sub, alt, n_ep, num = cfg[p + 5], cfg[p + 6], cfg[p + 3], cfg[p + 4], cfg[p + 2]
    end = p + max_len
    if cls == 0xFF and alt == 0 and n_ep <= 1:
        n = dlen(cfg, p)
        q = p + n
        eps = []
        if n_ep == 1 and dtype(cfg, q) == T_ENDPOINT:
            eps.append(cfg[q + 2])
            n += dlen(cfg, q)
        ctx["eps"] = eps
        return n
    if cls != AUDIO or sub != SUB_CONTROL or alt != 0:
        return 0
    if scoped and num != 0:
        return 0
    q = p + dlen(cfg, p)
    while q < end and dtype(cfg, q) == T_CS_INTERFACE:
        q += dlen(cfg, q)
    eps = []
    while q < end and dtype(cfg, q) == T_INTERFACE:
        if cfg[q + 5] != AUDIO or cfg[q + 6] != SUB_STREAMING:
            break
        q += dlen(cfg, q)
        while q < end and dtype(cfg, q) != T_INTERFACE:
            if dtype(cfg, q) == T_ENDPOINT:
                eps.append(cfg[q + 2])
            q += dlen(cfg, q)
    ctx["eps"] = sorted(set(eps))
    return q - p


def loopback_open(cfg, p, max_len, itf_lb_ac, ctx):
    """loopback.c lb_driver_open (scoped to ITF_NUM_LOOPBACK_AC)."""
    cls, sub, alt, num = cfg[p + 5], cfg[p + 6], cfg[p + 3], cfg[p + 2]
    if cls != AUDIO or sub != SUB_CONTROL or alt != 0 or num != itf_lb_ac:
        return 0
    return uac1_open(cfg, p, max_len, False, ctx)


def midi_open(cfg, p, max_len, ctx):
    """TinyUSB midi_device.c midid_open."""
    if cfg[p + 5] != AUDIO or cfg[p + 6] != SUB_CONTROL or cfg[p + 7] != 0:
        return 0
    drv_len = dlen(cfg, p)
    q = p + drv_len
    while dtype(cfg, q) == T_CS_INTERFACE and drv_len <= max_len:
        drv_len += dlen(cfg, q)
        q += dlen(cfg, q)
    if dtype(cfg, q) != T_INTERFACE:
        return 0
    if cfg[q + 5] != AUDIO or cfg[q + 6] != SUB_MIDI or cfg[q + 7] != 0:
        return 0
    n_ep = cfg[q + 4]
    drv_len += dlen(cfg, q)
    q += dlen(cfg, q)
    found, eps = 0, []
    while found < n_ep and drv_len <= max_len:
        if dtype(cfg, q) == T_ENDPOINT:
            eps.append(cfg[q + 2])
            drv_len += dlen(cfg, q)
            q += dlen(cfg, q)
            found += 1
        drv_len += dlen(cfg, q)
        q += dlen(cfg, q)
    ctx["eps"] = eps
    return drv_len


# ---------------------------------------------------------------------------
# process_set_config
# ---------------------------------------------------------------------------

def check(elf_path: str, source_dir: str) -> bool:
    elf = Elf(elf_path)
    dev = elf.read_symbol("device_descriptor")
    cfg = elf.read_symbol("usb_config_descriptor")
    midi = "midid_open" in elf.symbols
    loopback = "lb_driver_open" in elf.symbols
    defines = {d for d, on in (("DSPI_USB_MIDI", midi), ("DSPI_LOOPBACK", loopback)) if on}
    scoped = uac1_scoped(source_dir, defines)
    total = struct.unpack_from("<H", cfg, 2)[0]
    n_itf = cfg[4]
    bcd = struct.unpack_from("<H", dev, 12)[0]
    print(f"{elf_path}: bcdDevice {bcd:04x}, wTotalLength {total}, bNumInterfaces {n_itf}, "
          f"variant {sorted(defines) or ['default']}, UAC1 driver scoped to itf 0: {scoped}")
    ok = True

    def fail(msg):
        nonlocal ok
        ok = False
        print(f"  FAIL {msg}")

    if total != len(cfg):
        fail(f"wTotalLength {total} != array size {len(cfg)}")
    drivers = [("uac1", lambda p, m, c: uac1_open(cfg, p, m, scoped, c))]
    if loopback:
        drivers.append(("loopback", lambda p, m, c: loopback_open(cfg, p, m, 3, c)))
    if midi:
        drivers.append(("midi", lambda p, m, c: midi_open(cfg, p, m, c)))

    itf2drv, ep2drv = {}, {}
    p, end = cfg[0], min(total, len(cfg))
    while p < end:
        assoc = 1
        if dtype(cfg, p) == T_IAD:
            assoc = cfg[p + 3]
            p += dlen(cfg, p)
        if dtype(cfg, p) != T_INTERFACE:
            fail(f"offset {p}: expected an interface descriptor, found type {dtype(cfg, p):#x}")
            break
        num = cfg[p + 2]
        remaining = end - p
        for name, op in drivers:
            ctx = {}
            n = op(p, remaining, ctx)
            if 9 <= n <= remaining:
                if assoc == 1 and name == "midi":
                    assoc = 2
                for i in range(num, num + assoc):
                    if i in itf2drv:
                        fail(f"interface {i} bound twice ({itf2drv[i]}, {name})")
                    itf2drv[i] = name
                for ep in ctx.get("eps", []):
                    if ep in ep2drv and ep2drv[ep] != name:
                        fail(f"endpoint {ep:#04x} claimed by {ep2drv[ep]} and {name}")
                    ep2drv[ep] = name
                print(f"  interface {num}" + (f"..{num + assoc - 1}" if assoc > 1 else "") +
                      f" -> {name} ({n} bytes, endpoints {[hex(e) for e in ctx.get('eps', [])]})")
                p += n
                break
        else:
            fail(f"interface {num} (class {cfg[p + 5]:#04x}/{cfg[p + 6]:#04x}) at offset {p}: "
                 f"no driver accepts it -> SET_CONFIGURATION stalls")
            break

    if ok:
        for i in range(n_itf):
            if i not in itf2drv:
                fail(f"interface {i} not bound")
        want = {0: "uac1", 1: "uac1", 2: "uac1"}
        if midi:
            mi = 5 if loopback else 3
            want.update({mi: "midi", mi + 1: "midi"})
        if loopback:
            want.update({3: "loopback", 4: "loopback"})
        for i, d in want.items():
            if itf2drv.get(i) != d:
                fail(f"interface {i} bound to {itf2drv.get(i)}, expected {d}")
    print(f"  {'PASS' if ok else 'FAIL'}")
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--source", default=os.path.join(here, "..", "firmware", "DSPi"),
                    help="firmware/DSPi of the tree the ELF was built from")
    ap.add_argument("elf", nargs="+")
    args = ap.parse_args(argv)
    results = [check(e, args.source) for e in args.elf]
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
