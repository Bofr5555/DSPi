#!/usr/bin/env python3
"""siggen_sync_bench.py - the generators of several units started at one T (WP FWTB3).

The bench driver of PLAN-AT ATHIL Part A (H1-H4, H7, HS1): it talks to each
DSPi unit over its vendor requests (the USB MIDI control channel, or EP0 on
Linux usbfs), reads every unit's time base (EXT_TIME_BASE 0x8030,
extension_channel_spec.md 2.11.1), stages one finite SiggenConfig per unit
(REQ_SIGGEN_SET_CONFIG 0xA4), writes apply op 9 SIGGEN_START for one T on
the shared counter (EXT_APPLY_AT_FRAME 0x8006, 2.8.7.1), or op 7 CLICK, and
reads back the proof: EXT_SIGGEN_TIMELINE (0x8031, 2.11.2) per unit (started
on T, the discontinuities, the samples) and the disturbance counter and
`t - output_frames` of EXT_TIME_BASE before and after (the contract's C4:
nothing happened in between).  It does not capture audio: the recording and
its analysis are tools/alink/skew_xcorr.py's.

T per unit.  On the line and on S/PDIF every unit of a domain counts the same
F, so unit i gets T + stagger_i.  On USB-direct (time base 1) units count S
from origins that differ by multiples of 2048 * spf (alink_protocol_spec.md
11.1 C1); the driver resolves it from the reads: unit i's t, moved back by
the time between its read and unit 0's, lies within half of 2048 frames of a
whole number of wraps from unit 0's t.  The lead is counted from unit 0's t
at its read (T at the output then); the firmware measures it against the
sample entering the pipeline, which is D minus the ring fill later, and
refuses a lead under arm_lead (EXT_SIGGEN_TIMELINE.arm_lead_ms) at the write.

Safety.  Only finite configs (sweeps and patterns with a repeat count, the
firmware refuses anything else, R4), at -20 dBFS or below, on the outputs
named with --out; nothing plays on the host.  `abort` cancels every pending op
(CANCEL_ALL) and stops the generators (STOP_NOW) on every unit.

  siggen_sync_bench.py status  A=midi:/dev/snd/midiC2D0 B=usb:/dev/bus/usb/003/012
  siggen_sync_bench.py start   A=... B=... --type sweep --out 2 --lead-ms 400 [--stagger B=4807] [--json run.json]
  siggen_sync_bench.py click   A=... B=... --out 2 --lead-ms 300            (H1, HS1: op 7 at one T)
  siggen_sync_bench.py click-impulse A=... --out 2                          (H3: op 7 and op 9 IMPULSE at one T)
  siggen_sync_bench.py leads   A=... --leads 100,150,250 --out 2            (H7)
  siggen_sync_bench.py abort   A=... B=...
  siggen_sync_bench.py start --fake 2 ...                                   (in-process fakes, no device)

Exit status 0 = PASS, 1 = FAIL, 2 = usage or a unit that cannot take part.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import fcntl
import json
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MIDI_CTRL = os.path.join(os.path.dirname(HERE), "midi_ctrl")
sys.path.insert(0, HERE)                    # python -I drops the script dir
sys.path.insert(0, MIDI_CTRL)
import midi_ctrl as mc  # noqa: E402

REQ_EXTENSION = 0x00
REQ_SIGGEN_SET_CONFIG = 0xA4                # OUT, SiggenConfig (36 B)
REQ_SIGGEN_CONTROL = 0xA6                   # IN, wValue = SIGGEN_CTL_*
SIGGEN_CTL_STOP_NOW = 2
EXT_LIST = 0x0000
EXT_APPLY_AT_FRAME = 0x8006
EXT_TIME_BASE = 0x8030
EXT_SIGGEN_TIMELINE = 0x8031
OP_CANCEL_ALL, OP_CLICK, OP_SIGGEN_START, OP_SIGGEN_STOP = 0, 7, 9, 10
APPLY_RESULTS = {0: "applied", 1: "late", 2: "cancelled", 3: "refused"}

TB_LINE, TB_USB, TB_SPDIF = 0, 1, 3
TB_NAMES = {"line": TB_LINE, "usb": TB_USB, "spdif": TB_SPDIF}
TB_LOCKED = 3
TL_IDLE = 0
TL_STARTED = 1
WRAP_FRAMES = 2048                          # USB-direct: S origins differ by multiples of 2048 * spf

SIGGEN_CFG_VERSION = 1
SIGGEN_SWEEP_LOG, SIGGEN_IMPULSE = 4, 7
SIGGEN_TYPES = {"sweep": SIGGEN_SWEEP_LOG, "impulse": SIGGEN_IMPULSE}
LEVEL_MAX_DB = -20.0                        # ATHIL Part A: -20 dBFS, electrical only
RATES = (44100, 48000, 96000)


class Refused(Exception):
    """The unit refused a request (STALL on EP0, ERROR on the MIDI channel)."""


# ---------------------------------------------------------------------------
# Transports: get(breq, wvalue, length) -> bytes, set(breq, wvalue, payload)
# ---------------------------------------------------------------------------

class MidiUnit:
    """A unit behind the USB MIDI control channel (tools/midi_ctrl Client);
    its notifications (0xE2, 0xE4) are kept for the report."""

    def __init__(self, client: mc.Client):
        self.client = client

    def get(self, breq: int, wvalue: int = 0, length: int = 64) -> bytes:
        rep = self.client.get(breq, wvalue, wlength=length)
        if rep.status != mc.ST_OK:
            raise Refused(f"GET 0x{breq:02X}/0x{wvalue:04X}: {rep.status_name}")
        return bytes(rep.payload)

    def set(self, breq: int, wvalue: int, payload: bytes) -> None:
        rep = self.client.set(breq, wvalue, payload=bytes(payload))
        if rep.status != mc.ST_OK:
            raise Refused(f"SET 0x{breq:02X}/0x{wvalue:04X}: {rep.status_name}")

    def events(self) -> list[tuple[int, bytes]]:
        """(type, packet) of the notifications received so far (drained)."""
        self.client.pump(0.0)
        out = []
        while self.client.notifications:
            p = bytes(self.client.notifications.popleft().payload)
            if len(p) >= 2:
                out.append((p[1], p))
        return out

    def close(self) -> None:
        self.client.port.close()


class _Ctrl(ctypes.Structure):
    _fields_ = [("bRequestType", ctypes.c_uint8), ("bRequest", ctypes.c_uint8),
                ("wValue", ctypes.c_uint16), ("wIndex", ctypes.c_uint16),
                ("wLength", ctypes.c_uint16), ("timeout", ctypes.c_uint32),
                ("data", ctypes.c_void_p)]


USBDEVFS_CONTROL = (3 << 30) | (ctypes.sizeof(_Ctrl) << 16) | (ord("U") << 8) | 0
ITF_VENDOR = 2


class UsbUnit:
    """A unit's vendor interface (2) over Linux usbfs: vendor requests on EP0
    (as tools/alink/sender_usb.py, any bRequest).  No notifications here."""

    def __init__(self, node: str):
        self.node = node
        self.fd = os.open(node, os.O_RDWR)

    def _xfer(self, rtype: int, breq: int, wvalue: int, buf, n: int) -> int:
        c = _Ctrl(rtype, breq, wvalue, ITF_VENDOR, n, 1000, ctypes.addressof(buf))
        try:
            return fcntl.ioctl(self.fd, USBDEVFS_CONTROL, c)
        except OSError as e:
            if e.errno == errno.EPIPE:
                raise Refused(f"0x{breq:02X}/0x{wvalue:04X} STALL") from None
            raise

    def get(self, breq: int, wvalue: int = 0, length: int = 64) -> bytes:
        buf = ctypes.create_string_buffer(length)
        n = self._xfer(0xC1, breq, wvalue, buf, length)
        return buf.raw[:n]

    def set(self, breq: int, wvalue: int, payload: bytes) -> None:
        buf = ctypes.create_string_buffer(bytes(payload), len(payload))
        self._xfer(0x41, breq, wvalue, buf, len(payload))

    def events(self) -> list[tuple[int, bytes]]:
        return []

    def close(self) -> None:
        os.close(self.fd)


def open_unit(spec: str):
    """NAME=midi:/dev/snd/midiCxDy or NAME=usb:/dev/bus/usb/BBB/DDD -> (name, unit)."""
    name, sep, where = spec.partition("=")
    kind, sep2, node = where.partition(":")
    if not sep or not sep2 or not name or kind not in ("midi", "usb") or not node:
        raise SystemExit(f"unit '{spec}': use NAME=midi:/dev/snd/midiCxDy or NAME=usb:/dev/bus/usb/BBB/DDD")
    if kind == "midi":
        return name, MidiUnit(mc.Client(mc.RawMidiPort(node)))
    return name, UsbUnit(node)


# ---------------------------------------------------------------------------
# Wire structures
# ---------------------------------------------------------------------------

def pack_apply(frame: int, op: int, arg: int = 0, value: int = 0, tb: int = 0) -> bytes:
    """EXT_APPLY_AT_FRAME OUT (2.8.7): frame, op, arg, value, byte 12 = time base."""
    return struct.pack("<QBBhB3x", frame, op, arg, value, tb)


def unpack_apply_in(b: bytes) -> dict:
    n, op, res, _r, frame = struct.unpack_from("<BBBBQ", b)
    return dict(pending=n, last_op=op, last_result=res, last_frame=frame)


def unpack_time_base(b: bytes) -> dict:
    """EXT_TIME_BASE (2.11.1) -> dict with the entries."""
    if len(b) < 12:
        raise Refused(f"EXT_TIME_BASE: short reply ({len(b)} bytes)")
    ver, count, _res, us, frames = struct.unpack_from("<BBHII", b)
    entries = []
    for i in range(min(count, 2)):
        if len(b) < 12 + 24 * (i + 1):
            break
        tb, role, st, rs, dom, t, dist, d, pe = struct.unpack_from("<BBBBIQHHf", b, 12 + 24 * i)
        entries.append(dict(tb=tb, role=role, state=st, reason=rs, domain=dom, t=t, dist=dist, d=d, phase_error=pe))
    return dict(version=ver, time_us=us, output_frames=frames, entries=entries)


def unpack_timeline(b: bytes) -> dict:
    """EXT_SIGGEN_TIMELINE (2.11.2) -> dict."""
    if len(b) < 36:
        raise Refused(f"EXT_SIGGEN_TIMELINE: short reply ({len(b)} bytes)")
    ver, tb, st, res, disc, lead, start, pend, samples, held, run = struct.unpack_from("<BBBBHHQQIII", b)
    return dict(version=ver, tb=tb, state=st, result=res, result_name=mc.TL_RESULTS.get(res, str(res)),
                disc=disc, arm_lead_ms=lead, start=start, pending=pend, samples=samples, held=held, run_id=run)


def pack_config(kind: str, out_mask: int, level_db: float, repeat: int, duration_ms: int = 300,
                gap_ms: int = 20, f1: float = 20.0, f2: float = 20000.0, period_ms: float = 100.0) -> bytes:
    """A finite SiggenConfig (test_signals_spec.md 4.1): a log sweep or an impulse train."""
    if kind not in SIGGEN_TYPES:
        raise ValueError(f"type {kind}: sweep or impulse")
    if not 1 <= repeat <= 0xFFFF:
        raise ValueError("repeat 1..65535: a timed start needs a finite run (R4)")
    if not out_mask or out_mask >> 16:
        raise ValueError("no output")
    if not level_db <= LEVEL_MAX_DB:
        raise ValueError(f"level {level_db} dBFS above {LEVEL_MAX_DB} dBFS")
    t = SIGGEN_TYPES[kind]
    p1, p2 = (f1, f2) if t == SIGGEN_SWEEP_LOG else (period_ms, 0.0)
    dur = duration_ms if t == SIGGEN_SWEEP_LOG else 0
    return struct.pack("<BBHHBBfIHHffff", SIGGEN_CFG_VERSION, t, out_mask, 0, 0, 0, level_db, dur, repeat,
                       gap_ms, p1, p2, 0.0, 0.0)


def run_ms(kind: str, repeat: int, duration_ms: int, gap_ms: int, period_ms: float) -> float:
    return repeat * ((duration_ms if kind == "sweep" else period_ms) + gap_ms)


# ---------------------------------------------------------------------------
# The bench
# ---------------------------------------------------------------------------

class Bench:
    """Units by name; `sleep` and `now` are injectable (the tests drive fakes with them)."""

    def __init__(self, units: dict, rate: int = 48000, sleep=time.sleep, now=time.monotonic, log=print):
        if rate not in RATES:
            raise ValueError(f"rate {rate}")
        self.units = units
        self.rate = rate
        self.sleep = sleep
        self.now = now
        self.log = log

    # -- reads ---------------------------------------------------------------
    def capable(self, name: str) -> bool:
        b = self.units[name].get(REQ_EXTENSION, EXT_LIST, 128)
        ids = struct.unpack(f"<{len(b) // 2}H", b[:len(b) // 2 * 2])
        return EXT_TIME_BASE in ids and EXT_SIGGEN_TIMELINE in ids

    def time_base(self, name: str) -> dict:
        tb = unpack_time_base(self.units[name].get(REQ_EXTENSION, EXT_TIME_BASE, 64))
        tb["host_t"] = self.now()
        return tb

    def timeline(self, name: str) -> dict:
        return unpack_timeline(self.units[name].get(REQ_EXTENSION, EXT_SIGGEN_TIMELINE, 40))

    def apply_status(self, name: str) -> dict:
        return unpack_apply_in(self.units[name].get(REQ_EXTENSION, EXT_APPLY_AT_FRAME, 16))

    @staticmethod
    def entry(tbr: dict, tb: int | None) -> dict | None:
        for e in tbr["entries"]:
            if tb is None or e["tb"] == tb:
                return e
        return None

    def status(self) -> dict:
        out = {}
        for name in self.units:
            try:
                out[name] = dict(capable=self.capable(name), time_base=self.time_base(name),
                                 timeline=self.timeline(name))
            except Refused as e:
                out[name] = dict(capable=False, error=str(e))
        return out

    # -- the schedule ----------------------------------------------------------
    def plan(self, lead_ms: float, stagger: dict | None = None, tb: int | None = None) -> dict:
        """Read every unit's time base and give each its T.

        Refuses (Refused) a unit without the ops, without a LOCKED entry on
        the time base, or in another domain."""
        stagger = stagger or {}
        names = list(self.units)
        reads = {}
        for n in names:
            if not self.capable(n):
                raise Refused(f"{n}: EXT_TIME_BASE / EXT_SIGGEN_TIMELINE not listed (no ops 9/10 on this build)")
            reads[n] = self.time_base(n)
        e0 = self.entry(reads[names[0]], tb)
        if e0 is None:
            raise Refused(f"{names[0]}: no time-base entry")
        tb = e0["tb"]
        spf = self.rate // 1000
        plan = dict(time_base=tb, rate=self.rate, lead_ms=lead_ms, units={})
        t0 = e0["t"] + round(lead_ms * self.rate / 1000.0)
        for n in names:
            e = self.entry(reads[n], tb)
            if e is None or e["state"] != TB_LOCKED:
                st = mc.TB_STATES.get(e["state"], e["state"]) if e else "no entry"
                raise Refused(f"{n}: time base {mc.TB_IDS.get(tb, tb)} not LOCKED ({st})")
            if e["domain"] != e0["domain"]:
                raise Refused(f"{n}: domain 0x{e['domain']:08X}, {names[0]} 0x{e0['domain']:08X}: not one timeline")
            t = t0
            wraps = 0
            if tb == TB_USB:
                # C1: origins a whole number of 2048-frame wraps apart.
                elapsed = (reads[n]["host_t"] - reads[names[0]]["host_t"]) * self.rate
                diff = e["t"] - elapsed - e0["t"]
                wraps = round(diff / (WRAP_FRAMES * spf))
                t = t0 + wraps * WRAP_FRAMES * spf
            t += int(stagger.get(n, 0))
            plan["units"][n] = dict(T=t, wraps=wraps, domain=e["domain"], d=e["d"], dist_before=e["dist"],
                                    inv_before=(e["t"] - reads[n]["output_frames"]) & 0xFFFFFFFF,
                                    t_read=e["t"])
        return plan

    def write_all(self, plan: dict, op: int, arg: int = 0, value: int = 0) -> dict:
        res = {}
        for n, u in plan["units"].items():
            try:
                self.units[n].set(REQ_EXTENSION, EXT_APPLY_AT_FRAME, pack_apply(u["T"], op, arg, value,
                                                                                plan["time_base"]))
                res[n] = "ok"
            except Refused as e:
                res[n] = f"refused ({e})"
        return res

    def stage(self, cfg: bytes) -> None:
        for n, unit in self.units.items():
            try:
                unit.set(REQ_SIGGEN_SET_CONFIG, 0, cfg)
            except Refused as e:
                raise Refused(f"{n}: SET_CONFIG refused ({e})") from None

    def _wait(self, done, timeout_s: float, step_s: float = 0.05) -> bool:
        end = self.now() + timeout_s
        while True:
            if done():
                return True
            if self.now() >= end:
                return False
            self.sleep(step_s)

    def proof(self, plan: dict, name: str) -> dict:
        """EXT_TIME_BASE after: the disturbance counter and t - output_frames unchanged."""
        tbr = self.time_base(name)
        e = self.entry(tbr, plan["time_base"])
        u = plan["units"][name]
        if e is None:
            return dict(dist_after=None, inv_after=None, undisturbed=False, state_after=None)
        inv = (e["t"] - tbr["output_frames"]) & 0xFFFFFFFF
        d_inv = ((inv - u["inv_before"] + 0x80000000) & 0xFFFFFFFF) - 0x80000000
        return dict(dist_after=e["dist"], inv_after=inv, state_after=e["state"],
                    undisturbed=e["dist"] == u["dist_before"] and abs(d_inv) <= 1 and e["state"] == TB_LOCKED)

    # -- runs ------------------------------------------------------------------
    def start(self, cfg: bytes, run_ms_: float, lead_ms: float = 400.0, stagger: dict | None = None,
              tb: int | None = None) -> dict:
        """Stage cfg on every unit, op 9 at one T, wait for the runs, read back."""
        self.stage(cfg)
        plan = self.plan(lead_ms, stagger, tb)
        before = {n: self.timeline(n)["run_id"] for n in self.units}
        plan["write"] = self.write_all(plan, OP_SIGGEN_START)
        for n in self.units:
            self.units[n].events()           # what arrived before the run is not this run's
        live = [n for n, w in plan["write"].items() if w == "ok"]
        stag = max([abs(int(v)) for v in (stagger or {}).values()] + [0]) * 1000.0 / self.rate
        timeout = (lead_ms + run_ms_ + stag) / 1000.0 + 2.0

        def finished(n):
            # Pending, armed and running are not finished; idle with nothing
            # pending is (the run ended, or the START was refused or cancelled).
            tl = self.timeline(n)
            return tl["state"] == TL_IDLE and not tl["pending"]

        ok = self._wait(lambda: all(finished(n) for n in live), timeout)
        plan["timed_out"] = not ok
        return self._report(plan, before, kind="start")

    def _report(self, plan: dict, before: dict, kind: str) -> dict:
        verdict = not plan.get("timed_out")
        for n, u in plan["units"].items():
            tl = self.timeline(n)
            u["timeline"] = tl
            u.update(self.proof(plan, n))
            u["events"] = [dict(type=f"0x{t:02X}", hex=p.hex()) for t, p in self.units[n].events()]
            if kind == "start":
                u["ok"] = (plan["write"][n] == "ok" and tl["result"] == TL_STARTED and tl["start"] == u["T"]
                           and tl["run_id"] == before[n] + 1 and tl["disc"] == 0 and u["undisturbed"])
            else:
                st = self.apply_status(n)
                u["apply"] = dict(st, last_result_name=APPLY_RESULTS.get(st["last_result"], st["last_result"]))
                u["ok"] = (plan["write"][n] == "ok" and st["last_frame"] == u["T"] and st["last_result"] == 0
                           and u["undisturbed"])
            verdict = verdict and u["ok"]
        plan["verdict"] = "PASS" if verdict else "FAIL"
        return plan

    def click(self, out: int, lead_ms: float = 300.0, stagger: dict | None = None, tb: int | None = None) -> dict:
        """Op 7 CLICK on output `out` at one T on every unit (H1, HS1)."""
        plan = self.plan(lead_ms, stagger, tb)
        plan["write"] = self.write_all(plan, OP_CLICK, out)
        for n in self.units:
            self.units[n].events()
        span = max(int(v) for v in list((stagger or {}).values()) + [0]) * 1000.0 / self.rate
        live = [n for n, w in plan["write"].items() if w == "ok"]
        ok = self._wait(lambda: all(self.apply_status(n)["pending"] == 0 for n in live),
                        (lead_ms + span) / 1000.0 + 2.0)
        plan["timed_out"] = not ok
        return self._report(plan, {n: 0 for n in self.units}, kind="click")

    def click_impulse(self, name: str, cfg: bytes, out: int, lead_ms: float = 400.0, run_ms_: float = 200.0) -> dict:
        """H3 (R10b): op 7 CLICK and op 9 IMPULSE at the same T on one unit."""
        one = Bench({name: self.units[name]}, self.rate, self.sleep, self.now, self.log)
        one.stage(cfg)
        plan = one.plan(lead_ms)
        before = {name: one.timeline(name)["run_id"]}
        w9 = one.write_all(plan, OP_SIGGEN_START)
        w7 = one.write_all(plan, OP_CLICK, out)
        plan["write"] = {name: "ok" if w9[name] == w7[name] == "ok" else f"op 9 {w9[name]}, op 7 {w7[name]}"}
        ok = one._wait(lambda: one.apply_status(name)["pending"] == 0 and one.timeline(name)["state"] == TL_IDLE,
                       (lead_ms + run_ms_) / 1000.0 + 2.0)
        plan["timed_out"] = not ok
        rep = one._report(plan, before, kind="start")
        # Op 7 was queued after op 9 for the same sample, so it ran last.
        st = one.apply_status(name)
        u = rep["units"][name]
        u["click"] = dict(st, last_result_name=APPLY_RESULTS.get(st["last_result"], st["last_result"]))
        u["ok"] = u["ok"] and st["last_op"] == OP_CLICK and st["last_result"] == 0 and st["last_frame"] == u["T"]
        rep["verdict"] = "PASS" if u["ok"] and not plan["timed_out"] else "FAIL"
        return rep

    def leads(self, name: str, cfg: bytes, leads_ms: list, run_ms_: float) -> dict:
        """H7: op 9 with each lead on one unit: refused at the write when too short."""
        one = Bench({name: self.units[name]}, self.rate, self.sleep, self.now, self.log)
        out = dict(unit=name, arm_lead_ms=one.timeline(name)["arm_lead_ms"], runs=[])
        for lead in leads_ms:
            one.stage(cfg)
            plan = one.plan(lead)
            before = {name: one.timeline(name)["run_id"]}
            plan["write"] = one.write_all(plan, OP_SIGGEN_START)
            if plan["write"][name] == "ok":
                ok = one._wait(lambda: one.timeline(name)["state"] == TL_IDLE and not one.timeline(name)["pending"],
                               (lead + run_ms_) / 1000.0 + 2.0)
                plan["timed_out"] = not ok
            rep = one._report(plan, before, kind="start")
            u = rep["units"][name]
            out["runs"].append(dict(lead_ms=lead, write=plan["write"][name], result=u["timeline"]["result_name"],
                                    start_on_T=u["timeline"]["start"] == u["T"], ok=u["ok"]))
        # The expectation: refused at the write when the lead is under
        # arm_lead, accepted and started on T when it is over.  Within 5 ms
        # of arm_lead either is right: the firmware counts the lead from the
        # sample entering the pipeline (D minus the ring fill after the
        # read's t) less the request's round trip.
        def as_expected(r):
            if r["write"] == "ok" and not r["ok"]:
                return False
            if abs(r["lead_ms"] - out["arm_lead_ms"]) < 5.0:
                return True
            return (r["write"] == "ok") == (r["lead_ms"] > out["arm_lead_ms"])
        out["verdict"] = "PASS" if all(as_expected(r) for r in out["runs"]) else "FAIL"
        return out

    def abort(self) -> dict:
        """CANCEL_ALL on every time base the unit has, then STOP_NOW."""
        out = {}
        for n, unit in self.units.items():
            res = []
            try:
                tbs = [e["tb"] for e in self.time_base(n)["entries"]] or [0]
            except Refused:
                tbs = [0]
            for tb in sorted(set(t for t in tbs if t != 2)):
                try:
                    unit.set(REQ_EXTENSION, EXT_APPLY_AT_FRAME, pack_apply(0, OP_CANCEL_ALL, tb=tb))
                    res.append(f"cancel_all tb {tb}: ok")
                except Refused as e:
                    res.append(f"cancel_all tb {tb}: {e}")
            try:
                unit.get(REQ_SIGGEN_CONTROL, SIGGEN_CTL_STOP_NOW, 1)
                res.append("stop_now: ok")
            except Refused as e:
                res.append(f"stop_now: {e}")
            out[n] = res
        return out


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _stagger(items: list[str]) -> dict:
    out = {}
    for s in items or []:
        n, sep, v = s.partition("=")
        if not sep:
            raise SystemExit(f"--stagger {s}: NAME=FRAMES")
        out[n] = int(v, 0)
    return out


def fake_units(n: int) -> dict:
    """n in-process fake receivers on one line (tools/midi_ctrl/fake_device), locked."""
    import fake_device
    units = {}
    for i in range(n):
        dev = fake_device.FakeDevice(alink="receiver")
        dev.alink.config[0] = 1
        dev.alink_event(2, 1)
        dev.out.clear()
        units[chr(ord("A") + i)] = MidiUnit(mc.Client(fake_device.FakePort(dev), timeout=1.0))
    return units


def _print(rep: dict, as_json: str | None) -> None:
    text = json.dumps(rep, indent=2, default=str)
    if as_json:
        with open(as_json, "w") as f:
            f.write(text + "\n")
    print(text)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("status", "start", "click", "click-impulse", "leads", "abort"))
    ap.add_argument("units", nargs="*", help="NAME=midi:/dev/snd/midiCxDy or NAME=usb:/dev/bus/usb/BBB/DDD")
    ap.add_argument("--fake", type=int, default=0, help="N in-process fake receivers instead of devices")
    ap.add_argument("--rate", type=int, default=48000, choices=RATES)
    ap.add_argument("--time-base", choices=tuple(TB_NAMES), help="default: unit 1's first entry")
    ap.add_argument("--type", default="sweep", choices=tuple(SIGGEN_TYPES))
    ap.add_argument("--out", type=int, default=None, help="output index (0-based) the generator / CLICK plays on")
    ap.add_argument("--level", type=float, default=LEVEL_MAX_DB, help=f"dBFS, at most {LEVEL_MAX_DB}")
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--duration-ms", type=int, default=300, help="sweep length")
    ap.add_argument("--gap-ms", type=int, default=20)
    ap.add_argument("--period-ms", type=float, default=100.0, help="impulse period")
    ap.add_argument("--f1", type=float, default=20.0)
    ap.add_argument("--f2", type=float, default=20000.0)
    ap.add_argument("--lead-ms", type=float, default=400.0)
    ap.add_argument("--leads", default="100,150,250", help="leads: ms list (H7)")
    ap.add_argument("--stagger", action="append", help="NAME=FRAMES: that unit's T is T + FRAMES")
    ap.add_argument("--json", help="write the report here too")
    args = ap.parse_args(argv)
    if args.level > LEVEL_MAX_DB:
        ap.error(f"--level above {LEVEL_MAX_DB} dBFS")
    if args.cmd not in ("status", "abort") and args.out is None:
        ap.error("--out is required: the output the generator / CLICK plays on")
    if args.fake:
        units = fake_units(args.fake)
    else:
        if not args.units:
            ap.error("no units")
        units = dict(open_unit(s) for s in args.units)
    bench = Bench(units, args.rate)
    tb = TB_NAMES.get(args.time_base) if args.time_base else None
    try:
        cfg = None
        if args.cmd in ("start", "click-impulse", "leads"):
            kind = "impulse" if args.cmd == "click-impulse" else args.type
            cfg = pack_config(kind, 1 << args.out, args.level, args.repeat, args.duration_ms, args.gap_ms,
                              args.f1, args.f2, args.period_ms)
            length = run_ms(kind, args.repeat, args.duration_ms, args.gap_ms, args.period_ms)
        if args.cmd == "status":
            _print(bench.status(), args.json)
            return 0
        if args.cmd == "abort":
            _print(bench.abort(), args.json)
            return 0
        if args.cmd == "start":
            rep = bench.start(cfg, length, args.lead_ms, _stagger(args.stagger), tb)
        elif args.cmd == "click":
            rep = bench.click(args.out, args.lead_ms, _stagger(args.stagger), tb)
        elif args.cmd == "click-impulse":
            rep = bench.click_impulse(next(iter(units)), cfg, args.out, args.lead_ms, length)
        else:
            rep = bench.leads(next(iter(units)), cfg, [float(x) for x in args.leads.split(",")], length)
        _print(rep, args.json)
        return 0 if rep["verdict"] == "PASS" else 1
    except (Refused, ValueError) as e:
        print(f"siggen_sync_bench: {e}", file=sys.stderr)
        return 2
    finally:
        for u in units.values():
            u.close()


if __name__ == "__main__":
    sys.exit(main())
