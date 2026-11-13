#!/usr/bin/env python3
"""bench.py - drive link bench Picos over their USB consoles and judge the link.

The link bench image (firmware/link_bench, WP LK3) runs on spare Pico and
Pico 2 boards wired GPIO to GPIO.  This tool talks to each board's console
(USB CDC, /dev/ttyACM* or /dev/serial/by-id/...), runs a scripted test and
prints a PASS/FAIL summary.  Wiring, flashing and the expected numbers are in
Documentation/Bench/link_gpio_bench.md.

  bench.py id PORT [PORT...]                       who is who
  bench.py cmd PORT 'stat'                         one console command
  bench.py loop PORT [--cls L1] [--rate 48k] [--minutes 11]
                                                   one board, TX pin wired to RX pin
  bench.py pair --tx PORT --rx PORT [...]          two boards
  bench.py impair --tx PORT --imp PORT --rx PORT   three boards, IMPAIR in the middle
  bench.py matrix PORT|--tx P --rx P [--minutes 1] every class and rate in turn
  bench.py sender --rx PORT [--usb /dev/bus/usb/BBB/DDD] [...]
                                                   a link sender (firmware/DSPi-Sender, WP LK4)
                                                   to a bench RX board
  bench.py sender-diag --usb /dev/bus/usb/BBB/DDD [--pattern audio|prbs|idle|tone]
                                                   the sender's TX settings, superframe and DIAG

The sender is driven over its USB vendor interface (sender_usb.py, usbfs)
when --usb names its device node; without it the sender must run the bench
image (dspi_sender_bench: link 0 PRBS-7, L1 48 kHz 8 channels from boot).

Standard library only (termios), so it runs in the python:3.12-slim
container (tools/link/bench.sh) with the board's tty passed in, or on any
Linux host.  Never sets 1200 baud: that reboots a Pico into BOOTSEL.
"""

import argparse
import os
import select
import sys
import termios
import time

BAUD = {1: 15_360_000, 2: 30_720_000, 3: 61_440_000}
RATE_HZ = {"44k1": 44_100, "48k": 48_000, "96k": 96_000}
CLASSES = {"L1": 1, "L2": 2, "L3": 3}


# ---------------------------------------------------------------------------
# Console transport
# ---------------------------------------------------------------------------

class BenchError(Exception):
    pass


class SerialPort:
    """A raw tty, non-blocking, line oriented."""

    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0                                    # iflag
        attrs[1] = 0                                    # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0                                    # lflag: raw, no echo
        attrs[4] = attrs[5] = termios.B115200           # never B1200 (BOOTSEL reset)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = b""

    def write(self, data):
        view = memoryview(data)
        while view:
            _, w, _ = select.select([], [self.fd], [], 1.0)
            if not w:
                raise BenchError(f"{self.path}: write timeout")
            n = os.write(self.fd, view)
            view = view[n:]

    def readline(self, timeout):
        end = time.monotonic() + timeout
        while b"\n" not in self.buf:
            left = end - time.monotonic()
            if left <= 0:
                return None
            r, _, _ = select.select([self.fd], [], [], left)
            if r:
                chunk = os.read(self.fd, 4096)
                if not chunk:
                    raise BenchError(f"{self.path}: closed")
                self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode("ascii", "replace").strip()

    def close(self):
        os.close(self.fd)


def parse_kv(line):
    """'TAG k=v k=v' -> (tag, {k: v}) with ints where they parse."""
    parts = line.split()
    out = {}
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            try:
                out[k] = int(v, 0)
            except ValueError:
                out[k] = v
    return (parts[0] if parts else ""), out


class Bench:
    """One board's console: commands, replies, asynchronous EVT lines."""

    def __init__(self, port, name=None, log=None):
        self.port = port
        self.name = name or getattr(port, "path", "board")
        self.events = []
        self.log = log

    def _say(self, text):
        if self.log:
            self.log(f"[{self.name}] {text}")

    def command(self, line, until=("OK", "ERR"), timeout=3.0):
        """Send a command; return every reply line up to the first one whose
        tag is in `until`.  EVT lines are collected in self.events."""
        self._say(f"> {line}")
        self.port.write((line + "\n").encode("ascii"))
        lines = []
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            got = self.port.readline(max(left, 0.0)) if left > 0 else None
            if got is None:
                raise BenchError(f"{self.name}: no reply to '{line}'")
            if not got:
                continue
            self._say(f"< {got}")
            tag = got.split()[0]
            if tag == "EVT":
                self.events.append((time.monotonic(), got))
                continue
            lines.append(got)
            if tag in until:
                return lines

    def ok(self, line, timeout=3.0):
        r = self.command(line, timeout=timeout)
        if r[-1].startswith("ERR"):
            raise BenchError(f"{self.name}: '{line}' -> {r[-1]}")

    def tagged(self, line, tag, timeout=3.0):
        r = self.command(line, until=(tag, "ERR"), timeout=timeout)
        if r[-1].startswith("ERR"):
            raise BenchError(f"{self.name}: '{line}' -> {r[-1]}")
        return parse_kv(r[-1])[1], r

    def id(self):
        return self.tagged("id", "ID")[0]

    def stat(self):
        return self.tagged("stat", "STAT")[0]

    def wait_sync(self, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self.stat()
            if s.get("sync") == 1:
                return time.monotonic() - t0, s
            time.sleep(0.05)
        return None, self.stat()


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------

class Report:
    def __init__(self, out=print):
        self.rows = []
        self.out = out

    def add(self, name, verdict, detail=""):
        self.rows.append((name, verdict, detail))
        self.out(f"  {verdict:4}  {name}: {detail}")

    def check(self, name, cond, detail="", warn=False):
        self.add(name, "PASS" if cond else ("WARN" if warn else "FAIL"), detail)
        return cond

    def info(self, name, detail):
        self.add(name, "INFO", detail)

    def na(self, name, detail):
        self.add(name, "N/A", detail)

    @property
    def passed(self):
        return all(v != "FAIL" for _, v, _ in self.rows)

    def summary(self):
        n = {v: sum(1 for _, x, _ in self.rows if x == v) for v in ("PASS", "FAIL", "WARN", "N/A", "INFO")}
        verdict = "PASS" if self.passed else "FAIL"
        self.out(f"\n== {verdict}: {n['PASS']} passed, {n['FAIL']} failed, {n['WARN']} warnings, "
                 f"{n['N/A']} not applicable ==")
        return verdict


def delta(a, b, key):
    return int(b.get(key, 0)) - int(a.get(key, 0))


# ---------------------------------------------------------------------------
# Test steps
# ---------------------------------------------------------------------------

# TX impairment -> what the receiver must report.  BERT expectations are
# exact (it checks every bit); reference-receiver expectations apply only
# when the reference covered the window without a gap.  The same table is
# proven on the host in firmware/link_bench/tests (test_bench_logic.c).
IMPAIRMENTS = [
    # name, inject args, BERT expectations, reference expectations
    ("comma x1 (flywheel)", "comma 1", dict(errors=(1, 1), slips=0, losses=0), dict(comma=1, loss=0)),
    ("comma x2 (flywheel)", "comma 2", dict(errors=(2, 2), slips=0, losses=0), dict(comma=2, loss=0)),
    ("comma x3 (loss, re-lock)", "comma 3", dict(errors=(3, 3), slips=0, losses=0), dict(comma=3, loss=1, lock=1)),
    ("bit flip x5 (CRC-16, hold)", "flip 5", dict(errors=(5, 5), slips=0, losses=0), dict(crc16=5, loss=0)),
    ("slip +1", "slip 1", dict(errors=(0, 0), slips=1, slip_bits=1, losses=0), dict(slips=1, loss=0)),
    ("slip -1", "slip -1", dict(errors=(0, 1), slips=1, slip_bits=-1, losses=0), dict(slips=1, loss=0)),
    ("slip +2", "slip 2", dict(errors=(0, 0), slips=1, slip_bits=2, losses=0), dict(slips=1, loss=0)),
    ("slip -2", "slip -2", dict(errors=(0, 2), slips=1, slip_bits=-2, losses=0), dict(slips=1, loss=0)),
    ("slip +3 (outside window: loss)", "slip 3", dict(errors=(0, 0), slips=1, slip_bits=3, losses=0),
     dict(comma=3, loss=1, lock=1)),
    ("idle x2 (flywheel)", "idle 2", dict(errors=(1, None), slips=0, losses=0), dict(comma=2, loss=0)),
    ("idle x20 (loss, re-sync)", "idle 20", dict(errors=(1, None), slips=0, losses=1), dict(comma=3, loss=1, lock=1)),
]

REF_KEYS = {"comma": "comma", "loss": "loss", "lock": "lock", "crc16": "crc16", "slips": "slips"}


def step_lock(rep, rx, label, timeout=5.0):
    t, s = rx.wait_sync(timeout)
    rep.check(f"{label}: lock", t is not None,
              f"locked in {t:.2f} s (cls {s.get('rx_cls')} rate {s.get('rate')} ch {s.get('ch')})"
              if t is not None else f"no sync within {timeout} s (acq_tries {s.get('acq_tries')}, "
              f"rx_words {s.get('rx_words')}: is the wire on the right pins?)")
    return t is not None, s


def step_layout(rep, s, cls, rate, ch, label):
    ok = s.get("rx_cls") == cls and s.get("rate") == rate and s.get("ch") == ch and s.get("sf_flags") == 0x10
    rep.check(f"{label}: stream description", ok,
              f"superframe says {s.get('rx_cls')} {s.get('rate')} C={s.get('ch')} flags={s.get('sf_flags')}, "
              f"sent {cls} {rate} C={ch} PRBS")


def step_scan(rep, rx, label, dwell_ms=50, strict=True):
    kv, lines = rx.tagged(f"scan {dwell_ms}", "SCAN", timeout=60.0)
    steps = [parse_kv(x)[1] for x in lines if x.startswith("SCANSTEP")]
    detail = (f"clean {kv.get('lo')}..{kv.get('hi')}, used {kv.get('used')}, mask {kv.get('mask', 0):#07x}; errors per "
              "trim " + " ".join(f"{s['trim']}:{s['errors']}" for s in steps))
    state = kv.get("state")
    if state == 3:
        rep.check(f"{label}: trim scan", False, "no trim decoded clean; " + detail)
        return kv
    centred = abs(int(kv.get("used", 99))) <= 1 and abs(int(kv.get("lo", 0)) + int(kv.get("hi", 0))) <= 2
    rep.check(f"{label}: trim scan centred on 0 (+-1)", state == 2 and centred, detail, warn=not strict)
    return kv


def step_ber(rep, tx, rx, label, minutes, poll_s=5.0, out=print, loop=False):
    """Zero errors over the run; the frame counter tracks TX; nothing stalls."""
    rx.ok("reset")
    if tx is not rx and tx is not None:
        tx.ok("reset")
    time.sleep(0.3)
    s0 = rx.stat()
    t0 = time.monotonic()
    first_f = s0.get("frame", 0)
    end = t0 + minutes * 60.0
    worst_fdiff = 0.0 if tx is not None else None
    fs = RATE_HZ.get(s0.get("rate"), 48_000)
    baud = BAUD.get(CLASSES.get(s0.get("rx_cls"), 1))
    lost = False
    last = s0
    while True:
        now = time.monotonic()
        if now >= end:
            break
        time.sleep(min(poll_s, max(end - now, 0.05)))
        if tx is None:
            sr = rx.stat()
        elif tx is not rx:
            ts_a = time.monotonic()
            st = tx.stat()
            ts_b = time.monotonic()
            sr = rx.stat()
            ts_c = time.monotonic()
            # TX frame at (ts_a+ts_b)/2 vs RX frame at (ts_b+ts_c)/2.
            expect = (ts_c - ts_a) / 2.0 * fs
            fdiff = st.get("tx_frame", 0) - sr.get("frame", 0) + expect
            worst_fdiff = max(worst_fdiff, abs(fdiff))
        else:
            sr = rx.stat()
            worst_fdiff = max(worst_fdiff, abs(sr.get("fdelta", 0)))
        last = sr
        if sr.get("sync") != 1:
            lost = True
        bits = sr.get("bert_bits", 0)
        out(f"    {now - t0:6.0f} s  bits {bits:.3e}  errors {sr.get('bert_errors')}  "
            f"slips {sr.get('bert_slips')}  losses {sr.get('bert_losses')}  ref gaps {sr.get('ref_gaps')}")
    elapsed = time.monotonic() - t0
    s1 = last
    bits = s1.get("bert_bits", 0)
    errs = s1.get("bert_errors", 0)
    line_bits = elapsed * baud
    coverage = bits / line_bits if line_bits else 0.0
    ber_ub = 3.0 / bits if bits and not errs else (errs / bits if bits else float("nan"))
    rep.check(f"{label}: BER", bits > 0 and errs == 0 and not lost and s1.get("bert_losses", 0) == 0
              and s1.get("bert_slips", 0) == 0,
              f"{errs} errors in {bits:.3e} bits ({elapsed:.0f} s, {coverage:.1%} of the line checked); "
              f"BER < {ber_ub:.1e} at 95 % confidence" if not errs else f"{errs} errors in {bits:.3e} bits")
    rep.check(f"{label}: checker kept up", coverage >= 0.95 and s1.get("bert_skipped", 0) == 0,
              f"{coverage:.1%} of line bits compared, {s1.get('bert_skipped', 0)} frames skipped")
    stall = {k: s1.get(k, 0) for k in ("rx_stall",)}
    if tx is not None:
        stx = tx.stat() if tx is not rx else s1
        stall.update({k: stx.get(k, 0) for k in ("tx_late", "tx_stall")})
    rep.check(f"{label}: no FIFO stalls or late refills", not any(stall.values()),
              " ".join(f"{k}={v}" for k, v in stall.items()))
    tol = 0.05 * fs + 64
    if worst_fdiff is None:
        rep.na(f"{label}: frame counter tracks TX", "the sender's frame counter needs --usb")
    else:
        rep.check(f"{label}: frame counter tracks TX", worst_fdiff <= tol,
                  f"worst |F_tx - F_rx - transit| = {worst_fdiff:.0f} frames (limit {tol:.0f})")
    rate = (s1.get("frame", 0) - first_f) / elapsed if elapsed else 0
    rep.check(f"{label}: frame rate", abs(rate - fs) <= 0.01 * fs, f"{rate:.0f} frames/s for {fs} Hz")
    refd = {k: s1.get(k, 0) for k in ("crc8", "crc16", "slips", "loss", "sf_err", "comma", "counter", "code",
                                      "idle", "samples_bad")}
    rcov = s1.get("ref_bits", 0) / bits if bits else 0.0
    rep.check(f"{label}: reference receiver counters", not any(refd.values()),
              " ".join(f"{k}={v}" for k, v in refd.items()) + f"; reference covered {rcov:.0%} of the checked "
              f"bits ({s1.get('ref_gaps', 0)} gaps)")
    return s1


def step_impairments(rep, tx, rx, label, out=print):
    for name, args, bexp, rexp in IMPAIRMENTS:
        rx.ok("reset")
        if tx is not rx:
            tx.ok("reset")
        t, _ = rx.wait_sync(3.0)
        time.sleep(0.4)                     # let the reference receiver lock after its reset
        a = rx.stat()
        ta = tx.stat() if tx is not rx else a
        tx.ok(f"inject {args}")
        time.sleep(0.6)
        if "losses" in bexp and bexp["losses"]:
            rx.wait_sync(3.0)
            time.sleep(0.3)
        b = rx.stat()
        tb = tx.stat() if tx is not rx else b
        kind = args.split()[0]
        sent = delta(ta, tb, f"imp_{kind}")
        problems = []
        if sent < 1:
            problems.append(f"TX did not apply it (imp_{kind} +{sent})")
        lo, hi = bexp["errors"]
        e = delta(a, b, "bert_errors")
        if e < lo or (hi is not None and e > hi):
            problems.append(f"bert_errors +{e}, expected {lo}..{hi if hi is not None else 'many'}")
        for k in ("slips", "slip_bits", "losses"):
            if k in bexp and delta(a, b, f"bert_{k}") != bexp[k]:
                problems.append(f"bert_{k} +{delta(a, b, 'bert_' + k)}, expected {bexp[k]}")
        if b.get("sync") != 1:
            problems.append("receiver not back in sync")
        rep.check(f"{label}: {name}: checker", not problems, "; ".join(problems) or
                  f"errors +{e}, slips +{delta(a, b, 'bert_slips')}, losses +{delta(a, b, 'bert_losses')}")
        gaps = delta(a, b, "ref_gaps")
        if gaps or b.get("ref_bits", 0) <= a.get("ref_bits", 0):
            rep.na(f"{label}: {name}: reference counters", f"reference receiver had {gaps} gaps in the window "
                   "(CPU-limited at this rate); the checker result stands")
            continue
        rprob = [f"{k} +{delta(a, b, REF_KEYS[k])} (expected {v})" for k, v in rexp.items()
                 if delta(a, b, REF_KEYS[k]) != v]
        rep.check(f"{label}: {name}: reference counters", not rprob, "; ".join(rprob) or
                  " ".join(f"{k} +{delta(a, b, REF_KEYS[k])}" for k in rexp))


def step_unplug(rep, rx, label, prompt=input):
    a = rx.stat()
    prompt(f"  Pull the data jumper for about a second, put it back, then press Enter ({label}) ")
    t, b = rx.wait_sync(5.0)
    lost = delta(a, b, "bert_losses") >= 1 and delta(a, b, "bert_syncs") >= 1
    rep.check(f"{label}: unplug and replug", lost and t is not None,
              f"losses +{delta(a, b, 'bert_losses')}, re-syncs +{delta(a, b, 'bert_syncs')}, reference loss "
              f"+{delta(a, b, 'loss')}")


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def check_ids(rep, boards):
    for b in boards:
        i = b.id()
        rep.check(f"{b.name}: bench firmware", i.get("fw") == "link_bench",
                  f"{i.get('chip')} at {i.get('clock')} Hz, {i.get('ver')}, serial {i.get('serial')}, "
                  f"classes {i.get('classes')}")


def run_stream(rep, tx, rx, cls, rate, ch, args, out, loop):
    label = f"{cls} {rate}"
    ok, s = step_lock(rep, rx, label)
    if not ok:
        return
    step_layout(rep, s, cls, rate, ch, label)
    if not args.no_scan:
        step_scan(rep, rx, label, strict=loop or args.strict_scan)
    step_ber(rep, tx, rx, label, args.minutes, out=out, loop=loop)
    if not args.no_impair:
        step_impairments(rep, tx, rx, label, out=out)
    if args.unplug:
        step_unplug(rep, rx, label)


def default_ch(cls, rate):
    want = {"L1": 8, "L2": 16, "L3": 32}[cls]
    cap = {("L1", "44k1"): 8, ("L1", "48k"): 8, ("L1", "96k"): 3, ("L2", "44k1"): 19, ("L2", "48k"): 18,
           ("L2", "96k"): 8, ("L3", "44k1"): 41, ("L3", "48k"): 38, ("L3", "96k"): 18}[(cls, rate)]
    return min(want, cap)


def scenario_loop(rep, b, args, out, cls=None, rate=None):
    cls, rate = cls or args.cls, rate or args.rate
    ch = args.ch or default_ch(cls, rate)
    b.ok(f"loop {cls} {rate} ch={ch} txpins={args.txpin} rxpin={args.rxpin} drive={args.drive}")
    run_stream(rep, b, b, cls, rate, ch, args, out, loop=True)


def scenario_pair(rep, tx, rx, args, out, cls=None, rate=None):
    cls, rate = cls or args.cls, rate or args.rate
    ch = args.ch or default_ch(cls, rate)
    rx.ok(f"rx pin={args.rxpin} cls={cls}")
    tx.ok(f"tx {cls} {rate} ch={ch} pins={args.txpin} drive={args.drive}")
    run_stream(rep, tx, rx, cls, rate, ch, args, out, loop=False)


def scenario_impair(rep, tx, imp, rx, args, out):
    cls, rate = args.cls, args.rate
    ch = args.ch or default_ch(cls, rate)
    label = f"{cls} {rate} via IMPAIR"
    rx.ok(f"rx pin={args.rxpin} cls={cls}")
    tx.ok(f"tx {cls} {rate} ch={ch} pins={args.txpin} drive={args.drive}")
    imp.ok(f"impair in={args.imp_in} out={args.imp_out} div=1 drive={args.drive}")
    ok, s = step_lock(rep, rx, label)
    if not ok:
        return
    # Re-timing jitter: the pass-through quantises edges to div sys cycles.
    table = []
    for div in (1, 2, 3, 4, 6, 8, 10, 12):
        imp.ok(f"impair in={args.imp_in} out={args.imp_out} div={div} drive={args.drive}")
        t, _ = rx.wait_sync(3.0)
        rx.ok("reset")
        time.sleep(args.div_seconds)
        st = rx.stat()
        clean = t is not None and st.get("bert_errors", 1) == 0 and st.get("bert_losses", 1) == 0 and st.get("sync") == 1
        table.append((div, clean, st.get("bert_errors"), st.get("bert_bits")))
        out(f"    div {div:2}: {'clean' if clean else 'errors'} ({st.get('bert_errors')} in {st.get('bert_bits')} bits)")
    rep.check(f"{label}: pass-through at div 1 is clean", table[0][1], f"{table[0][2]} errors")
    clean_divs = [d for d, c, _, _ in table if c]
    rep.info(f"{label}: re-timing margin", f"clean up to div {max(clean_divs) if clean_divs else 0} "
             f"(edge quantisation {max(clean_divs or [0]) * 1e9 / 307.2e6:.1f} ns at 307.2 MHz)")
    imp.ok(f"impair in={args.imp_in} out={args.imp_out} div=1 drive={args.drive}")
    rx.wait_sync(3.0)
    # Bit-flip bursts: errors, no loss.
    rx.ok("reset")
    a = rx.stat()
    imp.ok("impair flip 40 100")
    time.sleep(5.0)
    imp.ok("impair flip 0 0")
    time.sleep(0.3)
    b = rx.stat()
    ib = imp.stat()
    rep.check(f"{label}: flip bursts (40 cycles / 100 ms)", delta(a, b, "bert_errors") > 0 and b.get("sync") == 1,
              f"{ib.get('imp_flips')} bursts -> {delta(a, b, 'bert_errors')} bit errors in "
              f"{delta(a, b, 'bert_err_frames')} frames, losses +{delta(a, b, 'bert_losses')}")
    # Dropouts: loss and re-lock each time.
    rx.ok("reset")
    time.sleep(0.3)
    a = rx.stat()
    ia = imp.stat()
    imp.ok("impair drop 2000 1000")
    time.sleep(5.5)
    imp.ok("impair drop 0 0")
    t, b = rx.wait_sync(3.0)
    ib = imp.stat()
    drops = delta(ia, ib, "imp_drops")
    rep.check(f"{label}: dropouts (2 ms / 1 s): loss and re-lock", t is not None and drops >= 1 and
              delta(a, b, "bert_losses") >= drops - 1 and delta(a, b, "bert_syncs") >= delta(a, b, "bert_losses"),
              f"{drops} dropouts -> checker losses +{delta(a, b, 'bert_losses')}, re-syncs "
              f"+{delta(a, b, 'bert_syncs')}, reference loss +{delta(a, b, 'loss')}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# A link sender (WP LK4) as the transmitter
# ---------------------------------------------------------------------------

def _sender_usb():
    # The module next to this one (also under python -I, which leaves the
    # script's directory off sys.path).
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    import sender_usb
    return sender_usb


class SenderTx:
    """The sender's USB extension block behind the console-board interface step_ber uses."""

    def __init__(self, dev):
        self.dev = dev

    def ok(self, line):
        if line == "reset":
            self.dev.stats_reset()

    def stat(self):
        f = self.dev.frame()
        d = self.dev.diag() or {}
        return dict(tx_frame=f, tx_late=d.get("tx_late", 0), tx_stall=d.get("tx_stall", 0))


def wait_for(rx, pred, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        s = rx.stat()
        if pred(s):
            return time.monotonic() - t0, s
        time.sleep(0.05)
    return None, rx.stat()


def step_sender_changes(rep, dev, rx, cls, rate, ch, out):
    """RATE scheduling, one apply at a time, a CTRL_MSG apply, a class change."""
    su = _sender_usb()
    SenderRefused, RATE_CODE = su.SenderRefused, su.RATE_CODE
    base = dev.link_tx()
    # 1. A rate change while the link runs: RATE at a multiple of 32, >= 2400 frames ahead, value = new C.
    new_rate = "96k" if rate != "96k" else "48k"
    new_ch = min(ch, default_ch(cls, new_rate))
    f0 = dev.frame()
    dev.set_link_tx(dict(base, rate=RATE_CODE[new_rate], channels=new_ch))
    c = dev.ctrl_in()
    ok = (c["op"] == 6 and c["arg"] == RATE_CODE[new_rate] and c["apply_frame"] % 32 == 0
          and c["apply_frame"] >= f0 + 2400 and c["value"] == (new_ch if new_ch != ch else 0))
    rep.check(f"{cls} {rate}: RATE scheduled", ok,
              f"op {c['op']} arg {c['arg']} value {c['value']} at F {c['apply_frame']} (F was {f0}, "
              f"lead {c['apply_frame'] - f0} frames, {c['apply_frame'] % 32 == 0 and 'on' or 'off'} a superframe boundary)")
    third = next(r for r in ("44k1", "48k", "96k") if r not in (rate, new_rate))
    try:
        dev.set_link_tx(dict(base, rate=RATE_CODE[third], channels=min(ch, default_ch(cls, third))))
        refused = False
    except SenderRefused:
        refused = True
    rep.check(f"{cls} {rate}: a second layout change is refused while RATE is pending", refused,
              "STALL" if refused else "accepted (should STALL)")
    t, s = wait_for(rx, lambda s: s.get("sync") == 1 and s.get("rate") == new_rate and s.get("ch") == new_ch, 5.0)
    rep.check(f"{cls} {rate}: receiver follows to {new_rate} C={new_ch}", t is not None,
              f"re-synced in {t:.2f} s at {s.get('rate')} C={s.get('ch')}" if t is not None else
              f"still at {s.get('rate')} C={s.get('ch')} sync={s.get('sync')}")
    rep.info(f"{cls} {rate}: reference receiver across the switch",
             f"loss={s.get('loss')} lock={s.get('lock')} (LK1 follows a RATE without loss when it saw the "
             f"announcement; ref gaps {s.get('ref_gaps')})")
    # Back (also a scheduled RATE).
    time.sleep(0.2)
    dev.set_link_tx(dict(base, rate=RATE_CODE[rate], channels=ch))
    wait_for(rx, lambda s: s.get("sync") == 1 and s.get("rate") == rate and s.get("ch") == ch, 5.0)

    # 2. A CTRL_MSG apply (SYNC_PULSE), one at a time, gone after its frame.
    time.sleep(0.2)
    target = (dev.frame() + 4800 + 31) // 32 * 32
    dev.ctrl(target, 8)
    c = dev.ctrl_in()
    try:
        dev.ctrl(target + 64, 1)
        second = False
    except SenderRefused:
        second = True
    rep.check(f"{cls} {rate}: CTRL_MSG apply carried, a second refused",
              c["op"] == 8 and c["apply_frame"] == target and second,
              f"IN shows op {c['op']} at {c['apply_frame']}, second write {'STALL' if second else 'accepted'}")
    fs = RATE_HZ[rate]
    time.sleep((target - dev.frame()) / fs + 0.1)
    c = dev.ctrl_in()
    rep.check(f"{cls} {rate}: the apply ends at its frame", c["op"] == 0 and c["apply_frame"] == 0,
              f"IN op {c['op']} frame {c['apply_frame']} at F {dev.frame()}")
    t, s = wait_for(rx, lambda s: s.get("sync") == 1, 5.0)
    rep.check(f"{cls} {rate}: receiver back in sync after the apply", t is not None, f"sync={s.get('sync')}")

    # 3. A class change restarts the links; the receiver re-detects the class.
    other = {"L1": "L2", "L2": "L1", "L3": "L2"}[cls]
    och = min(ch, default_ch(other, rate))
    dev.set_link_tx(dict(base, cls=CLASSES[other], channels=och))
    t, s = wait_for(rx, lambda s: s.get("sync") == 1 and s.get("rx_cls") == other, 10.0)
    rep.check(f"{cls} {rate}: class change {cls} -> {other} restarts the links, receiver re-detects", t is not None,
              f"locked at {s.get('rx_cls')} in {t:.2f} s" if t is not None else
              f"rx at {s.get('rx_cls')} sync={s.get('sync')} (is the RX board in cls=auto?)")
    dev.set_link_tx(base)
    wait_for(rx, lambda s: s.get("sync") == 1 and s.get("rx_cls") == cls, 10.0)


def step_sender_budget(rep, dev, cls, rate):
    d = dev.diag()
    if not d:
        rep.na(f"{cls} {rate}: encoder cycles", "EXT_LINK_DIAG is not the sender layout")
        return
    budget = d["cycles_per_frame"] or 1
    rep.check(f"{cls} {rate}: encoder within budget", d["enc_cycles_max"] < budget // 2,
              f"{d['enc_cycles_avg']} cycles per frame on average, worst {d['enc_cycles_max']}, of {budget} "
              f"({100.0 * d['enc_cycles_avg'] / budget:.1f} % of core 1; once for all links); ring lead "
              f"{d['lead_words']} words, tx_late {d['tx_late']}, tx_stall {d['tx_stall']}, core-1 pass worst "
              f"{d['loop_us_max']} us")


def scenario_sender(rep, dev, rx, args, out):
    RATE_CODE = _sender_usb().RATE_CODE
    cls, rate = args.cls, args.rate
    ch = args.ch or default_ch(cls, rate)
    label = f"{cls} {rate}"
    if dev is not None:
        info = dev.info()
        ids = dev.ext_list()
        rep.check("sender: link sender firmware", info[0] in (2, 4) and 0x8007 in ids,
                  f"role {info[0]}, protocol v{info[1]}, max_links {info[3]}, ids "
                  + " ".join(f"{i:04X}" for i in ids))
        tx = dev.link_tx()
        want = dict(tx, link_enable=1 << args.link, cls=CLASSES[cls], channels=ch, rate=RATE_CODE[rate],
                    test_pattern=1, sync_pin=args.sync, flags=0)
        dev.set_link_tx(want)
        got = dev.link_tx()
        rep.check("sender: EXT_LINK_TX written", all(got[k] == want[k] for k in ("link_enable", "cls", "channels",
                                                                                  "rate", "test_pattern", "sync_pin")),
                  f"link_enable {got['link_enable']:#06x} class {got['cls']} C {got['channels']} rate {got['rate']} "
                  f"PRBS {got['test_pattern'] == 1} sync GP{got['sync_pin']}")
        txi = SenderTx(dev)
    else:
        if (cls, rate, ch) != ("L1", "48k", 8):
            raise BenchError("without --usb the sender must be the bench image: L1 48k C=8 (drop --cls/--rate/--ch)")
        txi = None
    rx.ok(f"rx pin={args.rxpin} cls={'auto' if dev is not None and not args.no_changes else cls}")
    ok, s = step_lock(rep, rx, label, timeout=8.0)
    if not ok:
        return
    step_layout(rep, s, cls, rate, ch, label)
    if not args.no_scan:
        step_scan(rep, rx, label, strict=args.strict_scan)
    step_ber(rep, txi, rx, label, args.minutes, out=out)
    if dev is not None:
        step_sender_budget(rep, dev, cls, rate)
        if not args.no_changes:
            step_sender_changes(rep, dev, rx, cls, rate, ch, out)
    if args.unplug:
        step_unplug(rep, rx, label)


def open_sender_usb(path):
    return _sender_usb().SenderUsb(path)


def open_bench(path, name, verbose):
    return Bench(SerialPort(path), name=name, log=(print if verbose else None))


def main(argv=None, opener=open_bench, out=print, usb_opener=open_sender_usb):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="store_true", help="print the console traffic")
    sub = ap.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("id")
    p.add_argument("ports", nargs="+")
    p = sub.add_parser("cmd")
    p.add_argument("port")
    p.add_argument("line")

    def common(p):
        p.add_argument("--cls", default="L1", choices=sorted(CLASSES))
        p.add_argument("--rate", default="48k", choices=sorted(RATE_HZ))
        p.add_argument("--ch", type=int, default=0, help="channels (default: whole banks, capped)")
        p.add_argument("--minutes", type=float, default=11.0,
                       help="BER run (11 min at L1 = 1e10 bits, 3 min at L3)")
        p.add_argument("--txpin", default="2")
        p.add_argument("--rxpin", type=int, default=0)
        p.add_argument("--drive", type=int, default=4, choices=(2, 4, 8, 12))
        p.add_argument("--no-scan", action="store_true")
        p.add_argument("--strict-scan", action="store_true", help="fail (not warn) an off-centre scan on two boards")
        p.add_argument("--no-impair", action="store_true")
        p.add_argument("--unplug", action="store_true", help="also ask for a manual unplug/replug")

    p = sub.add_parser("loop")
    p.add_argument("port")
    common(p)
    p = sub.add_parser("pair")
    p.add_argument("--tx", required=True)
    p.add_argument("--rx", required=True)
    common(p)
    p = sub.add_parser("impair")
    p.add_argument("--tx", required=True)
    p.add_argument("--imp", required=True)
    p.add_argument("--rx", required=True)
    p.add_argument("--imp-in", type=int, default=0)
    p.add_argument("--imp-out", type=int, default=2)
    p.add_argument("--div-seconds", type=float, default=3.0)
    common(p)
    p = sub.add_parser("matrix")
    p.add_argument("port", nargs="?")
    p.add_argument("--tx")
    p.add_argument("--rx")
    common(p)

    p = sub.add_parser("sender-diag")
    p.add_argument("--usb", required=True)
    p.add_argument("--pattern", choices=("audio", "prbs", "idle", "tone"),
                   help="first set EXT_LINK_TX.test_pattern")
    p = sub.add_parser("sender")
    p.add_argument("--rx", required=True)
    p.add_argument("--usb", help="the sender's /dev/bus/usb/BBB/DDD node (else: the bench image's defaults)")
    p.add_argument("--link", type=int, default=0, help="sender link output wired to the RX board (0 = GP2)")
    p.add_argument("--sync", type=int, default=21, help="sender SYNC pin (scope trigger)")
    p.add_argument("--no-changes", action="store_true", help="skip the RATE / apply / class-change steps")
    common(p)

    args = ap.parse_args(argv)
    if args.mode == "id":
        for path in args.ports:
            b = opener(path, path, args.verbose)
            out(f"{path}: {b.command('id', until=('ID', 'ERR'))[-1]}")
        return 0
    if args.mode == "cmd":
        b = opener(args.port, args.port, args.verbose)
        until = {"stat": "STAT", "id": "ID", "diag": "DIAGF", "xstatus": "XSTATUS", "scan": "SCAN"}
        word = args.line.split()[0] if args.line.split() else ""
        tag = until.get(word, "OK")
        for line in b.command(args.line, until=(tag, "ERR"), timeout=60.0):
            out(line)
        return 0

    if args.mode == "sender-diag":
        dev = usb_opener(args.usb)
        if args.pattern:
            pat = ("audio", "prbs", "idle", "tone").index(args.pattern)
            dev.set_link_tx(dict(dev.link_tx(), test_pattern=pat))
        out(f"tx:    {dev.link_tx()}")
        out(f"sf:    {dev.ctrl_in()}")
        out(f"frame: {dev.frame()}")
        out(f"diag:  {dev.diag()}")
        return 0

    rep = Report(out)
    try:
        if args.mode == "loop":
            b = opener(args.port, "board", args.verbose)
            check_ids(rep, [b])
            scenario_loop(rep, b, args, out)
            b.ok("stop")
        elif args.mode == "pair":
            tx, rx = opener(args.tx, "tx", args.verbose), opener(args.rx, "rx", args.verbose)
            check_ids(rep, [tx, rx])
            scenario_pair(rep, tx, rx, args, out)
            tx.ok("stop"); rx.ok("stop")
        elif args.mode == "impair":
            tx = opener(args.tx, "tx", args.verbose)
            imp = opener(args.imp, "impair", args.verbose)
            rx = opener(args.rx, "rx", args.verbose)
            check_ids(rep, [tx, imp, rx])
            scenario_impair(rep, tx, imp, rx, args, out)
            for x in (tx, imp, rx):
                x.ok("stop")
        elif args.mode == "sender":
            rx = opener(args.rx, "rx", args.verbose)
            check_ids(rep, [rx])
            dev = usb_opener(args.usb) if args.usb else None
            scenario_sender(rep, dev, rx, args, out)
            rx.ok("stop")
        elif args.mode == "matrix":
            if args.port:
                b = opener(args.port, "board", args.verbose)
                boards, tx, rx = [b], b, b
            else:
                if not (args.tx and args.rx):
                    ap.error("matrix needs PORT or --tx and --rx")
                tx, rx = opener(args.tx, "tx", args.verbose), opener(args.rx, "rx", args.verbose)
                boards = [tx, rx]
            check_ids(rep, boards)
            classes = [c for c in ("L1", "L2", "L3") if c in str(rx.id().get("classes", "L1,L2,L3"))]
            for cls in classes:
                for rate in ("48k", "44k1", "96k"):
                    out(f"-- {cls} {rate}")
                    if args.port:
                        scenario_loop(rep, tx, args, out, cls, rate)
                    else:
                        scenario_pair(rep, tx, rx, args, out, cls, rate)
            for x in boards:
                x.ok("stop")
    except BenchError as e:
        rep.add("console", "FAIL", str(e))
    return 0 if rep.summary() == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
