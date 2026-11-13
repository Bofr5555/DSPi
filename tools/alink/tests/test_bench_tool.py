"""Tests for tools/link/bench.py (WP LK3) against simulated bench boards.

The simulated boards speak the link bench console protocol
(firmware/link_bench/link_bench.c) on a virtual clock: a healthy link
locks, counts bits at the line rate and answers each TX impairment the way
the firmware's checker and reference receiver do (the same expectations
firmware/link_bench/tests/test_bench_logic.c proves on the real code).
Faulty boards (bit errors, an off-centre scan, a missing sync, a stalling
TX) must turn the summary into FAIL.
"""

import os
import pty
import sys
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import bench  # noqa: E402

BAUD = {"L1": 15_360_000, "L2": 30_720_000, "L3": 61_440_000}
FS = {"44k1": 44_100, "48k": 48_000, "96k": 96_000}


class FakeTime:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += max(s, 0.0)


class Wire:
    """The stream between a simulated TX and RX (or one board in loopback)."""

    def __init__(self, clock):
        self.clock = clock
        self.stream = None          # (cls, rate, ch)
        self.t0 = None
        self.f0 = 0
        self.events = []            # (time, kind, n)

    def frame(self):
        if not self.stream:
            return 0
        return self.f0 + int((self.clock.t - self.t0) * FS[self.stream[1]])


# What one TX impairment does to the receiver's counters: (checker, reference).
def effects(kind, n):
    bert = dict(bert_errors=0, bert_slips=0, bert_slip_bits=0, bert_losses=0, bert_syncs=0, bert_err_frames=0)
    ref = dict(comma=0, loss=0, lock=0, crc16=0, slips=0)
    if kind == "comma":
        bert.update(bert_errors=n, bert_err_frames=n)
        ref.update(comma=n, loss=1 if n >= 3 else 0, lock=1 if n >= 3 else 0)
    elif kind == "flip":
        bert.update(bert_errors=n, bert_err_frames=n)
        ref.update(crc16=n)
    elif kind == "slip":
        bert.update(bert_slips=1, bert_slip_bits=n, bert_errors=1 if n < 0 else 0)
        if abs(n) >= 3:
            ref.update(comma=3, loss=1, lock=1)
        else:
            ref.update(slips=1)
    elif kind == "idle":
        lost = n >= 16
        bert.update(bert_errors=160 * min(n, 16), bert_err_frames=min(n, 16), bert_losses=1 if lost else 0,
                    bert_syncs=1 if lost else 0)
        ref.update(comma=min(n, 3), loss=1 if n >= 3 else 0, lock=1 if n >= 3 else 0)
    return bert, ref


class FakeBoard:
    """A board's console: write() takes commands, readline() returns replies."""

    def __init__(self, clock, wire, name, chip="rp2350", faults=()):
        self.clock, self.wire, self.name, self.chip = clock, wire, name, chip
        self.faults = set(faults)
        self.out = []
        self.role = "idle"
        self.counters = {}
        self.base = {}
        self.t_reset = clock.t
        self.imp = dict(imp_comma=0, imp_flip=0, imp_slip=0, imp_idle=0)
        self.div = 1
        self.flip = self.drop = (0, 0)
        self.imp_flips = self.imp_drops = self.flip_acc = 0
        self.flip_t0 = clock.t
        self.reset_counters()

    # Port interface.
    def write(self, data):
        for line in data.decode().splitlines():
            self.handle(line.strip())

    def readline(self, timeout):
        if not self.out:
            self.clock.sleep(timeout)
            return None
        return self.out.pop(0)

    # Simulation.
    def reset_counters(self):
        keys = ["bert_errors", "bert_slips", "bert_slip_bits", "bert_losses", "bert_syncs", "bert_err_frames",
                "comma", "loss", "lock", "crc16", "slips", "crc8", "sf_err", "counter", "code", "idle",
                "samples_bad", "ref_gaps"]
        self.counters = {k: 0 for k in keys}
        self.counters["lock"] = 1
        self.t_reset = self.clock.t
        self.seen = len(self.wire.events)

    def synced(self):
        w = self.wire
        if self.role not in ("rx", "loop") or not w.stream or w.stream[0] != self.cls:
            return False
        if "nosync" in self.faults:
            return False
        return self.clock.t - max(w.t0, self.t_start) > 0.05

    def apply_events(self):
        for t, kind, n in self.wire.events[self.seen:]:
            b, r = effects(kind, n)
            for k, v in list(b.items()) + list(r.items()):
                self.counters[k] += v
        self.seen = len(self.wire.events)

    def stat(self):
        c = dict(role=self.role, clock=307200000)
        w = self.wire
        if self.role in ("tx", "loop"):
            c.update(tx_cls=w.stream[0], tx_rate=w.stream[1], tx_ch=w.stream[2], tx_pay="prbs",
                     tx_frame=w.frame(), tx_late=0, tx_stall=3 if "txstall" in self.faults else 0, **self.imp)
        if self.role in ("rx", "loop"):
            self.apply_events()
            sync = self.synced()
            dt = self.clock.t - self.t_reset
            bits = int(BAUD[self.cls] * dt) if sync else 0
            errors = self.counters["bert_errors"] + (int(dt * 7) if "biterrors" in self.faults else 0)
            imp_board = getattr(self.wire, "impair", None)
            if imp_board is not None:
                errors += imp_board.errors_since(self.t_reset, self.cls)
                self.counters["bert_losses"] = imp_board.drops_since(self.t_reset)
                self.counters["bert_syncs"] = imp_board.drops_since(self.t_reset)
            c.update(rx_pin=0, rx_cls=self.cls, trim=0, sync=int(sync), frame=w.frame() - 20 if sync else 0,
                     rate=w.stream[1] if sync else "none", ch=w.stream[2] if sync else 0,
                     sf_flags=0x10, rx_stall=0, acq_tries=1, bert_bits=bits, bert_frames=bits // 320,
                     bert_bad_frames=0, bert_skipped=0, ref_bits=bits // 2, ref_frames=bits // 640,
                     **{k: (errors if k == "bert_errors" else v) for k, v in self.counters.items()})
            if self.role == "loop":
                c["fdelta"] = 20
        if self.role == "impair":
            c.update(imp_flips=self.imp_flips, imp_drops=self.imp_drops, imp_div=self.div)
        return "STAT " + " ".join(f"{k}={v}" for k, v in c.items())

    # IMPAIR effects seen by the downstream receiver.
    def errors_since(self, t0, cls):
        window = {"L1": 8, "L2": 3, "L3": 1}[cls]
        e = 0
        if self.div > window:
            e += int((self.clock.t - t0) * 1000)
        e += self.flips_now() * 12
        return e

    def flips_now(self):
        running = int((self.clock.t - self.flip_t0) * 1000 / self.flip[1]) if self.flip[1] else 0
        self.imp_flips = self.flip_acc + running
        return self.imp_flips

    def drops_since(self, t0):
        if self.drop[1]:
            self.imp_drops = 1 + int((self.clock.t - self.drop_t0) * 1000 / self.drop[1])
            return self.imp_drops
        return self.imp_drops if hasattr(self, "drop_frozen") else 0

    def handle(self, line):
        a = line.split()
        if not a:
            return
        kv = dict(x.split("=", 1) for x in a if "=" in x)
        cmd = a[0]
        say = self.out.append
        if cmd == "id":
            say(f"ID fw=link_bench ver=test chip={self.chip} clock=307200000 serial={self.name} max_links=7 "
                "classes=L1,L2,L3")
        elif cmd == "stat":
            say(self.stat())
        elif cmd == "stop":
            self.role = "idle"
            say("OK")
        elif cmd == "reset":
            self.reset_counters()
            self.imp = {k: 0 for k in self.imp}
            say("OK")
        elif cmd in ("tx", "loop"):
            self.wire.stream = (a[1], a[2], int(kv.get("ch", 8)))
            self.wire.t0 = self.clock.t
            self.role = cmd
            self.cls = a[1]
            self.t_start = self.clock.t
            self.reset_counters()
            say("OK")
        elif cmd == "rx":
            self.role = "rx"
            self.cls = kv.get("cls", "L1")
            self.t_start = self.clock.t
            self.reset_counters()
            say("OK")
        elif cmd == "inject":
            kind, n = a[1], int(a[2])
            self.imp[f"imp_{kind}"] += n if kind != "slip" else 1
            self.wire.events.append((self.clock.t, kind, n))
            say("OK")
        elif cmd == "scan":
            used = 4 if "offcentre" in self.faults else 0
            for t in range(-9, 10):
                clean = -8 + used <= t <= 8
                say(f"SCANSTEP trim={t} sync=1 clean={int(clean)} errors={0 if clean else 50} bits=760000")
            lo, hi = -8 + used, 8
            mask = sum(1 << (t + 9) for t in range(lo, hi + 1))
            say(f"SCAN state=2 lo={lo} hi={hi} used={(lo + hi) // 2} mask={mask:#07x} cls={self.cls}")
        elif cmd == "impair":
            if len(a) >= 4 and a[1] == "flip":
                self.flip_acc = self.flips_now()
                self.flip = (int(a[2]), int(a[3]))
                self.flip_t0 = self.clock.t
            elif len(a) >= 4 and a[1] == "drop":
                if int(a[3]) == 0 and self.drop[1]:
                    self.drops_since(0)
                    self.drop_frozen = True
                self.drop = (int(a[2]), int(a[3]))
                self.drop_t0 = self.clock.t
            else:
                self.role = "impair"
                self.div = int(kv.get("div", 1))
                self.flip = self.drop = (0, 0)
                self.imp_flips = self.imp_drops = self.flip_acc = 0
                self.flip_t0 = self.clock.t
                self.wire.impair = self
            say("OK")
        else:
            say("ERR unknown command (help)")


class ToolTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeTime()
        self.saved = bench.time
        bench.time = self.clock
        self.lines = []

    def tearDown(self):
        bench.time = self.saved

    def run_tool(self, argv, boards):
        def opener(path, name, verbose):
            return bench.Bench(boards[path], name=name)
        return bench.main(argv, opener=opener, out=self.lines.append)

    def text(self):
        return "\n".join(self.lines)

    def test_lk3_loop_healthy_passes(self):
        w = Wire(self.clock)
        rc = self.run_tool(["loop", "P", "--minutes", "0.5"], {"P": FakeBoard(self.clock, w, "A")})
        self.assertEqual(rc, 0, self.text())
        t = self.text()
        self.assertIn("PASS: ", t)
        self.assertIn("BER < ", t)
        for name, _, _, _ in bench.IMPAIRMENTS:
            self.assertIn(f"{name}: checker", t)
        self.assertNotIn("FAIL", t)

    def test_lk3_pair_healthy_passes_with_frame_counter(self):
        w = Wire(self.clock)
        boards = {"T": FakeBoard(self.clock, w, "T"), "R": FakeBoard(self.clock, w, "R", chip="rp2040")}
        rc = self.run_tool(["pair", "--tx", "T", "--rx", "R", "--cls", "L2", "--minutes", "0.2"], boards)
        self.assertEqual(rc, 0, self.text())
        self.assertIn("frame counter tracks TX", self.text())

    def test_lk3_bit_errors_fail(self):
        w = Wire(self.clock)
        rc = self.run_tool(["loop", "P", "--minutes", "0.2", "--no-impair"],
                           {"P": FakeBoard(self.clock, w, "A", faults={"biterrors"})})
        self.assertEqual(rc, 1)
        self.assertRegex(self.text(), r"FAIL  L1 48k: BER")

    def test_lk3_no_sync_fails(self):
        w = Wire(self.clock)
        rc = self.run_tool(["loop", "P", "--minutes", "0.1"], {"P": FakeBoard(self.clock, w, "A", faults={"nosync"})})
        self.assertEqual(rc, 1)
        self.assertIn("no sync within", self.text())

    def test_lk3_offcentre_scan_fails_in_loop_warns_on_pair(self):
        w = Wire(self.clock)
        rc = self.run_tool(["loop", "P", "--minutes", "0.1", "--no-impair"],
                           {"P": FakeBoard(self.clock, w, "A", faults={"offcentre"})})
        self.assertEqual(rc, 1)
        self.assertIn("FAIL  L1 48k: trim scan centred", self.text())
        self.lines.clear()
        w = Wire(self.clock)
        boards = {"T": FakeBoard(self.clock, w, "T"), "R": FakeBoard(self.clock, w, "R", faults={"offcentre"})}
        rc = self.run_tool(["pair", "--tx", "T", "--rx", "R", "--minutes", "0.1", "--no-impair"], boards)
        self.assertEqual(rc, 0, self.text())
        self.assertIn("WARN  L1 48k: trim scan centred", self.text())

    def test_lk3_tx_stall_fails(self):
        w = Wire(self.clock)
        boards = {"T": FakeBoard(self.clock, w, "T", faults={"txstall"}), "R": FakeBoard(self.clock, w, "R")}
        rc = self.run_tool(["pair", "--tx", "T", "--rx", "R", "--minutes", "0.1", "--no-impair"], boards)
        self.assertEqual(rc, 1)
        self.assertIn("FAIL  L1 48k: no FIFO stalls", self.text())

    def test_lk3_impair_chain(self):
        up, down = Wire(self.clock), Wire(self.clock)
        tx, imp, rx = FakeBoard(self.clock, up, "T"), FakeBoard(self.clock, down, "I"), FakeBoard(self.clock, down, "R")
        # The IMPAIR board passes the upstream stream through to the downstream wire.
        orig = imp.handle

        def handle(line):
            orig(line)
            down.stream, down.t0, down.f0 = up.stream, up.t0, up.f0
        imp.handle = handle
        rc = self.run_tool(["impair", "--tx", "T", "--imp", "I", "--rx", "R", "--div-seconds", "1"],
                           {"T": tx, "I": imp, "R": rx})
        self.assertEqual(rc, 0, self.text())
        t = self.text()
        self.assertIn("clean up to div 8", t)
        self.assertIn("dropouts (2 ms / 1 s): loss and re-lock", t)

    def test_lk3_parse_kv(self):
        tag, kv = bench.parse_kv("STAT role=loop sync=1 sf_flags=0x10 rate=48k slip_bits=-2")
        self.assertEqual(tag, "STAT")
        self.assertEqual(kv, {"role": "loop", "sync": 1, "sf_flags": 16, "rate": "48k", "slip_bits": -2})

    def test_lk3_impairment_table_matches_simulation(self):
        # The tool's expectations and the simulated firmware agree, so a
        # healthy run passes for the right reason.
        for name, args, bexp, rexp in bench.IMPAIRMENTS:
            kind, n = args.split()[0], int(args.split()[1])
            b, r = effects(kind, n)
            lo, hi = bexp["errors"]
            self.assertTrue(lo <= b["bert_errors"] and (hi is None or b["bert_errors"] <= hi), name)
            for k, v in rexp.items():
                self.assertEqual(r[k], v, name)


class SerialTransportTest(unittest.TestCase):
    """The real termios transport over a pty pair, a board simulated on the far side."""

    def test_lk3_serial_port_round_trip(self):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        clock = FakeTime()
        board = FakeBoard(clock, Wire(clock), "PTY")
        stop = threading.Event()

        def serve():
            buf = b""
            while not stop.is_set():
                try:
                    chunk = os.read(master, 256)
                except OSError:
                    return
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    board.write(line + b"\n")
                    while board.out:
                        os.write(master, (board.out.pop(0) + "\r\n").encode())
        th = threading.Thread(target=serve, daemon=True)
        th.start()
        out = []
        try:
            rc = bench.main(["id", path], out=out.append)
            self.assertEqual(rc, 0)
            self.assertIn("fw=link_bench", out[0])
            port = bench.SerialPort(path)
            b = bench.Bench(port)
            self.assertEqual(b.stat()["role"], "idle")
            b.ok("loop L1 48k")
            self.assertEqual(b.stat()["role"], "loop")
            r = b.command("bogus")
            self.assertTrue(r[-1].startswith("ERR"))
            port.close()
        finally:
            stop.set()
            os.close(slave)
            os.close(master)


if __name__ == "__main__":
    unittest.main()
