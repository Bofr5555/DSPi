"""Tests for siggen_sync_bench.py (WP FWTB3) against tools/midi_ctrl's fake device.

No device is opened: the units are FakeDevice receivers behind FakePort and
midi_ctrl.Client (the USB MIDI control channel's framing), and os.open is
guarded so that a /dev node would fail the test.  The fake's alink model
(FakeAlink, WP FWTB1: the op 9 queue, the lead window, the timeline record,
the loss cancel and stop) is extended here only by what a host test of the
driver needs and the fake does not model: REQ_SIGGEN_SET_CONFIG with the R4
fresh-config rule, STOP_NOW, a run that ends after its length, a time base
other than the line's (USB-direct S, with its own origin) and a virtual
clock that moves every unit's counter together.
"""

import io
import json
import os
import struct
import sys
import unittest
from contextlib import redirect_stdout, redirect_stderr

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "midi_ctrl"))
import fake_device  # noqa: E402
import midi_ctrl as mc  # noqa: E402
import siggen_sync_bench as sb  # noqa: E402

FS = 48000
WRAP = 2048 * 48                    # USB-direct: S origins differ by multiples of this


class GenAlink(fake_device.FakeAlink):
    """FakeAlink with the generator's run and a time-base id."""

    def __init__(self, tb=0, domain=0x5A):
        super().__init__("receiver")
        self.tb = tb
        self.domain = domain
        self.cfg = None
        self.fresh = False
        self.run_len = 0

    def time_base(self):
        b = bytearray(super().time_base())
        if b[1]:
            b[12] = self.tb
            struct.pack_into("<I", b, 16, (self.domain & 0xFFFFFF) | self.tb << 24)
        return bytes(b)

    def set(self, sub, p, dev):
        if sub == 0x8006:
            if p[12] != self.tb and p[8] != 0:
                return False                    # another time base: refused (VC_ERR_STATE)
            p = p[:12] + bytes(4)
        return super().set(sub, p, dev)

    def advance(self, frames, dev):
        # R4: a START whose config is not fresh is refused at T (not ready).
        for q in [q for q in self.queue if q[1] == 9 and q[0] <= self.frame + frames]:
            if not self.fresh:
                self.queue.remove(q)
                self.tl.update(result=3, pending=0)
                self.last = (9, 3, q[0])
                dev.notify_apply(9, 3, q[0])
        super().advance(frames, dev)
        if self.tl["state"] == 3:
            self.tl["samples"] = min(self.run_len, self.frame - self.tl["start"])
            if self.tl["samples"] >= self.run_len:
                self.tl["state"] = 0            # the run ended by itself (fresh stays)


class GenDevice(fake_device.FakeDevice):
    """FakeDevice with REQ_SIGGEN_SET_CONFIG (0xA4) and REQ_SIGGEN_CONTROL (0xA6)."""

    def __init__(self, tb=0, frame=1_000_000, domain=0x5A):
        super().__init__(alink="receiver")
        self.alink = GenAlink(tb, domain)
        self.alink.config[0] = 1
        self.alink.frame = frame
        self.alink_event(2, 1)
        self.out.clear()
        self.configs = 0

    def _dispatch(self, tag, is_get, r):
        if r["breq"] == 0xA4 and not is_get:
            p = r["payload"]
            ok = len(p) == 36 and p[0] == 1 and p[1] in (4, 7) and struct.unpack_from("<H", p, 2)[0]
            if not ok:
                return self._reply(tag, False, mc.ST_ERROR)
            ln = self.alink
            ln.cfg, ln.fresh = bytes(p), True
            kind, repeat = p[1], struct.unpack_from("<H", p, 16)[0]
            dur, gap = struct.unpack_from("<I", p, 12)[0], struct.unpack_from("<H", p, 18)[0]
            period = struct.unpack_from("<f", p, 20)[0]
            ln.run_len = int(repeat * ((dur if kind == 4 else period) + gap) * FS / 1000)
            self.configs += 1
            return self._reply(tag, False, mc.ST_OK)
        if r["breq"] == 0xA6 and is_get:
            if r["wvalue"] == 2:
                ln = self.alink
                ln.fresh = False
                ln.tl["state"] = 0
            return self._reply(tag, True, mc.ST_OK, b"\x00")
        return super()._dispatch(tag, is_get, r)


class World:
    """The virtual clock: every unit's counter moves with it; events at times."""

    def __init__(self, devs):
        self.devs = devs
        self.t = 100.0
        self.at = []                        # (time, fn)
        self.frac = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s
        self.frac += s * FS
        n = int(self.frac)
        self.frac -= n
        for d in self.devs:
            d.alink.advance(n, d)
        for item in [a for a in self.at if a[0] <= self.t]:
            self.at.remove(item)
            item[1]()


def bench_of(*devs, rate=FS):
    w = World(list(devs))
    units = {chr(ord("A") + i): sb.MidiUnit(mc.Client(fake_device.FakePort(d), timeout=1.0))
             for i, d in enumerate(devs)}
    return sb.Bench(units, rate, sleep=w.sleep, now=w.now, log=lambda *a: None), w


def sweep(repeat=2, out=2):
    return sb.pack_config("sweep", 1 << out, -20.0, repeat, 300, 20, role="woofer")


SWEEP_MS = sb.run_ms("sweep", 2, 300, 20, 0)


class NoDevices(unittest.TestCase):
    """Every test: opening a device node is an error."""

    def setUp(self):
        self._open = os.open

        def guarded(path, *a, **k):
            if str(path).startswith("/dev/"):
                raise AssertionError(f"test opened a device node: {path}")
            return self._open(path, *a, **k)
        os.open = guarded

    def tearDown(self):
        os.open = self._open


class TestWire(NoDevices):
    def test_apply_layout(self):
        b = sb.pack_apply(0x1122334455667788, 9, 0, 0, 1)
        self.assertEqual(len(b), 16)
        self.assertEqual(b[:8], bytes.fromhex("8877665544332211"))
        self.assertEqual(b[8:], bytes([9, 0, 0, 0, 1, 0, 0, 0]))
        self.assertEqual(sb.pack_apply(0, 0, tb=3)[12], 3)

    def test_config_layout_and_guards(self):
        c = sb.pack_config("sweep", 1 << 2, -20.0, 3, 300, 20, 20.0, 20000.0, role="woofer")
        self.assertEqual(len(c), 36)
        ver, typ, mask, inv, flags, _r, lvl, dur, rep, gap, p1, p2, p3, p4 = struct.unpack("<BBHHBBfIHHffff", c)
        self.assertEqual((ver, typ, mask, inv, flags, dur, rep, gap), (1, 4, 4, 0, 0, 300, 3, 20))
        self.assertEqual((lvl, p1, p2), (-20.0, 20.0, 20000.0))
        imp = sb.pack_config("impulse", 1, -30.0, 5, period_ms=50.0)
        self.assertEqual(struct.unpack_from("<B", imp, 1)[0], 7)
        self.assertEqual(struct.unpack_from("<I", imp, 12)[0], 0)
        self.assertEqual(struct.unpack_from("<f", imp, 20)[0], 50.0)
        with self.assertRaises(ValueError):
            sb.pack_config("sweep", 1, -10.0, 3, role="woofer")     # above -20 dBFS
        with self.assertRaises(ValueError):
            sb.pack_config("sweep", 1, -20.0, 0, role="woofer")     # not finite (R4)
        with self.assertRaises(ValueError):
            sb.pack_config("sweep", 0, -20.0, 1, role="woofer")     # no output
        with self.assertRaises(ValueError):
            sb.pack_config("sine", 1, -20.0, 1, role="woofer")      # continuous: not offered

    def test_review_fwbc_n4_tweeter_cap(self):
        # review fwbc N4: -20 dBFS only on a named woofer / full-range output;
        # a tweeter, or an output whose role is not named, gets -30 dBFS at most.
        for role in ("woofer", "full-range"):
            self.assertEqual(len(sb.pack_config("sweep", 1, -20.0, 1, role=role)), 36)
        for role in ("tweeter", None):
            with self.assertRaises(ValueError):
                sb.pack_config("sweep", 1, -20.0, 1, role=role)
            with self.assertRaises(ValueError):
                sb.pack_config("impulse", 1, -29.9, 1, role=role)
            c = sb.pack_config("sweep", 1, -30.0, 1, role=role)
            self.assertEqual(struct.unpack_from("<f", c, 8)[0], -30.0)

    def test_decoders_against_the_fake(self):
        d = GenDevice()
        tb = sb.unpack_time_base(d.alink.time_base())
        e = tb["entries"][0]
        self.assertEqual((e["tb"], e["state"], e["domain"], e["t"]), (0, 3, 0x5A, 1_000_000))
        self.assertEqual((e["t"] - tb["output_frames"]) & 0xFFFFFFFF, 1000)
        tl = sb.unpack_timeline(d.alink.timeline())
        self.assertEqual((tl["state"], tl["arm_lead_ms"], tl["run_id"]), (0, 150, 0))

    def test_unit_spec(self):
        with self.assertRaises(SystemExit):
            sb.open_unit("A=/dev/snd/midiC2D0")             # no kind: refused before anything opens
        with self.assertRaises(SystemExit):
            sb.open_unit("A=tty:/dev/ttyACM0")


class TestStart(NoDevices):
    def test_two_units_one_T(self):
        a, b = GenDevice(), GenDevice()
        bench, _w = bench_of(a, b)
        rep = bench.start(sweep(), SWEEP_MS, lead_ms=400)
        self.assertEqual(rep["verdict"], "PASS", json.dumps(rep, default=str, indent=1))
        ua, ub = rep["units"]["A"], rep["units"]["B"]
        self.assertEqual(ua["T"], ub["T"])
        self.assertEqual(ua["T"], 1_000_000 + 400 * 48)
        for u in (ua, ub):
            self.assertEqual(u["timeline"]["result_name"], "started")
            self.assertEqual(u["timeline"]["start"], u["T"])
            self.assertEqual(u["timeline"]["samples"], int(SWEEP_MS * 48))
            self.assertTrue(u["undisturbed"])
            # The 0xE2 of op 9 arrived (MIDI notifications).
            self.assertIn("0xE2", [e["type"] for e in u["events"]])
        self.assertEqual((a.configs, b.configs), (1, 1))

    def test_staggered(self):
        a, b, c = GenDevice(), GenDevice(), GenDevice()
        bench, _w = bench_of(a, b, c)
        rep = bench.start(sweep(), SWEEP_MS, lead_ms=400, stagger={"B": 4807, "C": -3})
        self.assertEqual(rep["verdict"], "PASS")
        t0 = rep["units"]["A"]["T"]
        self.assertEqual(rep["units"]["B"]["timeline"]["start"], t0 + 4807)
        self.assertEqual(rep["units"]["C"]["timeline"]["start"], t0 - 3)

    def test_usb_origins_resolved(self):
        # USB-direct S: unit B counts from an origin 3 wraps of 2048 frames
        # earlier (it booted later); the T it gets is A's moved by that.
        a, b = GenDevice(tb=1, frame=50_000_000, domain=0), GenDevice(tb=1, frame=50_000_000 - 3 * WRAP, domain=0)
        bench, _w = bench_of(a, b)
        rep = bench.start(sweep(), SWEEP_MS, lead_ms=400)
        self.assertEqual(rep["verdict"], "PASS", json.dumps(rep, default=str, indent=1))
        self.assertEqual(rep["time_base"], 1)
        self.assertEqual(rep["units"]["B"]["wraps"], -3)
        self.assertEqual(rep["units"]["B"]["T"], rep["units"]["A"]["T"] - 3 * WRAP)

    def test_refusals_before_anything_is_written(self):
        # Another domain (another sender), a unit not LOCKED, a unit without the ops.
        a, b = GenDevice(), GenDevice(domain=0x33)
        bench, _w = bench_of(a, b)
        with self.assertRaisesRegex(sb.Refused, "domain"):
            bench.start(sweep(), SWEEP_MS)
        self.assertEqual(a.alink.queue, [])
        a, b = GenDevice(), GenDevice()
        b.alink_event(1, 2)                                   # B searching
        bench, _w = bench_of(a, b)
        with self.assertRaisesRegex(sb.Refused, "not LOCKED"):
            bench.start(sweep(), SWEEP_MS)
        self.assertEqual(a.alink.queue, [])
        a, b = GenDevice(), GenDevice()
        b.alink.ids = tuple(i for i in b.alink.ids if i not in (0x8030, 0x8031))
        bench, _w = bench_of(a, b)
        with self.assertRaisesRegex(sb.Refused, "not listed"):
            bench.start(sweep(), SWEEP_MS)

    def test_short_lead_refused_at_the_write(self):
        a, b = GenDevice(), GenDevice()
        bench, _w = bench_of(a, b)
        rep = bench.start(sweep(), SWEEP_MS, lead_ms=100)    # under arm_lead (150 ms)
        self.assertEqual(rep["verdict"], "FAIL")
        self.assertTrue(all(w.startswith("refused") for w in rep["write"].values()))
        self.assertEqual(a.alink.queue, [])

    def test_stale_config_refused_at_T(self):
        # R4: STOP_NOW clears the fresh flag; a START written afterwards
        # without a new SET_CONFIG is refused at T, nothing plays.
        a = GenDevice()
        bench, w = bench_of(a)
        bench.stage(sweep())
        a.alink.fresh = False
        plan = bench.plan(400)
        plan["write"] = bench.write_all(plan, sb.OP_SIGGEN_START)
        self.assertEqual(plan["write"]["A"], "ok")
        w.sleep(0.5)
        tl = bench.timeline("A")
        self.assertEqual((tl["result_name"], tl["run_id"]), ("refused_not_ready", 0))

    def test_loss_mid_run(self):
        # B's line goes dark while its run plays (STOPPED_LOSS, the
        # discontinuity counted); C's later START is cancelled by its own
        # loss (CANCELLED_LOSS); A plays through.  The report fails B and C.
        a, b, c = GenDevice(), GenDevice(), GenDevice()
        bench, w = bench_of(a, b, c)
        w.at.append((100.0 + 0.4 + 0.3, lambda: b.alink_event(3, 2)))    # 300 ms into B's run
        w.at.append((100.0 + 0.4 + 0.1, lambda: c.alink_event(3, 2)))    # before C's T
        rep = bench.start(sweep(), SWEEP_MS, lead_ms=400, stagger={"C": 48000})
        self.assertEqual(rep["verdict"], "FAIL")
        ua, ub, uc = (rep["units"][n] for n in "ABC")
        self.assertTrue(ua["ok"])
        self.assertEqual(ub["timeline"]["result_name"], "stopped_loss")
        self.assertEqual(ub["timeline"]["disc"], 1)
        self.assertFalse(ub["undisturbed"])
        self.assertGreater(ub["dist_after"], ub["dist_before"])
        self.assertIn("0xE4", [e["type"] for e in ub["events"]])
        self.assertEqual(uc["timeline"]["result_name"], "cancelled_loss")
        self.assertFalse(uc["ok"])


class TestOthers(NoDevices):
    def test_click_one_T(self):
        a, b = GenDevice(), GenDevice()
        bench, _w = bench_of(a, b)
        rep = bench.click(2, lead_ms=300)
        self.assertEqual(rep["verdict"], "PASS")
        for u in rep["units"].values():
            self.assertEqual((u["apply"]["last_op"], u["apply"]["last_result_name"], u["apply"]["last_frame"]),
                             (7, "applied", u["T"]))

    def test_click_and_impulse_one_unit(self):
        a = GenDevice()
        bench, _w = bench_of(a)
        cfg = sb.pack_config("impulse", 1 << 2, -20.0, 2, period_ms=100.0, role="woofer")
        rep = bench.click_impulse("A", cfg, 2, lead_ms=400, run_ms_=200)
        self.assertEqual(rep["verdict"], "PASS", json.dumps(rep, default=str, indent=1))
        u = rep["units"]["A"]
        self.assertEqual(u["timeline"]["start"], u["T"])
        self.assertEqual(u["click"]["last_frame"], u["T"])

    def test_leads(self):
        a = GenDevice()
        bench, _w = bench_of(a)
        rep = bench.leads("A", sweep(1), [100, 150, 250], sb.run_ms("sweep", 1, 300, 20, 0))
        self.assertEqual(rep["verdict"], "PASS", json.dumps(rep, default=str, indent=1))
        self.assertEqual(rep["arm_lead_ms"], 150)
        self.assertTrue(rep["runs"][0]["write"].startswith("refused"))
        self.assertEqual([r["write"] for r in rep["runs"][1:]], ["ok", "ok"])
        self.assertTrue(all(r["start_on_T"] for r in rep["runs"][1:]))

    def test_abort(self):
        a = GenDevice()
        bench, _w = bench_of(a)
        bench.stage(sweep())
        plan = bench.plan(2000)
        bench.write_all(plan, sb.OP_SIGGEN_START)
        self.assertEqual(len(a.alink.queue), 1)
        out = bench.abort()
        self.assertEqual(out["A"], ["cancel_all tb 0: ok", "stop_now: ok"])
        self.assertEqual(a.alink.queue, [])
        self.assertEqual(bench.timeline("A")["result_name"], "cancelled_host")
        self.assertFalse(a.alink.fresh)

    def test_cli_fake(self):
        # The CLI against in-process fakes: status; --level above -20 dBFS
        # and a missing --out are usage errors before any request.
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            rc = sb.main(["status", "--fake", "2"])
        self.assertEqual(rc, 0)
        st = json.loads(out.getvalue())
        self.assertTrue(st["A"]["capable"] and st["B"]["capable"])
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                sb.main(["start", "--fake", "2", "--out", "2", "--role", "woofer", "--level", "-6"])
            with self.assertRaises(SystemExit):
                sb.main(["start", "--fake", "2"])              # --out required
            # review fwbc N4: the role is required; a tweeter takes -30 dBFS at most and no CLICK.
            with self.assertRaises(SystemExit):
                sb.main(["start", "--fake", "2", "--out", "2"])
            with self.assertRaises(SystemExit):
                sb.main(["start", "--fake", "2", "--out", "3", "--role", "tweeter", "--level", "-20"])
            with self.assertRaises(SystemExit):
                sb.main(["start", "--fake", "2", "--out", "3", "--role", "tweeter", "--level", "-29"])
            for cmd in ("click", "click-impulse"):
                with self.assertRaises(SystemExit):
                    sb.main([cmd, "--fake", "2", "--out", "3", "--role", "tweeter"])

    def test_review_fwbc_n4_tweeter_default_level(self):
        # The default level is the role's cap: a tweeter run stages -30 dBFS.
        staged = []
        real = sb.Bench.stage
        def spy(self, cfg, *a, **k):
            staged.append(struct.unpack_from("<f", cfg, 8)[0])
            return real(self, cfg, *a, **k)
        sb.Bench.stage = spy
        try:
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                sb.main(["start", "--fake", "2", "--out", "3", "--role", "tweeter", "--repeat", "1"])
        finally:
            sb.Bench.stage = real
        self.assertTrue(staged)
        self.assertTrue(all(v == -30.0 for v in staged), staged)


if __name__ == "__main__":
    unittest.main()
