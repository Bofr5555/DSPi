#!/usr/bin/env python3
"""mirror_diff.py - keep alink_phase_sim.c's copy of the receiver glue in step (WP LK6, review lk6).

The simulation runs line-for-line copies ("mirrors") of alink_input.c's
delivery, anchor, verification, servo-tick and holdover functions, because the
firmware file needs the Pico SDK.  This script extracts each mirrored function
from both files, removes the simulation's `rx_t *r` plumbing and comments, and
diffs them statement by statement.  The differences that remain are the port
itself (ring fill, DMA positions, the actuator, events, the flags byte); they
are recorded in mirror_expected.txt.  Any other difference means one copy
changed without the other: the check fails.

  python3 tools/alink_phase_sim/mirror_diff.py [ROOT]           check (exit 1 on drift)
  python3 tools/alink_phase_sim/mirror_diff.py [ROOT] --write   accept the current port differences

Based on review lk6's mirror_diff.py.
"""

import difflib
import os
import re
import sys

NAMES = ["deliver_rows", "silence_block", "deliver_span", "reanchor", "deliver_hold", "deliver_place", "deliver",
         "phase_error", "phase_act", "phase_verify", "phase_servo_tick", "enter_locked", "enter_holdover",
         "arrival_since", "apply_exec", "apply_run", "apply_report",
         # WP FWTB3: the line's time base and the write path (ops 9/10)
         "tb_line_now", "tb_line_note", "tb_line_read", "tb_line_next", "tb_line_drop", "alink_input_apply"]


def body(src, name):
    m = re.search(r"\n(?:static |DSP_TIME_CRITICAL\n)?[\w\s\*\(\)]*?\b" + name + r"\s*\(.*?\)\s*\{", src)
    if not m:
        return None
    i, depth = m.end(), 1
    while depth:
        c = src[i]
        depth += (c == "{") - (c == "}")
        i += 1
    return src[m.start():i]


def norm(s):
    s = re.sub(r"\brx_t \*r,\s*", "", s)
    s = re.sub(r"\brx_t \*r\b", "void", s)
    s = re.sub(r"\br->", "", s)
    s = re.sub(r"\(r,\s*", "(", s)
    s = re.sub(r"\(r\)", "()", s)
    s = re.sub(r"&\(alink_apply_limits_t\)\{[^}]*\}", "&apply_lim", s)
    s = re.sub(r"//.*", "", s)
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    s = re.sub(r"\s+", " ", s)
    return re.sub(r"\s*([{};(),])\s*", r"\1", s).split(";")


def report(root):
    fw = open(os.path.join(root, "firmware/DSPi/alink_input.c")).read()
    sim = open(os.path.join(root, "tools/alink_phase_sim/alink_phase_sim.c")).read()
    out = []
    for n in NAMES:
        a, b = body(fw, n), body(sim, n)
        if a is None or b is None:
            out.append(f"### {n}: missing fw={a is not None} sim={b is not None}")
            continue
        d = list(difflib.unified_diff(norm(a), norm(b), "fw", "sim", lineterm="", n=0))
        lines = [l for l in d[2:] if l[:1] in "+-"]
        out.append(f"### {n}: " + ("identical" if not lines else f"{len(lines)} port statements"))
        out += ["    " + l[:240] for l in lines]
    return "\n".join(out) + "\n"


def main(argv):
    args = [a for a in argv if not a.startswith("--")]
    root = args[0] if args else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    expected = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mirror_expected.txt")
    now = report(root)
    if "--write" in argv:
        open(expected, "w").write(now)
        print(f"mirror_diff: wrote {expected}")
        return 0
    want = open(expected).read() if os.path.exists(expected) else ""
    if now == want:
        print(f"mirror_diff: the simulation's {len(NAMES)} mirrored functions match alink_input.c (port lines as recorded)")
        return 0
    print("mirror_diff: FAIL, the simulation's glue and alink_input.c have drifted apart:")
    sys.stdout.writelines(difflib.unified_diff(want.splitlines(True), now.splitlines(True), "mirror_expected.txt",
                                               "now"))
    print("Change both copies; after checking that the remaining differences are port lines only, "
          "run mirror_diff.py --write.")
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
