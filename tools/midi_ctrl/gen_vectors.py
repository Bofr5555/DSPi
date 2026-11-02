#!/usr/bin/env python3
"""Regenerate firmware/tests/midi_vectors.json (shared C / Python test vectors).

One JSON object per line so the C test (test_midi_frame.c) can scan it
without a JSON library.  Fields: name, dir ("h2d" request / "d2h" device
frame), mfr (hex), tag, frame (hex, type byte .. CRC), sysex (hex, F0 .. F7).
The frame bytes are built here; the C test checks that its encoder produces
exactly `sysex` from `frame`, and decodes every h2d vector back.

    python3 -I gen_vectors.py > ../../firmware/tests/midi_vectors.json
"""

import json
import struct
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import midi_ctrl as mc  # noqa: E402


def vectors():
    dev = mc.MFR_DEV
    v = []

    def add(name, d, frame, tag, mfr=dev):
        v.append(dict(name=name, dir=d, mfr=mfr.hex(), tag=tag, frame=frame.hex(),
                      sysex=mc.encode(frame, tag, mfr).hex()))

    # The spec's worked example: GET 0x7F (platform), tag 0x01.
    add("get_platform", "h2d", mc.request_frame(mc.TYPE_GET, 0x7F), 0x01)
    # SET master volume -20.0 dB (bytes with MSBs set; 14 frame bytes = 2 full groups).
    add("set_master_volume", "h2d",
        mc.request_frame(mc.TYPE_SET, 0xD2, payload=struct.pack("<f", -20.0)), 0x02)
    add("get_status_9_capped", "h2d",
        mc.request_frame(mc.TYPE_GET, 0x50, wvalue=9, windex=2, wlength=26), 0x7E)
    add("set_wvalue_only", "h2d", mc.request_frame(mc.TYPE_SET, 0xD4, wvalue=0x0102), 0x10)
    add("set_64_bytes", "h2d",
        mc.request_frame(mc.TYPE_SET, 0x20, wvalue=3, payload=bytes(range(0x80, 0xC0))), 0x11)
    add("get_platform_3byte_mfr", "h2d", mc.request_frame(mc.TYPE_GET, 0x7F), 0x05,
        mfr=bytes([0x00, 0x21, 0x7E]))
    bulk = bytes((i * 7 + 3) & 0xFF for i in range(6136))
    add("set_all_params_6136", "h2d", mc.request_frame(mc.TYPE_SET, 0xA1, payload=bulk), 0x21)

    add("resp_get_master_volume", "d2h",
        mc.response_frame(mc.RESP_GET, 0, struct.pack("<f", -20.0)), 0x03)
    add("resp_set_ok", "d2h", mc.response_frame(mc.RESP_SET, 0), 0x02)
    add("resp_bulk_locked", "d2h", mc.response_frame(mc.RESP_SET, 4), 0x21)
    add("resp_frame_error_unknown_tag", "d2h", mc.response_frame(mc.RESP_SET, 7), 0x7F)
    add("resp_platform", "d2h", mc.response_frame(mc.RESP_GET, 0, bytes([1, 1, 0x16, 4, 1, 6, 0])), 0x01)
    pkt = bytes([2, 0x02, 0, 0x2A]) + struct.pack("<HH", 0x0AD4, 4) + bytes([10, 0, 0, 0]) + \
        struct.pack("<f", -12.0)
    add("notify_param_changed", "d2h", mc.response_frame(mc.TYPE_NOTIFY, 0, pkt), 0x00)
    return v


def main():
    for vec in vectors():
        print(json.dumps(vec, separators=(", ", ": ")))


if __name__ == "__main__":
    main()
