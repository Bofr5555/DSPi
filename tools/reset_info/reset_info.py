#!/usr/bin/env python3
"""Read a connected DSPi's firmware version and, on firmware built with
reset_info.c, why it last reset.

  reset_info.py            one-shot report
  reset_info.py --watch    poll; print a line whenever the device reboots

Read-only: only GET vendor requests (REQ_GET_PLATFORM, REQ_GET_STATUS).
Stock firmware has no wValue 64..68 (it answers them with 0); that is
reported as "not available", not as an error.
"""
import argparse
import struct
import sys
import time

import usb.core

VID, PID = 0x2E8B, 0xFEAA
BM_VENDOR_IN = 0xC1
WINDEX = 2                      # vendor interface, as in dspi_poke
REQ_GET_STATUS = 0x50
REQ_GET_PLATFORM = 0x7F

ST_SYS_CLK, ST_VREG_MV, ST_TEMP_CDEG = 13, 14, 16
ST_CHIP_RESET, ST_WATCHDOG, ST_BOOT_COUNT, ST_PREV_UPTIME, ST_UPTIME = 64, 65, 66, 67, 68

# CHIP_RESET cause bits (RP2350: POWMAN CHIP_RESET; RP2040: VREG_AND_CHIP_RESET).
RP2350_CAUSES = {
    16: "POR (power-on reset)",
    17: "BOR (brown-out detector)",
    18: "RUN pin low",
    19: "Arm debugger reset request",
    21: "debugger rescue reset",
    22: "watchdog -> powman (async)",
    23: "watchdog -> powman",
    24: "watchdog -> switched core",
    25: "switched-core power-down",
    26: "GLITCH DETECTOR (supply glitch)",
    27: "hazard debugger system reset",
    28: "watchdog -> power-on state machine",
}
RP2040_CAUSES = {
    8: "POR or BOR (power-on/brown-out)",
    16: "RUN pin low",
    20: "debug port / watchdog PSM restart",
}


def get(dev, req, wvalue, length):
    return bytes(dev.ctrl_transfer(BM_VENDOR_IN, req, wvalue, WINDEX, length, timeout=1000))


def status(dev, wvalue):
    try:
        return struct.unpack("<I", get(dev, REQ_GET_STATUS, wvalue, 4))[0]
    except usb.core.USBError:
        return None


def read_all(dev):
    plat = get(dev, REQ_GET_PLATFORM, 0, 4)
    info = {
        "rp2350": plat[0] == 1,
        "version": f"{plat[1]}.{plat[2] >> 4}.{plat[2] & 0xF}",
        "outputs": plat[3],
        "sys_clk": status(dev, ST_SYS_CLK),
        "vreg_mv": status(dev, ST_VREG_MV),
        "temp_cdeg": status(dev, ST_TEMP_CDEG),
    }
    for key, wv in (("chip_reset", ST_CHIP_RESET), ("watchdog", ST_WATCHDOG),
                    ("boot_count", ST_BOOT_COUNT), ("prev_uptime", ST_PREV_UPTIME),
                    ("uptime", ST_UPTIME)):
        info[key] = status(dev, wv)
    # Stock firmware answers unknown wValues with 0 rather than stalling; ours
    # always reports boot_count >= 1, so 0 means "no reset_info in this build".
    if not info["boot_count"]:
        for key in ("chip_reset", "watchdog", "boot_count", "prev_uptime", "uptime"):
            info[key] = None
    return info


def decode_cause(info):
    raw = info["chip_reset"]
    if raw is None:
        return "not available (firmware without reset_info)"
    table = RP2350_CAUSES if info["rp2350"] else RP2040_CAUSES
    names = [name for bit, name in table.items() if raw & (1 << bit)]
    wd = info["watchdog"] or 0
    if wd & 0x100:
        names.append("our 8 s watchdog TIMED OUT (firmware hang)")
    elif wd & 0x1:
        names.append("watchdog timer (not our timeout: reboot/bootloader request)")
    elif wd & 0x2:
        names.append("watchdog forced")
    return f"0x{raw:08X}: " + (", ".join(names) if names else "no cause bit set")


def report(info):
    print(f"platform      {'RP2350' if info['rp2350'] else 'RP2040'}, DSPi v{info['version']}, "
          f"{info['outputs']} outputs")
    if info["sys_clk"] is not None:
        print(f"clock / vreg  {info['sys_clk'] / 1e6:.1f} MHz, {info['vreg_mv']} mV core")
    if info["temp_cdeg"] is not None:
        print(f"temperature   {info['temp_cdeg'] / 100:.1f} C")
    print(f"last reset    {decode_cause(info)}")
    if info["boot_count"] is not None:
        print(f"boot count    {info['boot_count']} since last power-on/brown-out")
        print(f"prev uptime   {info['prev_uptime']} s (how long the previous run lasted; "
              f"0 after power-on/brown-out)")
        print(f"uptime        {info['uptime']} s")


def watch():
    last_boot = None
    print("watching; Ctrl-C to stop")
    while True:
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            if last_boot is not None:
                print(time.strftime("%H:%M:%S"), "device GONE from USB")
                last_boot = None
            time.sleep(0.5)
            continue
        try:
            info = read_all(dev)
        except usb.core.USBError:
            time.sleep(0.5)
            continue
        key = (info["boot_count"], info["chip_reset"])
        if key != last_boot:
            print(time.strftime("%H:%M:%S"), f"boot #{info['boot_count']}, prev run "
                  f"{info['prev_uptime']} s, cause {decode_cause(info)}")
            last_boot = key
        time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--watch", action="store_true", help="poll and log every reboot")
    args = ap.parse_args()
    if args.watch:
        try:
            watch()
        except KeyboardInterrupt:
            pass
        return
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit("no DSPi found (VID:PID 2E8B:FEAA)")
    try:
        report(read_all(dev))
    except usb.core.USBError as e:
        sys.exit(f"USB error: {e} (permissions? see the udev rule)")


if __name__ == "__main__":
    main()
