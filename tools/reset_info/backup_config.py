#!/usr/bin/env python3
"""Back up a connected DSPi's configuration using read-only vendor requests.

  backup_config.py OUTDIR

Writes OUTDIR/<serial>-<timestamp>/ with bulk_params.bin (live WireBulkParams,
i.e. the active preset's DSP state) and config.json (presets directory and
names, master volume, IO mapping, routing, channel names, DAC mute config).

Several DSPi "write-as-read" commands perform actions on a plain IN request
(preset save/load/delete, factory reset, enter bootloader...).  Every request
here goes through read() which refuses any opcode not in READ_ONLY.
Contents of non-active preset slots are not readable without PRESET_LOAD;
take a picotool flash dump for a byte-exact copy of all slots.
"""
import json
import os
import struct
import sys
import time

import usb.core

VID, PID = 0x2E8B, 0xFEAA
BM_VENDOR_IN = 0xC1
WINDEX = 2

READ_ONLY = {
    0x50: "GET_STATUS", 0x61: "GET_CROSSFEED_PRESET", 0x71: "GET_MATRIX_ROUTE",
    0x7D: "GET_OUTPUT_PIN", 0x7E: "GET_SERIAL", 0x7F: "GET_PLATFORM",
    0x93: "PRESET_GET_NAME", 0x95: "PRESET_GET_DIR", 0x97: "PRESET_GET_STARTUP",
    0x99: "GET_OUTPUT_CONFIG_MODE", 0x9A: "PRESET_GET_ACTIVE", 0x9C: "GET_CHANNEL_NAME",
    0xA0: "GET_ALL_PARAMS", 0xA2: "GET_ALL_PARAMS_CHUNK", 0xC1: "GET_OUTPUT_TYPE",
    0xC3: "GET_I2S_BCK_PIN", 0xC7: "GET_MCK_PIN", 0xD3: "GET_MASTER_VOLUME",
    0xD5: "GET_MASTER_VOLUME_MODE", 0xD7: "GET_SAVED_MASTER_VOLUME",
    0xE5: "GET_SPDIF_RX_PIN", 0xEB: "GET_DAC_HW_MUTE_CONFIG", 0xF2: "GET_I2S_RX_PIN",
    0xF9: "GET_CTRL_IFACE_STATUS",
}
PRESET_SLOTS = 10
MAX_INPUTS = 8          # matrix scan bound; out-of-range routes just stall


def read(dev, req, wvalue, length):
    if req not in READ_ONLY:
        raise RuntimeError(f"refusing non-read opcode 0x{req:02X}")
    try:
        return bytes(dev.ctrl_transfer(BM_VENDOR_IN, req, wvalue, WINDEX, length, timeout=2000))
    except usb.core.USBError:
        return None


def cstr(b):
    return None if b is None else b.split(b"\0", 1)[0].decode("utf-8", "replace")


def f32(b):
    return None if b is None or len(b) < 4 else struct.unpack("<f", b[:4])[0]


def hx(b):
    return None if b is None else b.hex()


def bulk_params(dev):
    """Chunked read if the firmware has 0xA2, else a single 0xA0 (usbfs caps at 4 KB)."""
    first = read(dev, 0xA2, 0, 2048)
    if first and len(first) >= 8:
        # WireHeader.payload_length (u16 @6) is the full size; offset 0 took the
        # snapshot, and reading through to the last byte releases its lock.
        total = struct.unpack_from("<H", first, 6)[0]
        data = bytearray(first)
        while len(data) < total:
            chunk = read(dev, 0xA2, len(data), min(2048, total - len(data)))
            if not chunk:
                break
            data += chunk
        return bytes(data[:total]), f"0xA2 chunked, header says {total} B"
    return read(dev, 0xA0, 0, 4096), "0xA0 single"


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit("no DSPi found (VID:PID 2E8B:FEAA)")

    plat = read(dev, 0x7F, 0, 4)
    serial = cstr(read(dev, 0x7E, 0, 16)) or "unknown"
    n_out = plat[3] if plat else 0
    cfg = {
        "captured": time.strftime("%Y-%m-%d %H:%M:%S"),
        "serial": serial,
        "platform": None if plat is None else ("RP2350" if plat[0] == 1 else "RP2040"),
        "fw_version": None if plat is None else f"{plat[1]}.{plat[2] >> 4}.{plat[2] & 0xF}",
        "num_outputs": n_out,
        "presets": {
            "directory_raw": hx(read(dev, 0x95, 0, 7)),
            "startup_raw": hx(read(dev, 0x97, 0, 3)),
            "active_slot": (lambda b: b and b[0])(read(dev, 0x9A, 0, 1)),
            "names": {s: cstr(read(dev, 0x93, s, 32)) for s in range(PRESET_SLOTS)},
        },
        "master_volume": {
            "live_db": f32(read(dev, 0xD3, 0, 4)),
            "saved_db": f32(read(dev, 0xD7, 0, 4)),
            "mode": (lambda b: b and b[0])(read(dev, 0xD5, 0, 1)),
        },
        "io": {
            "output_config_mode": (lambda b: b and b[0])(read(dev, 0x99, 0, 1)),
            "output_types": [hx(read(dev, 0xC1, i, 1)) for i in range(n_out)],
            "output_pins": [hx(read(dev, 0x7D, i, 1)) for i in range(n_out)],
            "i2s_bck_pin": hx(read(dev, 0xC3, 0, 1)),
            "mck_pin": hx(read(dev, 0xC7, 0, 1)),
            "i2s_rx_pins": [hx(read(dev, 0xF2, i, 1)) for i in range(4)],
            "spdif_rx_pins": [hx(read(dev, 0xE5, i, 1)) for i in range(4)],
            "dac_hw_mute_config": hx(read(dev, 0xEB, 0, 16)),
            "ctrl_iface_status": hx(read(dev, 0xF9, 0, 8)),
        },
        "matrix_routes": {f"{i}->{o}": hx(r) for i in range(MAX_INPUTS) for o in range(n_out)
                          if (r := read(dev, 0x71, (i << 8) | o, 8)) is not None},
        "channel_names": {c: cstr(read(dev, 0x9C, c, 32)) for c in range(MAX_INPUTS + n_out)},
    }
    bulk, how = bulk_params(dev)
    cfg["bulk_params"] = {"method": how, "bytes": None if bulk is None else len(bulk)}

    out = os.path.join(sys.argv[1], f"{serial}-{time.strftime('%Y%m%d-%H%M%S')}")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "config.json"), "w") as f:
        json.dump(cfg, f, indent=2)
    if bulk:
        with open(os.path.join(out, "bulk_params.bin"), "wb") as f:
            f.write(bulk)
    print(f"{cfg['platform']} DSPi v{cfg['fw_version']}, serial {serial}, {n_out} outputs")
    print(f"active preset {cfg['presets']['active_slot']}, "
          f"master volume live {cfg['master_volume']['live_db']} dB / "
          f"saved {cfg['master_volume']['saved_db']} dB")
    print(f"bulk params: {cfg['bulk_params']['bytes']} bytes via {how}")
    print(f"saved to {out}")


if __name__ == "__main__":
    main()
