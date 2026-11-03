#!/usr/bin/env bash
# Before/after regression check around an F1 (DSPI_USB_MIDI) flash.
# Coordinator-run; reads only (sysfs descriptors, ALSA listing, GET requests).
#
#   ./regress.sh before            # with the old firmware running
#   (coordinator flashes the MIDI build)
#   ./regress.sh after             # with the new firmware running
#   ./regress.sh compare           # diff: interfaces 0-2, state image, ALSA
#
# State image: GET 0xA2 chunks over the vendor interface (needs
# DSPI_USB_DEV=/dev/bus/usb/BBB/DDD); `after` also reads 0xA0 over MIDI
# (needs DSPI_MIDI_DEV) and checks it equals the vendor image.
#
# Audio: this script does NOT play anything.  The playback smoke test is the
# coordinator's guarded player (<= 80 %, no fallback, watchdog), run after
# `after`; regress.sh only checks that ALSA still lists the DSPi PCM.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$HERE/out/regress"
mkdir -p "$OUT"
R="$HERE/run.sh"
step="${1:-}"

state_dump() {   # $1 = file (inside the container: out/regress/...)
  if [ -n "${DSPI_USB_DEV:-}" ]; then
    "$R" -c "import sys, usb_vendor as u; d=u.VendorDevice(); b=d.get_all_params(); open(sys.argv[1],'wb').write(b); print('vendor 0xA2 image', len(b), 'bytes')" "$1"
  else
    echo "DSPI_USB_DEV not set: no vendor state image"
  fi
}

case "$step" in
  before|after)
    "$R" usb_desc.py snapshot "out/regress/$step.json"
    state_dump "out/regress/$step-state.bin"
    if [ "$step" = after ]; then
      if [ -n "${DSPI_MIDI_DEV:-}" ]; then
        "$R" midi_ctrl.py get 0x7F --port "$DSPI_MIDI_DEV"
        "$R" midi_ctrl.py get 0xA0 --port "$DSPI_MIDI_DEV" --out out/regress/after-state-midi.bin
      else
        echo "DSPI_MIDI_DEV not set: MIDI checks skipped"
      fi
    fi
    ;;
  compare)
    "$R" usb_desc.py compare out/regress/before.json out/regress/after.json
    if [ -f "$OUT/before-state.bin" ] && [ -f "$OUT/after-state.bin" ]; then
      if cmp -s "$OUT/before-state.bin" "$OUT/after-state.bin"; then
        echo "state image: identical (vendor, before vs after)"
      else
        echo "state image: DIFFERENT (vendor, before vs after)"; exit 1
      fi
    fi
    if [ -f "$OUT/after-state.bin" ] && [ -f "$OUT/after-state-midi.bin" ]; then
      if cmp -s "$OUT/after-state.bin" "$OUT/after-state-midi.bin"; then
        echo "state image: MIDI 0xA0 == vendor 0xA2"
      else
        echo "state image: MIDI 0xA0 != vendor 0xA2"; exit 1
      fi
    fi
    ;;
  *)
    sed -n '2,20p' "$0"; exit 2 ;;
esac
