#!/usr/bin/env bash
# Run a midi_ctrl tool in the container (rootless podman).  The firmware
# checkout is mounted read-only except tools/midi_ctrl/out; no network.
#
#   ./run.sh midi_ctrl.py list
#   DSPI_MIDI_DEV=/dev/snd/midiC2D0 ./run.sh midi_ctrl.py get 0x7F
#   ./run.sh -m unittest discover -s tests -v          # no device
#
# Device access is opt-in and narrow: DSPI_MIDI_DEV passes exactly one
# rawmidi node (/proc/asound, masked by podman by default, is unmasked
# read-only for the card lookup; /sys is the container's read-only sysfs);
# DSPI_USB_DEV additionally passes one /dev/bus/usb/BBB/DDD node for the
# optional vendor-interface reads (measure.py --vendor, soak.py --vendor).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
IMAGE="${DSPI_MIDI_IMAGE:-localhost/dspi-midi-ctrl:1}"
if ! podman image exists "$IMAGE"; then
  podman build -q -t "$IMAGE" -f "$HERE/Containerfile" "$HERE" >/dev/null
fi
mkdir -p "$HERE/out"
DEV=()
if [ -n "${DSPI_MIDI_DEV:-}" ]; then DEV+=(--device "$DSPI_MIDI_DEV" --group-add keep-groups); fi
if [ -n "${DSPI_USB_DEV:-}" ]; then DEV+=(--device "$DSPI_USB_DEV"); fi
TTY=(); [ -t 0 ] && TTY=(-it)
exec podman run --rm "${TTY[@]}" --network none --security-opt label=disable \
  --security-opt unmask=/proc/asound \
  -v "$ROOT:/work:ro" -v "$HERE/out:/work/tools/midi_ctrl/out:rw" \
  -e DSPI_BULK_FIXTURE="${DSPI_BULK_FIXTURE:-}" \
  "${DEV[@]}" "$IMAGE" "$@"
