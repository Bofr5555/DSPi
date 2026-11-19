#!/usr/bin/env bash
# System simulation of alink USB-direct units (WP USB1; generators WP FWTB3).
# Run it in a container with a C compiler, nm and objcopy, from the repo root:
#   podman run --rm --network none -v "$PWD:/r:ro" <gcc image> bash /r/tools/alink_usb_sim/run.sh [scenario]
# ALINK_USB_SIM_OUT: build directory (default: a temporary one, removed after).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HERE/../.."
if [ -n "${ALINK_USB_SIM_OUT:-}" ]; then
    OUT="$ALINK_USB_SIM_OUT"
    mkdir -p "$OUT"
else
    OUT="$(mktemp -d)"
    trap 'rm -rf "$OUT"' EXIT
fi
# One copy of the real siggen.c + timebase.c per unit (sg_unit.h).
SG=$(sh "$HERE/sg_unit.sh" "$OUT" "$ROOT/firmware")
"${CC:-cc}" -std=gnu11 -O2 -Wall -Wextra -Werror -I "$ROOT/firmware/DSPi" -I "$ROOT/firmware/tests/stubs" -I "$HERE" \
    "$HERE/alink_usb_sim.c" $SG -o "$OUT/sim" -lm
"$OUT/sim" "$@"
