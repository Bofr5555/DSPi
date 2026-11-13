#!/usr/bin/env bash
# System simulation of alink USB-direct units (WP USB1).  Run it in a container
# with a C compiler, from the repo root:
#   podman run --rm --network none -v "$PWD:/r:ro" <gcc image> bash /r/tools/alink_usb_sim/run.sh [scenario]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HERE/../.."
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
"${CC:-cc}" -std=gnu11 -O2 -Wall -Wextra -Werror -I "$ROOT/firmware/DSPi" "$HERE/alink_usb_sim.c" -o "$OUT/sim" -lm
"$OUT/sim" "$@"
