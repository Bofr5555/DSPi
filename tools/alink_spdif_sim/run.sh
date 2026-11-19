#!/usr/bin/env bash
# System simulation of the S/PDIF time base (WP FWSP2): one source, two
# followers, through the portable roles.  Run it in a container with a C
# compiler, from the repo root:
#   podman run --rm --network none -v "$PWD:/r:ro" <gcc image> bash /r/tools/alink_spdif_sim/run.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HERE/../.."
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
"${CC:-cc}" -std=gnu11 -O2 -Wall -Wextra -Werror -I "$ROOT/firmware/DSPi" -I "$ROOT/firmware/tests" \
  "$HERE/alink_spdif_sim.c" -o "$OUT/sim" -lm
"$OUT/sim" "$@"
