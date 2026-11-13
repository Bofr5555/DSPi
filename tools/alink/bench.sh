#!/usr/bin/env bash
# Run tools/alink/bench.py in its container venv (rootless podman, no network).
# Every /dev/... argument is passed into the container as a device, and only
# those: the container sees no other USB device.  The checkout is mounted
# read-only.
#
#   tools/alink/bench.sh id /dev/serial/by-id/usb-Raspberry_Pi_DSPi_alink_bench_*
#   tools/alink/bench.sh loop /dev/ttyACM0 --cls L1 --rate 48k
#   tools/alink/bench.sh pair --tx /dev/ttyACM0 --rx /dev/ttyACM1 --cls L2
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
IMAGE="${DSPI_ALINK_BENCH_IMAGE:-${DSPI_LINK_BENCH_IMAGE:-localhost/dspi-alink-bench:1}}"   # DSPI_LINK_BENCH_IMAGE: old name, accepted for one release
if ! podman image exists "$IMAGE"; then
  podman build -q -t "$IMAGE" -f "$HERE/Containerfile.bench" "$HERE" >/dev/null
fi
devs=()
args=()
for a in "$@"; do
  if [[ "$a" == /dev/* ]]; then
    real="$(readlink -f "$a")"
    devs+=(--device "$real")
    args+=("$real")
  else
    args+=("$a")
  fi
done
exec podman run --rm -it --network none --security-opt label=disable --group-add keep-groups \
  "${devs[@]}" -v "$ROOT:/work:ro" "$IMAGE" "${args[@]}"
