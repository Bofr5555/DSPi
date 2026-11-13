#!/usr/bin/env bash
# Run an alink tool in the container (rootless podman, no network, no devices).
# The checkout is mounted read-only.
#
#   ./run.sh -m unittest discover -s tests -v
#   ./run.sh gen_vectors.py > ../../firmware/tests/alink_vectors.json
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
IMAGE="${DSPI_ALINK_IMAGE:-${DSPI_LINK_IMAGE:-localhost/dspi-alink-tools:1}}"   # DSPI_LINK_IMAGE: old name, accepted for one release
if ! podman image exists "$IMAGE"; then
  podman build -q -t "$IMAGE" -f "$HERE/Containerfile" "$HERE" >/dev/null
fi
exec podman run --rm -i --network none --security-opt label=disable \
  -v "$ROOT:/work:ro" "$IMAGE" "$@"
