#!/usr/bin/env bash
# Run a link tool in the container (rootless podman, no network, no devices).
# The checkout is mounted read-only.
#
#   ./run.sh -m unittest discover -s tests -v
#   ./run.sh gen_vectors.py > ../../firmware/tests/link_vectors.json
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
IMAGE="${DSPI_LINK_IMAGE:-localhost/dspi-link-tools:1}"
if ! podman image exists "$IMAGE"; then
  podman build -q -t "$IMAGE" -f "$HERE/Containerfile" "$HERE" >/dev/null
fi
exec podman run --rm -i --network none --security-opt label=disable \
  -v "$ROOT:/work:ro" "$IMAGE" "$@"
