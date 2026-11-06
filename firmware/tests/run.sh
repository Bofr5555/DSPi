#!/usr/bin/env bash
# Host-compiled unit tests for pure firmware helpers (no Pico SDK needed;
# tests/stubs stands in for the few SDK headers a firmware source includes).
# Run it in a container with a C compiler, e.g.
#   podman run --rm --network none -v "$PWD/firmware:/fw:ro" <gcc image> bash /fw/tests/run.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
CC="${CC:-cc}"
for t in "$HERE"/test_*.c; do
  name="$(basename "$t" .c)"
  "$CC" -std=c11 -Wall -Wextra -Werror -I "$HERE/../DSPi" -I "$HERE/stubs" "$t" -o "$OUT/$name"
  "$OUT/$name"
done
