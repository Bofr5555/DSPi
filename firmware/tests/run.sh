#!/usr/bin/env bash
# Host-compiled unit tests for pure firmware helpers (no Pico SDK needed).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
CC="${CC:-cc}"
for t in "$HERE"/test_*.c; do
  name="$(basename "$t" .c)"
  "$CC" -std=c11 -Wall -Wextra -Werror -I "$HERE/../DSPi" "$t" -o "$OUT/$name"
  "$OUT/$name"
done
