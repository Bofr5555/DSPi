#!/usr/bin/env bash
# Host tests for the link bench image (WP LK3): the bench logic alone, then
# the bench logic driven through LK2's cycle-accurate PIO model.
#
#   firmware/link_bench/tests/run.sh            (from the repository root)
#
# Needs cc, c++ and cmake (pioasm is built from the pico-sdk). Run it in a
# container, e.g.
#   podman run --rm --network none -v "$PWD:/repo:ro" -v "$OUT:/out" \
#     -v "$SDK:/sdk:ro" -e PICO_SDK_PATH=/sdk -e LINK_BENCH_TEST_OUT=/out \
#     -w /repo localhost/dspi-console-build-debian firmware/link_bench/tests/run.sh
# Environment: PICO_SDK_PATH (default firmware/pico-sdk), PIOASM (an existing
# pioasm binary), LINK_BENCH_TEST_OUT (build directory, default
# build/link_bench_test, ignored by git).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
out=${LINK_BENCH_TEST_OUT:-$root/build/link_bench_test}
mkdir -p "$out/gen"
CC=${CC:-cc}
CFLAGS="-std=c11 -O2 -Wall -Wextra -Werror -I$root/firmware/DSPi/link -I$root/firmware/link_bench"
SRC="$root/firmware/DSPi/link/link_codec.c $root/firmware/DSPi/link/link_decoder.c
     $root/firmware/link_bench/bench_gen.c $root/firmware/link_bench/bench_check.c
     $root/firmware/link_bench/bench_trim.c $root/firmware/link_bench/bench_cmd.c"

# shellcheck disable=SC2086
$CC $CFLAGS -o "$out/test_bench_logic" "$here/test_bench_logic.c" $SRC
"$out/test_bench_logic"

if [ -z "${PIOASM:-}" ]; then
    sdk=${PICO_SDK_PATH:-$root/firmware/pico-sdk}
    [ -f "$sdk/tools/pioasm/CMakeLists.txt" ] || { echo "no pico-sdk at $sdk (set PICO_SDK_PATH or PIOASM)" >&2; exit 2; }
    if [ ! -x "$out/pioasm/pioasm" ]; then
        cmake -S "$sdk/tools/pioasm" -B "$out/pioasm" -DCMAKE_BUILD_TYPE=Release \
            -DPIOASM_VERSION_STRING=sdk >/dev/null
        cmake --build "$out/pioasm" -j4 >/dev/null
    fi
    PIOASM=$out/pioasm/pioasm
fi
for f in "$root"/firmware/DSPi/link_tx_*.pio "$root"/firmware/DSPi/link_rx_*.pio; do
    "$PIOASM" -o c-sdk "$f" "$out/gen/$(basename "$f").h"
done
# shellcheck disable=SC2086
$CC $CFLAGS -DPICO_NO_HARDWARE=1 -I"$out/gen" -I"$root/tools/link_rx_test" \
    -o "$out/test_bench_model" "$here/test_bench_model.c" $SRC -lm
"$out/test_bench_model" "$@"
# WP LK5: the production receiver against the reference on the model's output.
# shellcheck disable=SC2086
$CC $CFLAGS -DPICO_NO_HARDWARE=1 -I"$out/gen" -I"$root/tools/link_rx_test" \
    -o "$out/test_fast_model" "$here/test_fast_model.c" $SRC "$root/firmware/DSPi/link/link_fast.c" -lm
"$out/test_fast_model"
