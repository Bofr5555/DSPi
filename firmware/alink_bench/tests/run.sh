#!/usr/bin/env bash
# Host tests for the alink bench image (WP LK3): the bench logic alone, then
# the bench logic driven through LK2's cycle-accurate PIO model.
#
#   firmware/alink_bench/tests/run.sh           (from the repository root)
#
# Needs cc, c++ and cmake (pioasm is built from the pico-sdk). Run it in a
# container, e.g.
#   podman run --rm --network none -v "$PWD:/repo:ro" -v "$OUT:/out" \
#     -v "$SDK:/sdk:ro" -e PICO_SDK_PATH=/sdk -e ALINK_BENCH_TEST_OUT=/out \
#     -w /repo localhost/dspi-console-build-debian firmware/alink_bench/tests/run.sh
# Environment: PICO_SDK_PATH (default firmware/pico-sdk), PIOASM (an existing
# pioasm binary), ALINK_BENCH_TEST_OUT (build directory, default
# build/alink_bench_test, ignored by git).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
out=${ALINK_BENCH_TEST_OUT:-$root/build/alink_bench_test}
mkdir -p "$out/gen"
CC=${CC:-cc}
CFLAGS="-std=c11 -O2 -Wall -Wextra -Werror -I$root/firmware/DSPi/alink -I$root/firmware/alink_bench"
SRC="$root/firmware/DSPi/alink/alink_codec.c $root/firmware/DSPi/alink/alink_decoder.c
     $root/firmware/alink_bench/bench_gen.c $root/firmware/alink_bench/bench_check.c
     $root/firmware/DSPi/alink/bench_trim.c $root/firmware/alink_bench/bench_cmd.c"

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
for f in "$root"/firmware/DSPi/alink_tx_*.pio "$root"/firmware/DSPi/alink_rx_*.pio; do
    "$PIOASM" -o c-sdk "$f" "$out/gen/$(basename "$f").h"
done
# shellcheck disable=SC2086
$CC $CFLAGS -DPICO_NO_HARDWARE=1 -I"$out/gen" -I"$root/tools/alink_rx_test" \
    -o "$out/test_bench_model" "$here/test_bench_model.c" $SRC -lm
"$out/test_bench_model" "$@"
# WP LK5: the production receiver against the reference on the model's output.
# shellcheck disable=SC2086
$CC $CFLAGS -DPICO_NO_HARDWARE=1 -I"$out/gen" -I"$root/tools/alink_rx_test" \
    -o "$out/test_fast_model" "$here/test_fast_model.c" $SRC "$root/firmware/DSPi/alink/alink_fast.c" -lm
"$out/test_fast_model"
