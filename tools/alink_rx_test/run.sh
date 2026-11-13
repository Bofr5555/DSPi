#!/bin/sh
# run.sh; assemble the link PIO programs and run the cycle-accurate model.
#
#   tools/link_rx_test/run.sh [model args]      (from the repository root)
#
# Needs a C compiler, CMake and a C++ compiler (for pioasm). Environment:
#   PICO_SDK_PATH     pico-sdk checkout (default firmware/pico-sdk); pioasm is
#                     built from its tools/pioasm unless PIOASM is set
#   PIOASM            an existing pioasm binary
#   LINK_MODEL_BUILD  build directory (default build/link_rx_test, ignored by git)
# With no model args it runs the regression matrix; "sweep" prints the margin
# tables; see link_pio_model.c for single cases.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
out=${LINK_MODEL_BUILD:-$root/build/link_rx_test}
mkdir -p "$out/gen"

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
cc -std=c11 -O2 -Wall -Wextra -Werror -DPICO_NO_HARDWARE=1 -I"$out/gen" -I"$here" \
    -o "$out/link_pio_model" "$here/link_pio_model.c" -lm
exec "$out/link_pio_model" "$@"
