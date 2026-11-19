#!/bin/sh
# run.sh; build and run the two-receiver phase-lock simulation (WP LK6).
#
#   tools/alink_phase_sim/run.sh [scenario substring]    (from the repository root)
#   tools/alink_phase_sim/run.sh pio                     (the line model against LK2's PIO model)
#
# Needs a C compiler, nm and objcopy (WP FWTB3: one copy of the real siggen.c
# and timebase.c per receiver, tools/alink_usb_sim/sg_unit.sh) and Python 3
# (the glue drift check, mirror_diff.py); run it
# in a container, e.g.
#   podman run --rm --network none -v "$PWD:/repo:ro" -v "$OUT:/out" -w /repo \
#       localhost/dspi-console-build-debian sh tools/alink_phase_sim/run.sh
# ALINK_PHASE_SIM_OUT: build directory (default build/alink_phase_sim).  "pio"
# also needs CMake and a C++ compiler for pioasm (PICO_SDK_PATH, or PIOASM).
set -eu
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../.." && pwd)
out=${ALINK_PHASE_SIM_OUT:-$root/build/alink_phase_sim}
mkdir -p "$out"
fw=$root/firmware
if [ "${1:-}" = pio ]; then
    if [ -z "${PIOASM:-}" ]; then
        sdk=${PICO_SDK_PATH:-$fw/pico-sdk}
        if [ ! -x "$out/pioasm/pioasm" ]; then
            cmake -S "$sdk/tools/pioasm" -B "$out/pioasm" -DCMAKE_BUILD_TYPE=Release -DPIOASM_VERSION_STRING=sdk >/dev/null
            cmake --build "$out/pioasm" -j4 >/dev/null
        fi
        PIOASM=$out/pioasm/pioasm
    fi
    mkdir -p "$out/gen"
    for f in "$fw"/DSPi/alink_tx_*.pio "$fw"/DSPi/alink_rx_*.pio; do
        "$PIOASM" -o c-sdk "$f" "$out/gen/$(basename "$f").h"
    done
    cc -std=c11 -O2 -Wall -Wextra -Werror -DPICO_NO_HARDWARE=1 -I"$out/gen" -I"$root/tools/alink_rx_test" \
        -I"$fw/DSPi/alink" -I"$fw/DSPi-Sender" -o "$out/pio_timing" "$here/pio_timing.c" \
        "$fw/DSPi/alink/alink_codec.c" "$fw/DSPi-Sender/sender_alink.c" -lm
    exec "$out/pio_timing"
fi
# The simulation's copy of alink_input.c's glue must not drift (review lk6):
# mirror_diff.py fails unless the differences are the recorded port lines.
python3 -I "$here/mirror_diff.py" "$root"
# WP FWTB3: one copy of the real siggen.c + timebase.c per receiver (needs nm, objcopy).
mkdir -p "$out/sg"
sg=$(sh "$root/tools/alink_usb_sim/sg_unit.sh" "$out/sg" "$fw")
cc -std=c11 -O2 -Wall -Wextra -Werror -Wno-missing-field-initializers -I"$fw/DSPi/alink" -I"$fw/DSPi-Sender" -I"$fw/DSPi" -I"$fw/tests/stubs" \
    -I"$root/tools/alink_usb_sim" \
    -o "$out/alink_phase_sim" "$here/alink_phase_sim.c" \
    "$fw/DSPi/alink/alink_codec.c" "$fw/DSPi/alink/alink_fast.c" "$fw/DSPi/alink/alink_phase.c" \
    "$fw/DSPi-Sender/sender_alink.c" $sg -lm
exec "$out/alink_phase_sim" "$@"
