#!/usr/bin/env bash
# Host tests for the link sender core (WP LK4).  No Pico SDK needed.
#
#   firmware/DSPi-Sender/tests/run.sh            (from the repository root)
#
# Needs a C compiler; run it in a container, e.g.
#   podman run --rm --network none -v "$PWD:/repo:ro" -v "$OUT:/out" \
#     -e SENDER_TEST_OUT=/out -w /repo localhost/dspi-console-build-debian \
#     firmware/DSPi-Sender/tests/run.sh
# SENDER_TEST_OUT is the build directory (default build/sender_test, ignored
# by git).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
root=$(cd "$here/../../.." && pwd)
out=${SENDER_TEST_OUT:-$root/build/sender_test}
mkdir -p "$out"
CC=${CC:-cc}
CFLAGS="-std=c11 -O2 -Wall -Wextra -Werror -I$root/firmware/DSPi/link -I$root/firmware/DSPi-Sender"
CODEC="$root/firmware/DSPi/link/link_codec.c $root/firmware/DSPi/link/link_decoder.c"
for t in test_sender test_fanout; do
    [ -f "$here/$t.c" ] || continue
    # shellcheck disable=SC2086
    $CC $CFLAGS -o "$out/$t" "$here/$t.c" $root/firmware/DSPi-Sender/sender_link.c \
        $( [ "$t" = test_fanout ] && echo "$root/firmware/DSPi-Sender/sender_fanout.c" ) $CODEC -lm
    "$out/$t"
done
