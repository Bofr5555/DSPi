#!/bin/sh
# sg_unit.sh OUT FW - build the per-unit generator objects of sg_unit.h (WP FWTB3):
# sg_unit.c once, then SG_UNITS copies with every defined global renamed
# sgu<k>_<name> (objcopy).  Undefined symbols must be the C library's only:
# a port function siggen.c or timebase.c gains later fails the check below
# instead of binding silently to one definition shared by all units.
# Prints the objects to link.  Needs cc, nm, objcopy.
set -eu
out=$1 fw=$2
here=$(cd "$(dirname "$0")" && pwd)
cc=${CC:-cc}
"$cc" -std=c11 -O2 -Wall -Wextra -Werror -I "$fw/DSPi" -I "$fw/tests/stubs" -I "$here" \
    -c "$here/sg_unit.c" -o "$out/sg_unit.o"
bad=$(nm -u "$out/sg_unit.o" | awk '{print $2}' | grep -Ev '^(mem(set|cpy|move|cmp)|str[a-z]*|(sin|cos|tan|exp|log|pow|sqrt|floor|ceil|fabs|round|lround|lrint|fmod|atan2|exp2|log2|log10|fmin|fmax|tanh|modf|ldexp|frexp)f?|__[a-z_0-9]+|_GLOBAL_OFFSET_TABLE_|abort|printf|puts|putchar)$' || true)
if [ -n "$bad" ]; then
    echo "sg_unit.sh: sg_unit.o needs symbols the simulation would share between units: $bad" >&2
    exit 1
fi
nm -g --defined-only "$out/sg_unit.o" | awk '{print $3}' > "$out/sg_unit.syms"
k=0
while [ $k -lt 5 ]; do
    sed "s/.*/& sgu${k}_&/" "$out/sg_unit.syms" > "$out/sg_unit$k.map"
    objcopy --redefine-syms="$out/sg_unit$k.map" "$out/sg_unit.o" "$out/sg_unit$k.o"
    printf '%s ' "$out/sg_unit$k.o"
    k=$((k + 1))
done
echo
