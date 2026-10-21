#!/bin/sh
# Builds the DSP core of ../main.c (everything from the first #define to
# DSP-CORE-END: constants, then the register-free core) on the PC and runs
# the AHF + PLL simulation.
set -e
cd "$(dirname "$0")"
out=${TMPDIR:-/tmp}/ahf_sim_build
mkdir -p "$out"
sed -n '/^#define SYSCLK_HZ/,/DSP-CORE-END/p' ../main.c > "$out/core.h"
cc -std=c11 -O2 -Wall -Wextra -Wno-sign-compare -Wno-unused-function -I"$out" sim.c -lm -o "$out/sim"
"$out/sim"
