#!/bin/sh
# Builds the register-free parts of ../main.c on the PC and runs:
#   sim        - AHF selective-harmonic loop and grid PLL
#   relay_sim  - capacitor relay timing rules and point-on-wave switching,
#                for the current settings and for the previous firmware
set -e
cd "$(dirname "$0")"
out=${TMPDIR:-/tmp}/ahf_sim_build
mkdir -p "$out"
CFLAGS="-std=c11 -O2 -Wall -Wextra -Wno-sign-compare -Wno-unused-function -Wno-unused-variable -Wno-type-limits"

# constants + DSP core
sed -n '/^#define SYSCLK_HZ/,/DSP-CORE-END/p' ../main.c > "$out/core.h"
cc $CFLAGS -I"$out" sim.c -lm -o "$out/sim"
"$out/sim"

# process_window() with the real Butterworth filter and sensor-A display
sed -n '/^typedef struct {$/,/} win_t;/p' ../main.c > "$out/proc_glob.h"
cat >> "$out/proc_glob.h" <<'G'
static float w_v[N_CYC], w_i[N_CYC], w_a[N_CYC], w_c[N_CYC], w_t[N_CYC];
static float av[MAX_H + 1], bv[MAX_H + 1], ai[MAX_H + 1], bi[MAX_H + 1],
             aa[MAX_H + 1], ba[MAX_H + 1];
G
awk '$0 ~ "^static void process_window\\(" {p=1} p {print} p && /^}/ {exit}' ../main.c > "$out/proc_fn.h"
cc $CFLAGS -I"$out" proc_sim.c -lm -o "$out/proc_sim"
"$out/proc_sim"

# relay functions, copied verbatim
fn() { awk -v f="$1" '$0 ~ "^static .*[ *]" f "\\(" {p=1} p {print} p && /^}/ {exit}' ../main.c; }
{ fn relay_bsrr; fn relay_isr_step; fn locks_read; fn relay_service; } > "$out/relay_fns.h"
echo
cc $CFLAGS -I"$out" relay_sim.c -lm -o "$out/relay_sim"
"$out/relay_sim"

# same scenario with the previous firmware's rules (immediate switching, 3 s, no min ON)
sed -i -e 's/^\(#define RELAY_SYNC  *\)[0-9]*u/\10u/' \
       -e 's/^\(#define RELAY_MIN_ON_MS  *\)[0-9]*u/\10u/' \
       -e 's/^\(#define CAP_RECLOSE_MS  *\)[0-9]*u/\13000u/' "$out/core.h"
echo; echo "=== previous firmware, same scenario ==="
cc $CFLAGS -I"$out" relay_sim.c -lm -o "$out/relay_old"
"$out/relay_old"
