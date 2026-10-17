#!/usr/bin/env bash
# PID vs INDI campaign on sim/square.plan.
#
#   sim/run_campaign.sh [seeds...]        (default seeds: 1 2 3)
#
# Runs {pid, indi} x {calm, w6, w9} x seeds into runs/<ctrl>_<case>_s<seed>/.
#   calm  no wind, no turbulence, no moment pulses
#   w6    6 m/s wind from the west + light turbulence + 1.44 N.m roll/pitch pulses
#   w9    9 m/s wind from the west + light turbulence + 3.24 N.m roll/pitch pulses
# The pulse size is 1/2 rho u^2 S b/4 for a gust u of the same speed (sim/disturb.py).
#
# PID uses sim/stock_pid.param. INDI uses the same file with sim/indi.param
# appended (later lines win), so the two differ only in the Q_INDI_* lines.
# RUNS_DIR sets an independent output root; CASES="w6 w9" limits the cases.
# RTF=0 (headless) is uncapped, keeping the 1 ms physics step unchanged.
# Runs that already finished (flight.BIN present) are skipped.
set -uo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
seeds=("$@")
[[ ${#seeds[@]} -gt 0 ]] || seeds=(1 2 3)
read -r -a cases <<< "${CASES:-calm w6 w9}"
for case in "${cases[@]}"; do
    [[ "$case" == calm || "$case" == w6 || "$case" == w9 ]] || { echo "invalid case: $case" >&2; exit 1; }
done
for seed in "${seeds[@]}"; do
    [[ "$seed" =~ ^[0-9]+$ ]] || { echo "invalid seed: $seed" >&2; exit 1; }
done
run_root="${RUNS_DIR:-$root/runs}"
mkdir -p "$run_root"
run_root="$(cd "$run_root" && pwd)"

# EXTRA_PARAMS=<file> is appended to both parameter files (same for PID and INDI).
extra="${EXTRA_PARAMS:-/dev/null}"
pid_params="$run_root/pid_combined.param"
cat "$root/sim/stock_pid.param" "$extra" > "$pid_params"
indi_params="$run_root/indi_combined.param"
cat "$root/sim/stock_pid.param" "$root/sim/indi.param" "$extra" > "$indi_params"

failures=0
for seed in "${seeds[@]}"; do
    for case in "${cases[@]}"; do
        case "$case" in
            calm) wind=0; nm=0;    turb=0.0; tdir=0.0;  tvert=0.0 ;;
            w6)   wind=6; nm=1.44; turb=0.3; tdir=0.02; tvert=0.05 ;;
            w9)   wind=9; nm=3.24; turb=0.3; tdir=0.02; tvert=0.05 ;;
        esac
        for ctrl in pid indi; do
            name="${ctrl}_${case}_s${seed}"
            if [[ -f "$run_root/$name/flight.BIN" ]]; then
                echo "[$name] done, skipping"
                continue
            fi
            params="$pid_params"
            [[ "$ctrl" == "indi" ]] && params="$indi_params"
            echo "[$name] wind $wind m/s, pulses $nm N.m, seed $seed"
            RUNS_DIR="$run_root" PARAMS="$params" WIND_E="$wind" TURB_MAG="$turb" TURB_DIR="$tdir" TURB_VERT="$tvert" \
                SEED="$seed" DISTURB_NM="$nm" \
                bash "$root/sim/run_square_mission.sh" "$name" > "$run_root/${name}.console.log" 2>&1 \
                || { echo "[$name] FAILED (see $run_root/${name}.console.log)"; failures=$((failures + 1)); }
        done
    done
done
(( failures == 0 )) || { echo "$failures missions failed" >&2; exit 1; }
