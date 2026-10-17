#!/usr/bin/env bash
# The result runs: PID and INDI at 6 m/s and 9 m/s wind, no moment pulses,
# no IMU noise. Lock-step simulation, so each run takes about a minute and is
# repeatable.
#
#   sim/run_results.sh            -> runs/{pid,indi}_{w6,w9}/
#
# Then: analysis/flight_figures.py --out runs/figures runs/pid_w6 runs/indi_w6 runs/pid_w9 runs/indi_w9
set -uo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
indi_params="$root/runs/indi_combined.param"
cat "$root/sim/stock_pid.param" "$root/sim/indi.param" > "$indi_params"

for wind in 6 9; do
    for ctrl in pid indi; do
        name="${ctrl}_w${wind}"
        params="$root/sim/stock_pid.param"
        [[ "$ctrl" == "indi" ]] && params="$indi_params"
        echo "[$name] wind $wind m/s from the west, light turbulence, seed 1"
        PARAMS="$params" WIND_E="$wind" TURB_MAG=0.3 TURB_DIR=0.02 TURB_VERT=0.05 SEED=1 \
            bash "$root/sim/run_square_mission.sh" "$name" > "$root/runs/${name}.console.log" 2>&1 \
            || echo "[$name] FAILED (see runs/${name}.console.log)"
        grep -o '"reason": "[^"]*"' "$root/runs/$name/mission_status.json" 2>/dev/null
    done
done
