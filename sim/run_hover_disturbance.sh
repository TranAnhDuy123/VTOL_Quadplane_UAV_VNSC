#!/usr/bin/env bash
# Hover-disturbance test: QLOITER over the origin, 5 s roll then 5 s pitch torque.
#
#   bash sim/run_hover_disturbance.sh            -> runs/hover_test/{pid,indi}_{1p44,3p24}/
#
# Calm air, calibrated IMU noise, same parameter files as the campaign.
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="$root/runs/hover_test"
mkdir -p "$out"
cat "$root/sim/stock_pid.param" "$root/sim/indi.param" > "$out/indi_combined.param"
export IMU_NOISE="${IMU_NOISE:-$root/sim/imu_noise_matched.json}" GUI=0 RTF="${RTF:-0}" SPEEDUP="${SPEEDUP:-50}"
failures=0
for nm in 1.44 3.24; do
    for ctrl in pid indi; do
        name="${ctrl}_${nm/./p}"
        run_dir="$out/$name"
        [[ -f "$run_dir/flight.BIN" ]] && { echo "[$name] done, skipping"; continue; }
        params="$root/sim/stock_pid.param"
        [[ "$ctrl" == indi ]] && params="$out/indi_combined.param"
        echo "[$name] torque $nm N.m"
        rm -rf "$run_dir"; mkdir -p "$run_dir"
        (
            trap 'bash "$root/sim/stop_gazebo_sitl.sh"' EXIT
            RUNS_DIR="$out" PARAMS="$params" WIND_E=0 SEED=1 MAVPROXY=0 \
                bash "$root/sim/run_gazebo_sitl.sh" "$run_dir" > "$run_dir/launch.log" 2>&1
            sleep 10
            python3 -u "$root/sim/fly_hover_disturbance.py" --nm "$nm" \
                --status-out "$run_dir/mission_status.json" > "$run_dir/mission.log" 2>&1
            sleep 5
        )
        sleep 2
        bin=$(ls -t "$run_dir"/logs/*.BIN 2>/dev/null | head -1 || true)
        if [[ -n "$bin" ]] && python3 -c "import json,sys;s=json.load(open('$run_dir/mission_status.json'));sys.exit(0 if s['completed'] and all(p['ok'] for p in s['pulses']) else 1)"; then
            cp -f "$bin" "$run_dir/flight.BIN"; echo "[$name] ok"
        else
            echo "[$name] FAILED (see $run_dir/mission.log)"; failures=$((failures + 1))
        fi
    done
done
(( failures == 0 )) || exit 1
