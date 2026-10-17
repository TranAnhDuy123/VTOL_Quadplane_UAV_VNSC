#!/usr/bin/env bash
# Hover torque test with a camera recording, for the side-by-side video.
#
#   bash sim/run_hover_recording.sh      -> runs/hover_video/{pid,indi}_{1p44,3p24}/
#
# Same flight and parameters as sim/run_hover_disturbance.sh, plus a fixed
# camera (sim/worlds/hover_camera.sdf) rendered headless and recorded by
# sim/record_camera.py. Runs at real time (RTF=1) so no frame is dropped.
# Then: python analysis/hover_video.py runs/hover_video
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="$root/runs/hover_video"
mkdir -p "$out"
cat "$root/sim/stock_pid.param" "$root/sim/indi.param" > "$out/indi_combined.param"
export IMU_NOISE="${IMU_NOISE:-$root/sim/imu_noise_matched.json}" GUI=0 RTF=1.0 SPEEDUP=1
export WORLD_INSERT="$root/sim/worlds/hover_camera.sdf" HEADLESS_RENDER=1
failures=0
for nm in 1.44 3.24; do
    for ctrl in pid indi; do
        name="${ctrl}_${nm/./p}"
        run_dir="$out/$name"
        [[ -f "$run_dir/flight.BIN" && -s "$run_dir/rec/events.csv" ]] && { echo "[$name] done, skipping"; continue; }
        params="$root/sim/stock_pid.param"
        [[ "$ctrl" == indi ]] && params="$out/indi_combined.param"
        echo "[$name] torque $nm N.m, recording"
        rm -rf "$run_dir"; mkdir -p "$run_dir/rec"
        (
            trap 'kill -TERM $rec 2>/dev/null; wait $rec 2>/dev/null; bash "$root/sim/stop_gazebo_sitl.sh"' EXIT
            PARAMS="$params" WIND_E=0 SEED=1 MAVPROXY=0 \
                bash "$root/sim/run_gazebo_sitl.sh" "$run_dir" > "$run_dir/launch.log" 2>&1
            /usr/bin/python3 -u "$root/sim/record_camera.py" "$run_dir/rec" > "$run_dir/rec/record.log" 2>&1 &
            rec=$!
            sleep 10
            python3 -u "$root/sim/fly_hover_disturbance.py" --nm "$nm" \
                --status-out "$run_dir/mission_status.json" > "$run_dir/mission.log" 2>&1
            sleep 3
        )
        sleep 2
        bin=$(ls -t "$run_dir"/logs/*.BIN 2>/dev/null | head -1 || true)
        nframes=$(ls "$run_dir/rec/frames" 2>/dev/null | wc -l)
        if [[ -n "$bin" ]] && python3 -c "import json,sys;s=json.load(open('$run_dir/mission_status.json'));sys.exit(0 if s['completed'] and all(p['ok'] for p in s['pulses']) else 1)"; then
            cp -f "$bin" "$run_dir/flight.BIN"; echo "[$name] ok, $nframes frames"
        else
            echo "[$name] FAILED ($nframes frames; see $run_dir/mission.log)"; failures=$((failures + 1))
        fi
    done
done
(( failures == 0 )) || exit 1
