#!/usr/bin/env bash
# The 6 m/s square mission (case w6 of sim/run_campaign.sh) with a chase
# camera recording, for the side-by-side fixed-wing video.
#
#   bash sim/run_fw_recording.sh      -> runs/fw_video/{pid,indi}_w6/
#
# Same plan, wind, turbulence, seed, 1.44 N.m pulses and parameter files as
# runs/joint_final (stock PID / + INDI, both with sim/assist_sat.param), plus a
# camera fixed to the airframe (sim/worlds/chase_camera_sensor.sdf) rendered
# headless and recorded by sim/record_camera.py. Runs at real time (RTF=1).
# Only frames from 10 s before the transition pulses to the last wing-borne
# pulse + 15 s are kept (disk space). Then: python analysis/fw_video.py runs/fw_video
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="$root/runs/fw_video"
mkdir -p "$out"
cat "$root/sim/stock_pid.param" "$root/sim/assist_sat.param" > "$out/pid_combined.param"
cat "$root/sim/stock_pid.param" "$root/sim/indi.param" "$root/sim/assist_sat.param" > "$out/indi_combined.param"
export IMU_NOISE="${IMU_NOISE:-$root/sim/imu_noise_matched.json}" GUI=0 RTF=1.0 SPEEDUP=1
export WORLD_INSERT="$root/sim/worlds/render_sensors.sdf" MODEL_INSERT="$root/sim/worlds/chase_camera_sensor.sdf"
export HEADLESS_RENDER=1 RECORD_TOPIC=/chase_cam RECORD_QUALITY=85
export PLAN="$root/sim/square.plan" RUNS_DIR="$out"
failures=0
for ctrl in pid indi; do
    name="${ctrl}_w6"
    run_dir="$out/$name"
    [[ -f "$run_dir/flight.BIN" && -s "$run_dir/rec/events.csv" ]] && { echo "[$name] done, skipping"; continue; }
    echo "[$name] square, wind 6 m/s, pulses 1.44 N.m, recording"
    PARAMS="$out/${ctrl}_combined.param" WIND_E=6 TURB_MAG=0.3 TURB_DIR=0.02 TURB_VERT=0.05 SEED=1 DISTURB_NM=1.44 \
        bash "$root/sim/run_square_mission.sh" "$name" > "$out/$name.console.log" 2>&1 \
        || { echo "[$name] FAILED (see $out/$name.console.log)"; failures=$((failures + 1)); continue; }
    python3 - "$run_dir/rec" <<'PY'
import csv
import sys
from pathlib import Path
rec = Path(sys.argv[1])
on = [float(e["sim_s"]) for e in csv.DictReader(open(rec / "events.csv")) if e["event"].startswith("torque_on")]
# torque_on order: take-off roll, pitch, transition roll, pitch, ..., landing roll, pitch
lo, hi = on[2] - 10.0, on[-3] + 15.0
frames = sorted((rec / "frames").glob("*.jpg"))
drop = [f for f in frames if not lo <= int(f.stem) * 1e-3 <= hi]
for f in drop:
    f.unlink()
print("kept %d of %d frames (sim %.1f-%.1f s)" % (len(frames) - len(drop), len(frames), lo, hi))
PY
    echo "[$name] ok"
done
(( failures == 0 )) || exit 1
