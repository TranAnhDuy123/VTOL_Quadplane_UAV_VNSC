#!/usr/bin/env bash
# Start Gazebo + stock ArduPlane SITL for the quadplane.
#
#   sim/run_gazebo_sitl.sh [run-dir]
#
# run-dir (default: runs/manual) becomes SITL's working directory, so the
# DataFlash logs land in <run-dir>/logs/ and there is no stale eeprom.bin from
# an earlier run (stale eeprom values silently override --defaults).
#
# Environment:
#   PARAMS=<file>   parameter file (default: sim/stock_pid.param, two columns)
#   WIND_E, WIND_N  steady wind in m/s, Gazebo ENU (default 0 0 = calm)
#   TURB_MAG        horizontal turbulence stddev, m/s (default 0)
#   SEED            Gazebo random seed (default 1), fixes the turbulence so
#                   PID and INDI runs see seeded gusts (not perfectly lockstep)
#   RTF             Gazebo real-time factor (default 0 = as fast as possible;
#                   1 with GUI=1). Keeps the 1 ms physics step; faster runs
#                   can scatter because SITL/Gazebo are not fully lockstep.
#   SPEEDUP         SITL --speedup (default 50; it only caps SITL's own pace)
#   IMU_NOISE=<json> add Gaussian IMU noise for this run only (sim/imu_noise.json:
#                   matched to the real aircraft's hover IMU noise). Off by default.
#   GUI=1           open the Gazebo GUI (default: headless server)
#   MAVPROXY=1      also start MAVProxy on tcp:5760 -> udp:14550 (for QGC)
#   WORLD_INSERT=<file>  SDF added to the world (e.g. sim/worlds/hover_camera.sdf)
#   MODEL_INSERT=<file>  SDF added to the aircraft's base_link, this run only
#                   (e.g. sim/worlds/chase_camera_sensor.sdf)
#   HEADLESS_RENDER=1  render camera sensors without a window
#
# SITL MAVLink ports: tcp:5760 (SERIAL0), tcp:5762 (SERIAL1), tcp:5763 (SERIAL2).
# Only one client per port behaves reliably.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${1:-$root/runs/manual}"
mkdir -p "$run_dir"
run_dir="$(cd "$run_dir" && pwd)"
param_file="$(realpath "${PARAMS:-$root/sim/stock_pid.param}")"
arduplane="$root/ardupilot/build/sitl/bin/arduplane"
pid_file="/tmp/quadplane_sitl.pids"

[[ -x "$arduplane" ]] || { echo "not built: $arduplane (cd ardupilot && ./waf configure --board sitl && ./waf plane)" >&2; exit 1; }
[[ -f "$param_file" ]] || { echo "no such param file: $param_file" >&2; exit 1; }

# Stop anything left over from a previous run.
bash "$root/sim/stop_gazebo_sitl.sh" >/dev/null 2>&1 || true
pkill -f "build/sitl/bin/arduplane" >/dev/null 2>&1 || true
pkill -f "gz sim" >/dev/null 2>&1 || true
fuser -k 5760/tcp >/dev/null 2>&1 || true
sleep 1

# World: the template with this run's wind substituted in.
world="$run_dir/world.sdf"
sed -e "s/@WIND_E@/${WIND_E:-0.0}/g" -e "s/@WIND_N@/${WIND_N:-0.0}/g" \
    -e "s/@TURB_MAG@/${TURB_MAG:-0.0}/g" -e "s/@TURB_DIR@/${TURB_DIR:-0.0}/g" \
    -e "s/@TURB_VERT@/${TURB_VERT:-0.0}/g" \
    -e "s/@RTF@/${RTF:-$([[ "${GUI:-0}" == "1" ]] && echo 1.0 || echo 0)}/g" \
    "$root/sim/worlds/quadplane_runway.sdf.in" > "$world"
# WORLD_INSERT=<file>: extra SDF inserted before </world> (e.g. a recording camera)
if [[ -n "${WORLD_INSERT:-}" ]]; then
    python3 - "$world" "$WORLD_INSERT" <<'PY'
import sys
w, ins = sys.argv[1], sys.argv[2]
s = open(w).read()
i = s.rindex("</world>")
open(w, "w").write(s[:i] + open(ins).read() + "\n  " + s[i:])
PY
fi
cp "$param_file" "$run_dir/params.param"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$root/sim/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$root/sim/ardupilot_gazebo/models:$root/sim/ardupilot_gazebo/worlds:$root/sim/gazebo_models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if [[ -n "${IMU_NOISE:-}" ]]; then
    cp "$IMU_NOISE" "$run_dir/imu_noise.json"
    python3 "$root/sim/make_noisy_model.py" "$IMU_NOISE" "$run_dir/models" >/dev/null
    export GZ_SIM_RESOURCE_PATH="$run_dir/models:$GZ_SIM_RESOURCE_PATH"
fi
# MODEL_INSERT=<file>: extra SDF inserted into the aircraft's base_link (e.g. a
# chase camera), in this run's copy of the model only
if [[ -n "${MODEL_INSERT:-}" ]]; then
    python3 - "$root/sim/gazebo_models/VTOL_Quadplane" "$run_dir/models/VTOL_Quadplane" "$MODEL_INSERT" <<'PY'
import os
import sys
from pathlib import Path
src, out, ins = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
out.mkdir(parents=True, exist_ok=True)
for item in src.iterdir():
    if item.name != "model.sdf" and not (out / item.name).exists():
        os.symlink(item, out / item.name)
sdf = (out / "model.sdf") if (out / "model.sdf").exists() else (src / "model.sdf")
s = sdf.read_text()
i = s.index("</link>", s.index('<link name="base_link">'))
i = s.rindex("\n", 0, i) + 1
(out / "model.sdf").write_text(s[:i] + ins.read_text() + s[i:])
PY
    [[ "$GZ_SIM_RESOURCE_PATH" == "$run_dir/models:"* ]] || export GZ_SIM_RESOURCE_PATH="$run_dir/models:$GZ_SIM_RESOURCE_PATH"
fi

gz_flags=(-v2 -r --seed "${SEED:-1}")
[[ "${GUI:-0}" == "1" ]] || gz_flags+=(-s)
# HEADLESS_RENDER=1: render camera sensors without a window (EGL)
[[ "${HEADLESS_RENDER:-0}" == "1" ]] && gz_flags+=(--headless-rendering)
nohup gz sim "${gz_flags[@]}" "$world" >"$run_dir/gz.log" 2>&1 &
gz_pid=$!
sleep 5

(cd "$run_dir" && exec nohup "$arduplane" \
    --model JSON --speedup "${SPEEDUP:-50}" --slave 0 \
    --defaults "$param_file" --sim-address=127.0.0.1 -I0 \
    >"$run_dir/sitl.log" 2>&1) &
sitl_pid=$!

mavproxy_pid=""
if [[ "${MAVPROXY:-0}" == "1" ]]; then
    sleep 3
    nohup mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550 \
        --daemon --non-interactive --nowait >"$run_dir/mavproxy.log" 2>&1 &
    mavproxy_pid=$!
fi

cat > "$pid_file" <<EOF
gz=$gz_pid
sitl=$sitl_pid
mavproxy=$mavproxy_pid
run_dir=$run_dir
EOF

echo "Gazebo PID $gz_pid, SITL PID $sitl_pid${mavproxy_pid:+, MAVProxy PID $mavproxy_pid}"
echo "Run dir: $run_dir (logs in $run_dir/logs)"
