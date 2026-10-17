#!/usr/bin/env bash
# One unattended run: start Gazebo + SITL, fly a mission plan in AUTO
# (default sim/square.plan: VTOL take-off -> transition -> square circuit ->
# VTOL land), stop the sim, and copy the DataFlash log out.
# PLAN=<file.plan> flies another plan (e.g. "sim/Default square zigzag path.plan").
#
#   sim/run_square_mission.sh <run-name>
#
# Output: runs/<run-name>/ with world.sdf, params.param, gz/sitl logs,
# mission.log, mission_status.json, logs/*.BIN, and flight.BIN (the log).
# Same environment variables as run_gazebo_sitl.sh (PARAMS, WIND_E, SEED, GUI, ...),
# plus DISTURB_NM: size of the roll/pitch moment pulses in N.m (default 0 = none).
set -euo pipefail

name="${1:?usage: sim/run_square_mission.sh <run-name>}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo "invalid run name: $name" >&2; exit 1; }
run_dir="${RUNS_DIR:-$root/runs}/$name"
rm -rf "$run_dir"
mkdir -p "$run_dir"

rec_pid=""
stop_rec() { [[ -n "$rec_pid" ]] && { kill -TERM "$rec_pid" 2>/dev/null; wait "$rec_pid" 2>/dev/null || true; rec_pid=""; }; return 0; }
trap 'stop_rec; bash "$root/sim/stop_gazebo_sitl.sh"' EXIT

MAVPROXY=0 bash "$root/sim/run_gazebo_sitl.sh" "$run_dir"
# RECORD_TOPIC=<camera topic>: save its frames, the pose and the torque events
# to <run>/rec/ with sim/record_camera.py (needs a camera, see run_fw_recording.sh)
if [[ -n "${RECORD_TOPIC:-}" ]]; then
    mkdir -p "$run_dir/rec"
    /usr/bin/python3 -u "$root/sim/record_camera.py" "$run_dir/rec" "$RECORD_TOPIC" "${RECORD_QUALITY:-90}" \
        > "$run_dir/rec/record.log" 2>&1 &
    rec_pid=$!
fi
sleep 10

python3 -u "$root/sim/fly_plan_mission.py" \
    --connect tcp:127.0.0.1:5760 \
    --plan "${PLAN:-$root/sim/square.plan}" \
    --case "$name" \
    --status-out "$run_dir/mission_status.json" \
    --disturb-nm "${DISTURB_NM:-0}" \
    2>&1 | tee "$run_dir/mission.log"

python3 - "$run_dir/mission_status.json" "${DISTURB_NM:-0}" <<'PY'
import json
import sys

status = json.load(open(sys.argv[1]))
pulses = status.get("pulses", [])
windows = [w["window"] for w in status.get("windows", [])]
if not (status.get("completed") and status.get("transitioned")
        and status.get("waypoints_reached", 0) >= status.get("n_items", 6)):
    raise SystemExit("incomplete mission: " + str(status.get("reason")))
if float(sys.argv[2]) > 0:
    need = {"hover_takeoff", "transition", "hover_land"}
    fw = [w for w in windows if w.startswith(("corner_", "mid_"))]
    if not need <= set(windows) or not fw:
        raise SystemExit("pulse windows missing: have %s" % windows)
    if len(pulses) != 4 * len(windows) or not all(p["ok"] for p in pulses):
        raise SystemExit("missing or failed moment pulses (%d/%d)" % (len(pulses), 4 * len(windows)))
PY

# Let the logger flush after disarm before killing SITL.
sleep 5
stop_rec
bash "$root/sim/stop_gazebo_sitl.sh"
trap - EXIT
sleep 2

bin=$(ls -t "$run_dir"/logs/*.BIN 2>/dev/null | head -1 || true)
if [[ -n "$bin" ]]; then
    cp -f "$bin" "$run_dir/flight.BIN"
    echo "[$name] log: $run_dir/flight.BIN ($(du -h "$bin" | cut -f1))"
else
    echo "[$name] WARNING: no .BIN produced" >&2
    exit 1
fi
