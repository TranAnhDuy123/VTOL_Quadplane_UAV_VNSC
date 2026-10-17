#!/usr/bin/env bash
# One system-identification flight (calm air): start Gazebo + SITL, fly
# sim/fly_sysid.py, stop, copy the log to runs/<run-name>/flight.BIN.
#
#   sim/run_sysid.sh <run-name> hover|fw
#
# Same environment variables as run_gazebo_sitl.sh (PARAMS, ...).
set -euo pipefail

name="${1:?usage: sim/run_sysid.sh <run-name> hover|fw}"
phase="${2:?usage: sim/run_sysid.sh <run-name> hover|fw}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="$root/runs/$name"
rm -rf "$run_dir"
mkdir -p "$run_dir"

trap 'bash "$root/sim/stop_gazebo_sitl.sh"' EXIT

MAVPROXY=0 bash "$root/sim/run_gazebo_sitl.sh" "$run_dir"
sleep 10

python3 -u "$root/sim/fly_sysid.py" --phase "$phase" 2>&1 | tee "$run_dir/mission.log"

sleep 5
bash "$root/sim/stop_gazebo_sitl.sh"
trap - EXIT
sleep 2

bin=$(ls -t "$run_dir"/logs/*.BIN 2>/dev/null | head -1 || true)
[[ -n "$bin" ]] || { echo "[$name] no .BIN produced" >&2; exit 1; }
cp -f "$bin" "$run_dir/flight.BIN"
echo "[$name] log: $run_dir/flight.BIN ($(du -h "$bin" | cut -f1))"
