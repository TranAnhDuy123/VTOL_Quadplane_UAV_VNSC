#!/usr/bin/env bash
# Stop the processes started by sim/run_gazebo_sitl.sh. Leaves the run
# directory (logs, world, params) in place.
set -uo pipefail

pid_file="/tmp/quadplane_sitl.pids"

if [[ -f "$pid_file" ]]; then
    source "$pid_file"
    for pid in "${mavproxy:-}" "${sitl:-}" "${gz:-}"; do
        if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" || true
        fi
    done
    rm -f "$pid_file"
fi

# Children that outlive their parent PID (gz spawns server/gui processes).
pkill -f "build/sitl/bin/arduplane" >/dev/null 2>&1 || true
pkill -f "gz sim" >/dev/null 2>&1 || true
pkill -f "mavproxy.py --master=tcp:127.0.0.1:5760" >/dev/null 2>&1 || true
exit 0
