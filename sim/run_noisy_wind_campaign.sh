#!/usr/bin/env bash
# Rerun the same square plan and seeded 6/9 m/s moment-pulse cases with the
# calibrated Gazebo IMU sensor noise. Never overwrite the original campaign.
# Usage: bash sim/run_noisy_wind_campaign.sh [seeds...]  (default: 1 2 3)
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RUNS_DIR="${RUNS_DIR:-$root/runs/noisy_wind}"
export CASES="w6 w9"
export IMU_NOISE="${IMU_NOISE:-$root/sim/imu_noise_matched.json}"
export GUI=0
export RTF="${RTF:-0}"
export SPEEDUP="${SPEEDUP:-50}"
echo "physics: 1 ms step, headless, RTF=$RTF (0 = uncapped), SITL speedup=$SPEEDUP"
echo "noise: $IMU_NOISE; outputs: $RUNS_DIR"
bash "$root/sim/run_campaign.sh" "$@"