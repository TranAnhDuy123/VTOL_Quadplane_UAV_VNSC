#!/usr/bin/env bash
# PID vs INDI (joint motor/surface allocation) on both mission profiles,
# with the calibrated IMU noise, one seed.
#
#   bash sim/run_joint_campaign.sh [seeds...]      (default seed: 1)
#
# Output: runs/joint/<profile>/<ctrl>_<case>_s<seed>/ for profile square, zigzag;
# cases calm / w6 / w9 as in sim/run_campaign.sh. Finished runs are skipped.
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
seeds=("$@")
[[ ${#seeds[@]} -gt 0 ]] || seeds=(1)
export IMU_NOISE="${IMU_NOISE:-$root/sim/imu_noise_matched.json}"
export GUI=0 RTF="${RTF:-0}" SPEEDUP="${SPEEDUP:-50}"
failures=0
for profile in square zigzag; do
    case "$profile" in
        square) plan="$root/sim/square.plan" ;;
        zigzag) plan="$root/sim/Default square zigzag path.plan" ;;
    esac
    echo "== $profile: $plan"
    PLAN="$plan" RUNS_DIR="${OUT_ROOT:-$root/runs/joint}/$profile" CASES="calm w6 w9" \
        bash "$root/sim/run_campaign.sh" "${seeds[@]}" || failures=$((failures + 1))
done
(( failures == 0 )) || { echo "$failures profile(s) had failed missions" >&2; exit 1; }
