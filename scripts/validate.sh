#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$INDOOR_ROOT"
exec > >(tee -i "$INDOOR_ROOT/results/validation_$(date +%Y%m%d_%H%M%S).log") 2>&1
if [[ "${1:-}" == '--narrow-isolated' ]]; then
    shift
    python3 tests/run_narrow_isolated.py "$@"
    exit
fi
if [[ "${1:-}" == '--avoidance-isolated' ]]; then
    shift
    python3 tests/run_avoidance_isolated.py "$@"
    exit
fi
if [[ "${1:-}" == '--avoidance-peek' ]]; then
    ROS_DOMAIN_ID=189 python3 tests/capture_tracking.py
    exit
fi
if [[ "${1:-}" == '--avoidance-diagnose' ]]; then
    python3 tests/diagnose_avoidance.py
    exit
fi
if [[ "${1:-}" == '--health' ]]; then
    python3 tests/live_health.py
    exit
fi
if [[ "${1:-}" == '--display' ]]; then
    export ROS_DOMAIN_ID=188 INDOOR_ROS_DOMAIN_ID=188
    python3 tests/display_and_goal_test.py
    exit
fi
python3 tests/check_assets.py
python3 tests/planner_matrix.py
if [[ "${1:-}" == '--avoidance-regression' ]]; then
    indoor_report_dir="${INDOOR_TEST_REPORT_DIR:-$INDOOR_ROOT/results}"
    python3 tests/geometry_clearance.py
    python3 tests/navigation_smoke.py --obstacle --existing-obstacle unit_box --box-size 1.0 \
      --start-near 9 0 --obstacle-goal 0 4 --timeout 180 --report-dir "$indoor_report_dir"
    python3 tests/navigation_smoke.py --obstacle --existing-obstacle unit_box --box-size 1.0 \
      --start-near 0 4 --obstacle-goal 0 0 --timeout 120 --report-dir "$indoor_report_dir"
    python3 tests/navigation_smoke.py --obstacle --timeout 120 --report-dir "$indoor_report_dir"
elif [[ "${1:-}" == '--all' ]]; then
    python3 tests/navigation_smoke.py --obstacle
    python3 tests/navigation_smoke.py --goal 0 0
    python3 tests/navigation_smoke.py
else
    python3 tests/navigation_smoke.py "$@"
fi
