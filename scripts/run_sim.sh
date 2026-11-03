#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
if [[ ! -f "$INDOOR_ROOT/install/indoor_bringup/share/indoor_bringup/package.xml" ]]; then
    echo '请先运行 ./scripts/build.sh' >&2
    exit 1
fi
python3 "$INDOOR_ROOT/tests/check_launch_executables.py" --quiet
mkdir -p "${INDOOR_RESULTS_DIR:-$INDOOR_ROOT/results}"
cd "${INDOOR_RESULTS_DIR:-$INDOOR_ROOT/results}"
indoor_log_link="${INDOOR_RUN_LOG:-$INDOOR_ROOT/results/simulation.log}"
indoor_log_stamp="$(date +%Y%m%d_%H%M%S)_${BASHPID}"
indoor_log_file="${indoor_log_link%.log}_${indoor_log_stamp}.log"
if [[ -f "$indoor_log_link" && ! -L "$indoor_log_link" ]]; then
    mv "$indoor_log_link" "${indoor_log_link%.log}_previous_${indoor_log_stamp}.log"
fi
ln -sfn "$(basename "$indoor_log_file")" "$indoor_log_link"
exec > >(tee -i "$indoor_log_file") 2>&1
exec ros2 launch indoor_bringup simulation.launch.py "$@"
