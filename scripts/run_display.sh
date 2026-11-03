#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
python3 "$INDOOR_ROOT/tests/check_launch_executables.py" --quiet
exec ros2 launch indoor_bringup display.launch.py "$@"
