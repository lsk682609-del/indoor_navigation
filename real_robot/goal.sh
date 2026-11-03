#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec python3 "$INDOOR_ROOT/real_robot/send_goal.py" "$@"
