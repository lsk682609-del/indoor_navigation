#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$INDOOR_ROOT"
exec python3 tests/navigation_smoke.py "$@"
