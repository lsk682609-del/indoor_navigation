#!/usr/bin/env bash
set -eo pipefail
INDOOR_SCENARIO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$INDOOR_SCENARIO_ROOT/scripts/run_sim.sh" \
  "world:=$INDOOR_SCENARIO_ROOT/src/indoor_bringup/worlds/indoor_obstacle.world" \
  x:=9.0 y:=0.0 yaw:=3.141592653589793 "$@"
