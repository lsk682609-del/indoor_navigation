#!/usr/bin/env bash
set -eo pipefail
INDOOR_SKIP_OVERLAY=1 source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$INDOOR_ROOT"
export CMAKE_BUILD_PARALLEL_LEVEL="${INDOOR_BUILD_JOBS:-4}"
export MAKEFLAGS="-j${INDOOR_BUILD_JOBS:-4}"
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DUSE_SYSTEM_FMT=ON "$@"
