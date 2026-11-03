#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$INDOOR_ROOT"
cmake -S tests/theta_star_regression -B build/theta_star_regression -DCMAKE_BUILD_TYPE=Debug
cmake --build build/theta_star_regression -j4
ASAN_OPTIONS=detect_leaks=0:abort_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
  ./build/theta_star_regression/theta_star_regression
