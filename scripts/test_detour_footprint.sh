#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$INDOOR_ROOT"
export ROS_DOMAIN_ID=188
cmake -S tests/theta_star_regression -B build/theta_star_regression -DCMAKE_BUILD_TYPE=Debug
cmake --build build/theta_star_regression --target detour_footprint_regression -j4
./build/theta_star_regression/detour_footprint_regression
