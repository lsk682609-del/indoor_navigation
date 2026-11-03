#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$INDOOR_ROOT"
python3 tests/check_assets.py
python3 tests/check_launch_executables.py
ros2 launch indoor_bringup simulation.launch.py --show-args
for indoor_package in indoor_bringup nav2_msgs nav2_net_planner nav2_a_star_rpp vehicle_gazebo_sim ros2_livox_simulation; do
    indoor_prefix="$(ros2 pkg prefix "$indoor_package")"
    if [[ "$indoor_prefix" != "$INDOOR_ROOT/install/"* ]]; then
        echo "错误：$indoor_package 来自其他工作空间：$indoor_prefix" >&2
        exit 1
    fi
done
echo 'PASS: 自定义导航与模型包均来自本目录'
