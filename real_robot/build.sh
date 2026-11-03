#!/usr/bin/env bash
set -eo pipefail
INDOOR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL="${INDOOR_BUILD_JOBS:-4}"
export MAKEFLAGS="-j${INDOOR_BUILD_JOBS:-4}"
if [[ -f "$INDOOR_ROOT/real_robot/deps/usr/include/asio.hpp" ]]; then
    export CPLUS_INCLUDE_PATH="$INDOOR_ROOT/real_robot/deps/usr/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
elif [[ ! -f /usr/include/asio.hpp ]]; then
    echo '缺少 libasio-dev（Scout SDK 编译依赖）；安装该包后重新构建。' >&2
    exit 1
fi
cd "$INDOOR_ROOT/real_robot/drivers_ws"
colcon build --base-paths src --symlink-install --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/local_setup.bash
cd "$INDOOR_ROOT"
# Exclude the simulation message-only Livox package; use the full driver underlay.
colcon --log-base log_real build --base-paths src/navigation2 src/indoor_bringup \
    --build-base build_real --install-base install_real --symlink-install --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DUSE_SYSTEM_FMT=ON "$@"
