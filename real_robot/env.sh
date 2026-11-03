#!/usr/bin/env bash
# Source in every real-vehicle terminal. Isolated from simulation domain 88.
INDOOR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
source /opt/ros/humble/setup.bash
if [[ -n "${INDOOR_LOCALIZATION_SETUP:-}" ]]; then
    if [[ ! -f "$INDOOR_LOCALIZATION_SETUP" ]]; then
        echo "定位工作空间不存在：$INDOOR_LOCALIZATION_SETUP" >&2
        return 1
    fi
    source "$INDOOR_LOCALIZATION_SETUP"
fi
for indoor_setup in "$INDOOR_ROOT/real_robot/drivers_ws/install/local_setup.bash" "$INDOOR_ROOT/install_real/local_setup.bash"; do
    if [[ ! -f "$indoor_setup" ]]; then
        echo '请先运行 ./real_robot/build.sh' >&2
        return 1
    fi
    source "$indoor_setup"
done
export ROS_DOMAIN_ID="${INDOOR_REAL_DOMAIN_ID:-89}"
export ROS_LOCALHOST_ONLY="${INDOOR_REAL_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="$INDOOR_ROOT/results/real_robot/ros_logs"
mkdir -p "$ROS_LOG_DIR"
