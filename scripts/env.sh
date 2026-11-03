#!/usr/bin/env bash
# Source this file in a terminal. Build and run scripts source it automatically.
INDOOR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Use system dependencies plus this workspace, never a previously sourced overlay.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
unset GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH
source /opt/ros/humble/setup.bash
if [[ -f /usr/share/gazebo/setup.sh ]]; then
    source /usr/share/gazebo/setup.sh
fi
if [[ "${INDOOR_SKIP_OVERLAY:-0}" != 1 && -f "$INDOOR_ROOT/install/local_setup.bash" ]]; then
    source "$INDOOR_ROOT/install/local_setup.bash"
fi
export ROS_DOMAIN_ID="${INDOOR_ROS_DOMAIN_ID:-88}"
export GAZEBO_MASTER_URI="${INDOOR_GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="${INDOOR_ROS_LOG_DIR:-$INDOOR_ROOT/results/ros_logs}"
mkdir -p "$ROS_LOG_DIR"
