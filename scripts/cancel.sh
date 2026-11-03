#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec ros2 service call /car1/navigation_request nav2_msgs/srv/NavigationRequest \
  '{command_id: indoor_cancel, cmd: Cancel, target_pose: {}}'
