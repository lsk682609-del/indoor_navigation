#!/usr/bin/env python3
"""Resolve the same installed executables as launch_ros, including execute permission."""
import argparse
import sys

from launch import LaunchContext
from launch_ros.substitutions import ExecutableInPackage


def check(quiet=False):
    context = LaunchContext()
    for package, executable in [
        ('indoor_bringup', 'road_network_display.py'),
        ('tf2_ros', 'static_transform_publisher'),
        ('robot_state_publisher', 'robot_state_publisher'),
        ('gazebo_ros', 'spawn_entity.py'),
        ('rviz2', 'rviz2'),
        ('nav2_input_bridge', 'input_bridge_node'),
        ('nav2_net_planner', 'net_planner_action_server'),
        ('nav2_net_system', 'net_navigator_node'),
        ('nav2_controller', 'controller_server'),
        ('nav2_log_record', 'log_server_node'),
        ('nav2_lifecycle_manager', 'lifecycle_manager'),
    ]:
        try:
            ExecutableInPackage(package=package, executable=executable).perform(context)
        except Exception as exc:
            print(f'启动检查失败：{package}/{executable} 未安装或没有可执行权限。\n{exc}', file=sys.stderr)
            if package == 'indoor_bringup':
                print('请先执行：chmod +x src/indoor_bringup/scripts/road_network_display.py', file=sys.stderr)
            print('然后执行：./scripts/build.sh --packages-select indoor_bringup', file=sys.stderr)
            return 1
    if not quiet:
        print('PASS: launch_ros 能找到全部 11 个已安装的可执行程序（包含权限检查）')
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quiet', action='store_true')
    raise SystemExit(check(parser.parse_args().quiet))
