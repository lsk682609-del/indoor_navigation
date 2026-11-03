"""Attach the complete road network and RViz to an already running indoor simulation."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('indoor_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('topo_map_path', default_value=os.path.join(share, 'maps', 'indoor_topology.yaml')),
        DeclareLaunchArgument('scene_world', default_value=os.path.join(share, 'worlds', 'indoor.world')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'indoor.rviz')),
        Node(package='indoor_bringup', executable='road_network_display.py',
             namespace='car1', output='screen', parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'topo_map_path': LaunchConfiguration('topo_map_path'),
                 'scene_world': ParameterValue(LaunchConfiguration('scene_world'), value_type=str)}]),
        Node(package='rviz2', executable='rviz2', name='indoor_map_view',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
