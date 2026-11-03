"""Scout real vehicle; requires a real map topology and localization odometry."""
import os
import importlib.util
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        value = arg(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(name + ' must be true or false')
        return value == 'true'

    share = get_package_share_directory('indoor_bringup')
    spec = importlib.util.spec_from_file_location('indoor_topology_reader', os.path.join(
        share, '..', '..', 'lib', 'indoor_bringup', 'road_network_display.py'))
    display = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(display)
    # Fail before any hardware starts when given an invalid/non-local topology.
    display.read_topology(arg('topo_map_path'))
    with open(arg('hardware_config'), encoding='utf-8') as stream:
        hardware = yaml.safe_load(stream)
    topics, frames = hardware['topics'], hardware['frames']
    if topics['nav_cmd_vel'] == topics['cmd_vel']:
        raise ValueError('nav_cmd_vel must differ from guarded cmd_vel')
    if arg('localization_odom_topic') == topics['wheel_odom']:
        raise ValueError('Localization and wheel odometry must use separate topics')
    if arg('tf_mode') not in ('map_base', 'map_odom', 'bridge'):
        raise ValueError('tf_mode must be map_base, map_odom or bridge')
    for path_arg in ('params_file', 'hardware_config'):
        if not Path(arg(path_arg)).is_file():
            raise ValueError('File missing: ' + arg(path_arg))
    if arg('map_yaml') and not Path(arg('map_yaml')).is_file():
        raise ValueError('Occupancy map YAML missing: ' + arg('map_yaml'))
    nodes = []
    if arg('map_yaml'):
        nodes.extend([
            Node(package='nav2_map_server', executable='map_server', name='real_map_server',
                 parameters=[{'use_sim_time': False, 'yaml_filename': arg('map_yaml'),
                              'frame_id': 'map', 'topic_name': '/map'}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='real_map_lifecycle', parameters=[{
                     'use_sim_time': False, 'autostart': True, 'node_names': ['real_map_server']}]),
        ])
    if enabled('start_localization'):
        loc = hardware['localization']
        source = os.path.join(get_package_share_directory(loc['package']), 'launch', loc['launch'])
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(source),
                     launch_arguments={k: str(v) for k, v in loc.get('arguments', {}).items()}.items()))
    if enabled('start_base'):
        nodes.append(Node(package='scout_base', executable='scout_base_node', name='scout',
                          output='screen', parameters=[dict(hardware['base'],
                              use_sim_time=False, simulated_robot=False,
                              odom_topic_name=topics['wheel_odom'], odom_frame='odom',
                              base_frame='base_link', publish_tf=arg('tf_mode') == 'map_odom')],
                          remappings=[('/cmd_vel', topics['cmd_vel'])]))
    if enabled('start_lidar'):
        # Use a separate launch so the actual JSON network settings are explicit.
        if not Path(arg('lidar_config')).is_file():
            raise ValueError('MID360 network JSON missing: ' + arg('lidar_config'))
        nodes.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                          name='livox_lidar_publisher', output='screen', parameters=[{
                              'use_sim_time': False, 'xfer_format': 1, 'multi_topic': 0,
                              'data_src': 0, 'publish_freq': 10.0, 'output_data_type': 0,
                              'frame_id': frames['lidar'], 'user_config_path': arg('lidar_config'),
                              'cmdline_input_bd_code': 'livox0000000001', 'lvx_file_path': ''}],
                          remappings=[('livox/lidar', topics['custom_cloud'])]))
    if enabled('publish_lidar_tf'):
        ext = hardware['extrinsics']
        values = [part for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw') for part in ('--' + key, str(ext[key]))]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name='real_base_to_lidar', arguments=values + [
                              '--frame-id', 'base_link', '--child-frame-id', frames['lidar']],
                          parameters=[{'use_sim_time': False}]))
    if enabled('convert_livox'):
        nodes.append(Node(package='indoor_bringup', executable='livox_cloud_converter',
                          output='screen', parameters=[{'use_sim_time': False,
                              'input_topic': topics['custom_cloud'],
                              'output_topic': topics['navigation_cloud'], 'sensor_frame': frames['lidar']}]))
    nodes.append(Node(package='indoor_bringup', executable='velocity_guard.py', output='screen',
                      parameters=[dict(hardware['guard'], use_sim_time=False,
                          input_topic=topics['nav_cmd_vel'], output_topic=topics['cmd_vel'],
                          localization_topic='/car1/odom_local', wheel_topic=topics['wheel_odom'],
                          cloud_topic='/car1/cur_lid', lidar_frame=frames['lidar'])]))
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'params_file': arg('params_file'), 'topo_map_path': arg('topo_map_path'),
            'localization_odom_topic': arg('localization_odom_topic'),
            'velocity_odom_topic': topics['wheel_odom'], 'cloud_topic': topics['navigation_cloud'],
            'lidar_frame': frames['lidar'], 'cmd_vel_topic': topics['nav_cmd_vel'],
            'use_sim_time': 'false', 'use_rviz': arg('use_rviz'), 'scene_world': '',
            'publish_odom_tf': str(arg('tf_mode') == 'bridge').lower(),
            'odom_twist_from_pose': 'false', 'strict_frames': 'true',
            'publish_base_tf': arg('publish_base_tf'), 'rviz_config': os.path.join(share, 'rviz', 'real_robot.rviz'),
        }.items()))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('indoor_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('topo_map_path', description='Real local-XYZ topology in localization map coordinates'),
        DeclareLaunchArgument('localization_odom_topic', description='Actual map localization Odometry topic, not wheel odom'),
        DeclareLaunchArgument('hardware_config', default_value=os.path.join(share, 'config', 'scout_hardware.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'real_nav.yaml')),
        DeclareLaunchArgument('lidar_config', default_value='', description='Actual MID360 network JSON path'),
        DeclareLaunchArgument('map_yaml', default_value='', description='Optional real occupancy map for RViz; omit when localizer publishes /map'),
        DeclareLaunchArgument('tf_mode', default_value='map_base',
                              description='map_base: localizer owns map-base; map_odom: localizer map-odom + Scout odom-base; bridge: bridge owns map-base'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_localization', default_value='false'),
        DeclareLaunchArgument('publish_lidar_tf', default_value='true'),
        DeclareLaunchArgument('publish_base_tf', default_value='true'),
        DeclareLaunchArgument('convert_livox', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
