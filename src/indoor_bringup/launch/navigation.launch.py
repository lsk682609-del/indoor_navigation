"""Indoor navigation only. Input odometry positions must already be in map coordinates."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('indoor_bringup')
    args = [
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'indoor_nav.yaml')),
        DeclareLaunchArgument('topo_map_path', default_value=os.path.join(share, 'maps', 'indoor_topology.yaml')),
        DeclareLaunchArgument('scene_world', default_value=os.path.join(share, 'worlds', 'indoor.world')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('localization_odom_topic', default_value=LaunchConfiguration('odom_topic')),
        DeclareLaunchArgument('velocity_odom_topic', default_value=LaunchConfiguration('odom_topic')),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar/pointcloud'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('publish_base_tf', default_value='true'),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('odom_twist_from_pose', default_value='true'),
        DeclareLaunchArgument('strict_frames', default_value='false'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share, 'rviz', 'indoor.rviz')),
    ]
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    nodes = [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='indoor_base_to_footprint',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
        parameters=[{'use_sim_time': sim_time}],
        condition=IfCondition(LaunchConfiguration('publish_base_tf')),
    )]
    for package, executable, name in [
        ('nav2_input_bridge', 'input_bridge_node', 'nav2_input_bridge'),
        ('nav2_net_planner', 'net_planner_action_server', 'net_planner_action_server'),
        ('nav2_net_system', 'net_navigator_node', 'net_navigator'),
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_log_record', 'log_server_node', 'nav2_log_record'),
        ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation'),
    ]:
        overrides = {'use_sim_time': sim_time}
        remaps = []
        if name == 'nav2_input_bridge':
            overrides.update(local_odom_topic=LaunchConfiguration('localization_odom_topic'),
                             lidar_pcl_topic=LaunchConfiguration('cloud_topic'),
                             lidar_frame=LaunchConfiguration('lidar_frame'),
                             publish_odom_tf=ParameterValue(LaunchConfiguration('publish_odom_tf'), value_type=bool),
                             odom_twist_from_pose=ParameterValue(LaunchConfiguration('odom_twist_from_pose'), value_type=bool),
                             strict_frames=ParameterValue(LaunchConfiguration('strict_frames'), value_type=bool))
            remaps += [('/mavros/global_position/set_gp_origin', 'unused_gps/set_gp_origin'),
                       ('/mavros/global_position/gp_origin', 'unused_gps/gp_origin')]
        elif name == 'net_planner_action_server':
            overrides['topo_map_path'] = LaunchConfiguration('topo_map_path')
        elif name == 'net_navigator':
            remaps += [('goal_pose', '/goal_pose')]
        elif name == 'controller_server':
            overrides['odom_topic'] = LaunchConfiguration('velocity_odom_topic')
            remaps += [('cur_lid', '/car1/cur_lid'),
                       ('cmd_vel', LaunchConfiguration('cmd_vel_topic'))]
        nodes.append(Node(package=package, executable=executable, name=name,
                          namespace='car1', output='screen',
                          parameters=[LaunchConfiguration('params_file'), overrides],
                          remappings=remaps))
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'display.launch.py')),
        launch_arguments={key: LaunchConfiguration(key) for key in
                          ['use_sim_time', 'use_rviz', 'topo_map_path', 'scene_world', 'rviz_config']}.items()))
    return LaunchDescription(args + nodes)
