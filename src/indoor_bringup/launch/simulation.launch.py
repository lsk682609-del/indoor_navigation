"""One entry point for the self-contained indoor world, original vehicle and navigation."""
import os
from pathlib import Path
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def instance_conflicts(proc_root=Path('/proc'), domain=None, master_uri=None, own_pid=None):
    """Catch both a live server and a navigation stack left after its server died."""
    domain = str(domain if domain is not None else os.environ.get('ROS_DOMAIN_ID', '0'))
    master_uri = master_uri or os.environ.get('GAZEBO_MASTER_URI', 'http://localhost:11345')
    own_pid = os.getpid() if own_pid is None else own_pid
    master = urlparse(master_uri)
    conflicts = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            args = (entry / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            exe = Path(args[0]).name if args and args[0] else ''
            is_launch = ('launch' in args and 'indoor_bringup' in args and 'simulation.launch.py' in args)
            is_navigator = (exe == 'net_navigator_node' and '__ns:=/car1' in args)
            if not (is_launch or is_navigator or exe == 'gzserver'):
                continue
            env = {}
            for item in (entry / 'environ').read_bytes().split(b'\0'):
                if item.startswith((b'ROS_DOMAIN_ID=', b'GAZEBO_MASTER_URI=')):
                    key, value = item.decode(errors='replace').split('=', 1)
                    env[key] = value
            other_master = urlparse(env.get('GAZEBO_MASTER_URI', 'http://localhost:11345'))
            local_hosts = {'localhost', '127.0.0.1', '::1'}
            same_host = (master.hostname == other_master.hostname or
                         {master.hostname, other_master.hostname} <= local_hosts)
            same_master = same_host and (master.port or 11345) == (other_master.port or 11345)
            if ((is_launch or is_navigator) and env.get('ROS_DOMAIN_ID', '0') == domain) or (exe == 'gzserver' and same_master):
                conflicts.append((int(entry.name), ' '.join(arg for arg in args if arg)))
        except (OSError, ValueError):
            continue
    return conflicts


def guard_simulation(context):
    conflicts = instance_conflicts()
    if conflicts:
        details = '\n'.join(f'  PID {pid}: {cmd}' for pid, cmd in conflicts)
        raise RuntimeError('检测到同 ROS 域的室内仿真/导航或占用当前端口的 Gazebo：\n' + details +
                           '\n请先在旧启动终端按 Ctrl+C 退出完整实例，再启动；单独关闭 Gazebo 窗口不会结束旧导航。')
    return []


def generate_launch_description():
    share = get_package_share_directory('indoor_bringup')
    vehicle = get_package_share_directory('vehicle_gazebo_sim')
    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'indoor.world')),
        DeclareLaunchArgument('topo_map_path', default_value=os.path.join(share, 'maps', 'indoor_topology.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'indoor_nav.yaml')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vehicle, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'),
            'rviz': 'false', 'use_sim_time': 'true',
            'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'topo_map_path': LaunchConfiguration('topo_map_path'),
            'scene_world': LaunchConfiguration('world'),
            'use_sim_time': 'true', 'use_rviz': LaunchConfiguration('use_rviz'),
        }.items())
    return LaunchDescription(args + [OpaqueFunction(function=guard_simulation), simulation,
                                    TimerAction(period=5.0, actions=[navigation])])
