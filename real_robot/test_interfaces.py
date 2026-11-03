#!/usr/bin/env python3
"""ROS integration tests with synthetic data; never start a hardware driver.

All test command topics use /real_test, including the guard's output.
Run with an isolated ROS_DOMAIN_ID after sourcing real_robot/env.sh.
"""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = rclpy.create_node('real_interface_test')
    processes, files, checks = [], [], []
    output = Path(__file__).resolve().parents[1] / 'results/real_robot/interface_tests'
    output.mkdir(parents=True, exist_ok=True)

    def start(package, executable, parameters):
        log = (output / (executable.replace('.py', '') + '.log')).open('w')
        files.append(log)
        cmd = ['ros2', 'run', package, executable, '--ros-args']
        for key, value in parameters.items():
            cmd += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
        processes.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

    def check(condition, label):
        if not condition:
            raise AssertionError(label)
        checks.append(label)
        print('PASS:', label, flush=True)

    received = {'cloud': None, 'odom': None, 'command': None, 'status': ''}
    dynamic_frames = []
    def save(key):
        return lambda msg: received.__setitem__(key, msg)
    node.create_subscription(PointCloud2, '/real_test/filtered', save('cloud'), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/real_test/pose', save('odom'), qos_profile_sensor_data)
    node.create_subscription(Twist, '/real_test/output', save('command'), 10)
    node.create_subscription(String, '/velocity_guard/status',
                             lambda msg: received.__setitem__('status', msg.data), 10)
    node.create_subscription(TFMessage, '/tf',
                             lambda msg: dynamic_frames.extend(t.child_frame_id for t in msg.transforms), 10)
    cloud_pub = node.create_publisher(CustomMsg, '/real_test/custom', 10)
    pose_pub = node.create_publisher(Odometry, '/real_test/raw_pose', 10)
    wheel_pub = node.create_publisher(Odometry, '/real_test/wheel', 10)
    cmd_pub = node.create_publisher(Twist, '/real_test/input', 10)
    static_tf, dynamic_tf = StaticTransformBroadcaster(node), TransformBroadcaster(node)
    static = TransformStamped()
    static.header.stamp = node.get_clock().now().to_msg()
    static.header.frame_id, static.child_frame_id = 'map', 'tracking_frame'
    static.transform.translation.x, static.transform.translation.y = 10.0, 20.0
    static.transform.rotation.z = math.sin(math.pi / 4)
    static.transform.rotation.w = math.cos(math.pi / 4)
    sensor = TransformStamped()
    sensor.header.stamp, sensor.header.frame_id, sensor.child_frame_id = static.header.stamp, 'base_link', 'test_sensor'
    sensor.transform.translation.x, sensor.transform.rotation.w = 0.16, 1.0
    static_tf.sendTransform([static, sensor])
    flags = dict(cloud=True, pose=True, wheel=True, tf=True, command=True,
                 frame='livox_frame', source='tracking_frame', child='base_link', future=False, empty=False, nan=False)

    def feed():
        stamp = node.get_clock().now().to_msg()
        if flags['tf']:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'map', 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = 10.0, 21.0
            tf.transform.rotation.z = math.sin(math.pi / 4)
            tf.transform.rotation.w = math.cos(math.pi / 4)
            dynamic_tf.sendTransform(tf)
        if flags['pose']:
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp, flags['source']
            if flags['future']:
                odom.header.stamp.sec += 5
            odom.child_frame_id = flags['child']
            odom.pose.pose.position.x = 1.16 if flags['child'] == 'test_sensor' else 1.0
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = 0.6
            odom.twist.twist.angular.z = 0.2
            pose_pub.publish(odom)
        if flags['wheel']:
            wheel = Odometry()
            wheel.header.stamp, wheel.header.frame_id, wheel.child_frame_id = stamp, 'odom', 'base_link'
            wheel.pose.pose.orientation.w = 1.0
            wheel.twist.twist.linear.x = 0.4
            wheel_pub.publish(wheel)
        if flags['cloud']:
            cloud = CustomMsg()
            cloud.header.stamp, cloud.header.frame_id = stamp, flags['frame']
            cloud.timebase = stamp.sec * 1000000000 + stamp.nanosec
            if not flags['empty']:
                cloud.points = [CustomPoint(x=2.0, y=0.0, z=0.6, reflectivity=37),
                                CustomPoint(x=float('nan'), y=0.0, z=0.6)]
            cloud.point_num = len(cloud.points)
            cloud_pub.publish(cloud)
        if flags['command']:
            command = Twist()
            command.linear.x = float('nan') if flags['nan'] else 2.0
            command.angular.z = 1.5
            cmd_pub.publish(command)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            feed()
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A component exited; inspect ' + str(output))

    def stopped():
        cmd = received['command']
        return cmd is not None and cmd.linear.x == 0.0 and cmd.angular.z == 0.0

    try:
        start('indoor_bringup', 'livox_cloud_converter', {
            'input_topic': '/real_test/custom', 'output_topic': '/real_test/pointcloud'})
        start('nav2_input_bridge', 'input_bridge_node', {
            'local_odom_topic': '/real_test/raw_pose', 'odom_output_topic': '/real_test/pose',
            'lidar_pcl_topic': '/real_test/pointcloud', 'lidar_output_topic': '/real_test/filtered',
            'lidar_frame': 'livox_frame', 'strict_frames': True,
            'publish_odom_tf': False, 'odom_twist_from_pose': False})
        start('indoor_bringup', 'velocity_guard.py', {
            'input_topic': '/real_test/input', 'output_topic': '/real_test/output',
            'localization_topic': '/real_test/pose', 'wheel_topic': '/real_test/wheel',
            'cloud_topic': '/real_test/filtered'})
        configure = node.create_client(ChangeState, '/nav2_input_bridge/change_state')
        if not configure.wait_for_service(timeout_sec=15):
            raise RuntimeError('Bridge lifecycle unavailable')
        request = ChangeState.Request()
        request.transition.id = 1
        future = configure.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        check(future.done() and future.result().success, 'bridge configures')
        flags['tf'] = False
        pump(0.5)
        check('base_link' not in dynamic_frames, 'bridge TF disabled without duplicate base transform')
        flags['tf'] = True
        flags['pose'] = False
        pump(1.0)
        check(stopped(), 'missing localization blocks commands')
        flags['pose'] = True
        pump(1.0)
        cloud, odom = received['cloud'], received['odom']
        check(cloud is not None and cloud.width == 1 and cloud.header.frame_id == 'livox_frame',
              'CustomMsg converts and filters NaN without relabeling frame')
        check(struct.unpack_from('<ffff', bytes(cloud.data)) == (2.0, 0.0, struct.unpack('<f', struct.pack('<f', 0.6))[0], 37.0),
              'XYZ and reflectivity survive conversion')
        age = node.get_clock().now().nanoseconds * 1e-9 - (cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9)
        check(0 <= age < 0.3, 'measurement timestamp preserved')
        check(odom is not None and abs(odom.pose.pose.position.x - 10) < 1e-5 and
              abs(odom.pose.pose.position.y - 21) < 1e-5, 'pose uses actual map transform')
        check(abs(odom.twist.twist.linear.x - 0.6) < 1e-5 and
              abs(odom.twist.twist.angular.z - 0.2) < 1e-5, 'input body twist preserved')
        check(received['status'] == 'ready' and abs(received['command'].linear.x - 1) < 1e-5 and
              abs(received['command'].angular.z - 0.75) < 1e-5, 'fresh data enables bounded commands preserving turn curvature')
        flags['child'] = 'test_sensor'
        pump(0.4)
        check(abs(received['odom'].pose.pose.position.x - 10) < 1e-5 and
              abs(received['odom'].pose.pose.position.y - 21) < 1e-5,
              'sensor reference pose converts through base extrinsic')
        flags['child'] = 'base_link'
        for key in ('command', 'cloud', 'wheel', 'pose', 'tf'):
            flags[key] = False
            pump(0.85)
            check(stopped(), key + ' timeout stops motion')
            flags[key] = True
            pump(0.35)
            check(received['status'] == 'ready', key + ' recovery accepts fresh command')
        flags['frame'] = 'world'
        pump(0.85)
        check(stopped(), 'wrong sensor frame never masquerades as sensor coordinates')
        flags['frame'] = 'livox_frame'
        flags['source'] = 'unknown_world'
        pump(0.85)
        check(stopped(), 'unknown localization frame is rejected')
        flags['source'] = 'tracking_frame'
        flags['future'] = True
        pump(0.35)
        check(stopped(), 'future localization timestamp stops motion')
        flags['future'] = False
        flags['empty'] = True
        pump(0.35)
        check(stopped(), 'empty cloud stops motion')
        flags['empty'] = False
        flags['nan'] = True
        pump(0.35)
        check(stopped(), 'non-finite command rejected')
        flags['nan'] = False
        pump(0.35)
        check(received['status'] == 'ready', 'valid inputs restore control')
        # SIGINT must publish a final zero while the ROS context is still alive.
        os.killpg(processes[-1].pid, signal.SIGINT)
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        check(stopped(), 'guard shutdown sends zero command')
        print(f'{len(checks)} checks passed; no hardware drivers started.', flush=True)
        (output / 'result.txt').write_text('\n'.join(checks) + '\n')
        return 0
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        for stream in files:
            stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
