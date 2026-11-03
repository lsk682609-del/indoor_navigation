#!/usr/bin/env python3
"""Exercise the real launch with a kinematic plant and synthetic sensors, no hardware."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav2_msgs.msg import NavigationStatus
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster


def main():
    root = Path(__file__).resolve().parents[1]
    output = root / 'results/real_robot/navigation_test'
    output.mkdir(parents=True, exist_ok=True)
    hardware = yaml.safe_load((root / 'src/indoor_bringup/config/scout_hardware.yaml').read_text())
    hardware['topics'].update(wheel_odom='/real_test/wheel', custom_cloud='/real_test/custom',
                              navigation_cloud='/real_test/cloud', nav_cmd_vel='/real_test/input',
                              cmd_vel='/real_test/output')
    profile = output / 'hardware.yaml'
    profile.write_text(yaml.safe_dump(hardware))
    topology = output / 'topology.yaml'
    topology.write_text('map:\n0 synthetic_corridor\nlla_mode:\n0\nnode:\n'
                        '0 0 0 0 0 0 nan nan nan\n1 0 0 3 0 0 nan nan nan\n'
                        'edge:\n0 0 0 0 1 0\n')
    rclpy.init()
    node = rclpy.create_node('real_navigation_test')
    plant = dict(x=0.0, y=0.0, yaw=0.0, speed=0.0, angular=0.0, status=-1,
                 max_speed=0.0, motion=False)
    def command(msg):
        plant.update(speed=msg.linear.x, angular=msg.angular.z,
                     max_speed=max(plant['max_speed'], abs(msg.linear.x)))
        plant['motion'] = plant['motion'] or abs(msg.linear.x) > 0.05
    node.create_subscription(Twist, '/real_test/output', command, 10)
    node.create_subscription(NavigationStatus, '/car1/navigation_status',
                             lambda msg: plant.__setitem__('status', msg.state),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pose_pub = node.create_publisher(Odometry, '/real_test/localization', 10)
    wheel_pub = node.create_publisher(Odometry, '/real_test/wheel', 10)
    cloud_pub = node.create_publisher(CustomMsg, '/real_test/custom', 10)
    tf_pub = TransformBroadcaster(node)
    previous = time.monotonic()
    def step():
        nonlocal previous
        now = time.monotonic()
        dt, previous = min(now - previous, 0.05), now
        plant['x'] += plant['speed'] * math.cos(plant['yaw']) * dt
        plant['y'] += plant['speed'] * math.sin(plant['yaw']) * dt
        plant['yaw'] += plant['angular'] * dt
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'map', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = plant['x'], plant['y']
        odom.pose.pose.orientation.z = math.sin(plant['yaw']/2)
        odom.pose.pose.orientation.w = math.cos(plant['yaw']/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = plant['speed'], plant['angular']
        pose_pub.publish(odom)
        odom.header.frame_id = 'odom'
        wheel_pub.publish(odom)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'map', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = plant['x'], plant['y']
        tf.transform.rotation = odom.pose.pose.orientation
        tf_pub.sendTransform(tf)
        cloud = CustomMsg()
        cloud.header.stamp, cloud.header.frame_id = stamp, 'livox_frame'
        # Clear test corridor; distant returns still exercise CustomMsg conversion.
        cloud.points = [CustomPoint(x=10.0, y=10.0, z=1.0, reflectivity=20)]
        cloud.point_num = 1
        cloud.timebase = stamp.sec * 1000000000 + stamp.nanosec
        cloud_pub.publish(cloud)
    node.create_timer(0.025, step)
    process = None
    stream = (output/'launch.log').open('w')
    try:
        process = subprocess.Popen(['ros2', 'launch', 'indoor_bringup', 'real_robot.launch.py',
                    'topo_map_path:=' + str(topology), 'hardware_config:=' + str(profile),
                    'localization_odom_topic:=/real_test/localization',
                    'start_base:=false', 'start_lidar:=false', 'use_rviz:=false'],
                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        state_client = node.create_client(GetState, '/car1/net_navigator/get_state')
        deadline, active = time.monotonic() + 30, False
        while time.monotonic() < deadline and not active:
            rclpy.spin_once(node, timeout_sec=0.025)
            if process.poll() is not None:
                raise RuntimeError('Launch exited before navigation became active')
            if state_client.service_is_ready():
                future = state_client.call_async(GetState.Request())
                while not future.done() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.025)
                active = future.done() and future.result().current_state.id == 3
        if not active:
            raise RuntimeError('Navigation activation timeout')
        print('PASS: real launch activates navigation with synthetic localization and sensors', flush=True)
        sender = subprocess.Popen(['python3', str(root/'real_robot/send_goal.py'), '3', '0'],
                                  stdout=stream, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and plant['status'] != NavigationStatus.SUCCEEDED:
            rclpy.spin_once(node, timeout_sec=0.025)
            if plant['status'] == NavigationStatus.FAILED:
                raise RuntimeError('Navigation failed')
        if sender.wait(timeout=5) != 0:
            raise RuntimeError('Goal helper rejected goal')
        error = math.hypot(plant['x']-3, plant['y'])
        assert plant['status'] == NavigationStatus.SUCCEEDED, 'goal did not succeed'
        assert error < 0.3, 'plant did not reach goal'
        assert plant['motion'] and plant['max_speed'] <= 1.00001, 'guard speed contract failed'
        result = f'PASS: goal accepted, SUCCEEDED, final error={error:.3f} m, maximum speed={plant["max_speed"]:.3f} m/s'
        print(result, flush=True)
        (output/'result.txt').write_text(result+'\nNo hardware drivers started.\n')
        return 0
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
