#!/usr/bin/env python3
"""Gate real Scout commands on fresh localization, wheel feedback, cloud and TF."""
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class VelocityGuard(Node):
    def __init__(self):
        super().__init__('velocity_guard')
        defaults = {
            'input_topic': '/car1/nav_cmd_vel', 'output_topic': '/cmd_vel',
            'localization_topic': '/car1/odom_local', 'wheel_topic': '/wheel_odom',
            'cloud_topic': '/car1/cur_lid', 'localization_health_topic': '',
            'global_frame': 'map', 'base_frame': 'base_link', 'lidar_frame': 'livox_frame',
            'command_timeout': 0.25, 'localization_timeout': 0.5,
            'wheel_timeout': 0.5, 'cloud_timeout': 0.5, 'tf_timeout': 0.5,
            'max_linear_speed': 1.0, 'max_angular_speed': 0.8,
        }
        self.p = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        for key in ('command_timeout', 'localization_timeout', 'wheel_timeout',
                    'cloud_timeout', 'tf_timeout', 'max_linear_speed', 'max_angular_speed'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.samples = {}
        self.command = None
        self.reason = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, self.p['output_topic'], 1)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.create_subscription(Twist, self.p['input_topic'], self.on_command, 1)
        self.create_subscription(Odometry, self.p['localization_topic'],
                                 lambda msg: self.on_odom('localization', msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['wheel_topic'],
                                 lambda msg: self.on_odom('wheel', msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.p['cloud_topic'], self.on_cloud, qos_profile_sensor_data)
        if self.p['localization_health_topic']:
            self.create_subscription(Bool, self.p['localization_health_topic'],
                                     self.on_health, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick)

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        self.command = (time.monotonic(), msg) if all(map(math.isfinite, values)) else None

    def record(self, key, msg, valid):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples[key] = (time.monotonic(), stamp, valid)

    def on_odom(self, key, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  v.linear.x, v.linear.y, v.angular.z)
        valid = (all(map(math.isfinite, values)) and
                 abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 0.01 and
                 msg.child_frame_id == self.p['base_frame'] and bool(msg.header.frame_id))
        if key == 'localization':
            valid = valid and msg.header.frame_id == self.p['global_frame']
        self.record(key, msg, valid)

    def on_cloud(self, msg):
        fields = {f.name for f in msg.fields}
        valid = (msg.header.frame_id == self.p['lidar_frame'] and msg.width * msg.height > 0 and
                 {'x', 'y', 'z'} <= fields and msg.point_step > 0 and
                 len(msg.data) >= msg.row_step * msg.height)
        self.record('cloud', msg, valid)

    def on_health(self, msg):
        self.samples['health'] = (time.monotonic(), self.get_clock().now().nanoseconds * 1e-9, msg.data)

    def blocked_reason(self):
        monotonic_now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        keys = ['localization', 'wheel', 'cloud']
        if self.p['localization_health_topic']:
            keys.append('health')
        for key in keys:
            sample = self.samples.get(key)
            limit = self.p['localization_timeout' if key == 'health' else key + '_timeout']
            if sample is None:
                return key + '_missing'
            received, stamp, valid = sample
            if not valid:
                return key + '_invalid'
            if stamp <= 0 or not -0.1 <= ros_now - stamp <= limit or monotonic_now - received > limit:
                return key + '_stale'
        try:
            transform = self.tf_buffer.lookup_transform(self.p['global_frame'], self.p['base_frame'], Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            if stamp <= 0 or not -0.1 <= ros_now - stamp <= self.p['tf_timeout']:
                return 'tf_stale'
        except TransformException:
            return 'tf_missing'
        if self.command is None or monotonic_now - self.command[0] > self.p['command_timeout']:
            return 'command_stale'
        return 'ready'

    def tick(self):
        reason = self.blocked_reason()
        output = Twist()
        if reason == 'ready':
            cmd = self.command[1]
            # Scout Mini differential/skid drive accepts only x and yaw.
            # Scale the pair together: clipping yaw alone changes the turn radius
            # from the trajectory checked by the controller's collision predictor.
            scale = max(1.0, abs(cmd.linear.x) / self.p['max_linear_speed'],
                        abs(cmd.angular.z) / self.p['max_angular_speed'])
            output.linear.x = cmd.linear.x / scale
            output.angular.z = cmd.angular.z / scale
        else:
            # A command received while sensors are invalid may never become a
            # delayed movement when they recover; require a new control command.
            self.command = None
        self.pub.publish(output)
        self.status_pub.publish(String(data=reason))
        if reason != self.reason:
            self.get_logger().info('速度输出状态: ' + reason)
            self.reason = reason

    def stop(self):
        if rclpy.ok():
            self.pub.publish(Twist())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def shutdown_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, shutdown_signal)
    signal.signal(signal.SIGTERM, shutdown_signal)
    node = None
    try:
        node = VelocityGuard()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
