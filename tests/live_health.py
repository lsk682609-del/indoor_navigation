#!/usr/bin/env python3
"""Read-only live startup check: Gazebo pause state and actual input freshness."""
import time

import rclpy
from gazebo_msgs.srv import GetPhysicsProperties
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('indoor_live_health')
counts = {'clock': 0, 'odom': 0, 'raw_cloud': 0, 'cur_lid': 0}


def count(key):
    def receive(_):
        counts[key] += 1
    return receive


for message, topic, key in [(Clock, '/clock', 'clock'), (Odometry, '/odom', 'odom'),
                            (PointCloud2, '/livox/lidar/pointcloud', 'raw_cloud'),
                            (PointCloud2, '/car1/cur_lid', 'cur_lid')]:
    node.create_subscription(message, topic, count(key), qos_profile_sensor_data)
client = node.create_client(GetPhysicsProperties, '/get_physics_properties')
future = None
states = []
for name in ['net_navigator', 'controller_server', 'net_planner_action_server']:
    state_client = node.create_client(GetState, f'/car1/{name}/get_state')
    available = state_client.wait_for_service(timeout_sec=2)
    states.append((name, state_client, state_client.call_async(GetState.Request()) if available else None))
if client.wait_for_service(timeout_sec=2):
    future = client.call_async(GetPhysicsProperties.Request())
deadline = time.monotonic() + 6
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
print('Messages received in 6 s:', counts)
for name, _, state_future in states:
    print(name, 'state:', (state_future.result().current_state.label if state_future is not None and state_future.done() else
                           'service unavailable' if state_future is None else 'request did not respond'))
if future is not None and future.done():
    response = future.result()
    print('Gazebo paused:', response.pause, 'time_step:', response.time_step)
else:
    print('Gazebo physics property service did not respond')
node.destroy_node()
rclpy.shutdown()
