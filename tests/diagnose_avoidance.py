#!/usr/bin/env python3
"""Capture the actual rolling costmap and model poses before an avoidance repair."""
from collections import deque
import json
from pathlib import Path
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import NavigationRequest
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

ROOT = Path(__file__).resolve().parents[1]
rclpy.init()
node = Node('indoor_avoidance_capture')
messages = {}
retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
for kind, topic, key, qos in [
    (Costmap, '/car1/local_costmap/costmap_raw', 'costmap', retained),
    (ModelStates, '/indoor_sim/model_states', 'models', qos_profile_sensor_data),
    (Odometry, '/odom', 'odom', qos_profile_sensor_data),
    (NavPath, '/car1/topology_plan', 'path', retained),
]:
    node.create_subscription(kind, topic, lambda msg, key=key: messages.update({key: msg}), qos)
client = node.create_client(NavigationRequest, '/car1/navigation_request')
if client.wait_for_service(timeout_sec=3):
    request = NavigationRequest.Request()
    request.cmd = 'Cancel'
    future = client.call_async(request)
else:
    future = None
deadline = time.monotonic() + 5
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
if future is not None and future.done():
    print('Current task canceled for repair:', future.result().success)
assert 'costmap' in messages and 'odom' in messages, f'Missing inputs: {list(messages)}'
costmap = messages['costmap']
meta = costmap.metadata
pose = messages['odom'].pose.pose.position
snapshot = dict(width=meta.size_x, height=meta.size_y, resolution=meta.resolution,
                origin=[meta.origin.position.x, meta.origin.position.y],
                robot=[pose.x, pose.y], data=list(costmap.data))
if 'path' in messages:
    snapshot['path'] = [[p.pose.position.x, p.pose.position.y] for p in messages['path'].poses]
if 'models' in messages:
    snapshot['models'] = {name: [p.position.x, p.position.y, p.position.z]
                          for name, p in zip(messages['models'].name, messages['models'].pose)}
output = ROOT / 'results/avoidance_costmap_snapshot.json'
output.write_text(json.dumps(snapshot))
print('Snapshot:', output)
print('Models:', snapshot.get('models'))
print('Robot:', snapshot['robot'], 'costmap origin:', snapshot['origin'])
width, height, resolution = meta.size_x, meta.size_y, meta.resolution
start = (int((pose.x-meta.origin.position.x)/resolution), int((pose.y-meta.origin.position.y)/resolution))
for threshold in [83, 100, 140, 180, 253]:
    reached = {start}
    todo = deque([start])
    while todo:
        x, y = todo.popleft()
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            q = x+dx, y+dy
            if 0 <= q[0] < width and 0 <= q[1] < height and q not in reached and snapshot['data'][q[1]*width+q[0]] < threshold:
                reached.add(q)
                todo.append(q)
    candidates = []
    for x, y in snapshot.get('path', []):
        cell = (int((x-meta.origin.position.x)/resolution), int((y-meta.origin.position.y)/resolution))
        if cell in reached and ((x-pose.x)**2+(y-pose.y)**2)**0.5 > 2:
            candidates.append([round(x,2), round(y,2)])
    print('Threshold', threshold, 'reachable cells:', len(reached), 'forward path targets:', candidates[-3:])
node.destroy_node()
rclpy.shutdown()
