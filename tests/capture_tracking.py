#!/usr/bin/env python3
"""Read-only path/heading snapshot for the isolated avoidance regression."""
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

rclpy.init()
node=Node('avoidance_tracking_capture')
data={}
for topic in ['theta_star_path_raw','received_global_plan','topology_plan']:
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic!='received_global_plan' else 10
    def receive(msg, topic=topic):
        data[topic]={'frame':msg.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in msg.poses]}
    node.create_subscription(NavPath,'/car1/'+topic,receive,qos)
def odom(msg):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    data['robot']=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
node.create_subscription(PointStamped,'/car1/lookahead_point',lambda m:data.update(carrot=[m.point.x,m.point.y]),10)
deadline=time.monotonic()+3
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
root=Path(__file__).resolve().parents[1]
(root/'results/isolated_avoidance/tracking.json').write_text(json.dumps(data))
for key,value in data.items():
    if isinstance(value,dict):
        points=value['points'];print(key,value['frame'],len(points), 'start:',points[:8], 'end:',points[-3:])
    else:print(key,value)
node.destroy_node();rclpy.shutdown()
