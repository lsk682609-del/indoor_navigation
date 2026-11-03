#!/usr/bin/env python3
"""Compare simulated lidar hits with the independently known gate surfaces before motion."""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--world',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
world=ET.parse(args.world).getroot().find('world')
sizes={name:tuple(map(float,world.find(f"model[@name='{name}']/link/collision/geometry/box/size").text.split()))
       for name in ['unit_box_1','unit_box_2']}
rclpy.init();node=Node('lidar_gate_geometry');models={};errors=[]
def model_cb(msg):
    models.update(zip(msg.name,msg.pose))
def cloud_cb(msg):
    if not all(name in models for name in ['simple_car',*sizes]):return
    car=models['simple_car'];q=car.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    if abs(car.position.x-8)>0.05 or abs(car.position.y)>0.05:return
    c,s=math.cos(yaw),math.sin(yaw)
    for x,y,z in point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True):
        # Ground-reference extrinsic from the actual URDF: (0.24,0,0.30).
        gx=car.position.x+c*(x+.24)-s*y;gy=car.position.y+s*(x+.24)+c*y;gz=car.position.z+z+.30
        if not (9.2<gx<10.8 and abs(gy)<1.8 and .4<gz<.9):continue
        distances=[]
        for name,(sx,sy,sz) in sizes.items():
            p=models[name].position
            d=[abs(gx-p.x)-sx/2,abs(gy-p.y)-sy/2,abs(gz-p.z)-sz/2]
            distances.append(math.sqrt(sum(max(0.,v)**2 for v in d)) if max(d)>0 else -max(d))
        errors.append(min(distances))
node.create_subscription(ModelStates,'/indoor_sim/model_states',model_cb,10)
node.create_subscription(PointCloud2,'/livox/lidar/pointcloud',cloud_cb,qos_profile_sensor_data)
deadline=time.monotonic()+35
while time.monotonic()<deadline and len(errors)<300:rclpy.spin_once(node,timeout_sec=.1)
node.destroy_node();rclpy.shutdown()
errors.sort()
assert len(errors)>50,f'insufficient gate hits: {len(errors)}'
result=dict(points=len(errors),median_surface_error_m=errors[len(errors)//2],p95_surface_error_m=errors[int(.95*(len(errors)-1))])
args.output.write_text(json.dumps(result,indent=2)+'\n')
print('Lidar vs known gate surfaces:',result,flush=True)
assert result['median_surface_error_m']<.01 and result['p95_surface_error_m']<.03,'lidar points do not match physical gate surfaces'
