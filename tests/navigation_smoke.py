#!/usr/bin/env python3
"""Sequential live navigation test, checked against Gazebo model pose when available."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import NavigationStatus
from nav2_msgs.srv import NavigationRequest
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import yaml
from geometry_clearance import clearance, circle_clearance, transform

ROOT = Path(__file__).resolve().parents[1]


class NavigationTest(Node):
    def __init__(self):
        super().__init__('indoor_navigation_test')
        self.client = self.create_client(NavigationRequest, '/car1/navigation_request')
        self.pose = None
        self.truth = None
        self.odom_received = self.cloud_received = self.truth_received = 0.0
        self.path_points = 0
        self.max_speed = 0.0
        self.detour_points = 0
        self.status = None
        self.tracking = self.seen_active = False
        self.last_status_print = 0.0
        self.last_printed_state = None
        self.test_box = None
        self.models = {}
        self.min_box_clearance = math.inf
        self.trajectory = []
        self.sent_goal = False
        self.last_motion_sample = None
        self.pause_started = None
        self.longest_translation_pause = 0.0
        self.translation_pause_total = 0.0
        self.scene_obstacles = {}
        self.scene_clearances = {}
        self.door_phase_counts = {}
        config = yaml.safe_load((ROOT / 'src/indoor_bringup/config/indoor_nav.yaml').read_text())
        costmap = config['/**/local_costmap']['local_costmap']['ros__parameters']
        padding = float(costmap.get('footprint_padding', 0.0))
        self.footprint = [(x+math.copysign(padding,x), y+math.copysign(padding,y))
                          for x, y in yaml.safe_load(costmap['footprint'])]
        retained = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(ModelStates, '/indoor_sim/model_states', self.on_truth, 10)
        self.create_subscription(PointCloud2, '/car1/cur_lid', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(NavigationStatus, '/car1/navigation_status', self.on_status, retained)
        self.create_subscription(NavPath, '/car1/topology_plan', self.on_path, retained)
        self.create_subscription(Twist, '/cmd_vel', self.on_velocity, 10)
        self.create_subscription(NavPath, '/car1/theta_star_path_raw', self.on_detour, retained)
        self.create_subscription(String, '/log/record', self.on_record, 100)

    def on_record(self, msg):
        fields = msg.data.split(',')
        if self.tracking and len(fields) > 3 and fields[1:3] == ['ctrl','door']:
            phase = fields[3]
            self.door_phase_counts[phase] = self.door_phase_counts.get(phase, 0)+1

    def load_obstacle_world(self, path):
        world = ET.parse(path).getroot().find('world')
        for model in world.findall('model'):
            for collision in model.findall('link/collision'):
                box = collision.find('geometry/box')
                cylinder = collision.find('geometry/cylinder')
                if box is not None:
                    sx, sy, sz = map(float,box.findtext('size').split())
                    if sz > 0.12:
                        self.scene_obstacles[model.get('name')] = ('box',sx,sy)
                elif cylinder is not None:
                    self.scene_obstacles[model.get('name')] = ('circle',float(cylinder.findtext('radius')))

    def on_odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.odom_received = time.monotonic()
        if self.tracking:
            now = self.odom_received
            stopped = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y) < 0.03
            if stopped:
                if self.pause_started is None:
                    self.pause_started = now
                self.longest_translation_pause = max(self.longest_translation_pause, now-self.pause_started)
                if self.last_motion_sample is not None:
                    self.translation_pause_total += min(now-self.last_motion_sample, 0.25)
            else:
                self.pause_started = None
            self.last_motion_sample = now

    def on_truth(self, msg):
        self.models = dict(zip(msg.name, msg.pose))
        if 'simple_car' in msg.name:
            model_pose = self.models['simple_car']
            position = model_pose.position
            self.truth = (position.x, position.y)
            self.truth_received = time.monotonic()
            if self.tracking:
                q = model_pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                self.trajectory.append([position.x, position.y, yaw])
                body = transform(self.footprint,position.x,position.y,yaw)
                for name, shape in self.scene_obstacles.items():
                    if name not in self.models:
                        continue
                    p = self.models[name]; q = p.orientation
                    angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    if shape[0] == 'box':
                        sx,sy = shape[1]/2,shape[2]/2
                        d = clearance(body,transform([(-sx,-sy),(sx,-sy),(sx,sy),(-sx,sy)],
                                                     p.position.x,p.position.y,angle))
                    else:
                        d = circle_clearance(body,p.position.x,p.position.y,shape[1])
                    self.scene_clearances[name] = min(self.scene_clearances.get(name,math.inf),d)
                if self.test_box is not None and self.test_box['name'] in self.models:
                    box = self.models[self.test_box['name']]
                    bq = box.orientation
                    byaw = math.atan2(2*(bq.w*bq.z+bq.x*bq.y), 1-2*(bq.y*bq.y+bq.z*bq.z))
                    half = self.test_box['size']/2
                    rectangle = [(-half,-half),(half,-half),(half,half),(-half,half)]
                    distance = clearance(transform(self.footprint,position.x,position.y,yaw),
                                         transform(rectangle,box.position.x,box.position.y,byaw))
                    self.min_box_clearance = min(self.min_box_clearance,distance)

    def on_cloud(self, msg):
        if msg.width * msg.height:
            self.cloud_received = time.monotonic()

    def on_path(self, msg):
        self.path_points = len(msg.poses)

    def on_velocity(self, msg):
        self.max_speed = max(self.max_speed, abs(msg.linear.x))

    def on_detour(self, msg):
        if self.tracking:
            self.detour_points = max(self.detour_points, len(msg.poses))

    def obstacle(self, spawn):
        service_type, name = (SpawnEntity, '/spawn_entity') if spawn else (DeleteEntity, '/delete_entity')
        client = self.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError(f'{name} not available')
            request = service_type.Request()
            request.name = 'indoor_test_box'
            if spawn:
                request.reference_frame = 'world'
                request.initial_pose.position.x = self.test_box['x']
                request.initial_pose.position.y = self.test_box['y']
                size = self.test_box['size']
                height = max(0.6, size)
                request.initial_pose.position.z = height/2
                request.initial_pose.orientation.w = 1.0
                request.xml = f'''<sdf version="1.7"><model name="indoor_test_box"><static>true</static>
                  <link name="box"><collision name="collision"><geometry><box><size>{size} {size} {height}</size>
                  </box></geometry></collision><visual name="visual"><geometry><box><size>0.5 0.5 0.6</size>
                  </box></geometry></visual></link></model></sdf>'''
                request.xml = request.xml.replace('0.5 0.5 0.6', f'{size} {size} {height}')
            future = client.call_async(request)
            if not self.spin_until(future.done, 15):
                raise RuntimeError(f'{name} timeout')
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(f'{name} failed: {response}')
            # Let several sensor/costmap updates observe the changed environment.
            self.spin_until(lambda: False, 2)
        finally:
            self.destroy_client(client)

    def on_status(self, msg):
        if self.tracking:
            if msg.state in (NavigationStatus.PLANNING, NavigationStatus.NAVIGATING):
                self.seen_active = True
            if self.seen_active:
                self.status = msg
                if msg.state != self.last_printed_state or time.monotonic() - self.last_status_print >= 5:
                    names = {1: '规划中', 2: '行驶中', 3: '已到达', 4: '已取消', 5: '失败'}
                    remaining = f'，剩余 {msg.distance_remaining:.2f} 米' if msg.distance_remaining >= 0 else ''
                    print(f'导航状态：{names.get(msg.state, str(msg.state))}{remaining}。{msg.message}', flush=True)
                    self.last_status_print = time.monotonic()
                    self.last_printed_state = msg.state

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        for name in ['net_navigator', 'net_planner_action_server', 'controller_server']:
            client = self.create_client(GetState, f'/car1/{name}/get_state')
            try:
                while time.monotonic() < deadline:
                    if not client.wait_for_service(timeout_sec=0.5):
                        continue
                    future = client.call_async(GetState.Request())
                    if self.spin_until(future.done, min(3, max(0, deadline - time.monotonic()))):
                        response = future.result()
                        if response and response.current_state.id == 3:
                            break
                else:
                    raise RuntimeError(f'{name} did not become Active')
            finally:
                self.destroy_client(client)
        if not self.spin_until(lambda: self.client.service_is_ready()
                               and time.monotonic() - self.odom_received < 2
                               and time.monotonic() - self.cloud_received < 2,
                               max(0, deadline - time.monotonic())):
            raise RuntimeError('navigation service, fresh odometry or cloud not ready')

    def cancel(self):
        if self.client.service_is_ready():
            request = NavigationRequest.Request()
            request.cmd = 'Cancel'
            future = self.client.call_async(request)
            self.spin_until(future.done, 3)

    def run_goal(self, route, tolerance, require_truth):
        self.status = None
        self.seen_active = False
        self.tracking = True
        self.last_printed_state = None
        self.path_points = 0
        self.max_speed = 0.0
        self.detour_points = 0
        self.min_box_clearance = math.inf
        self.trajectory = []
        request = NavigationRequest.Request()
        self.last_motion_sample = self.pause_started = None
        self.longest_translation_pause = self.translation_pause_total = 0.0
        self.scene_clearances = {}
        self.door_phase_counts = {}
        request.command_id = f'indoor_{time.time_ns()}'
        request.cmd = 'New'
        request.target_pose.header.frame_id = 'map'
        request.target_pose.pose.position.x = float(route['x'])
        request.target_pose.pose.position.y = float(route['y'])
        request.target_pose.pose.orientation.w = 1.0
        start = time.monotonic()
        self.sent_goal = True
        future = self.client.call_async(request)
        if not self.spin_until(future.done, 10):
            raise RuntimeError('navigation request timed out')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f'goal rejected: {response}')
        print(f'目标已受理：map ({route["x"]}, {route["y"]}) 米，等待小车到达……', flush=True)
        terminal = self.spin_until(lambda: self.status is not None and self.status.state in (3, 4, 5),
                                   float(route.get('timeout', 120)))
        self.tracking = False
        target = (float(route['x']), float(route['y']))
        odom_error = math.dist(self.pose, target) if self.pose else None
        truth_fresh = self.truth is not None and time.monotonic() - self.truth_received < 2
        truth_error = math.dist(self.truth, target) if truth_fresh else None
        checked_error = truth_error if truth_fresh else odom_error
        success = (terminal and self.status.state == 3 and checked_error is not None
                   and checked_error <= tolerance and self.path_points > 0
                   and (truth_fresh or not require_truth))
        result = dict(name=route['name'], goal=list(target), success=success,
                      elapsed_wall_s=round(time.monotonic() - start, 3),
                      status=int(self.status.state) if self.status else None,
                      message=self.status.message if self.status else 'timed out without terminal status',
                      odometry_error_m=odom_error, gazebo_truth_error_m=truth_error,
                      path_points=self.path_points, max_command_speed_mps=self.max_speed,
                      theta_star_path_points=self.detour_points,
                      longest_translation_pause_s=round(self.longest_translation_pause, 3),
                      translation_pause_total_s=round(self.translation_pause_total, 3),
                      door_phase_counts=self.door_phase_counts,
                      scene_clearances_m=self.scene_clearances,
                      minimum_box_clearance_m=self.min_box_clearance if math.isfinite(self.min_box_clearance) else None)
        if route.get('require_detour') and self.detour_points == 0:
            result['success'] = success = False
            result['message'] += '; expected a Theta* detour but none was received'
        if self.test_box is not None and (not math.isfinite(self.min_box_clearance) or self.min_box_clearance <= 0.02):
            result['success'] = success = False
            result['message'] += '; test box clearance was not measured or was below 0.02 m'
        if self.scene_obstacles and (set(self.scene_clearances) != set(self.scene_obstacles) or
                                     min(self.scene_clearances.values(),default=0.) <= 0.02):
            result['success'] = success = False
            result['message'] += '; scene body clearance missing or below 0.02 m'
        if route.get('require_door') and not self.door_phase_counts.get('3',0):
            result['success'] = success = False
            result['message'] += '; expected door CROSS servo but none was observed'
        if (route.get('max_translation_pause') is not None and
                self.longest_translation_pause > route['max_translation_pause']):
            result['success'] = success = False
            result['message'] += '; longest translation pause exceeded configured limit'
        print(json.dumps(result, ensure_ascii=False), flush=True)
        result['trajectory'] = self.trajectory
        if not terminal:
            self.cancel()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--routes', type=Path, default=ROOT / 'src/indoor_bringup/config/test_routes.yaml')
    parser.add_argument('--ready-timeout', type=float, default=90)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--max-translation-pause', type=float,
                        help='fail if a continuous period with measured translation speed <0.03 m/s exceeds this many seconds')
    parser.add_argument('--report-dir', type=Path, default=ROOT / 'results')
    parser.add_argument('--obstacle', action='store_true',
                        help='single obstacle test from near (0,0) to (6,0); cleans up the box afterward')
    parser.add_argument('--box-size', type=float, default=0.5)
    parser.add_argument('--box-x', type=float, default=3.0)
    parser.add_argument('--box-y', type=float, default=0.0)
    parser.add_argument('--existing-obstacle', help='use this existing Gazebo model without creating/deleting it')
    parser.add_argument('--obstacle-world',type=Path,help='check padded body against all box/cylinder models in this planar fixture')
    parser.add_argument('--require-door-servo',action='store_true')
    parser.add_argument('--obstacle-goal', nargs=2, type=float, default=[6.0,0.0])
    parser.add_argument('--start-near', nargs=2, type=float, default=[0.0,0.0])
    parser.add_argument('--allow-odometry-only', action='store_true',
                        help='allow external navigation without the Gazebo truth topic')
    args = parser.parse_args()
    config = yaml.safe_load(args.routes.read_text())
    routes = ([dict(name='manual_goal', x=args.goal[0], y=args.goal[1], timeout=args.timeout)]
              if args.goal else config['routes'])
    if args.obstacle:
        routes = [dict(name='static_obstacle', x=args.obstacle_goal[0], y=args.obstacle_goal[1],
                       timeout=args.timeout, require_detour=True)]
    if args.require_door_servo:
        for route in routes:
            route['require_detour'] = False
            route['require_door'] = True
    if args.max_translation_pause is not None:
        if not math.isfinite(args.max_translation_pause) or args.max_translation_pause <= 0:
            parser.error('--max-translation-pause must be positive and finite')
        for route in routes:
            route['max_translation_pause'] = args.max_translation_pause
    if any(not all(math.isfinite(float(route[key])) for key in ('x', 'y', 'timeout'))
           or float(route['timeout']) <= 0 for route in routes):
        parser.error('coordinates must be finite and timeouts positive')
    rclpy.init()
    node = NavigationTest()
    if args.obstacle_world:
        node.load_obstacle_world(args.obstacle_world)
    if args.obstacle:
        if not math.isfinite(args.box_size) or args.box_size <= 0:
            parser.error('box size must be positive and finite')
        node.test_box = dict(name=args.existing_obstacle or 'indoor_test_box',
                             size=args.box_size,x=args.box_x,y=args.box_y)
    report = dict(started_utc=datetime.now(timezone.utc).isoformat(), results=[])
    passed = False
    obstacle_spawned = False
    try:
        print('等待导航节点激活、里程计和雷达点云就绪……', flush=True)
        node.wait_ready(args.ready_timeout)
        print('导航已就绪。', flush=True)
        if not args.goal and not args.obstacle and (
                node.pose is None or math.dist(node.pose, (0.0, 0.0)) > 0.5):
            raise RuntimeError('route suite must start near (0,0); run ./scripts/goal.sh 0 0 first')
        if args.obstacle:
            if node.pose is None or math.dist(node.pose, args.start_near) > 0.5:
                raise RuntimeError(f'obstacle test must start near {args.start_near}')
            if args.existing_obstacle:
                if args.existing_obstacle not in node.models:
                    raise RuntimeError(f'obstacle model {args.existing_obstacle} not found')
            else:
                node.obstacle(True)
                obstacle_spawned = True
        for route in routes:
            result = node.run_goal(route, float(config['position_tolerance']),
                                   not args.allow_odometry_only)
            report['results'].append(result)
            if not result['success']:
                break
        passed = len(report['results']) == len(routes) and all(r['success'] for r in report['results'])
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or 'interrupted'
        print(report['error'], flush=True)
        if node.sent_goal:
            node.cancel()
    finally:
        if obstacle_spawned:
            try:
                node.obstacle(False)
            except Exception as exc:
                report['cleanup_error'] = str(exc)
                passed = False
        report['success'] = passed
        output = args.report_dir / ('navigation_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(f'Report: {output}', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
