#!/usr/bin/env python3
"""Isolated ROS checks: retained topology markers and CLI request delivery to a fake server."""
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import NavigationStatus
from nav2_msgs.srv import NavigationRequest
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/indoor_bringup/scripts'))
from road_network_display import RoadNetworkDisplay, environment_markers, read_topology, topology_markers
from check_launch_executables import check as check_executables


def retained():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def check_markers():
    nodes, edges = read_topology(ROOT / 'src/indoor_bringup/maps/indoor_topology.yaml')
    graph = topology_markers(nodes, edges)
    assert len([m for m in graph.markers if m.ns == 'nodes']) == 12
    assert {m.text for m in graph.markers if m.ns == 'node_ids'} == {str(i) for i in range(12)}
    assert len(graph.markers[0].points) == 24
    environment = environment_markers(ROOT / 'src/indoor_bringup/worlds/indoor.world')
    assert len(environment.markers) == 6
    assert all(m.header.frame_id == 'map' and m.color.a > 0 for m in graph.markers + environment.markers)
    # Verify one-way arrows as well as the bidirectional default graph.
    directed = topology_markers({0: (0., 0., 0.), 1: (1., 0., 0.)}, [(9, 1, 0, 1)])
    assert any(m.type == Marker.ARROW and len(m.points) == 2 for m in directed.markers)
    print('PASS: 12 node markers, 12 labels, 12 edges, 5 walls and floor; one-way arrow', flush=True)

    display = RoadNetworkDisplay()
    # The display publishes once in its constructor; do not spin its timer.
    time.sleep(0.3)
    listener = Node('late_display_listener')
    received = {}
    listener.create_subscription(MarkerArray, '/car1/road_network',
                                 lambda msg: received.update(graph=msg), retained())
    listener.create_subscription(MarkerArray, '/car1/indoor_environment',
                                 lambda msg: received.update(world=msg), retained())
    deadline = time.monotonic() + 10
    while len(received) < 2 and time.monotonic() < deadline:
        rclpy.spin_once(listener, timeout_sec=0.1)
    assert len(received) == 2, 'RViz late subscribers did not receive retained markers'
    assert len(received['graph'].markers) == 25 and len(received['world'].markers) == 6
    listener.destroy_node()
    display.destroy_node()
    print('PASS: late ROS subscribers receive the full graph and world without a navigation goal', flush=True)


def check_installed_display():
    assert check_executables() == 0
    listener = Node('installed_display_listener')
    received = []
    listener.create_subscription(MarkerArray, '/car1/road_network', received.append, retained())
    try:
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(['ros2', 'launch', 'indoor_bringup', 'display.launch.py',
                                        'use_rviz:=false', 'use_sim_time:=false'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic() + 12
                while not received and process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(listener, timeout_sec=0.1)
                assert process.poll() is None and received and len(received[-1].markers) == 25
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
            log.seek(0)
            output = log.read()
            assert '显示完整路网' in output and 'Traceback' not in output and '[ERROR]' not in output, output
        print('PASS: installed display.launch.py starts its executable and publishes the full road network', flush=True)
    finally:
        listener.destroy_node()


class FakeNavigation(Node):
    def __init__(self):
        super().__init__('fake_navigation_for_cli_test')
        self.pose = (0., 0.)
        self.requested = None
        self.finish_at = None
        self.state = NavigationStatus.IDLE
        for name in ['net_navigator', 'net_planner_action_server', 'controller_server']:
            self.create_service(GetState, f'/car1/{name}/get_state', self.get_state)
        self.create_service(NavigationRequest, '/car1/navigation_request', self.request)
        self.odom = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.cloud = self.create_publisher(PointCloud2, '/car1/cur_lid', qos_profile_sensor_data)
        self.truth = self.create_publisher(ModelStates, '/indoor_sim/model_states', 10)
        self.status = self.create_publisher(NavigationStatus, '/car1/navigation_status', retained())
        self.path = self.create_publisher(NavPath, '/car1/topology_plan', retained())
        self.create_timer(0.1, self.update)

    def get_state(self, _, response):
        response.current_state.id = 3
        response.current_state.label = 'active'
        return response

    def request(self, request, response):
        response.success = True
        response.command_id = request.command_id
        if request.cmd == 'New':
            assert request.target_pose.header.frame_id == 'map'
            self.requested = (request.target_pose.pose.position.x, request.target_pose.pose.position.y)
            self.state = NavigationStatus.PLANNING
            self.finish_at = time.monotonic() + 0.7
            response.message = 'accepted by isolated CLI test server'
        else:
            self.state = NavigationStatus.CANCELED
        self.update()
        return response

    def update(self):
        if self.finish_at and time.monotonic() >= self.finish_at:
            self.pose = self.requested
            self.state = NavigationStatus.SUCCEEDED
        odom = Odometry()
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose
        self.odom.publish(odom)
        cloud = PointCloud2()
        cloud.width = cloud.height = 1
        self.cloud.publish(cloud)
        truth = ModelStates()
        truth.name = ['simple_car']
        truth.pose = [Pose()]
        truth.pose[0].position.x, truth.pose[0].position.y = self.pose
        self.truth.publish(truth)
        status = NavigationStatus()
        status.state = self.state
        status.message = 'isolated interface test (no vehicle motion)'
        self.status.publish(status)
        if self.requested:
            path = NavPath()
            path.poses = [PoseStamped(), PoseStamped()]
            path.poses[-1].pose.position.x, path.poses[-1].pose.position.y = self.requested
            self.path.publish(path)


def check_cli():
    wrong = subprocess.run([str(ROOT / 'scripts/goal.sh'), '3.0'], text=True, capture_output=True)
    assert wrong.returncode == 2 and './scripts/goal.sh 3 0' in wrong.stderr
    assert 'expected 2 arguments' not in wrong.stderr
    print('PASS: one-number goal exits before ROS and explains the two-coordinate format in Chinese', flush=True)
    fake = FakeNavigation()
    try:
        with tempfile.TemporaryDirectory(prefix='indoor_cli_test_') as directory:
            env = dict(os.environ, INDOOR_ROS_DOMAIN_ID=os.environ['ROS_DOMAIN_ID'])
            process = subprocess.Popen([str(ROOT / 'scripts/goal.sh'), '3', '0',
                                        '--ready-timeout', '20', '--timeout', '5',
                                        '--report-dir', directory], env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            deadline = time.monotonic() + 35
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(fake, timeout_sec=0.1)
            if process.poll() is None:
                process.terminate()
            output, _ = process.communicate(timeout=5)
            assert process.returncode == 0, output
            assert fake.requested == (3., 0.), fake.requested
            assert '目标已受理' in output and '已到达' in output, output
            reports = list(Path(directory).glob('navigation_*.json'))
            assert len(reports) == 1
            report = json.loads(reports[0].read_text())
            assert report['success'] and report['results'][0]['goal'] == [3., 0.]
            print('PASS: ./scripts/goal.sh 3 0 sends map (3,0), prints acceptance/status and saves a success report', flush=True)
    finally:
        fake.destroy_node()


def main():
    assert os.environ['ROS_DOMAIN_ID'] == '188', 'This test must run in isolated ROS domain 188'
    rclpy.init(args=['--ros-args', '-r', '__ns:=/car1', '-p',
                     'topo_map_path:=' + str(ROOT / 'src/indoor_bringup/maps/indoor_topology.yaml'),
                     '-p', 'scene_world:=' + str(ROOT / 'src/indoor_bringup/worlds/indoor.world')])
    try:
        check_installed_display()
        check_markers()
        check_cli()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
