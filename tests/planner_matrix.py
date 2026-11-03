#!/usr/bin/env python3
"""Exercise the built C++ planner for every node pair, including the same-node case."""
import math
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node

from check_assets import SHARE, load_graph


def wait(node, future, timeout=10):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError('planner action timeout')
    return future.result()


def main():
    rclpy.init()
    node = Node('indoor_planner_matrix')
    client = ActionClient(node, ComputePathToPose, '/car1/compute_path_to_pose')
    try:
        if not client.wait_for_server(timeout_sec=60):
            raise RuntimeError('planner not available; start the simulation first')
        state_client = node.create_client(GetState, '/car1/net_planner_action_server/get_state')
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if state_client.wait_for_service(timeout_sec=0.5):
                state = wait(node, state_client.call_async(GetState.Request()))
                if state.current_state.id == 3:
                    break
            rclpy.spin_once(node, timeout_sec=0.2)
        else:
            raise RuntimeError('planner did not become Active')
        points, _ = load_graph(SHARE / 'maps/indoor_topology.yaml')
        tested = 0
        for start_id, start in points.items():
            for goal_id, goal in points.items():
                request = ComputePathToPose.Goal()
                request.use_start = True
                for field, xy in [(request.start, start), (request.goal, goal)]:
                    field.header.frame_id = 'map'
                    field.pose.position.x, field.pose.position.y = xy
                    field.pose.orientation.w = 1.0
                handle = wait(node, client.send_goal_async(request))
                if not handle.accepted:
                    raise RuntimeError(f'{start_id}->{goal_id}: rejected')
                response = wait(node, handle.get_result_async())
                assert response.status == 4, f'{start_id}->{goal_id}: action status {response.status}'
                poses = response.result.path.poses
                assert poses, f'{start_id}->{goal_id}: empty path'
                for pose, xy in [(poses[0], start), (poses[-1], goal)]:
                    assert math.dist((pose.pose.position.x, pose.pose.position.y), xy) < 1e-5
                tested += 1
        print(f'PASS: C++ planner {tested}/{len(points) ** 2} node pairs, endpoints verified')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
