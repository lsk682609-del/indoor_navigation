#!/usr/bin/env python3
"""Send a map goal or cancellation using the current NavigationRequest interface."""
import argparse
import math
import uuid

import rclpy
from nav2_msgs.srv import NavigationRequest


def main():
    parser = argparse.ArgumentParser(description='实车目标（map 坐标，米），仅报告请求受理结果')
    parser.add_argument('xy', nargs='*', type=float)
    parser.add_argument('--yaw', type=float, default=0.0, help='目标朝向，弧度')
    parser.add_argument('--cancel', action='store_true')
    args = parser.parse_args()
    if (not args.cancel and len(args.xy) != 2) or (args.cancel and args.xy):
        parser.error('目标格式：goal.sh X Y；取消格式：goal.sh --cancel')
    if not all(math.isfinite(v) for v in args.xy + [args.yaw]):
        parser.error('坐标和角度必须为有限数值')
    rclpy.init()
    node = rclpy.create_node('real_robot_goal')
    try:
        client = node.create_client(NavigationRequest, '/car1/navigation_request')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('未发现导航服务，请检查实车入口和 ROS_DOMAIN_ID')
        request = NavigationRequest.Request()
        request.command_id = 'real_' + uuid.uuid4().hex[:12]
        request.cmd = 'Cancel' if args.cancel else 'New'
        request.target_pose.header.frame_id = 'map'
        request.target_pose.header.stamp = node.get_clock().now().to_msg()
        if not args.cancel:
            request.target_pose.pose.position.x, request.target_pose.pose.position.y = args.xy
        request.target_pose.pose.orientation.z = math.sin(args.yaw / 2)
        request.target_pose.pose.orientation.w = math.cos(args.yaw / 2)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done():
            raise RuntimeError('导航服务响应超时')
        response = future.result()
        print(('请求已受理：' if response.success else '请求被拒绝：') + response.message)
        return 0 if response.success else 1
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
