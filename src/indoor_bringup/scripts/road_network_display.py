#!/usr/bin/env python3
"""Publish the complete topology and SDF collision geometry as retained RViz markers."""
import math
import signal
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def read_topology(path):
    nodes, edges = {}, []
    section = ''
    for raw in Path(path).read_text().splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        if line.endswith(':'):
            section = line[:-1]
            continue
        fields = line.split()
        if section == 'lla_mode' and line != '0':
            raise ValueError('Indoor display requires lla_mode: 0 (local XYZ coordinates)')
        if section == 'node':
            node_id = int(fields[0])
            xyz = tuple(map(float, fields[3:6]))
            if node_id in nodes or len(xyz) != 3 or not all(map(math.isfinite, xyz)):
                raise ValueError(f'Invalid or duplicate node: {line}')
            nodes[node_id] = xyz
        elif section == 'edge':
            edges.append((int(fields[0]), int(fields[1]), int(fields[3]), int(fields[4])))
    if not nodes:
        raise ValueError('Topology has no nodes')
    for edge_id, kind, a, b in edges:
        if a not in nodes or b not in nodes or kind not in (0, 1):
            raise ValueError(f'Invalid edge {edge_id}: {a}->{b}')
    return nodes, edges


def marker(namespace, marker_id, kind, color):
    item = Marker()
    item.header.frame_id = 'map'
    # Static map markers use the latest transform and remain visible while paused.
    item.ns, item.id, item.type = namespace, marker_id, kind
    item.action = Marker.ADD
    item.pose.orientation.w = 1.0
    item.color.r, item.color.g, item.color.b, item.color.a = color
    return item


def topology_markers(nodes, edges):
    result = MarkerArray()
    lines = marker('edges', 0, Marker.LINE_LIST, (0.0, 0.85, 1.0, 1.0))
    lines.scale.x = 0.09
    for _, kind, a, b in edges:
        for node_id in (a, b):
            x, y, z = nodes[node_id]
            lines.points.append(Point(x=x, y=y, z=z + 0.07))
    result.markers.append(lines)
    for node_id, (x, y, z) in nodes.items():
        dot = marker('nodes', node_id, Marker.CYLINDER, (1.0, 0.75, 0.05, 1.0))
        dot.pose.position = Point(x=x, y=y, z=z + 0.10)
        dot.scale.x = dot.scale.y = 0.30
        dot.scale.z = 0.06
        result.markers.append(dot)
        label = marker('node_ids', node_id, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        label.pose.position = Point(x=x + 0.3, y=y + 0.35, z=z + 0.3)
        label.scale.z = 0.38
        label.text = str(node_id)
        result.markers.append(label)
    for edge_id, kind, a, b in edges:
        if kind == 1:
            arrow = marker('directions', edge_id, Marker.ARROW, (0.0, 0.85, 1.0, 1.0))
            for fraction in (0.4, 0.65):
                xyz = [nodes[a][i] + fraction * (nodes[b][i] - nodes[a][i]) for i in range(3)]
                arrow.points.append(Point(x=xyz[0], y=xyz[1], z=xyz[2] + 0.08))
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.1, 0.3, 0.3
            result.markers.append(arrow)
    return result


def sdf_pose(element):
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('relative_to SDF frames need an explicit frame resolver')
    values = tuple(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))
    if len(values) != 6 or not all(map(math.isfinite, values)):
        raise ValueError('Invalid SDF pose')
    if abs(values[3]) > 1e-8 or abs(values[4]) > 1e-8:
        raise ValueError('Indoor background currently supports horizontal models only')
    return values[0], values[1], values[2], values[5]


def compose(parent, child):
    x, y, z, yaw = parent
    u, v, w, angle = child
    return (x + math.cos(yaw) * u - math.sin(yaw) * v,
            y + math.sin(yaw) * u + math.cos(yaw) * v, z + w, yaw + angle)


def environment_markers(path):
    result = MarkerArray()
    world = ET.parse(path).getroot().find('world')
    if world is None:
        raise ValueError('SDF file has no world')
    bounds = []
    for model in world.findall('model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is None or geometry.find('plane') is not None:
                    continue
                box = geometry.find('box')
                cylinder = geometry.find('cylinder')
                if box is not None:
                    sx, sy, sz = map(float, box.findtext('size').split())
                    kind = Marker.CUBE
                elif cylinder is not None:
                    sx = sy = 2 * float(cylinder.findtext('radius'))
                    sz = float(cylinder.findtext('length'))
                    kind = Marker.CYLINDER
                else:
                    raise ValueError(f'Unsupported background geometry in {model.get("name")}')
                x, y, z, yaw = compose(compose(sdf_pose(model), sdf_pose(link)), sdf_pose(collision))
                item = marker('walls', len(result.markers), kind, (0.48, 0.52, 0.58, 0.75))
                item.pose.position = Point(x=x, y=y, z=z)
                item.pose.orientation.z, item.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                item.scale.x, item.scale.y, item.scale.z = sx, sy, sz
                result.markers.append(item)
                hx = (abs(math.cos(yaw)) * sx + abs(math.sin(yaw)) * sy) / 2
                hy = (abs(math.sin(yaw)) * sx + abs(math.cos(yaw)) * sy) / 2
                bounds.append((x - hx, y - hy, x + hx, y + hy))
    if bounds:
        xmin, ymin = min(b[0] for b in bounds), min(b[1] for b in bounds)
        xmax, ymax = max(b[2] for b in bounds), max(b[3] for b in bounds)
        floor = marker('floor', 0, Marker.CUBE, (0.18, 0.22, 0.25, 1.0))
        floor.pose.position = Point(x=(xmin + xmax) / 2, y=(ymin + ymax) / 2, z=-0.04)
        floor.scale.x, floor.scale.y, floor.scale.z = xmax - xmin + 1, ymax - ymin + 1, 0.04
        result.markers.insert(0, floor)
    return result


class RoadNetworkDisplay(Node):
    def __init__(self):
        super().__init__('road_network_display')
        topology = self.declare_parameter('topo_map_path', '').value
        world = self.declare_parameter('scene_world', '').value
        nodes, edges = read_topology(topology)
        self.topology = topology_markers(nodes, edges)
        self.environment = MarkerArray()
        if world:
            try:
                self.environment = environment_markers(world)
            except (ValueError, OSError, ET.ParseError) as exc:
                self.get_logger().error(f'Cannot display world background: {exc}')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.graph_pub = self.create_publisher(MarkerArray, 'road_network', qos)
        self.world_pub = self.create_publisher(MarkerArray, 'indoor_environment', qos)
        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info(f'显示完整路网：{len(nodes)} 个节点、{len(edges)} 条边；节点编号为白色文字')
        self.publish()

    def publish(self):
        # Retained ADD markers alone leave deleted obstacles/edges visible when
        # switching from a larger scene. Reset each display in the same batch.
        reset = Marker()
        reset.header.frame_id = 'map'
        reset.action = Marker.DELETEALL
        self.graph_pub.publish(MarkerArray(markers=[reset] + self.topology.markers))
        self.world_pub.publish(MarkerArray(markers=[reset] + self.environment.markers))


def main():
    rclpy.init()
    node = None
    try:
        node = RoadNetworkDisplay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # The terminal and launch may both deliver SIGINT during shutdown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
