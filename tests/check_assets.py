#!/usr/bin/env python3
"""Check the actual graph against world collision geometry and the configured footprint."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT / 'src/indoor_bringup'


def load_graph(path):
    nodes, edges = {}, []
    section = ''
    for raw in path.read_text().splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        if line.endswith(':'):
            section = line[:-1]
            continue
        fields = line.split()
        if section == 'node':
            node_id = int(fields[0])
            assert node_id not in nodes, f'duplicate node {node_id}'
            nodes[node_id] = (float(fields[3]), float(fields[4]))
        elif section == 'edge':
            edges.append((int(fields[3]), int(fields[4])))
        elif section == 'lla_mode':
            assert line == '0', 'indoor graph must use local XYZ'
    assert nodes and edges
    return nodes, edges


def main():
    nodes, edges = load_graph(SHARE / 'maps/indoor_topology.yaml')
    adj = {key: set() for key in nodes}
    for a, b in edges:
        assert a in nodes and b in nodes, f'missing endpoint {a}->{b}'
        adj[a].add(b)
        adj[b].add(a)
    for start in nodes:
        seen, todo = {start}, [start]
        while todo:
            for nxt in adj[todo.pop()]:
                if nxt not in seen:
                    seen.add(nxt)
                    todo.append(nxt)
        assert len(seen) == len(nodes), f'disconnected graph from {start}'
    config = yaml.safe_load((SHARE / 'config/indoor_nav.yaml').read_text())
    costmap = config['/**/local_costmap']['local_costmap']['ros__parameters']
    footprint = yaml.safe_load(costmap['footprint'])
    radius = max(math.hypot(x, y) for x, y in footprint) + costmap['footprint_padding']
    boxes = []
    for model in ET.parse(SHARE / 'worlds/indoor.world').getroot().find('world').findall('model'):
        box = model.find('link/collision/geometry/box')
        if box is not None:
            x, y, _, roll, pitch, yaw = map(float, model.findtext('pose').split())
            assert max(abs(roll), abs(pitch), abs(yaw)) < 1e-9
            sx, sy, _ = map(float, box.findtext('size').split())
            boxes.append((x, y, sx / 2, sy / 2))
    minimum = math.inf
    for a, b in edges:
        count = max(1, math.ceil(math.dist(nodes[a], nodes[b]) / 0.025))
        for i in range(count + 1):
            x, y = [s + (e - s) * i / count for s, e in zip(nodes[a], nodes[b])]
            for cx, cy, hx, hy in boxes:
                distance = math.hypot(max(abs(x - cx) - hx, 0), max(abs(y - cy) - hy, 0))
                minimum = min(minimum, distance)
                assert distance > radius + 0.1, f'edge {a}->{b} lacks vehicle clearance'
    # Source independence: source files must not be symlinked into the old project.
    for path in (ROOT / 'src').rglob('*'):
        if path.is_symlink():
            assert path.resolve().is_relative_to(ROOT), f'external source link: {path}'
    print(f'PASS: {len(nodes)} nodes, {len(edges)} edges, all pairs connected; '
          f'minimum centerline clearance={minimum:.3f} m, footprint radius+padding={radius:.3f} m')


if __name__ == '__main__':
    main()
