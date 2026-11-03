#!/usr/bin/env python3
"""Generate the indoor corridor world. No online models or other workspace assets."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def child(parent, tag, text=None, **attrs):
    element = ET.SubElement(parent, tag, attrs)
    if text is not None:
        element.text = text
    return element


def generate():
    sdf = ET.Element('sdf', version='1.7')
    world = child(sdf, 'world', name='indoor_corridors')
    child(world, 'gravity', '0 0 -9.8')
    physics = child(world, 'physics', name='indoor_physics', type='ode')
    child(physics, 'max_step_size', '0.002')
    child(physics, 'real_time_update_rate', '500')
    scene = child(world, 'scene')
    child(scene, 'ambient', '0.65 0.65 0.65 1')
    child(scene, 'background', '0.85 0.85 0.85 1')
    light = child(world, 'light', name='indoor_light', type='directional')
    child(light, 'pose', '0 0 12 0 0 0')
    child(light, 'diffuse', '0.8 0.8 0.8 1')
    child(light, 'direction', '-0.3 0.1 -0.9')
    ground = child(world, 'model', name='floor')
    child(ground, 'static', 'true')
    link = child(ground, 'link', name='floor_link')
    for kind in ['collision', 'visual']:
        item = child(link, kind, name='floor_' + kind)
        plane = child(child(item, 'geometry'), 'plane')
        child(plane, 'normal', '0 0 1')
        child(plane, 'size', '40 30')
        if kind == 'visual':
            material = child(item, 'material')
            child(material, 'ambient', '0.65 0.68 0.7 1')
            child(material, 'diffuse', '0.65 0.68 0.7 1')
    # Outside bounds [-2,14] x [-2,10]. Four corridors surround a central room block.
    boxes = [
        ('south_wall', 6, -2, 16.2, 0.2),
        ('north_wall', 6, 10, 16.2, 0.2),
        ('west_wall', -2, 4, 0.2, 12),
        ('east_wall', 14, 4, 0.2, 12),
        ('central_room_block', 6, 4, 8, 4),
    ]
    for name, x, y, sx, sy in boxes:
        model = child(world, 'model', name=name)
        child(model, 'static', 'true')
        child(model, 'pose', f'{x} {y} 1.25 0 0 0')
        link = child(model, 'link', name='wall')
        for kind in ['collision', 'visual']:
            item = child(link, kind, name=name + '_' + kind)
            child(child(child(item, 'geometry'), 'box'), 'size', f'{sx} {sy} 2.5')
            if kind == 'visual':
                material = child(item, 'material')
                child(material, 'ambient', '0.8 0.75 0.65 1')
                child(material, 'diffuse', '0.8 0.75 0.65 1')
    state = child(world, 'plugin', name='indoor_ground_truth', filename='libgazebo_ros_state.so')
    child(child(state, 'ros'), 'namespace', '/indoor_sim')
    child(state, 'update_rate', '20.0')
    gui = child(world, 'gui')
    camera = child(gui, 'camera', name='indoor_overview')
    child(camera, 'pose', '6 -9 17 0 0.9 1.57')
    child(camera, 'view_controller', 'orbit')
    ET.indent(sdf, space='  ')
    target = ROOT / 'src/indoor_bringup/worlds/indoor.world'
    ET.ElementTree(sdf).write(target, encoding='unicode', xml_declaration=True)
    print(target)


if __name__ == '__main__':
    generate()
