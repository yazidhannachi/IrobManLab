#!/usr/bin/env python3
"""Create physical lab objects, without object-pose or joint-state publishers."""
import json
import math
from pathlib import Path
import random
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

COLORS = [(0.85, 0.04, 0.04), (0.04, 0.65, 0.08), (0.04, 0.12, 0.85)]


def object_sdf(name, shape, size, color):
    mass = 0.066
    if shape == 'box':
        x, y, z = size
        geometry = f'<box><size>{x} {y} {z}</size></box>'
        ixx, iyy, izz = [mass * n / 12 for n in (y*y+z*z, x*x+z*z, x*x+y*y)]
    else:
        radius, height = size
        geometry = f'<cylinder><radius>{radius}</radius><length>{height}</length></cylinder>'
        ixx = iyy = mass * (3*radius**2 + height**2) / 12
        izz = mass * radius**2 / 2
    rgba = ' '.join(map(str, (*color, 1)))
    return f'''<sdf version="1.9"><model name="{name}"><link name="{name}_body">
      <inertial><mass>{mass}</mass><inertia>
        <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
      </inertia></inertial>
      <collision name="{name}_collision"><geometry>{geometry}</geometry>
        <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>
      </collision>
      <visual name="{name}_visual"><geometry>{geometry}</geometry><material>
        <ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>
      </material></visual>
    </link></model></sdf>'''


def entity(sdf, x, y, z, yaw=0):
    return (f'data: {{sdf: {json.dumps(sdf)} pose: {{position: {{x: {x} y: {y} z: {z}}} '
            f'orientation: {{z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}}}}}')


def main():
    rclpy.init()
    node = Node('lab_scene')
    environment = node.declare_parameter('environment', 'cubes').value
    count = node.declare_parameter('cube_count', 6).value
    rng = random.Random(node.declare_parameter('seed', 1).value)
    if environment not in ('cubes', 'drawer') or not 1 <= count <= 12:
        raise ValueError('environment must be cubes/drawer and cube_count must be 1–12')
    # Wait for Gazebo readiness rather than assuming a startup duration.
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        services = subprocess.run(['gz', 'service', '-l'], capture_output=True,
                                  text=True, timeout=5).stdout.splitlines()
        if '/world/franka_table/create_multiple/blocking' in services:
            break
        time.sleep(1)
    else:
        raise RuntimeError('Gazebo create service did not become ready')

    entities = []
    if environment == 'cubes':
        cells = [(x, y) for x in (.35, .47, .59, .71) for y in (-.22, 0, .22)]
        for i, (x, y) in enumerate(rng.sample(cells, count)):
            entities.append(entity(
                object_sdf(f'cube_{i}', 'box', (.045,)*3, COLORS[i % 3]),
                x+rng.uniform(-.01, .01), y+rng.uniform(-.01, .01), .8255,
                rng.uniform(-math.pi/4, math.pi/4)))
    else:
        pkg = Path(get_package_share_directory('franka_sim_bringup'))
        entities.append(entity((pkg / 'models/drawer/model.sdf').read_text(), .62, 0, .8, math.pi))
        for i, (shape, size, x) in enumerate([
            ('box', (.045, .045, .045), .43),
            ('box', (.035, .05, .055), .56),
            ('cylinder', (.018, .05), .69),
        ]):
            entities.append(entity(
                object_sdf(f'object_{i}', shape, size, COLORS[i]),
                x+rng.uniform(-.005, .005), -.23+rng.uniform(-.005, .005),
                .8+size[-1]/2+.003, rng.uniform(-math.pi/4, math.pi/4)))
    # Finish creating all objects before the launch starts the GUI.
    result = subprocess.run([
        'gz', 'service', '-s', '/world/franka_table/create_multiple/blocking',
        '--reqtype', 'gz.msgs.EntityFactory_V', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', ' '.join(entities),
    ], capture_output=True, text=True, timeout=10)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError(f'Gazebo could not create the scene: {result.stdout} {result.stderr}')
    node.get_logger().info(f'{environment} scene ready. Robot awaits student commands.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
