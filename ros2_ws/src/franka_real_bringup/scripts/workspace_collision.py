#!/usr/bin/env python3
"""Add the measured static cell obstacles (config/workspace.yaml) to MoveIt."""
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def main():
    rclpy.init()
    node = Node('lab_workspace_collision')
    default = Path(get_package_share_directory('franka_real_bringup')) / 'config/workspace.yaml'
    config = yaml.safe_load(Path(node.declare_parameter('config', str(default)).value).read_text())
    client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=90):
        raise RuntimeError('MoveIt planning scene service is unavailable')
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    for box in config.get('boxes', []):
        obstacle = CollisionObject(id=box['id'], operation=CollisionObject.ADD)
        obstacle.header.frame_id = 'world'
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, box['position'])
        pose.orientation.w = 1.
        obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                              dimensions=[float(v) for v in box['size']])]
        obstacle.primitive_poses = [pose]
        request.scene.world.collision_objects.append(obstacle)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    if not future.done() or not future.result().success:
        raise RuntimeError('Could not add the workspace obstacles to MoveIt')
    names = [o.id for o in request.scene.world.collision_objects]
    node.get_logger().info(f'MoveIt workspace obstacles ready: {names}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
