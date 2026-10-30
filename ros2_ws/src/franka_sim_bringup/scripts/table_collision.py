#!/usr/bin/env python3
"""Add the fixed work surface to MoveIt; object obstacles are a student task."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    rclpy.init()
    node = Node('lab_table_collision')
    client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=90):
        raise RuntimeError('MoveIt planning scene service is unavailable')
    table = CollisionObject(id='lab_table')
    table.header.frame_id = 'world'
    table.operation = CollisionObject.ADD
    # The robot pedestal occupies the rear of the table. Model the usable
    # surface from x=0.15 to 0.90 so it does not collide with the fixed base.
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.75, 1.0, .05])
    pose = Pose()
    pose.position.x, pose.position.z, pose.orientation.w = .525, .775, 1.0
    table.primitives, table.primitive_poses = [box], [pose]
    request = ApplyPlanningScene.Request()
    request.scene.is_diff = True
    request.scene.world.collision_objects = [table]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    if not future.done() or not future.result().success:
        raise RuntimeError('Could not add the table to MoveIt')
    node.get_logger().info('MoveIt work surface ready.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
