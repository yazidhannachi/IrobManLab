#!/usr/bin/env python3
"""Read-only checks for the lab infrastructure. Does not move the robot."""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPlanningScene
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('lab_check_setup')
    samples = {}
    clock = []
    def record(topic, msg):
        samples[topic] = msg
        if topic == '/clock':
            clock.append(msg.clock.sec + msg.clock.nanosec * 1e-9)
    topics = [('/clock', Clock), ('/joint_states', JointState),
              ('/zed_x_mini/rgbd/image', Image), ('/zed_x_mini/rgbd/depth_image', Image),
              ('/zed_x_mini/rgbd/camera_info', CameraInfo), ('/zed_x_mini/rgbd/points', PointCloud2),
              ('/zed_x_mini/right/image', Image), ('/zed_x_mini/right/camera_info', CameraInfo),
              ('/zed_x_mini/imu', Imu)]
    for topic, kind in topics:
        node.create_subscription(kind, topic, lambda msg, t=topic: record(t, msg),
                                 qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.2)
        if len(samples) == len(topics) and len(clock) > 1 and clock[-1]-clock[0] > 2:
            break
    checks = {f'{name} received': name in samples for name, _ in topics}
    checks['simulation clock advances'] = len(clock) > 1 and clock[-1] > clock[0]
    state = samples.get('/joint_states')
    checks['arm and gripper joint states'] = state is not None and set(
        [f'fr3_joint{i}' for i in range(1, 8)] + ['fr3_finger_joint1', 'fr3_finger_joint2']
    ).issubset(state.name)
    for topic in ['/zed_x_mini/rgbd/image', '/zed_x_mini/rgbd/depth_image', '/zed_x_mini/rgbd/camera_info']:
        msg = samples.get(topic)
        checks[f'{topic} optical frame'] = msg is not None and msg.header.frame_id == 'left_camera_link_optical'
    checks['camera TF'] = buffer.can_transform('world', 'left_camera_link_optical', rclpy.time.Time())
    left = samples.get('/zed_x_mini/rgbd/camera_info')
    right = samples.get('/zed_x_mini/right/camera_info')
    checks['ZED X Mini stereo calibration'] = left is not None and right is not None and all(
        (c.width, c.height) == (1920, 1200)
        and abs(c.k[0] - 737) < .1
        and abs(c.k[4] - 737) < .1
        and abs(c.k[2] - 960) < .1 and abs(c.k[5] - 600) < .1
        and abs(c.p[0] - c.k[0]) < .1 and abs(c.p[5] - c.k[4]) < .1
        for c in (left, right)) and abs(left.p[3]) < 1e-6 and abs(
            right.p[3] + right.p[0] * .05) < 1e-6
    checks['ZED X Mini 50 mm baseline TF'] = False
    if buffer.can_transform('left_camera_link', 'right_camera_link', rclpy.time.Time()):
        baseline = buffer.lookup_transform('left_camera_link', 'right_camera_link',
                                           rclpy.time.Time()).transform.translation
        checks['ZED X Mini 50 mm baseline TF'] = (
            abs(baseline.x) < 1e-6 and abs(baseline.y + .05) < 1e-6
            and abs(baseline.z) < 1e-6)
    depth = samples.get('/zed_x_mini/rgbd/depth_image')
    checks['depth encoding is 32FC1'] = depth is not None and depth.encoding == '32FC1'
    checks['right camera optical frame'] = right is not None and right.header.frame_id == 'right_camera_link_optical'
    checks['TCP TF'] = buffer.can_transform('world', 'fr3_hand_tcp', rclpy.time.Time())
    for name, kind in [('/fr3_arm_controller/follow_joint_trajectory', FollowJointTrajectory),
                       ('/fr3_gripper/follow_joint_trajectory', FollowJointTrajectory),
                       ('/move_action', MoveGroup)]:
        checks[name] = ActionClient(node, kind, name).wait_for_server(timeout_sec=3)

    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    checks['controllers active'] = False
    if client.wait_for_service(timeout_sec=3):
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if future.done() and future.result():
            active = {c.name for c in future.result().controller if c.state == 'active'}
            checks['controllers active'] = {'joint_state_broadcaster', 'fr3_arm_controller', 'fr3_gripper'} <= active
    client = node.create_client(GetPlanningScene, '/get_planning_scene')
    checks['MoveIt table obstacle'] = False
    if client.wait_for_service(timeout_sec=3):
        request = GetPlanningScene.Request()
        request.components.components = request.components.WORLD_OBJECT_NAMES
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if future.done() and future.result():
            checks['MoveIt table obstacle'] = any(
                o.id == 'lab_table' for o in future.result().scene.world.collision_objects)
    checks['no object ground-truth feeds'] = not any(
        name.startswith('/model/') or name == '/drawer/joint_states'
        for name, _ in node.get_topic_names_and_types())
    for name, ok in checks.items():
        print(f'{"PASS" if ok else "FAIL"}: {name}')
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
