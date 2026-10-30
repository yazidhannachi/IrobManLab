#!/usr/bin/env python3
"""Read-only checks for the real-robot bringup. Never commands the robot."""
from collections import deque
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaRobotState
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener

JOINTS = [f'fr3_joint{i}' for i in range(1, 8)] + ['fr3_finger_joint1', 'fr3_finger_joint2']
# The ZED X Mini receiver (scripts/run.sh --zed) publishes the simulation's camera topics.
# The Jetson's stream carries no IMU data, so /zed_x_mini/imu is only reported.
CAMERA = [('/zed_x_mini/rgbd/image', Image), ('/zed_x_mini/rgbd/camera_info', CameraInfo),
          ('/zed_x_mini/rgbd/depth_image', Image), ('/zed_x_mini/rgbd/points', PointCloud2),
          ('/zed_x_mini/right/image', Image), ('/zed_x_mini/right/camera_info', CameraInfo)]
IMU = '/zed_x_mini/imu'
MODES = {getattr(FrankaRobotState, n): n[len('ROBOT_MODE_'):] for n in dir(FrankaRobotState)
         if n.startswith('ROBOT_MODE_')}


def main():
    rclpy.init()
    node = Node('lab_check_real_setup')
    states, robot = [], deque(maxlen=1)
    node.create_subscription(JointState, '/joint_states', states.append, 10)
    # Published at 1 kHz; keep only the latest message.
    node.create_subscription(FrankaRobotState, '/franka_robot_state_broadcaster/robot_state',
                             lambda msg: robot.append(msg), 1)
    camera = {}
    for topic, kind in [*CAMERA, (IMU, Imu)]:
        node.create_subscription(kind, topic, lambda msg, t=topic: camera.setdefault(t, msg),
                                 qos_profile_sensor_data)
    buffer = Buffer()
    TransformListener(buffer, node)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and len(states) < 20:
        rclpy.spin_once(node, timeout_sec=.1)
    checks = {}
    latest = states[-1] if states else None
    checks['/joint_states has all arm and finger joints'] = (
        latest is not None and set(JOINTS).issubset(latest.name))
    if len(states) > 1:
        span = (states[-1].header.stamp.sec - states[0].header.stamp.sec
                + (states[-1].header.stamp.nanosec - states[0].header.stamp.nanosec) * 1e-9)
        checks['/joint_states uses wall-clock time (no /clock needed)'] = (
            states[-1].header.stamp.sec > 1e9 and span > 0)
    checks['TF world -> fr3_hand_tcp'] = buffer.can_transform('world', 'fr3_hand_tcp', rclpy.time.Time())
    checks['TF world -> left_camera_link_optical'] = buffer.can_transform(
        'world', 'left_camera_link_optical', rclpy.time.Time())

    def call(service, kind):
        client = node.create_client(kind, service)
        if not client.wait_for_service(timeout_sec=5):
            return None
        future = client.call_async(kind.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        return future.result() if future.done() else None

    hardware = call('/controller_manager/list_hardware_components', ListHardwareComponents)
    real = hardware is not None and any(
        c.plugin_name == 'franka_hardware/FrankaHardwareInterface' for c in hardware.component)
    controllers = call('/controller_manager/list_controllers', ListControllers)
    state = {c.name: c.state for c in controllers.controller} if controllers else {}
    checks['controller joint_state_broadcaster active'] = state.get('joint_state_broadcaster') == 'active'
    # arm:=inactive leaves the arm controller loaded but not commanding the robot.
    checks['controller fr3_arm_controller loaded'] = state.get('fr3_arm_controller') in (
        'active', 'inactive')
    if real:
        checks['controller franka_robot_state_broadcaster active'] = (
            state.get('franka_robot_state_broadcaster') == 'active')
        checks['/franka_robot_state_broadcaster/robot_state received'] = bool(robot)

    actions = [('/fr3_arm_controller/follow_joint_trajectory', FollowJointTrajectory),
               ('/fr3_gripper/follow_joint_trajectory', FollowJointTrajectory),
               ('/move_action', MoveGroup)]
    if real:
        actions += [('/franka_gripper/move', Move), ('/franka_gripper/grasp', Grasp)]
    for name, kind in actions:
        checks[f'action {name}'] = ActionClient(node, kind, name).wait_for_server(timeout_sec=10)
    scene = node.create_client(GetPlanningScene, '/get_planning_scene')
    checks['MoveIt planning scene service'] = scene.wait_for_service(timeout_sec=10)

    # The camera is required on the robot; with mock hardware it is checked if it is running.
    if real or node.count_publishers(CAMERA[0][0]):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not all(t in camera for t, _ in CAMERA):
            rclpy.spin_once(node, timeout_sec=.1)
        for topic, _ in CAMERA:
            checks[f'{topic} received'] = topic in camera
        image = camera.get('/zed_x_mini/rgbd/image')
        if image is not None:
            checks[f'TF world -> {image.header.frame_id} (camera image frame)'] = (
                buffer.can_transform('world', image.header.frame_id, rclpy.time.Time()))

    width = None
    if latest is not None and set(JOINTS).issubset(latest.name):
        positions = dict(zip(latest.name, latest.position))
        width = positions['fr3_finger_joint1'] + positions['fr3_finger_joint2']
    for name, ok in checks.items():
        print(f'[{"PASS" if ok else "FAIL"}] {name}')
    print(f'[INFO] hardware: {"real FR3 (franka_hardware)" if real else "mock hardware"}')
    arm = state.get('fr3_arm_controller', 'not loaded')
    print(f'[INFO] fr3_arm_controller {arm}: ' + (
        f'the arm holds its pose{" under torque control" if real else ""} and accepts goals'
        if arm == 'active'
        else 'the arm is not commanded'))
    if robot:
        errors = robot[-1].current_errors
        active_errors = [n for n in errors.get_fields_and_field_types() if getattr(errors, n)]
        print(f'[INFO] robot mode {MODES.get(robot[-1].robot_mode, robot[-1].robot_mode)}, '
              f'current errors: {", ".join(active_errors) or "none"}')
    if width is not None:
        print(f'[INFO] gripper opening {width*1000:.1f} mm')
    info = camera.get('/zed_x_mini/rgbd/camera_info')
    if info is not None:
        print(f'[INFO] camera {info.width} x {info.height}, fx {info.k[0]:.1f} px, '
              f'frame {info.header.frame_id}')
        print(f'[INFO] {IMU}: {"received" if IMU in camera else "no data (not in the stream)"}')
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__ == '__main__':
    main()
