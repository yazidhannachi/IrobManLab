#!/usr/bin/env python3
"""First supervised arm test: move one joint by a small angle and back.

Without --execute this is a dry run: it prints the planned goal and sends
nothing. With --execute it asks for typed confirmation, then sends one slow
goal to /fr3_arm_controller/follow_joint_trajectory and cancels it on Ctrl+C.
Only execute on the real robot while someone watches it with the external
activation/stop device in hand.
"""
import argparse
from collections import deque
from pathlib import Path
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
MAX_DELTA = .3        # [rad]
MAX_SPEED = .2        # [rad/s]
LIMIT_MARGIN = .1     # [rad] kept from the joint position limits
QUINTIC_PEAK = 1.875  # peak/mean velocity of a rest-to-rest quintic segment


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--joint', type=int, default=7, choices=range(1, 8),
                        help='joint to move (default 7, the wrist rotation)')
    parser.add_argument('--delta', type=float, default=.1,
                        help=f'angle to move and return [rad], |delta| <= {MAX_DELTA}')
    parser.add_argument('--speed', type=float, default=.1,
                        help=f'peak joint speed [rad/s], at most {MAX_SPEED}')
    parser.add_argument('--execute', action='store_true',
                        help='send the goal after confirmation (default: dry run)')
    args = parser.parse_args()
    if not 0 < abs(args.delta) <= MAX_DELTA:
        parser.error(f'|delta| must be in (0, {MAX_DELTA}] rad')
    if not 0 < args.speed <= MAX_SPEED:
        parser.error(f'speed must be in (0, {MAX_SPEED}] rad/s')
    return args


def joint_limits(joint):
    path = Path(get_package_share_directory('franka_description')) / 'robots/fr3/joint_limits.yaml'
    limit = yaml.safe_load(path.read_text())[f'joint{joint}']['limit']
    return limit['lower'], limit['upper']


def main():
    args = parse_args()
    # Without rclpy's handlers, Ctrl+C raises KeyboardInterrupt but keeps the
    # context alive, so a running goal can still be cancelled.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('lab_arm_motion_test')
    try:
        return run(node, args)
    except KeyboardInterrupt:
        print('Interrupted before a goal was sent.')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def call(node, service, kind):
    client = node.create_client(kind, service)
    if not client.wait_for_service(timeout_sec=5):
        return None
    future = client.call_async(kind.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    return future.result() if future.done() else None


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.01)


def run(node, args):
    # The broadcaster's own topic; /joint_states keeps republishing stale values.
    states = deque(maxlen=1)
    node.create_subscription(JointState, '/franka/joint_states', lambda msg: states.append(msg), 10)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and not states:
        rclpy.spin_once(node, timeout_sec=.1)
    if not states:
        print('No /franka/joint_states received; is real.launch.py running?')
        return 1
    spin_for(node, .2)
    state = states[-1]
    age = (node.get_clock().now() - rclpy.time.Time.from_msg(state.header.stamp)).nanoseconds * 1e-9
    if age > .5:
        print(f'/franka/joint_states is {age:.1f} s old; check the bringup and its clock.')
        return 1
    position = dict(zip(state.name, state.position))
    velocity = dict(zip(state.name, state.velocity))
    if not set(JOINTS) <= position.keys():
        print('/franka/joint_states does not contain all arm joints.')
        return 1
    if any(abs(velocity.get(j, 0.)) > .02 for j in JOINTS):
        print('The arm is moving; wait until it is at rest.')
        return 1

    name = f'fr3_joint{args.joint}'
    start = [position[j] for j in JOINTS]
    lower, upper = joint_limits(args.joint)
    target = position[name] + args.delta
    if not lower + LIMIT_MARGIN <= target <= upper - LIMIT_MARGIN:
        print(f'{name} target {target:.3f} rad is within {LIMIT_MARGIN} rad of its limits '
              f'[{lower}, {upper}]; choose the other sign or a smaller delta.')
        return 1
    segment = max(2., QUINTIC_PEAK * abs(args.delta) / args.speed)

    trajectory = JointTrajectory(joint_names=JOINTS)
    for t, value in [(segment, target), (2 * segment, position[name])]:
        positions = list(start)
        positions[args.joint - 1] = value
        # Zero velocities and accelerations: smooth rest-to-rest quintic segments.
        trajectory.points.append(JointTrajectoryPoint(
            positions=positions, velocities=[0.] * 7, accelerations=[0.] * 7,
            time_from_start=Duration(seconds=t).to_msg()))

    print(f'{name}: {position[name]:+.3f} -> {target:+.3f} -> {position[name]:+.3f} rad, '
          f'{segment:.1f} s per segment, peak speed {QUINTIC_PEAK * abs(args.delta) / segment:.3f} '
          'rad/s. All other joints hold their current positions.')
    if not args.execute:
        print('Dry run: nothing was sent. Add --execute to send the goal.')
        return 0

    hardware = call(node, '/controller_manager/list_hardware_components', ListHardwareComponents)
    real = hardware is not None and any(
        c.plugin_name == 'franka_hardware/FrankaHardwareInterface' for c in hardware.component)
    controllers = call(node, '/controller_manager/list_controllers', ListControllers)
    if controllers is None or not any(c.name == 'fr3_arm_controller' and c.state == 'active'
                                      for c in controllers.controller):
        print('fr3_arm_controller is not active (arm:=inactive?). Activate it first.')
        return 1
    client = ActionClient(node, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=5):
        print('The arm trajectory action is unavailable.')
        return 1
    target_name = 'the REAL ROBOT' if real else 'mock hardware (no robot)'
    try:
        answer = input(f'This moves {target_name}. Area clear and stop device in hand? '
                       'Type "move" to send: ')
    except EOFError:
        answer = ''
    if answer.strip() != 'move':
        print('Cancelled; nothing was sent.')
        return 0

    # The goal must start where the arm is now, not where it was before the prompt.
    spin_for(node, .2)
    now = dict(zip(states[-1].name, states[-1].position))
    if max(abs(now[j] - p) for j, p in zip(JOINTS, start)) > .01:
        print('The arm moved since the goal was planned; nothing was sent. Run again.')
        return 1
    sent = client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
    try:
        rclpy.spin_until_future_complete(node, sent, timeout_sec=5)
        handle = sent.result() if sent.done() else None
        if handle is None or not handle.accepted:
            print('The controller did not accept the goal.')
            return 1
        print('Moving. Press Ctrl+C to cancel (the arm then holds its current pose).')
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(node, done, timeout_sec=2 * segment + 10)
    except KeyboardInterrupt:
        rclpy.spin_until_future_complete(node, sent, timeout_sec=2)
        if sent.done() and sent.result() is not None and sent.result().accepted:
            cancel = sent.result().cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2)
            print('Cancel requested; the arm holds its current pose.')
        return 1
    if not done.done():
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=2)
        print('No result in time; cancel requested.')
        return 1
    result = done.result().result
    print(f'Result: error_code {result.error_code} {result.error_string}'.strip())
    return 0 if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL else 1


if __name__ == '__main__':
    raise SystemExit(main())
