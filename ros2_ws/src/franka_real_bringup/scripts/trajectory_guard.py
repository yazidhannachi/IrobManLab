#!/usr/bin/env python3
"""Check arm trajectory goals before they reach the real arm controller.

Offers /fr3_arm_controller/follow_joint_trajectory, the action MoveIt and
students use, and forwards valid goals to the controller's own action, which
the bringup moves to a hidden name. A goal is rejected (aborted with
INVALID_GOAL or INVALID_JOINTS and the reason in error_string) unless:
  - it names exactly the 7 arm joints and contains only finite numbers,
  - its times increase and every point lies within the joint position limits,
  - it starts at the current measured position (a point at time 0 must be
    within start_tolerance), and
  - no segment, including the one from the current position to the first
    point, is faster on average than the MoveIt velocity caps, nor any given
    point velocity.
Feedback, cancel requests and the controller's result are passed through.
"""
import math
from pathlib import Path
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
Result = FollowJointTrajectory.Result


class TrajectoryGuard(Node):
    def __init__(self):
        super().__init__('trajectory_guard')
        share = Path(get_package_share_directory('franka_real_bringup'))
        caps = yaml.safe_load((share / 'config/joint_limits.yaml').read_text())['joint_limits']
        description = Path(get_package_share_directory('franka_description'))
        limits = yaml.safe_load((description / 'robots/fr3/joint_limits.yaml').read_text())
        self.speed = [caps[j]['max_velocity'] for j in JOINTS]
        self.lower = [limits[f'joint{i}']['limit']['lower'] for i in range(1, 8)]
        self.upper = [limits[f'joint{i}']['limit']['upper'] for i in range(1, 8)]
        self.speed_tolerance = self.declare_parameter('speed_tolerance', 1.05).value
        self.start_tolerance = self.declare_parameter('start_tolerance', .02).value
        target = self.declare_parameter(
            'controller_action', '/fr3_arm_controller/_unchecked/follow_joint_trajectory').value
        self.state = None
        group = ReentrantCallbackGroup()
        # The controller's own joint states: /joint_states keeps republishing stale values.
        self.create_subscription(JointState, '/franka/joint_states', self.on_state, 1,
                                 callback_group=group)
        self.controller = ActionClient(self, FollowJointTrajectory, target, callback_group=group)
        self.server = ActionServer(
            self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute, goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.get_logger().info(f'Checking arm trajectory goals for {target}.')

    def on_state(self, msg):
        self.state = (time.monotonic(), dict(zip(msg.name, msg.position)))

    def check(self, trajectory):
        """Return (error_code, reason) for an invalid trajectory, else None."""
        names = list(trajectory.joint_names)
        if sorted(names) != sorted(JOINTS):
            return Result.INVALID_JOINTS, f'joint_names must be exactly {JOINTS}, got {names}'
        if not trajectory.points:
            return Result.INVALID_GOAL, 'the trajectory has no points'
        if self.state is None or time.monotonic() - self.state[0] > .5:
            return Result.INVALID_GOAL, 'no current joint state on /franka/joint_states'
        order = [names.index(j) for j in JOINTS]
        previous_time, previous = 0., [self.state[1][j] for j in JOINTS]
        for k, point in enumerate(trajectory.points):
            if len(point.positions) != len(JOINTS):
                return Result.INVALID_GOAL, f'point {k} has {len(point.positions)} positions, not 7'
            values = [*point.positions, *point.velocities, *point.accelerations]
            if not all(math.isfinite(v) for v in values):
                return Result.INVALID_GOAL, f'point {k} contains NaN or infinite values'
            q = [point.positions[i] for i in order]
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if t < previous_time or (k > 0 and t == previous_time):
                return Result.INVALID_GOAL, f'time_from_start of point {k} does not increase'
            for j, value, low, high in zip(JOINTS, q, self.lower, self.upper):
                if not low <= value <= high:
                    return (Result.INVALID_GOAL,
                            f'point {k}: {j} = {value:.3f} rad is outside [{low}, {high}]')
            if t == 0.:
                offset = max(abs(a - b) for a, b in zip(q, previous))
                if offset > self.start_tolerance:
                    return (Result.INVALID_GOAL,
                            f'point 0 at time 0 is {offset:.3f} rad from the current position; '
                            'start at the current position or give it time to get there')
            else:
                for j, a, b, cap in zip(JOINTS, q, previous, self.speed):
                    speed = abs(a - b) / (t - previous_time)
                    if speed > cap * self.speed_tolerance:
                        start = 'the current position' if k == 0 else f'point {k - 1}'
                        return (Result.INVALID_GOAL,
                                f'{j} would move from {start} to point {k} at {speed:.2f} rad/s '
                                f'(limit {cap} rad/s); allow more time')
            if point.velocities:
                for j, i, cap in zip(JOINTS, order, self.speed):
                    if abs(point.velocities[i]) > cap * self.speed_tolerance:
                        return (Result.INVALID_GOAL, f'point {k}: {j} velocity '
                                f'{point.velocities[i]:.2f} rad/s exceeds {cap} rad/s')
            previous_time, previous = t, q
        return None

    def execute(self, handle):
        result = Result()
        problem = self.check(handle.request.trajectory)
        if problem:
            result.error_code, result.error_string = problem
            self.get_logger().warn(f'Rejected arm goal: {result.error_string}')
            handle.abort()
            return result
        if not self.controller.wait_for_server(timeout_sec=2.):
            return self.fail(handle, result, 'the arm trajectory controller is unavailable')
        sent = self.controller.send_goal_async(
            handle.request, feedback_callback=lambda f: handle.publish_feedback(f.feedback))
        if not self.wait(sent) or not sent.result().accepted:
            return self.fail(handle, result, 'fr3_arm_controller did not accept the goal; '
                             'is it active? (ros2 control list_controllers)')
        inner = sent.result()
        done = inner.get_result_async()
        while not done.done():
            if handle.is_cancel_requested:
                self.wait(inner.cancel_goal_async())
                self.wait(done, 5.)
                handle.canceled()
                return done.result().result if done.done() else result
            time.sleep(.01)
        response = done.result()
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            handle.succeed()
        elif response.status == GoalStatus.STATUS_CANCELED:
            handle.canceled()
        else:
            handle.abort()
        return response.result

    def fail(self, handle, result, error):
        self.get_logger().error(error)
        result.error_code, result.error_string = Result.INVALID_GOAL, error
        handle.abort()
        return result

    @staticmethod
    def wait(future, timeout=5.):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(.01)
        return future.done()


def main():
    rclpy.init()
    node = TrajectoryGuard()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
