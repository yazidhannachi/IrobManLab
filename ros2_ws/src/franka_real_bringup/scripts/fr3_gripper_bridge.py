#!/usr/bin/env python3
"""Offer the Franka Hand as /fr3_gripper/follow_joint_trajectory, as in simulation.

In the lab simulation the gripper is a trajectory controller on both finger
joints. On hardware franka_gripper_node offers move and grasp actions instead.
The last trajectory point gives the total opening (sum of both fingers; MoveIt
sends only fr3_finger_joint1, which fr3_finger_joint2 mimics, so it counts twice):
opening goals become moves, closing goals become grasps that keep holding with
`grasp_force`. A grasp succeeds wherever the fingers stop, like the simulated
controller; clients check the measured opening on /joint_states for contact.
With fake:=true no hardware is used and finger states are simulated.
"""
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from franka_msgs.action import Grasp, Move
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

JOINTS = ['fr3_finger_joint1', 'fr3_finger_joint2']
MAX_OPENING = .08
TIMEOUT = 20.


class GripperBridge(Node):
    def __init__(self):
        super().__init__('fr3_gripper_bridge')
        self.fake = self.declare_parameter('fake', False).value
        self.force = float(self.declare_parameter('grasp_force', 20.).value)
        self.speed = min(float(self.declare_parameter('speed', .05).value), .1)
        self.epsilon = float(self.declare_parameter('grasp_epsilon', .08).value)
        self.opening = 0. if self.fake else None
        self.busy = threading.Lock()
        group = ReentrantCallbackGroup()
        if self.fake:
            self.states = self.create_publisher(JointState, '/franka_gripper/joint_states', 10)
            self.create_timer(1/30, self.publish_fake_state, callback_group=group)
        else:
            self.create_subscription(JointState, '/franka_gripper/joint_states', self.on_state, 10,
                                     callback_group=group)
            self.move = ActionClient(self, Move, '/franka_gripper/move', callback_group=group)
            self.grasp = ActionClient(self, Grasp, '/franka_gripper/grasp', callback_group=group)
            self.stop = self.create_client(Trigger, '/franka_gripper/stop', callback_group=group)
        self.server = ActionServer(
            self, FollowJointTrajectory, '/fr3_gripper/follow_joint_trajectory',
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.get_logger().info(f'Gripper bridge ready ({"fake" if self.fake else "franka_gripper"}).')

    def on_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in JOINTS):
            self.opening = sum(positions[j] for j in JOINTS)

    def publish_fake_state(self):
        msg = JointState(name=JOINTS, position=[self.opening/2]*2, velocity=[0.]*2, effort=[0.]*2)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.states.publish(msg)

    def on_goal(self, goal):
        trajectory = goal.trajectory
        if sorted(trajectory.joint_names) not in (JOINTS, JOINTS[:1]) or not trajectory.points:
            self.get_logger().error(f'Gripper goals must command {JOINTS} or {JOINTS[0]}')
            return GoalResponse.REJECT
        if len(trajectory.points[-1].positions) != len(trajectory.joint_names):
            return GoalResponse.REJECT
        if self.busy.locked():
            self.get_logger().error('Gripper is busy; wait for or cancel the active goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, handle):
        with self.busy:
            point = handle.request.trajectory.points[-1]
            opening = sum(point.positions) * len(JOINTS) / len(point.positions)
            target = min(max(opening, 0.), MAX_OPENING)
            duration = point.time_from_start.sec + point.time_from_start.nanosec*1e-9
            result = FollowJointTrajectory.Result()
            if self.opening is None:
                return self.finish(handle, result, 'No /franka_gripper/joint_states received')
            if self.fake:
                return self.simulate(handle, result, target, duration)
            if target >= self.opening - .001:
                client, goal = self.move, Move.Goal(width=target, speed=self.speed)
            else:
                goal = Grasp.Goal(width=target, speed=self.speed, force=self.force)
                goal.epsilon.inner = goal.epsilon.outer = self.epsilon
                client = self.grasp
            if not client.wait_for_server(timeout_sec=2.):
                return self.finish(handle, result, 'franka_gripper actions are unavailable')
            sent = client.send_goal_async(goal)
            if not self.wait(sent, handle) or not sent.result().accepted:
                return self.interrupt(handle, result, 'franka_gripper did not accept the goal')
            done = sent.result().get_result_async()
            if not self.wait(done, handle):
                sent.result().cancel_goal_async()
                if self.stop.service_is_ready():
                    self.stop.call_async(Trigger.Request())
                return self.interrupt(handle, result, 'Gripper motion timed out')
            outcome = done.result().result
            if not outcome.success:
                return self.finish(handle, result, f'franka_gripper: {outcome.error}')
            handle.succeed()
            return result

    def simulate(self, handle, result, target, duration):
        start, begin = self.opening, time.monotonic()
        duration = max(duration, abs(target-start)/self.speed)
        while (elapsed := time.monotonic()-begin) < duration:
            if handle.is_cancel_requested:
                handle.canceled()
                return result
            self.opening = start + (target-start)*elapsed/duration
            time.sleep(.02)
        self.opening = target
        handle.succeed()
        return result

    def wait(self, future, handle):
        end = time.monotonic() + TIMEOUT
        while not future.done():
            if handle.is_cancel_requested or time.monotonic() > end:
                return False
            time.sleep(.01)
        return True

    def interrupt(self, handle, result, error):
        if handle.is_cancel_requested:
            handle.canceled()
            return result
        return self.finish(handle, result, error)

    def finish(self, handle, result, error):
        self.get_logger().error(error)
        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        result.error_string = error
        handle.abort()
        return result


def main():
    rclpy.init()
    node = GripperBridge()
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
