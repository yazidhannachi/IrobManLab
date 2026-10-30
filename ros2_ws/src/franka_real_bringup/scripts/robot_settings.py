#!/usr/bin/env python3
"""Apply the payload and collision thresholds, then activate the arm controller.

Runs once at bringup. franka_hardware accepts these settings only while no
controller commands the robot, so the arm controller is spawned inactive and
activated here afterwards (activate_arm). With fake:=true there is no robot to
configure and only the activation is done. Any failure exits non-zero, which
stops the bringup with the arm still inactive.
"""
import time

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController
from franka_msgs.srv import SetFullCollisionBehavior, SetLoad

ARM = 'fr3_arm_controller'
TIMEOUT = 60.


class RobotSettings(Node):
    def __init__(self):
        super().__init__('robot_settings')
        self.fake = self.declare_parameter('fake', False).value
        self.activate = self.declare_parameter('activate_arm', True).value
        self.mass = float(self.declare_parameter('load_mass', 0.).value)
        self.com = [float(v) for v in self.declare_parameter('load_center_of_mass', [0.] * 3).value]
        self.inertia = [float(v) for v in self.declare_parameter('load_inertia', [0.] * 9).value]
        self.torques = [float(v) for v in
                        self.declare_parameter('collision_torque_thresholds', [20.] * 7).value]
        self.forces = [float(v) for v in
                       self.declare_parameter('collision_force_thresholds', [20.] * 6).value]

    def call(self, kind, name, request):
        client = self.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=TIMEOUT):
            raise RuntimeError(f'{name} is unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{name} did not answer')
        return future.result()

    def arm_state(self):
        end = time.monotonic() + TIMEOUT
        while time.monotonic() < end:
            controllers = self.call(ListControllers, '/controller_manager/list_controllers',
                                    ListControllers.Request()).controller
            state = {c.name: c.state for c in controllers}.get(ARM)
            if state in ('inactive', 'active'):
                return state
            time.sleep(.5)
        raise RuntimeError(f'{ARM} was not loaded')

    def run(self):
        if self.arm_state() == 'active':
            raise RuntimeError(f'{ARM} is already active; these settings need an uncontrolled robot')
        if self.fake:
            self.get_logger().info('Mock hardware: no payload or collision thresholds to set.')
        else:
            result = self.call(SetLoad, '/service_server/set_load', SetLoad.Request(
                mass=self.mass, center_of_mass=self.com, load_inertia=self.inertia))
            if not result.success:
                raise RuntimeError(f'set_load failed: {result.error}')
            torques, forces = self.torques, self.forces
            result = self.call(
                SetFullCollisionBehavior, '/service_server/set_full_collision_behavior',
                SetFullCollisionBehavior.Request(
                    lower_torque_thresholds_acceleration=torques,
                    upper_torque_thresholds_acceleration=torques,
                    lower_torque_thresholds_nominal=torques,
                    upper_torque_thresholds_nominal=torques,
                    lower_force_thresholds_acceleration=forces,
                    upper_force_thresholds_acceleration=forces,
                    lower_force_thresholds_nominal=forces,
                    upper_force_thresholds_nominal=forces))
            if not result.success:
                raise RuntimeError(f'set_full_collision_behavior failed: {result.error}')
            self.get_logger().info(
                f'Payload {self.mass:.4f} kg at {self.com} m and collision thresholds '
                f'{torques} Nm / {forces} N set.')
        if not self.activate:
            self.get_logger().info(f'{ARM} left inactive (arm:=inactive).')
            return
        result = self.call(SwitchController, '/controller_manager/switch_controller',
                           SwitchController.Request(activate_controllers=[ARM],
                                                    strictness=SwitchController.Request.STRICT))
        if not result.ok:
            raise RuntimeError(f'could not activate {ARM}: {result.message}')
        self.get_logger().info(f'{ARM} active: the arm holds its pose and accepts goals.')


def main():
    rclpy.init()
    node = RobotSettings()
    try:
        node.run()
    except RuntimeError as error:
        node.get_logger().fatal(str(error))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
