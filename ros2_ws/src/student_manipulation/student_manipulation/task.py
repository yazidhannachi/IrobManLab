"""Idle task node. Implement your team's orchestration here."""
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def main():
    rclpy.init()
    # Simulation time in the simulator container, wall-clock time on the real robot.
    sim = os.environ.get('LAB_MODE', 'sim') == 'sim'
    node = Node('student_task', parameter_overrides=[Parameter('use_sim_time', value=sim)])
    node.get_logger().info('Student skeleton is idle. Implement your pipeline here.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
