"""Suggested execution interfaces; no commands are sent by these stubs."""
from moveit_msgs.msg import RobotTrajectory


def execute(trajectory: RobotTrajectory) -> bool:
    """TODO: execute a trajectory and evaluate completion from feedback."""
    raise NotImplementedError


def set_gripper(opening_m: float) -> bool:
    """TODO: command the total finger separation and evaluate the result."""
    raise NotImplementedError
