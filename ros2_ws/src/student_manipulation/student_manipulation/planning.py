"""Suggested planning interface."""
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from .perception import ObjectEstimate


def plan(goal: PoseStamped, objects: list[ObjectEstimate]) -> RobotTrajectory:
    """TODO: represent obstacles, choose a motion and check its feasibility."""
    raise NotImplementedError
