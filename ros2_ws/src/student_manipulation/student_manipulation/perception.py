"""Suggested perception interface; choose your team's own implementation."""
from dataclasses import dataclass
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import CameraInfo, Image


@dataclass
class ObjectEstimate:
    label: str
    pose: PoseStamped  # explicit frame and observation timestamp
    size: Vector3     # metres


def detect(rgb: Image, depth: Image, calibration: CameraInfo) -> list[ObjectEstimate]:
    """TODO: estimate labels, poses and sizes from synchronized observations."""
    raise NotImplementedError
