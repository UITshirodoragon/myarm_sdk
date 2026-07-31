"""ROS-independent contracts implemented by plugin adapters."""

from .camera import CameraInterface
from .controller import ControllerInterface
from .kinematics import KinematicsInterface
from .robot_arm import RobotArmInterface
from .trajectory import TrajectoryInterface

__all__ = [
    "CameraInterface",
    "ControllerInterface",
    "KinematicsInterface",
    "RobotArmInterface",
    "TrajectoryInterface",
]
