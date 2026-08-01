"""ROS-independent contracts implemented by plugin adapters."""

from .camera import CameraInterface
from .kinematics import KinematicsInterface
from .motion_execution import MotionExecutionInterface
from .robot_arm import RobotArmInterface
from .trajectory import TrajectoryPlannerInterface

__all__ = [
    "CameraInterface",
    "KinematicsInterface",
    "MotionExecutionInterface",
    "RobotArmInterface",
    "TrajectoryPlannerInterface",
]
