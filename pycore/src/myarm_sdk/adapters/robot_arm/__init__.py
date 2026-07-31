"""Robot-arm adapters."""

from .fake_robot_arm import FakeRobotArm
from .pymycobot_robot_arm import PymycobotRobotArm

__all__ = ["FakeRobotArm", "PymycobotRobotArm"]
