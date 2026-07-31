"""Stateful fake robot arm for development and tests."""

from myarm_sdk.model import JointPositions


class FakeRobotArm:
    def __init__(self, initial: JointPositions = None) -> None:
        self._joints = initial or JointPositions((0, 0, 0, 0, 0, 0))
        self._closed = False

    def read_joints(self) -> JointPositions:
        self._require_open()
        return self._joints

    def move_joints(self, target: JointPositions, speed: int = 50) -> None:
        self._require_open()
        if not 1 <= speed <= 100:
            raise ValueError("speed must be in the range 1..100")
        self._joints = target

    def close(self) -> None:
        self._closed = True

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("robot arm is closed")
