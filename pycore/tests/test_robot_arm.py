import math

import pytest
from myarm_sdk.core import (
    JointMetadata,
    JointPositions,
    RobotArmLifecycleError,
    RobotArmLimitError,
    RobotArmProtocolError,
)
from myarm_sdk.plugin_adapter.robot_arm import (
    FakeRobotArm,
    FakeRobotArmAdapter,
    MyArmM750RobotArm,
    MyArmM750RobotArmAdapter,
)

HOME = JointPositions((0.0, -0.35, 0.70, 0.0, -0.35, 0.0))
TARGET = JointPositions((0.15, -0.20, 0.30, -0.40, 0.50, -0.60))
ZERO = JointPositions((0.0,) * 6)


class _FakeSerialTransport:
    def __init__(self):
        self.is_closed = False

    def close(self):
        self.is_closed = True


class _FakeMyArmMControl:
    def __init__(self, _port, baudrate, timeout, debug):
        self.connection = {
            "baudrate": baudrate,
            "timeout": timeout,
            "debug": debug,
        }
        self.fresh_mode = 0
        self.powered = 1
        self.moving = 0
        self.angles = [0.0, 10.0, -10.0, 0.0, 0.0, 0.0]
        self.write_calls = []
        self.stop_calls = 0
        self._serial_port = _FakeSerialTransport()

    def set_fresh_mode(self, mode):
        self.fresh_mode = mode
        return 1

    def get_fresh_mode(self):
        return self.fresh_mode

    def is_powered_on(self):
        return self.powered

    def power_on(self):
        self.powered = 1
        return 1

    def power_off(self):
        self.powered = 0
        return 1

    def get_angles(self):
        return self.angles

    def write_angles(self, angles, speed):
        self.write_calls.append((list(angles), speed))
        return 1

    def stop(self):
        self.stop_calls += 1
        self.moving = 0
        return 1

    def is_moving(self):
        return self.moving


class _FreshModeMismatchMyArmMControl(_FakeMyArmMControl):
    def set_fresh_mode(self, _mode):
        return 1


def _joint_metadata():
    return tuple(
        JointMetadata(
            name=name,
            axis_xyz=(0.0, 0.0, 1.0),
            lower_limit_rad=-1.0,
            upper_limit_rad=1.0,
            velocity_limit_rad_s=1.0,
        )
        for name in FakeRobotArm.DEFAULT_JOINT_NAMES
    )


def test_fake_robot_arm_is_a_deterministic_memory_robot():
    arm = FakeRobotArm(
        initial_joint_positions=HOME,
        joint_metadata=_joint_metadata(),
        start_connected=False,
        start_powered=False,
    )

    with pytest.raises(RobotArmLifecycleError):
        arm.write_joint_positions(TARGET)

    arm.connect()
    arm.power_on()
    command = arm.write_joint_positions(TARGET, speed_scale=0.35)

    assert command.requested_joint_positions == TARGET
    assert command.accepted_joint_positions == TARGET
    assert arm.state.last_command == command
    assert arm.state.measured_joint_positions == TARGET
    assert arm.read_state().measured_joint_positions == TARGET

    arm.stop()
    assert arm.state.is_moving is False
    arm.power_off()
    with pytest.raises(RobotArmLifecycleError):
        arm.write_joint_positions(HOME)


def test_fake_robot_arm_defaults_to_the_configured_safe_home_pose():
    arm = FakeRobotArm(joint_metadata=_joint_metadata())

    assert arm.state.measured_joint_positions == HOME


def test_fake_robot_arm_rejects_a_target_outside_injected_urdf_limits():
    arm = FakeRobotArm(joint_metadata=_joint_metadata())

    with pytest.raises(RobotArmLimitError):
        arm.write_joint_positions(
            JointPositions((1.01, -0.20, 0.30, -0.40, 0.50, -0.60))
        )


def test_robot_arm_adapter_aliases_preserve_existing_imports():
    assert FakeRobotArmAdapter is FakeRobotArm
    assert MyArmM750RobotArmAdapter is MyArmM750RobotArm


def test_myarm_m750_robot_arm_uses_myarm_mcontrol_and_keeps_actual_state_separate():
    vendor = _FakeMyArmMControl("ignored", 0, 0, False)
    factory_calls = []

    def vendor_factory(*args, **kwargs):
        factory_calls.append((args, kwargs))
        return vendor

    arm = MyArmM750RobotArm(
        serial_port="/dev/myarm_m750",
        joint_metadata=_joint_metadata(),
        vendor_factory=vendor_factory,
    )
    connected = arm.connect()

    assert connected.is_connected is True
    assert connected.is_powered is True
    assert connected.measured_joint_positions is not None
    assert connected.measured_joint_positions.values == pytest.approx(ZERO.values)
    assert vendor.fresh_mode == 1
    assert factory_calls == [
        (
            ("/dev/myarm_m750",),
            {"baudrate": 1_000_000, "timeout": 0.1, "debug": False},
        )
    ]

    measured = arm.read_state()
    assert measured.measured_joint_positions is not None
    assert measured.measured_joint_positions.values == pytest.approx(ZERO.values)
    measured_before_command = measured.measured_joint_positions

    command = arm.write_joint_positions(TARGET, speed_scale=0.42)

    assert vendor.write_calls[0][1] == 42
    expected_hardware_deg = [
        math.degrees(value)
        for value in arm.hardware_from_model_positions(TARGET)
    ]
    assert vendor.write_calls[0][0] == pytest.approx(expected_hardware_deg)
    assert arm.state.measured_joint_positions == measured_before_command
    assert arm.state.last_command == command
    assert command.requested_joint_positions == TARGET
    assert command.accepted_joint_positions != TARGET

    arm.stop()
    assert vendor.stop_calls == 1
    assert arm.state.is_moving is None
    assert arm.read_motion_state().is_moving is False
    arm.disconnect()
    assert vendor._serial_port.is_closed is True
    assert arm.state.is_connected is False


def test_myarm_m750_robot_arm_rejects_vendor_sentinel_and_does_not_replace_measurement():
    vendor = _FakeMyArmMControl("ignored", 0, 0, False)
    arm = MyArmM750RobotArm(
        serial_port="/dev/myarm_m750",
        joint_metadata=_joint_metadata(),
        vendor_factory=lambda *_args, **_kwargs: vendor,
    )
    arm.connect()
    original = arm.read_state().measured_joint_positions
    vendor.angles = -1

    with pytest.raises(RobotArmProtocolError):
        arm.read_state()

    assert arm.state.measured_joint_positions == original
    assert arm.state.consecutive_error_count == 1
    assert arm.state.last_error_message is not None


def test_myarm_m750_robot_arm_rejects_invalid_fresh_mode_and_feedback_limit():
    mismatch_vendor = _FreshModeMismatchMyArmMControl("ignored", 0, 0, False)
    arm = MyArmM750RobotArm(
        serial_port="/dev/myarm_m750",
        joint_metadata=_joint_metadata(),
        vendor_factory=lambda *_args, **_kwargs: mismatch_vendor,
    )

    with pytest.raises(RobotArmProtocolError, match="fresh mode"):
        arm.connect()

    assert arm.state.is_connected is False
    assert arm.state.consecutive_error_count == 1
    assert arm.state.last_error_message is not None

    vendor = _FakeMyArmMControl("ignored", 0, 0, False)
    arm = MyArmM750RobotArm(
        serial_port="/dev/myarm_m750",
        joint_metadata=_joint_metadata(),
        vendor_factory=lambda *_args, **_kwargs: vendor,
    )
    original = arm.connect().measured_joint_positions
    vendor.angles[0] = 70.0

    with pytest.raises(RobotArmLimitError):
        arm.read_state()

    assert arm.state.measured_joint_positions == original
    assert arm.state.consecutive_error_count == 1
