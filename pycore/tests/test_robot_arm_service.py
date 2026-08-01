from copy import deepcopy
from pathlib import Path

import pytest
from myarm_sdk.core import (
    JointPositions,
    RobotArmLifecycleError,
    load_sdk_yaml,
    load_urdf_joint_metadata,
)
from myarm_sdk.plugin_adapter.robot_arm import FakeRobotArm, MyArmM750RobotArm
from myarm_sdk.service import RobotArmService, RobotArmServiceError

PROJECT_ROOT = Path(__file__).resolve().parents[2]
HOME = JointPositions((0.0, -0.35, 0.70, 0.0, -0.35, 0.0))
TARGET = JointPositions((0.15, -0.20, 0.30, -0.40, 0.50, -0.60))


class _FakeMyArmMControl:
    def __init__(self, _port, baudrate, timeout, debug):
        self.connection = (baudrate, timeout, debug)
        self.fresh_mode = 0
        self.powered = 0
        self.angles = [0.0, 10.0, -10.0, 0.0, 0.0, 0.0]
        self.power_on_calls = 0
        self.gripper_enabled = False
        self.gripper_value = 0

    def set_fresh_mode(self, mode):
        self.fresh_mode = mode
        return 1

    def get_fresh_mode(self):
        return self.fresh_mode

    def is_powered_on(self):
        return self.powered

    def power_on(self):
        self.power_on_calls += 1
        self.powered = 1
        return 1

    def power_off(self):
        self.powered = 0
        return 1

    def get_angles(self):
        return self.angles

    def write_angles(self, _angles, _speed):
        return 1

    def stop(self):
        return 1

    def is_moving(self):
        return 0

    def set_gripper_enabled(self):
        self.gripper_enabled = True
        return 1

    def get_gripper_value(self):
        return self.gripper_value

    def set_gripper_value(self, value, _speed):
        self.gripper_value = value
        return 1

    def is_gripper_moving(self):
        return 0


def _package_share_directory(package_name):
    return str(PROJECT_ROOT / "ros2_ws" / "src" / package_name)


def _config():
    return load_sdk_yaml("service/config/services.yaml")


def test_framework_free_urdf_loader_returns_canonical_order_axes_and_limits():
    config = _config()
    metadata = load_urdf_joint_metadata(
        PROJECT_ROOT / "ros2_ws/src/myarm_description/urdf/myarm_m750_poe_v3_2.urdf",
        config["robot"]["joint_order"]["names"],
    )

    assert tuple(item.name for item in metadata) == (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    )
    assert tuple(item.axis_xyz for item in metadata) == (
        (0.0, 0.0, 1.0),
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
    )
    assert metadata[0].lower_limit_rad == pytest.approx(-2.879793265790644)
    assert metadata[4].upper_limit_rad == pytest.approx(2.0943951023931953)


def test_urdf_resolver_allows_colcon_symlink_install_without_directory_escape(
    tmp_path,
):
    source_urdf_directory = tmp_path / "source" / "myarm_description" / "urdf"
    source_urdf_directory.mkdir(parents=True)
    (source_urdf_directory / "robot.urdf").write_text("<robot name='test'/>")
    package_share = tmp_path / "install" / "share" / "myarm_description"
    package_share.mkdir(parents=True)
    (package_share / "urdf").symlink_to(source_urdf_directory, target_is_directory=True)
    robot_config = {
        "robot_description": {
            "package": "myarm_description",
            "relative_path": "urdf/robot.urdf",
        }
    }

    result = RobotArmService._resolve_urdf_path(
        robot_config,
        lambda _package_name: str(package_share),
    )

    assert result == package_share / "urdf" / "robot.urdf"
    assert result.is_file()
    assert result.resolve() == source_urdf_directory / "robot.urdf"

    escaped = {
        "robot_description": {
            "package": "myarm_description",
            "relative_path": "../outside.urdf",
        }
    }
    with pytest.raises(ValueError, match="escapes package share"):
        RobotArmService._resolve_urdf_path(
            escaped,
            lambda _package_name: str(package_share),
        )


def test_fake_robot_service_reads_feedback_then_accepts_execution_setpoint():
    config = _config()
    service = RobotArmService.from_config(
        config["services"]["robot_arm"],
        _package_share_directory,
        config["robot"],
    )

    assert service.joint_names[0] == "shoulder_pan_joint"
    assert service.state.measured_joint_positions == HOME
    assert service.accepts_execution_setpoints is True
    assert service.accepts_gripper_commands is True
    assert service.update_rate_hz == 5.0

    feedback = service.read_feedback(
        now_monotonic_s=service.state.measured_at_monotonic_s + 0.1
    )
    command = service.send_joint_setpoint(TARGET)

    assert feedback.feedback_error is None
    assert feedback.feedback_updated is True
    assert feedback.measured_state_fresh is True
    assert feedback.measured_joint_positions == HOME
    assert command.requested_joint_positions == TARGET
    assert service.state.measured_joint_positions == TARGET

    gripper_command = service.send_gripper_opening(0.08)
    gripper_feedback = service.read_gripper_feedback()
    assert gripper_command.accepted_opening_width_m == pytest.approx(0.08)
    assert gripper_feedback.feedback_updated is True
    assert gripper_feedback.state.opening_width_m == pytest.approx(0.08)


def test_feedback_and_setpoint_gateway_report_failures_without_queuing():
    arm = FakeRobotArm(
        initial_joint_positions=HOME,
        start_connected=False,
        start_powered=False,
    )
    service = RobotArmService(
        robot_arm=arm,
        update_rate_hz=5.0,
        feedback_stale_after_s=0.5,
        accepts_execution_setpoints=True,
    )

    disconnected = service.read_feedback(now_monotonic_s=1.0)
    assert disconnected.feedback_error is not None
    with pytest.raises(RobotArmLifecycleError, match="disconnected"):
        service.send_joint_setpoint(TARGET)

    service.connect()
    unpowered = service.read_feedback(now_monotonic_s=2.0)
    assert unpowered.feedback_error is None
    with pytest.raises(RobotArmLifecycleError, match="not powered"):
        service.send_joint_setpoint(TARGET)

    service.power_on()
    command = service.send_joint_setpoint(TARGET)
    assert command.requested_joint_positions == TARGET
    assert service.state.measured_joint_positions == TARGET


def test_physical_profile_requires_explicit_execution_hardware_opt_in_and_no_constructor_io():
    config = _config()
    physical_service_config = deepcopy(config["services"]["robot_arm"])
    physical_service_config["plugin_adapter"] = "myarm_m750_robot_arm"
    physical_service_config["plugin_config"] = (
        "plugin_adapter/robot_arm/config/myarm_m750_robot_arm.yaml"
    )
    physical_service_config["transport"]["accept_internal_setpoints"] = True
    physical_service_config["transport"]["allow_physical_motion"] = False
    factory_calls = []

    def vendor_factory(*args, **kwargs):
        factory_calls.append((args, kwargs))
        return _FakeMyArmMControl(*args, **kwargs)

    service = RobotArmService.from_config(
        physical_service_config,
        _package_share_directory,
        config["robot"],
        vendor_factory=vendor_factory,
    )

    assert service.accepts_execution_setpoints is False
    assert service.state.is_connected is False
    assert factory_calls == []
    with pytest.raises(RobotArmServiceError, match="internal execution setpoints are disabled"):
        service.send_joint_setpoint(TARGET)


def test_physical_profile_defers_power_until_an_explicit_service_request():
    config = _config()
    physical_service_config = deepcopy(config["services"]["robot_arm"])
    physical_service_config["plugin_adapter"] = "myarm_m750_robot_arm"
    physical_service_config["plugin_config"] = (
        "plugin_adapter/robot_arm/config/myarm_m750_robot_arm.yaml"
    )
    vendor_instances = []

    def vendor_factory(*args, **kwargs):
        vendor = _FakeMyArmMControl(*args, **kwargs)
        vendor_instances.append(vendor)
        return vendor

    service = RobotArmService.from_config(
        physical_service_config,
        _package_share_directory,
        config["robot"],
        vendor_factory=vendor_factory,
    )
    assert vendor_instances == []
    state = service.connect()

    assert state.is_connected is True
    assert state.is_powered is False
    assert vendor_instances[0].power_on_calls == 0


def test_physical_gripper_maps_total_opening_to_vendor_scale_after_explicit_opt_in():
    config = _config()
    physical_service_config = deepcopy(config["services"]["robot_arm"])
    physical_service_config["plugin_adapter"] = "myarm_m750_robot_arm"
    physical_service_config["plugin_config"] = (
        "plugin_adapter/robot_arm/config/myarm_m750_robot_arm.yaml"
    )
    physical_service_config["transport"]["allow_physical_motion"] = True
    physical_service_config["gripper"]["allow_physical_actuation"] = True
    vendor_instances = []

    def vendor_factory(*args, **kwargs):
        vendor = _FakeMyArmMControl(*args, **kwargs)
        vendor_instances.append(vendor)
        return vendor

    service = RobotArmService.from_config(
        physical_service_config,
        _package_share_directory,
        config["robot"],
        vendor_factory=vendor_factory,
    )
    service.connect()
    service.power_on()
    service.enable_gripper()
    command = service.send_gripper_opening(0.08)
    feedback = service.read_gripper_feedback()

    assert service.accepts_gripper_commands is True
    assert vendor_instances[0].gripper_enabled is True
    assert vendor_instances[0].gripper_value == 100
    assert command.accepted_opening_width_m == pytest.approx(0.08)
    assert feedback.state.opening_width_m == pytest.approx(0.08)


def test_connect_can_power_only_after_a_successful_explicit_connection():
    config = _config()
    metadata = load_urdf_joint_metadata(
        PROJECT_ROOT / "ros2_ws/src/myarm_description/urdf/myarm_m750_poe_v3_2.urdf",
        config["robot"]["joint_order"]["names"],
    )
    vendor_instances = []

    def vendor_factory(*args, **kwargs):
        vendor = _FakeMyArmMControl(*args, **kwargs)
        vendor_instances.append(vendor)
        return vendor

    service = RobotArmService(
        robot_arm=MyArmM750RobotArm(
            serial_port="/dev/myarm_m750",
            joint_metadata=metadata,
            vendor_factory=vendor_factory,
        ),
        update_rate_hz=5.0,
        feedback_stale_after_s=0.5,
        power_on_on_connect=True,
        joint_metadata=metadata,
    )

    assert vendor_instances == []
    state = service.connect()

    assert state.is_connected is True
    assert state.is_powered is True
    assert vendor_instances[0].power_on_calls == 1
