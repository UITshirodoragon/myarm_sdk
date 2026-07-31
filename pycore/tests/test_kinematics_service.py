import pytest
from myarm_sdk.core import JointPositions, Pose, load_sdk_yaml
from myarm_sdk.plugin_adapter.kinematics import InverseKinematicsError
from myarm_sdk.service.kinematics import KinematicsService, KinematicsServiceError


class FakeKinematics:
    def forward(self, joints):
        return Pose(
            position=(joints.values[0], joints.values[1], joints.values[2]),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

    def inverse(self, pose, seed):
        return JointPositions(
            (pose.position[0], pose.position[1], pose.position[2]) + seed.values[3:]
        )


class FailingKinematics(FakeKinematics):
    def inverse(self, pose, seed):
        raise InverseKinematicsError("target is unreachable")


def test_kinematics_service_publishes_initial_pose_then_latest_target():
    service = KinematicsService(
        kinematics=FakeKinematics(),
        joint_names=("j1", "j2", "j3", "j4", "j5", "j6"),
        base_frame="base_link",
        initial_joint_positions=JointPositions((0.0,) * 6),
    )

    initial = service.step()
    assert initial.target_active is False
    assert initial.joint_positions == JointPositions((0.0,) * 6)

    service.set_target_pose(
        Pose(position=(0.1, 0.2, 0.3), orientation=(0.0, 0.0, 0.0, 1.0))
    )
    target = service.step()
    assert target.target_active is True
    assert target.joint_positions.values[:3] == (0.1, 0.2, 0.3)
    assert target.tcp_pose.position == (0.1, 0.2, 0.3)


def test_service_manifest_has_enabled_kinematics_at_five_hz():
    config = load_sdk_yaml("service/config/services.yaml")
    kinematics = config["services"]["kinematics"]

    assert kinematics["enabled"] is True
    assert kinematics["update_rate_hz"] == 5.0


def test_kinematics_service_hides_the_plugin_specific_ik_error():
    service = KinematicsService(
        kinematics=FailingKinematics(),
        joint_names=("j1", "j2", "j3", "j4", "j5", "j6"),
        base_frame="base_link",
        initial_joint_positions=JointPositions((0.0,) * 6),
    )
    service.set_target_pose(
        Pose(position=(0.1, 0.2, 0.3), orientation=(0.0, 0.0, 0.0, 1.0))
    )

    with pytest.raises(KinematicsServiceError, match="target is unreachable"):
        service.step()
