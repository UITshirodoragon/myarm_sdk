"""Bring up MyArm plus the static NeuGrasp application scene on one host."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_SUPPORTED_WRIST_CAMERA_PROFILES = frozenset((
    "generic",
    "logitech_c925e_wrist_v1",
))


def _as_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("true", "1", "yes"):
        return True
    if normalized in ("false", "0", "no"):
        return False
    raise ValueError(f"{name} must be true or false")


def _vector(document, field_name: str) -> str:
    if not isinstance(document, (list, tuple)) or len(document) != 3:
        raise ValueError(f"camera calibration {field_name} must contain three numbers")
    try:
        values = tuple(float(value) for value in document)
    except (TypeError, ValueError) as error:
        raise ValueError(f"camera calibration {field_name} must be numeric") from error
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"camera calibration {field_name} must be finite")
    return " ".join(format(value, ".17g") for value in values)


def _calibration_payload_sha256(calibration: dict) -> str:
    """Hash a calibration record without its self-referential digest field."""
    payload = {
        key: value for key, value in calibration.items() if key != "calibration_sha256"
    }
    try:
        canonical = json.dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as error:
        raise RuntimeError(
            "camera calibration payload is not canonical JSON serializable"
        ) from error
    return "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _camera_calibration_actions(context, *args, **kwargs):
    """Resolve one named camera profile/calibration into Xacro arguments."""
    if not _as_bool(LaunchConfiguration("use_wrist_camera").perform(context), "use_wrist_camera"):
        return []
    calibration_value = LaunchConfiguration("camera_calibration").perform(context).strip()
    if not calibration_value:
        raise RuntimeError(
            "use_wrist_camera:=true requires camera_calibration:=/absolute/or/package path"
        )
    calibration_path = Path(calibration_value).expanduser()
    if not calibration_path.is_file():
        raise RuntimeError(f"camera calibration does not exist: {calibration_path}")
    try:
        with calibration_path.open("r", encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise RuntimeError(f"camera calibration is invalid YAML: {calibration_path}") from error
    if not isinstance(calibration, dict):
        raise RuntimeError("camera calibration must be a mapping")
    if calibration.get("schema_version") != 1:
        raise RuntimeError("camera calibration schema_version must be 1")
    status = calibration.get("status")
    if status not in ("CALIBRATED", "FAKE"):
        raise RuntimeError("camera calibration status must be CALIBRATED or FAKE")
    calibration_id = calibration.get("calibration_id")
    provenance_hash = calibration.get("calibration_sha256")
    if not isinstance(calibration_id, str) or not calibration_id.strip():
        raise RuntimeError("camera calibration calibration_id must be non-empty")
    if not isinstance(provenance_hash, str) or not provenance_hash.strip():
        raise RuntimeError("camera calibration calibration_sha256 must be non-empty")
    if status == "CALIBRATED":
        expected_hash = _calibration_payload_sha256(calibration)
        if provenance_hash != expected_hash:
            raise RuntimeError(
                "camera calibration calibration_sha256 does not match its canonical payload; "
                f"expected {expected_hash}"
            )
    expected_frames = {
        "parent_frame": "gripper_base_link",
        "camera_body_frame": "wrist_camera_link",
        "camera_optical_frame": "wrist_camera_optical_frame",
    }
    for key, expected in expected_frames.items():
        if calibration.get(key) != expected:
            raise RuntimeError(
                f"camera calibration {key} must be {expected!r}, got {calibration.get(key)!r}"
            )
    # Existing named records without this field retain the generic transform
    # path; new records should always write the profile explicitly.
    camera_profile = calibration.get("camera_profile", "generic")
    if camera_profile not in _SUPPORTED_WRIST_CAMERA_PROFILES:
        raise RuntimeError(
            "camera calibration camera_profile must be one of {}, got {!r}".format(
                sorted(_SUPPORTED_WRIST_CAMERA_PROFILES), camera_profile
            )
        )
    required_adapter = LaunchConfiguration("required_robot_arm_plugin_adapter").perform(context).strip()
    if status == "FAKE" and required_adapter != "fake_robot_arm":
        raise RuntimeError(
            "a FAKE wrist-camera calibration is accepted only with "
            "required_robot_arm_plugin_adapter:=fake_robot_arm"
        )
    mount = calibration.get("mount")
    camera_body = calibration.get("camera_body")
    camera_optical = calibration.get("camera_optical")
    if (
        not isinstance(mount, dict)
        or not isinstance(camera_body, dict)
        or not isinstance(camera_optical, dict)
    ):
        raise RuntimeError(
            "camera calibration requires mount, camera_body and camera_optical mappings"
        )
    return [
        SetLaunchConfiguration("wrist_camera_profile", camera_profile),
        SetLaunchConfiguration(
            "wrist_camera_mount_xyz", _vector(mount.get("translation_m"), "mount.translation_m")
        ),
        SetLaunchConfiguration(
            "wrist_camera_mount_rpy", _vector(mount.get("rpy_rad"), "mount.rpy_rad")
        ),
        SetLaunchConfiguration(
            "wrist_camera_xyz",
            _vector(camera_body.get("translation_m"), "camera_body.translation_m"),
        ),
        SetLaunchConfiguration(
            "wrist_camera_rpy", _vector(camera_body.get("rpy_rad"), "camera_body.rpy_rad"),
        ),
        SetLaunchConfiguration(
            "wrist_camera_optical_xyz",
            _vector(
                camera_optical.get("translation_m"),
                "camera_optical.translation_m",
            ),
        ),
        SetLaunchConfiguration(
            "wrist_camera_optical_rpy",
            _vector(camera_optical.get("rpy_rad"), "camera_optical.rpy_rad"),
        ),
    ]


def generate_launch_description():
    """Run the only RSP plus the only owner of static NeuGrasp scene TF."""
    neugrasp_share = Path(get_package_share_directory("neugrasp_bringup"))
    myarm_bringup_share = Path(get_package_share_directory("myarm_bringup"))

    enable_driver = LaunchConfiguration("enable_driver")
    enable_kinematics = LaunchConfiguration("enable_kinematics")
    enable_motion_execution = LaunchConfiguration("enable_motion_execution")
    enable_cartesian_trajectory = LaunchConfiguration("enable_cartesian_trajectory")
    enable_cartesian_execution = LaunchConfiguration("enable_cartesian_execution")
    services_config = LaunchConfiguration("services_config")
    required_robot_arm_plugin_adapter = LaunchConfiguration("required_robot_arm_plugin_adapter")
    driver_visualization_joint_state = LaunchConfiguration("driver_visualization_joint_state")
    enable_scene_frames = LaunchConfiguration("enable_scene_frames")
    use_wrist_camera = LaunchConfiguration("use_wrist_camera")
    scene_config = LaunchConfiguration("scene_config")

    application_xacro = PathJoinSubstitution([
        FindPackageShare("myarm_description"),
        "urdf",
        "myarm_m750_neugrasp.urdf.xacro",
    ])
    robot_description = Command([
        FindExecutable(name="xacro"), " ", application_xacro, " ",
        "use_wrist_camera:=", use_wrist_camera, " ",
        "wrist_camera_profile:=", LaunchConfiguration("wrist_camera_profile"), " ",
        "wrist_camera_mount_xyz:='", LaunchConfiguration("wrist_camera_mount_xyz"), "' ",
        "wrist_camera_mount_rpy:='", LaunchConfiguration("wrist_camera_mount_rpy"), "' ",
        "wrist_camera_xyz:='", LaunchConfiguration("wrist_camera_xyz"), "' ",
        "wrist_camera_rpy:='", LaunchConfiguration("wrist_camera_rpy"), "' ",
        "wrist_camera_optical_xyz:='", LaunchConfiguration("wrist_camera_optical_xyz"), "' ",
        "wrist_camera_optical_rpy:='", LaunchConfiguration("wrist_camera_optical_rpy"), "'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("enable_driver", default_value="true"),
        DeclareLaunchArgument("enable_kinematics", default_value="true"),
        DeclareLaunchArgument("enable_motion_execution", default_value="true"),
        DeclareLaunchArgument("enable_cartesian_trajectory", default_value="false"),
        DeclareLaunchArgument("enable_cartesian_execution", default_value="false"),
        DeclareLaunchArgument("services_config", default_value="service/config/services.yaml"),
        DeclareLaunchArgument("required_robot_arm_plugin_adapter", default_value=""),
        DeclareLaunchArgument("driver_visualization_joint_state", default_value="/joint_states"),
        DeclareLaunchArgument("enable_scene_frames", default_value="true"),
        DeclareLaunchArgument("use_wrist_camera", default_value="false"),
        DeclareLaunchArgument(
            "camera_calibration",
            default_value="",
            description=(
                "Required named calibration YAML when use_wrist_camera is true. "
                "Use a status=FAKE file only with the fake adapter."
            ),
        ),
        # Internal values consumed by Xacro. They are populated exclusively
        # by _camera_calibration_actions from the named calibration record.
        DeclareLaunchArgument("wrist_camera_profile", default_value="generic"),
        DeclareLaunchArgument("wrist_camera_mount_xyz", default_value="0 0 0"),
        DeclareLaunchArgument("wrist_camera_mount_rpy", default_value="0 0 0"),
        DeclareLaunchArgument("wrist_camera_xyz", default_value="0 0 0"),
        DeclareLaunchArgument("wrist_camera_rpy", default_value="0 0 0"),
        DeclareLaunchArgument("wrist_camera_optical_xyz", default_value="0 0 0"),
        DeclareLaunchArgument(
            "wrist_camera_optical_rpy",
            default_value="-1.570796326794897 0 -1.570796326794897",
        ),
        DeclareLaunchArgument(
            "scene_config",
            default_value=str(neugrasp_share / "config" / "neugrasp_scene.yaml"),
        ),
        OpaqueFunction(function=_camera_calibration_actions),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(myarm_bringup_share / "launch" / "myarm_system.launch.py")
            ),
            launch_arguments={
                "enable_driver": enable_driver,
                "enable_kinematics": enable_kinematics,
                "enable_motion_execution": enable_motion_execution,
                "enable_cartesian_trajectory": enable_cartesian_trajectory,
                "enable_cartesian_execution": enable_cartesian_execution,
                "services_config": services_config,
                "required_robot_arm_plugin_adapter": required_robot_arm_plugin_adapter,
                "driver_visualization_joint_state": driver_visualization_joint_state,
                # This application launch owns the only robot_state_publisher.
                "enable_robot_state_publisher": "false",
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(robot_description, value_type=str),
            }],
            output="screen",
        ),
        Node(
            package="neugrasp_bringup",
            executable="neugrasp_static_scene_frames_node",
            name="neugrasp_static_scene_frames",
            parameters=[{"scene_config": scene_config}],
            condition=IfCondition(enable_scene_frames),
            output="screen",
        ),
    ])
