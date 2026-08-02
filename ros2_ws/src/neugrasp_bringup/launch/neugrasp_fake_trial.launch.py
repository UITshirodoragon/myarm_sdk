"""Run one autonomous NeuGrasp tensor trial against FakeRobotArm only.

This launch deliberately composes the application system itself instead of
including either the replay or scan launch. The trial node owns phase-gated
artifact snapshots, so no model visualization is published before Predict.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_run_dir(context, *args, **kwargs):
    """Fail before starting motion-capable nodes when no artifact run is named."""
    value = LaunchConfiguration("run_dir").perform(context).strip()
    if not value:
        raise RuntimeError(
            "run_dir is required and must name a completed run containing inference/*.npy"
        )
    path = Path(value).expanduser()
    if not path.is_dir():
        raise RuntimeError(f"run_dir does not exist or is not a directory: {path}")
    return []


def generate_launch_description():
    """Start exactly one fake NeuGrasp trial coordinator and its dependencies."""
    share = Path(get_package_share_directory("neugrasp_bringup"))
    rviz_share = Path(get_package_share_directory("myarm_rviz2"))

    run_dir = LaunchConfiguration("run_dir")
    services_config = LaunchConfiguration("services_config")
    scene_config = LaunchConfiguration("scene_config")
    scan_config = LaunchConfiguration("scan_config")
    trial_config = LaunchConfiguration("trial_config")
    scan_profile_id = LaunchConfiguration("scan_profile_id")
    base_frame = LaunchConfiguration("base_frame")
    volume_frame = LaunchConfiguration("volume_frame")
    tool_frame = LaunchConfiguration("tool_frame")
    camera_calibration = LaunchConfiguration("camera_calibration")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "run_dir",
            default_value="",
            description=(
                "Required completed run directory. Only inference/tsdf_vol.npy, "
                "qual_vol_raw.npy, rot_vol_raw.npy and width_vol_raw.npy are read."
            ),
        ),
        DeclareLaunchArgument(
            "services_config", default_value="service/config/services.yaml"
        ),
        DeclareLaunchArgument(
            "scene_config", default_value=str(share / "config" / "neugrasp_scene.yaml")
        ),
        DeclareLaunchArgument(
            "scan_config",
            default_value=str(share / "config" / "neugrasp_scan_profiles.yaml"),
        ),
        DeclareLaunchArgument(
            "trial_config",
            default_value=str(share / "config" / "neugrasp_fake_trial.yaml"),
        ),
        DeclareLaunchArgument(
            "scan_profile_id",
            default_value="neugrasp_simulation_views_16_19",
            description=(
                "Explicit ScanWorkspace profile for this trial; it does not use "
                "trajectory.active_profile."
            ),
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("volume_frame", default_value="neugrasp_volume"),
        DeclareLaunchArgument("tool_frame", default_value="tool0"),
        DeclareLaunchArgument(
            "camera_calibration",
            default_value=str(
                share / "config" / "neugrasp_fake_wrist_camera_calibration.yaml"
            ),
            description="FAKE-only current camera geometry used by the fake scan action.",
        ),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config", default_value=str(rviz_share / "config" / "neugrasp.rviz")
        ),
        OpaqueFunction(function=_validate_run_dir),
        # Do not include neugrasp_replay.launch.py or neugrasp_fake_scan.launch.py:
        # this launch must own one coherent graph without duplicate RSP, scene,
        # scan or driver nodes.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "neugrasp_system.launch.py")),
            launch_arguments={
                "enable_driver": "true",
                "enable_kinematics": "false",
                "enable_motion_execution": "true",
                "enable_cartesian_trajectory": "false",
                "enable_cartesian_execution": "false",
                "required_robot_arm_plugin_adapter": "fake_robot_arm",
                "services_config": services_config,
                "scene_config": scene_config,
                "enable_scene_frames": "true",
                "use_wrist_camera": "true",
                "camera_calibration": camera_calibration,
            }.items(),
        ),
        Node(
            package="myarm_neugrasp",
            executable="neugrasp_scan_node",
            name="neugrasp_scan",
            parameters=[{
                "scan_config": scan_config,
                "services_config": services_config,
                # The trial action also carries this explicitly.  Supplying it
                # here keeps the idle scan node's visible default aligned with
                # the one-trial profile rather than trajectory.active_profile.
                "default_profile": scan_profile_id,
            }],
            output="screen",
        ),
        Node(
            package="myarm_neugrasp",
            executable="neugrasp_trial_node",
            name="neugrasp_trial",
            parameters=[{
                "run_dir": run_dir,
                "trial_config": trial_config,
                "scan_profile_id": scan_profile_id,
                "scan_config": scan_config,
                "services_config": services_config,
                "base_frame": base_frame,
                "volume_frame": volume_frame,
                "tool_frame": tool_frame,
            }],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(rviz_share / "launch" / "neugrasp_rviz_remote.launch.py")
            ),
            launch_arguments={"rviz_config": rviz_config}.items(),
            condition=IfCondition(start_rviz),
        ),
    ])
