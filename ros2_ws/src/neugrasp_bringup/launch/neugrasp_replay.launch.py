"""Replay a completed NeuGrasp run without camera, inference or robot motion."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start base-frame replay without assuming a legacy scene calibration."""
    share = Path(get_package_share_directory("neugrasp_bringup"))
    rviz_share = Path(get_package_share_directory("myarm_rviz2"))
    run_dir = LaunchConfiguration("run_dir")
    scene_config = LaunchConfiguration("scene_config")
    enable_scene_frames = LaunchConfiguration("enable_scene_frames")
    base_frame = LaunchConfiguration("base_frame")
    allow_legacy_frame_relabel = LaunchConfiguration("allow_legacy_frame_relabel")
    start_rviz = LaunchConfiguration("start_rviz")
    top_k = LaunchConfiguration("top_k")
    return LaunchDescription([
        DeclareLaunchArgument("run_dir", default_value=""),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Legacy artifacts are authored in base_link; changing this is guarded.",
        ),
        DeclareLaunchArgument("allow_legacy_frame_relabel", default_value="false"),
        DeclareLaunchArgument(
            "scene_config", default_value=str(share / "config" / "neugrasp_scene.yaml")
        ),
        DeclareLaunchArgument(
            "enable_scene_frames",
            default_value="false",
            description=(
                "Opt in only after scene_config is verified to match the replay run; "
                "legacy runs must not use the current deployment calibration by default."
            ),
        ),
        DeclareLaunchArgument("top_k", default_value="50"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "neugrasp_system.launch.py")),
            launch_arguments={
                "enable_driver": "false",
                "enable_kinematics": "false",
                "enable_motion_execution": "false",
                "enable_cartesian_trajectory": "false",
                "enable_cartesian_execution": "false",
                "use_wrist_camera": "false",
                "scene_config": scene_config,
                "enable_scene_frames": enable_scene_frames,
            }.items(),
        ),
        Node(
            package="myarm_neugrasp",
            executable="neugrasp_replay_node",
            name="neugrasp_replay",
            parameters=[{
                "run_dir": run_dir,
                "top_k": top_k,
                "base_frame": base_frame,
                "allow_legacy_frame_relabel": allow_legacy_frame_relabel,
            }],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(rviz_share / "launch" / "neugrasp_rviz_remote.launch.py")
            ),
            condition=IfCondition(start_rviz),
        ),
    ])
