"""Replay run artifacts while the current bringup owns all frame relations."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start TSDF, grasp and diagnostic replay with current robot/scene TF."""
    share = Path(get_package_share_directory("neugrasp_bringup"))
    rviz_share = Path(get_package_share_directory("myarm_rviz2"))
    run_dir = LaunchConfiguration("run_dir")
    scene_config = LaunchConfiguration("scene_config")
    enable_scene_frames = LaunchConfiguration("enable_scene_frames")
    source_frame = LaunchConfiguration("source_frame")
    target_frame = LaunchConfiguration("target_frame")
    use_wrist_camera = LaunchConfiguration("use_wrist_camera")
    camera_calibration = LaunchConfiguration("camera_calibration")
    start_rviz = LaunchConfiguration("start_rviz")
    return LaunchDescription([
        DeclareLaunchArgument("run_dir", default_value=""),
        DeclareLaunchArgument(
            "scene_config", default_value=str(share / "config" / "neugrasp_scene.yaml")
        ),
        DeclareLaunchArgument(
            "source_frame",
            default_value="base_link",
            description="Known coordinate frame encoded by both legacy PLY files.",
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="neugrasp_volume",
            description="Current scene frame used in every replay message header.",
        ),
        DeclareLaunchArgument(
            "enable_scene_frames",
            default_value="true",
            description=(
                "Publish workspace and volume frames from the current scene_config. "
                "Run calibration and historical TF are never read."
            ),
        ),
        DeclareLaunchArgument(
            "use_wrist_camera",
            default_value="false",
            description=(
                "Append the wrist-camera Xacro frames from the current named calibration; "
                "never read a calibration from run_dir."
            ),
        ),
        DeclareLaunchArgument(
            "camera_calibration",
            default_value="",
            description="Required current calibration YAML when use_wrist_camera is true.",
        ),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "neugrasp_system.launch.py")),
            launch_arguments={
                "enable_driver": "false",
                "enable_kinematics": "false",
                "enable_motion_execution": "false",
                "enable_cartesian_trajectory": "false",
                "enable_cartesian_execution": "false",
                "use_wrist_camera": use_wrist_camera,
                "camera_calibration": camera_calibration,
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
                "source_frame": source_frame,
                "target_frame": target_frame,
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
