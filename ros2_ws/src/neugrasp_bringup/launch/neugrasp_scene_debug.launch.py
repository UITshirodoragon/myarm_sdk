"""Visualize a NeuGrasp scan profile with fake feedback and no executor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start scene/profile visualization without a motion action server."""
    share = Path(get_package_share_directory("neugrasp_bringup"))
    rviz_share = Path(get_package_share_directory("myarm_rviz2"))
    scan_config = LaunchConfiguration("scan_config")
    start_rviz = LaunchConfiguration("start_rviz")
    return LaunchDescription([
        DeclareLaunchArgument(
            "scan_config", default_value=str(share / "config" / "neugrasp_scan_profiles.yaml")
        ),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "neugrasp_system.launch.py")),
            launch_arguments={
                "enable_driver": "true",
                "enable_kinematics": "false",
                "enable_motion_execution": "false",
                "enable_cartesian_trajectory": "false",
                "enable_cartesian_execution": "false",
                "required_robot_arm_plugin_adapter": "fake_robot_arm",
                "use_wrist_camera": "true",
                "camera_calibration": str(
                    share / "config" / "neugrasp_fake_wrist_camera_calibration.yaml"
                ),
            }.items(),
        ),
        Node(
            package="myarm_neugrasp",
            executable="neugrasp_scan_node",
            name="neugrasp_scan",
            parameters=[{"scan_config": scan_config}],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(rviz_share / "launch" / "neugrasp_rviz_remote.launch.py")
            ),
            condition=IfCondition(start_rviz),
        ),
    ])
