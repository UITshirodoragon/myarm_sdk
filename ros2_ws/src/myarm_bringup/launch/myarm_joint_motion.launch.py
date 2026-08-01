"""Convenience launch for the established joint/one-shot-IK runtime."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start driver, one-shot kinematics, joint planner/executor and TF."""
    share = Path(get_package_share_directory("myarm_bringup"))
    services_config = LaunchConfiguration("services_config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "services_config", default_value="service/config/services.yaml"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "myarm_system.launch.py")),
            launch_arguments={
                "enable_driver": "true",
                "enable_kinematics": "true",
                "enable_motion_execution": "true",
                "enable_cartesian_trajectory": "false",
                "enable_robot_state_publisher": "true",
                "services_config": services_config,
            }.items(),
        ),
    ])
