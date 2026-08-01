"""Cartesian planning plus the existing joint executor, without RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Expose plan and FollowJointTrajectory actions without RViz.

    The Cartesian planner remains plan-only.  It never submits its own result
    to motion execution, so starting this launch alone cannot turn a TCP goal
    into an implicit physical motion command.
    """
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
                "enable_kinematics": "false",
                "enable_motion_execution": "true",
                "enable_cartesian_trajectory": "true",
                # Headless means no local RViz. Keep RSP so TF remains
                # available to remote RViz and PoseStamped TF conversion.
                "enable_robot_state_publisher": "true",
                "services_config": services_config,
            }.items(),
        ),
    ])
