"""Exercise Cartesian planning and joint execution with the checked-in fake arm.

This launch is intentionally a fake-only integration environment. The
plan-only Cartesian action remains separate; the executor additionally owns
``/myarm/follow_cartesian_trajectory`` for an explicit plan + preflight +
execution request. With the checked-in runtime configuration the robot-arm
adapter is ``FakeRobotArm``; do not use this launch as a physical-hardware
profile.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start fake feedback, planner, executor and the sole state publisher."""
    share = Path(get_package_share_directory("myarm_bringup"))
    services_config = LaunchConfiguration("services_config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "services_config", default_value="service/config/services.yaml"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "launch" / "myarm_system.launch.py")
            ),
            launch_arguments={
                "enable_driver": "true",
                "enable_kinematics": "false",
                "enable_motion_execution": "true",
                "enable_cartesian_trajectory": "true",
                "enable_cartesian_execution": "true",
                "enable_robot_state_publisher": "true",
                "required_robot_arm_plugin_adapter": "fake_robot_arm",
                "services_config": services_config,
            }.items(),
        ),
    ])
