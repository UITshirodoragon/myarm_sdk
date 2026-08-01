"""Bring up the optional MyArm driver, kinematics and TF publisher."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create a single-machine MyArm system without RViz or legacy bridges."""
    description_share = Path(get_package_share_directory("myarm_description"))
    robot_description = (
        description_share / "urdf" / "myarm_m750_poe_v3_2.urdf"
    ).read_text()

    enable_driver = LaunchConfiguration("enable_driver")
    enable_kinematics = LaunchConfiguration("enable_kinematics")
    enable_motion_execution = LaunchConfiguration("enable_motion_execution")
    enable_robot_state_publisher = LaunchConfiguration(
        "enable_robot_state_publisher"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_driver",
            default_value="true",
            description="Start MyArmRobotDriverNode.",
        ),
        DeclareLaunchArgument(
            "enable_kinematics",
            default_value="true",
            description="Start MyArmKinematicsNode.",
        ),
        DeclareLaunchArgument(
            "enable_motion_execution",
            default_value="true",
            description="Start MyArmMotionExecutionNode.",
        ),
        DeclareLaunchArgument(
            "enable_robot_state_publisher",
            default_value="true",
            description="Start the sole robot_state_publisher instance.",
        ),
        Node(
            package="myarm_robot_driver",
            executable="myarm_robot_driver_node",
            name="myarm_robot_driver",
            condition=IfCondition(enable_driver),
            output="screen",
        ),
        Node(
            package="myarm_kinematics",
            executable="kinematics_node",
            name="myarm_kinematics",
            condition=IfCondition(enable_kinematics),
            output="screen",
        ),
        Node(
            package="myarm_motion_execution",
            executable="myarm_motion_execution_node",
            name="myarm_motion_execution",
            condition=IfCondition(enable_motion_execution),
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(enable_robot_state_publisher),
            output="screen",
        ),
    ])
