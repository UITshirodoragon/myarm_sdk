"""Bring up optional MyArm runtime capabilities without RViz.

The robot-side host owns state, planning and TF.  Remote RViz launches stay
in ``myarm_rviz2`` so this composition is equally suitable for headless,
fake-robot and remote-visualisation workflows.
"""

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
    enable_cartesian_trajectory = LaunchConfiguration("enable_cartesian_trajectory")
    enable_cartesian_execution = LaunchConfiguration("enable_cartesian_execution")
    enable_robot_state_publisher = LaunchConfiguration(
        "enable_robot_state_publisher"
    )
    services_config = LaunchConfiguration("services_config")
    required_robot_arm_plugin_adapter = LaunchConfiguration(
        "required_robot_arm_plugin_adapter"
    )
    driver_visualization_joint_state = LaunchConfiguration(
        "driver_visualization_joint_state"
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
            "enable_cartesian_trajectory",
            default_value="false",
            description="Start the plan/preview-only Cartesian trajectory node.",
        ),
        DeclareLaunchArgument(
            "enable_cartesian_execution",
            default_value="false",
            description=(
                "Expose FollowCartesianTrajectory in motion execution. "
                "Current implementation accepts FakeRobotArm only."
            ),
        ),
        DeclareLaunchArgument(
            "enable_robot_state_publisher",
            default_value="true",
            description="Start the sole robot_state_publisher instance.",
        ),
        DeclareLaunchArgument(
            "services_config",
            default_value="service/config/services.yaml",
            description=(
                "SDK-relative runtime service manifest used by every MyArm node."
            ),
        ),
        DeclareLaunchArgument(
            "required_robot_arm_plugin_adapter",
            default_value="",
            description=(
                "Optional fail-closed backend assertion; fake launch profiles "
                "set this to fake_robot_arm."
            ),
        ),
        DeclareLaunchArgument(
            "driver_visualization_joint_state",
            default_value="/joint_states",
            description=(
                "Topic used only for driver visualisation JointState. "
                "Cartesian preview remaps it away from /joint_states."
            ),
        ),
        Node(
            package="myarm_robot_driver",
            executable="myarm_robot_driver_node",
            name="myarm_robot_driver",
            condition=IfCondition(enable_driver),
            remappings=[("/joint_states", driver_visualization_joint_state)],
            parameters=[{
                "services_config": services_config,
                "required_robot_arm_plugin_adapter": (
                    required_robot_arm_plugin_adapter
                ),
            }],
            output="screen",
        ),
        Node(
            package="myarm_kinematics",
            executable="kinematics_node",
            name="myarm_kinematics",
            condition=IfCondition(enable_kinematics),
            parameters=[{"services_config": services_config}],
            output="screen",
        ),
        Node(
            package="myarm_motion_execution",
            executable="myarm_motion_execution_node",
            name="myarm_motion_execution",
            condition=IfCondition(enable_motion_execution),
            parameters=[{
                "services_config": services_config,
                "enable_cartesian_execution": enable_cartesian_execution,
            }],
            output="screen",
        ),
        Node(
            package="myarm_cartesian_trajectory",
            executable="myarm_cartesian_trajectory_node",
            name="myarm_cartesian_trajectory",
            condition=IfCondition(enable_cartesian_trajectory),
            parameters=[{"services_config": services_config}],
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
