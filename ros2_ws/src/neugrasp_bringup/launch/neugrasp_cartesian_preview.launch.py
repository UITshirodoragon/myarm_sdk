"""Preview a Cartesian plan with Neugrasp scene frames and no physical motion.

The fake driver supplies fresh canonical feedback for the planner.  Its normal
visualisation stream is remapped away so the preview player is the sole source
of ``/joint_states`` for the application robot_state_publisher.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the safe Neugrasp Cartesian-plan preview composition."""
    neugrasp_share = Path(get_package_share_directory("neugrasp_bringup"))
    services_config = LaunchConfiguration("services_config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "services_config", default_value="service/config/services.yaml"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(neugrasp_share / "launch" / "neugrasp_system.launch.py")
            ),
            launch_arguments={
                "enable_driver": "true",
                "enable_kinematics": "false",
                "enable_motion_execution": "false",
                "enable_cartesian_trajectory": "true",
                "driver_visualization_joint_state": "/myarm/actual_joint_states",
                "required_robot_arm_plugin_adapter": "fake_robot_arm",
                "services_config": services_config,
            }.items(),
        ),
        Node(
            package="myarm_cartesian_trajectory",
            executable="myarm_trajectory_preview_player_node",
            name="myarm_trajectory_preview_player",
            parameters=[{
                # neugrasp_system owns robot_state_publisher; the driver
                # stream was remapped above, leaving this as its only source.
                "services_config": services_config,
                "output_joint_states_topic": "/joint_states",
            }],
            output="screen",
        ),
    ])
