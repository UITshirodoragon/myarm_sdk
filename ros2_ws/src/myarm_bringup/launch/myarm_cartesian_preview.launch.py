"""Safe fake-feedback Cartesian trajectory preview without physical execution."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the Cartesian planner and preview player as the only TF state source."""
    share = Path(get_package_share_directory("myarm_bringup"))
    description_share = Path(get_package_share_directory("myarm_description"))
    robot_description = (
        description_share / "urdf" / "myarm_m750_poe_v3_2.urdf"
    ).read_text()
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
                "enable_motion_execution": "false",
                "enable_cartesian_trajectory": "true",
                "enable_robot_state_publisher": "false",
                "driver_visualization_joint_state": "/myarm/actual_joint_states",
                # Never allow a preview launch to silently open a physical
                # adapter after somebody changes the shared runtime profile.
                "required_robot_arm_plugin_adapter": "fake_robot_arm",
                "services_config": services_config,
            }.items(),
        ),
        Node(
            package="myarm_cartesian_trajectory",
            executable="myarm_trajectory_preview_player_node",
            name="myarm_trajectory_preview_player",
            parameters=[{
                # The driver stream was remapped above, so this is the sole
                # JointState source consumed by this preview RSP instance.
                "services_config": services_config,
                "output_joint_states_topic": "/joint_states",
            }],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ])
