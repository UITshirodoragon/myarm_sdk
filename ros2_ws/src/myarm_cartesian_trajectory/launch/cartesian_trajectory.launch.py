"""Start plan-only Cartesian trajectory and optional synthetic preview nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch no driver, executor, or robot_state_publisher by design."""
    services_config = LaunchConfiguration("services_config")
    enable_preview_player = LaunchConfiguration("enable_preview_player")
    action_name = LaunchConfiguration("action_name")
    measured_joint_state_topic = LaunchConfiguration("measured_joint_state_topic")
    reference_path_topic = LaunchConfiguration("reference_path_topic")
    joint_preview_topic = LaunchConfiguration("joint_preview_topic")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    output_joint_states_topic = LaunchConfiguration("output_joint_states_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "services_config", default_value="service/config/services.yaml"
        ),
        DeclareLaunchArgument("enable_preview_player", default_value="true"),
        # Empty values preserve the configured topics in services.yaml.
        DeclareLaunchArgument("action_name", default_value=""),
        DeclareLaunchArgument("measured_joint_state_topic", default_value=""),
        DeclareLaunchArgument(
            "reference_path_topic", default_value="",
        ),
        DeclareLaunchArgument(
            "joint_preview_topic", default_value="",
        ),
        DeclareLaunchArgument(
            "diagnostics_topic", default_value="",
        ),
        # Safe default: never contend with a real driver publishing /joint_states.
        DeclareLaunchArgument(
            "output_joint_states_topic",
            default_value="/myarm/cartesian_trajectory/preview_joint_states",
        ),
        Node(
            package="myarm_cartesian_trajectory",
            executable="myarm_cartesian_trajectory_node",
            name="myarm_cartesian_trajectory",
            parameters=[{
                "services_config": services_config,
                "action_name": action_name,
                "measured_joint_state_topic": measured_joint_state_topic,
                "reference_path_topic": reference_path_topic,
                "joint_preview_topic": joint_preview_topic,
                "diagnostics_topic": diagnostics_topic,
            }],
            output="screen",
        ),
        Node(
            package="myarm_cartesian_trajectory",
            executable="myarm_trajectory_preview_player_node",
            name="myarm_cartesian_trajectory_preview",
            parameters=[{
                # The same empty-or-explicit override is passed to both
                # planner and player.  Empty makes both read the one shared
                # cartesian_trajectory_planner topic from services.yaml.
                "services_config": services_config,
                "joint_preview_topic": joint_preview_topic,
                "output_joint_states_topic": output_joint_states_topic,
            }],
            condition=IfCondition(enable_preview_player),
            output="screen",
        ),
    ])
