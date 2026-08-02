"""Launch one generic MyArm camera node for each enabled manifest instance."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from myarm_camera.camera_config import (
    camera_instance,
    camera_ros_config,
    resolve_camera_profile,
)


def _camera_nodes(context, *args, **kwargs):
    services_config = LaunchConfiguration("services_config").perform(context)
    camera_profile = LaunchConfiguration("camera_profile").perform(context).strip()
    actions = []
    for instance_id in resolve_camera_profile(services_config, camera_profile):
        instance = camera_instance(services_config, instance_id)
        ros_config = camera_ros_config(instance)
        actions.append(
            Node(
                package="myarm_camera",
                executable="myarm_camera_node",
                name=str(ros_config["node_name"]),
                namespace=str(ros_config["namespace"]),
                parameters=[{
                    "services_config": services_config,
                    "camera_instance": instance_id,
                }],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "services_config", default_value="service/config/services.yaml"
        ),
        DeclareLaunchArgument("camera_profile", default_value="none"),
        OpaqueFunction(function=_camera_nodes),
    ])
