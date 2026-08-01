"""Publish Neugrasp-owned static scene frames from one validated YAML file."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker


class NeugraspStaticSceneFramesNode(Node):
    """Own only non-robot static TF frames for one Neugrasp deployment."""

    def __init__(self) -> None:
        super().__init__("neugrasp_static_scene_frames")
        default_config = (
            Path(get_package_share_directory("neugrasp_bringup"))
            / "config"
            / "neugrasp_scene.yaml"
        )
        self.declare_parameter("scene_config", str(default_config))
        self.declare_parameter("workspace_marker_republish_hz", 1.0)
        config_path = Path(str(self.get_parameter("scene_config").value))
        transforms, workspace_marker = self._load_scene(config_path)
        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)
        self._workspace_marker = workspace_marker
        self._workspace_marker_publisher = self.create_publisher(
            Marker,
            "/neugrasp/workspace_marker",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        if self._workspace_marker is not None:
            self._publish_workspace_marker()
            marker_rate_hz = self._positive_float(
                self.get_parameter("workspace_marker_republish_hz").value,
                "workspace_marker_republish_hz",
            )
            # The application runs in distinct repeated phases, so a low-rate
            # refresh is sufficient and lets a volatile remote RViz join later.
            self.create_timer(1.0 / marker_rate_hz, self._publish_workspace_marker)
        self.get_logger().info(
            f"Published {len(transforms)} Neugrasp static scene transforms from {config_path}"
        )

    @classmethod
    def _load_transforms(cls, config_path: Path) -> Iterable[TransformStamped]:
        """Compatibility helper for callers that only need static transforms."""
        transforms, _ = cls._load_scene(config_path)
        return transforms

    @classmethod
    def _load_scene(cls, config_path: Path):
        if not config_path.is_file():
            raise ValueError(f"Neugrasp scene config does not exist: {config_path}")
        try:
            with config_path.open("r", encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise ValueError(f"Neugrasp scene config is invalid YAML: {config_path}") from error
        if not isinstance(document, dict):
            raise TypeError("Neugrasp scene config must be a mapping")
        frames = document.get("static_frames")
        if not isinstance(frames, list):
            raise TypeError("Neugrasp scene config static_frames must be a list")

        children = set()
        result = []
        for index, frame in enumerate(frames):
            if not isinstance(frame, dict):
                raise TypeError(f"static_frames[{index}] must be a mapping")
            if frame.get("enabled") is not True:
                continue
            parent = cls._frame_name(frame.get("parent_frame"), index, "parent_frame")
            child = cls._frame_name(frame.get("child_frame"), index, "child_frame")
            if parent == child:
                raise ValueError(f"static_frames[{index}] parent and child must differ")
            if child in children:
                raise ValueError(f"duplicate static child frame: {child}")
            children.add(child)
            translation = cls._vector(frame.get("translation_m"), index, "translation_m")
            rotation = cls._quaternion(frame.get("rotation_xyzw"), index)
            transform = TransformStamped()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x = translation[0]
            transform.transform.translation.y = translation[1]
            transform.transform.translation.z = translation[2]
            transform.transform.rotation.x = rotation[0]
            transform.transform.rotation.y = rotation[1]
            transform.transform.rotation.z = rotation[2]
            transform.transform.rotation.w = rotation[3]
            result.append(transform)
        workspace_marker = cls._workspace_marker(document, children, result)
        return result, workspace_marker

    @staticmethod
    def _frame_name(value: Any, index: int, field_name: str) -> str:
        if not isinstance(value, str) or not value or value.startswith("/"):
            raise ValueError(
                f"static_frames[{index}].{field_name} must be a non-empty relative TF frame"
            )
        return value

    @staticmethod
    def _vector(value: Any, index: int, field_name: str, size: int = 3):
        if not isinstance(value, (list, tuple)) or len(value) != size:
            raise ValueError(
                f"static_frames[{index}].{field_name} must contain {size} numbers"
            )
        try:
            normalized = tuple(float(item) for item in value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"static_frames[{index}].{field_name} must be numeric") from error
        if not all(math.isfinite(item) for item in normalized):
            raise ValueError(f"static_frames[{index}].{field_name} must be finite")
        return normalized

    @classmethod
    def _quaternion(cls, value: Any, index: int):
        quaternion = cls._vector(value, index, "rotation_xyzw", size=4)
        norm = math.sqrt(sum(item * item for item in quaternion))
        if norm < 1e-12:
            raise ValueError(f"static_frames[{index}].rotation_xyzw must not be zero")
        return tuple(item / norm for item in quaternion)

    @classmethod
    def _workspace_marker(cls, document: dict, children: set, transforms: list):
        """Derive volume TF and workspace CUBE marker from one bbox source."""
        workspace = document.get("workspace")
        if workspace is None:
            return None
        if not isinstance(workspace, dict):
            raise TypeError("Neugrasp scene config workspace must be a mapping")
        workspace_frame = cls._workspace_frame(workspace.get("frame_id"), "workspace.frame_id")
        volume_frame = cls._workspace_frame(workspace.get("volume_frame"), "workspace.volume_frame")
        if volume_frame in children:
            raise ValueError(
                f"workspace.volume_frame duplicates a static child frame: {volume_frame}"
            )
        bbox = workspace.get("bbox_m")
        if not isinstance(bbox, dict):
            raise TypeError("workspace.bbox_m must be a mapping")
        minimum = cls._workspace_vector(bbox.get("min"), "workspace.bbox_m.min")
        maximum = cls._workspace_vector(bbox.get("max"), "workspace.bbox_m.max")
        extent = tuple(upper - lower for lower, upper in zip(minimum, maximum))
        if not all(value > 0.0 for value in extent):
            raise ValueError("workspace.bbox_m.max must be greater than min on every axis")
        if workspace.get("derive_volume_origin_from_bbox_min") is not True:
            raise ValueError("workspace.derive_volume_origin_from_bbox_min must be true")
        volume_transform = TransformStamped()
        volume_transform.header.frame_id = workspace_frame
        volume_transform.child_frame_id = volume_frame
        (
            volume_transform.transform.translation.x,
            volume_transform.transform.translation.y,
            volume_transform.transform.translation.z,
        ) = minimum
        volume_transform.transform.rotation.w = 1.0
        transforms.append(volume_transform)
        children.add(volume_frame)

        marker = Marker()
        marker.header.frame_id = workspace_frame
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = minimum[0] + extent[0] / 2.0
        marker.pose.position.y = minimum[1] + extent[1] / 2.0
        marker.pose.position.z = minimum[2] + extent[2] / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = extent
        marker.color.r = 0.10
        marker.color.g = 0.75
        marker.color.b = 0.95
        marker.color.a = 0.18
        return marker

    @staticmethod
    def _workspace_frame(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value or value.startswith("/"):
            raise ValueError(f"{name} must be a non-empty relative TF frame")
        return value

    @staticmethod
    def _workspace_vector(value: Any, name: str):
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"{name} must contain three numbers")
        try:
            normalized = tuple(float(item) for item in value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not all(math.isfinite(item) for item in normalized):
            raise ValueError(f"{name} must be finite")
        return normalized

    @staticmethod
    def _positive_float(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric")
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return normalized

    def _publish_workspace_marker(self) -> None:
        if self._workspace_marker is None:
            return
        self._workspace_marker.header.stamp = self.get_clock().now().to_msg()
        self._workspace_marker_publisher.publish(self._workspace_marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = NeugraspStaticSceneFramesNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
