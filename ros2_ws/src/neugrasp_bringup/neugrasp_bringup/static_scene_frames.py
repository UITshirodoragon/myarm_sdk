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
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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
        config_path = Path(str(self.get_parameter("scene_config").value))
        transforms = self._load_transforms(config_path)
        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f"Published {len(transforms)} Neugrasp static scene transforms from {config_path}"
        )

    @classmethod
    def _load_transforms(cls, config_path: Path) -> Iterable[TransformStamped]:
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
        return result

    @staticmethod
    def _frame_name(value: Any, index: int, field_name: str) -> str:
        if not isinstance(value, str) or not value or value.startswith("/"):
            raise ValueError(
                f"static_frames[{index}].{field_name} must be a non-empty relative TF frame"
            )
        return value

    @staticmethod
    def _vector(value: Any, index: int, field_name: str):
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"static_frames[{index}].{field_name} must contain three numbers")
        try:
            normalized = tuple(float(item) for item in value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"static_frames[{index}].{field_name} must be numeric") from error
        if not all(math.isfinite(item) for item in normalized):
            raise ValueError(f"static_frames[{index}].{field_name} must be finite")
        return normalized

    @classmethod
    def _quaternion(cls, value: Any, index: int):
        quaternion = cls._vector(value, index, "rotation_xyzw")
        norm = math.sqrt(sum(item * item for item in quaternion))
        if norm < 1e-12:
            raise ValueError(f"static_frames[{index}].rotation_xyzw must not be zero")
        return tuple(item / norm for item in quaternion)


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
