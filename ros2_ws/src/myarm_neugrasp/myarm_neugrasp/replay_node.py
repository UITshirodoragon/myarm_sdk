"""Visualize geometric NeuGrasp replay products in the current volume frame."""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .math3d import RigidTransform, finite_vector, normalize_quaternion, rotate_vector


_VISUALIZATION_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class NeugraspReplayNode(Node):
    """Replay TSDF/candidates without publishing historical TF or image feeds."""

    def __init__(self) -> None:
        super().__init__("neugrasp_replay")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("scene_config", "")
        self.declare_parameter("volume_frame", "neugrasp_volume")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cloud_source", "auto")
        self.declare_parameter("tsdf_threshold_low", -0.2)
        self.declare_parameter("tsdf_threshold_high", 0.2)
        self.declare_parameter("voxel_resolution", 40)
        self.declare_parameter("max_cloud_points", 50000)
        self.declare_parameter("top_k", 50)
        self.declare_parameter("republish_period_s", 2.0)
        self.declare_parameter("publish_on_start", True)

        run_dir_value = str(self.get_parameter("run_dir").value).strip()
        self._run_dir = Path(run_dir_value).expanduser() if run_dir_value else None
        self._volume_frame = self._frame(self.get_parameter("volume_frame").value, "volume_frame")
        self._base_frame = self._frame(self.get_parameter("base_frame").value, "base_frame")
        self._cloud_source = str(self.get_parameter("cloud_source").value).strip().lower()
        if self._cloud_source not in ("auto", "tsdf", "ply"):
            raise ValueError("cloud_source must be one of auto, tsdf, ply")
        self._threshold_low = self._finite_float("tsdf_threshold_low")
        self._threshold_high = self._finite_float("tsdf_threshold_high")
        if self._threshold_low >= self._threshold_high:
            raise ValueError("tsdf_threshold_low must be below tsdf_threshold_high")
        self._max_cloud_points = self._positive_int("max_cloud_points")
        self._top_k = self._positive_int("top_k")
        self._voxel_resolution = self._positive_int("voxel_resolution")
        self._volume_extent_m = self._load_volume_extent()
        self._ply_voxel_size_m = tuple(
            extent / self._voxel_resolution for extent in self._volume_extent_m
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._cloud_publisher = self.create_publisher(
            PointCloud2, "/neugrasp/tsdf_cloud", _VISUALIZATION_QOS
        )
        self._candidate_publisher = self.create_publisher(
            MarkerArray, "/neugrasp/grasp_candidates", _VISUALIZATION_QOS
        )
        self._selected_pose_publisher = self.create_publisher(
            PoseStamped, "/neugrasp/selected_grasp", _VISUALIZATION_QOS
        )
        self._selected_marker_publisher = self.create_publisher(
            Marker, "/neugrasp/selected_grasp_marker", _VISUALIZATION_QOS
        )
        self._status_publisher = self.create_publisher(
            String, "/neugrasp/replay/status", _VISUALIZATION_QOS
        )

        self._cloud = self._point_cloud(())
        self._ply_vertices: Optional[List[Tuple[float, float, float, int]]] = None
        self._ply_waiting_for_tf = False
        self._candidates = self._delete_all_marker_array()
        self._selected_pose: Optional[PoseStamped] = None
        self._selected_marker = self._delete_all_marker()
        self._load_run()
        if bool(self.get_parameter("publish_on_start").value):
            self._publish()
        period_s = self._nonnegative_float("republish_period_s")
        if period_s > 0.0:
            self.create_timer(period_s, self._publish)

    def _load_volume_extent(self) -> Tuple[float, float, float]:
        value = str(self.get_parameter("scene_config").value).strip()
        if not value:
            raise ValueError("scene_config is required to derive the current neugrasp_volume size")
        path = Path(value).expanduser()
        try:
            with path.open("r", encoding="utf-8") as stream:
                scene = yaml.safe_load(stream)
            bbox = scene["workspace"]["bbox_m"]
            lower = finite_vector(bbox["min"], "workspace.bbox_m.min", 3)
            upper = finite_vector(bbox["max"], "workspace.bbox_m.max", 3)
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            raise ValueError(f"invalid scene_config volume bbox: {path}: {error}") from error
        extent = tuple(upper[index] - lower[index] for index in range(3))
        if not all(value > 0.0 for value in extent):
            raise ValueError("scene_config workspace.bbox_m must have positive extents")
        if max(extent) - min(extent) > 1e-9:
            raise ValueError(
                "scene_config workspace.bbox_m must be cubic for RViz PointCloud2 Boxes"
            )
        return extent

    def _load_run(self) -> None:
        self._cloud = self._point_cloud(())
        self._ply_vertices = None
        self._ply_waiting_for_tf = False
        self._candidates = self._delete_all_marker_array()
        self._selected_pose = None
        self._selected_marker = self._delete_all_marker()
        if self._run_dir is None:
            self._publish_status("idle: run_dir is empty")
            return
        if not self._run_dir.is_dir():
            self._publish_status(f"error: run_dir does not exist: {self._run_dir}")
            return

        statuses = [self._load_cloud(), self._load_candidates(), self._load_selected()]
        self._publish_status("; ".join(statuses))

    def _load_cloud(self) -> str:
        assert self._run_dir is not None
        tsdf_path = self._run_dir / "inference" / "tsdf_vol.npy"
        ply_path = self._run_dir / "visualizations" / "tsdf_near_surface_base.ply"
        try_tsdf = self._cloud_source in ("auto", "tsdf")
        if try_tsdf and tsdf_path.is_file():
            try:
                self._cloud = self._cloud_from_tsdf(tsdf_path)
                return f"cloud=tsdf_volume:{self._cloud.width}"
            except (OSError, ValueError, TypeError) as error:
                if self._cloud_source == "tsdf":
                    return f"cloud=tsdf_error:{error}"
        if self._cloud_source in ("auto", "ply") and ply_path.is_file():
            try:
                self._ply_vertices = self._read_ascii_ply(ply_path)
                self._ply_waiting_for_tf = True
                return f"cloud=ply_base_pending_current_tf:{len(self._ply_vertices)}"
            except (OSError, ValueError, IndexError, OverflowError) as error:
                return f"cloud=ply_error:{error}"
        if self._cloud_source == "tsdf":
            return "cloud=tsdf_missing"
        if self._cloud_source == "ply":
            return "cloud=ply_missing"
        return "cloud=missing"

    def _cloud_from_tsdf(self, path: Path) -> PointCloud2:
        volume = np.asarray(np.load(path, allow_pickle=False)).squeeze()
        if volume.ndim != 3 or min(volume.shape) <= 0:
            raise ValueError(f"tsdf must reduce to a non-empty 3D tensor, got {volume.shape}")
        if len(set(volume.shape)) != 1:
            raise ValueError(
                f"tsdf resolution must be cubic for RViz PointCloud2 Boxes, got {volume.shape}"
            )
        indices = np.argwhere((volume > self._threshold_low) & (volume < self._threshold_high))
        if len(indices) > self._max_cloud_points:
            indices = indices[::int(math.ceil(len(indices) / self._max_cloud_points))]
        steps = tuple(self._volume_extent_m[index] / volume.shape[index] for index in range(3))
        vertices = []
        for index in indices:
            i, j, k = (int(value) for value in index)
            value = float(volume[i, j, k])
            if not math.isfinite(value):
                continue
            normalized = max(0.0, min(1.0, (value - self._threshold_low) /
                                      (self._threshold_high - self._threshold_low)))
            red = int(255.0 * normalized)
            blue = int(255.0 * (1.0 - normalized))
            rgb = (red << 16) | (80 << 8) | blue
            # PointCloud2 Boxes are centered on each point.  Publish voxel
            # centers, not index corners, so a Box of exactly `steps` fills
            # the configured neugrasp_volume without a half-voxel offset.
            vertices.append((
                (i + 0.5) * steps[0],
                (j + 0.5) * steps[1],
                (k + 0.5) * steps[2],
                value,
                rgb,
            ))
        return self._point_cloud(vertices)

    def _update_ply_cloud_from_current_tf(self) -> None:
        if not self._ply_waiting_for_tf or self._ply_vertices is None:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._volume_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            base_to_volume = RigidTransform(
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            vertices = []
            for x, y, z, rgb in self._ply_vertices:
                rotated = rotate_vector(base_to_volume.rotation, (x, y, z))
                vertices.append((
                    base_to_volume.translation[0] + rotated[0] + self._ply_voxel_size_m[0] / 2.0,
                    base_to_volume.translation[1] + rotated[1] + self._ply_voxel_size_m[1] / 2.0,
                    base_to_volume.translation[2] + rotated[2] + self._ply_voxel_size_m[2] / 2.0,
                    0.0,
                    rgb,
                ))
            self._cloud = self._point_cloud(vertices)
            self._ply_waiting_for_tf = False
            self._publish_status(f"cloud=ply_base_to_current_volume:{len(vertices)}")
        except TransformException:
            # The static scene may start after this node.  Retry on the next
            # low-rate publish instead of assigning base coordinates to volume.
            return

    def _load_candidates(self) -> str:
        assert self._run_dir is not None
        path = self._run_dir / "inference" / "candidates.json"
        if not path.is_file():
            return "candidates=missing"
        try:
            candidates = self._read_candidates(path)
            self._candidates = self._candidate_markers(candidates)
            return f"candidates=volume:{len(candidates)}"
        except (OSError, ValueError, TypeError, KeyError) as error:
            return f"candidates=error:{error}"

    def _load_selected(self) -> str:
        assert self._run_dir is not None
        path = self._run_dir / "inference" / "selected_grasp.json"
        if not path.is_file():
            return "selected=missing"
        try:
            selected = self._read_candidate(path)
            if selected is None:
                return "selected=missing_pose_cube"
            pose, width_m = selected
            self._selected_pose = self._pose_stamped(pose)
            self._selected_marker = self._gripper_marker(
                "selected_grasp", 0, pose, width_m, (1.0, 0.80, 0.10, 1.0)
            )
            return "selected=volume:1"
        except (OSError, ValueError, TypeError, KeyError) as error:
            return f"selected=error:{error}"

    def _read_candidates(self, path: Path) -> List[Tuple[RigidTransform, float]]:
        with path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        if isinstance(document, dict):
            document = document.get("candidates", [])
        if not isinstance(document, list):
            raise ValueError("candidates.json must contain a list")
        result = []
        for item in document[:self._top_k]:
            candidate = self._candidate_from_document(item)
            if candidate is not None:
                result.append(candidate)
        return result

    def _read_candidate(self, path: Path) -> Optional[Tuple[RigidTransform, float]]:
        with path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        if isinstance(document, dict) and "selected_grasp" in document:
            document = document["selected_grasp"]
        return self._candidate_from_document(document)

    def _candidate_from_document(self, document: Any) -> Optional[Tuple[RigidTransform, float]]:
        if not isinstance(document, dict):
            return None
        # This local pose is the only candidate pose valid under the current
        # volume TF.  Never use pose_table_grasp or pose_base_grasp from a run.
        pose_document = document.get("pose_cube_grasp")
        if not isinstance(pose_document, dict):
            return None
        pose = RigidTransform(
            finite_vector(pose_document["translation_m"], "pose_cube_grasp.translation_m", 3),
            normalize_quaternion(pose_document["quat_xyzw"]),
        )
        width_m = float(document.get("width_m", 0.04))
        if not math.isfinite(width_m) or width_m < 0.0:
            raise ValueError("width_m must be finite and non-negative")
        return pose, width_m

    def _read_ascii_ply(self, path: Path) -> List[Tuple[float, float, float, int]]:
        with path.open("r", encoding="utf-8") as stream:
            lines = stream.readlines()
        if not lines or lines[0].strip() != "ply":
            raise ValueError(f"not an ASCII PLY: {path}")
        vertex_count = None
        properties: List[str] = []
        in_vertex = False
        header_end = None
        for index, line in enumerate(lines[1:], start=1):
            fields = line.strip().split()
            if fields[:2] == ["element", "vertex"]:
                vertex_count = int(fields[2])
                in_vertex = True
            elif fields and fields[0] == "element":
                in_vertex = False
            elif fields and fields[0] == "property" and in_vertex:
                properties.append(fields[-1])
            elif fields and fields[0] == "end_header":
                header_end = index + 1
                break
        if vertex_count is None or vertex_count < 0 or header_end is None:
            raise ValueError("PLY header has no valid vertex section")
        if not {"x", "y", "z"}.issubset(properties) or len(lines) < header_end + vertex_count:
            raise ValueError("PLY vertex section is invalid or truncated")
        fields_by_name = {name: index for index, name in enumerate(properties)}
        vertices = []
        for line in lines[header_end:header_end + vertex_count]:
            values = line.strip().split()
            x = float(values[fields_by_name["x"]])
            y = float(values[fields_by_name["y"]])
            z = float(values[fields_by_name["z"]])
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise ValueError("PLY contains non-finite point")
            red = int(values[fields_by_name["red"]]) if "red" in fields_by_name else 180
            green = int(values[fields_by_name["green"]]) if "green" in fields_by_name else 180
            blue = int(values[fields_by_name["blue"]]) if "blue" in fields_by_name else 180
            vertices.append((x, y, z, (max(0, min(red, 255)) << 16) |
                             (max(0, min(green, 255)) << 8) | max(0, min(blue, 255))))
        return vertices

    def _point_cloud(self, vertices: Iterable[Tuple[float, float, float, float, int]]) -> PointCloud2:
        rows = list(vertices)
        message = PointCloud2()
        message.header.frame_id = self._volume_frame
        message.height = 1
        message.width = len(rows)
        message.is_bigendian = False
        message.is_dense = True
        message.fields = [
            self._field("x", 0, PointField.FLOAT32),
            self._field("y", 4, PointField.FLOAT32),
            self._field("z", 8, PointField.FLOAT32),
            self._field("tsdf", 12, PointField.FLOAT32),
            self._field("rgb", 16, PointField.UINT32),
        ]
        message.point_step = 20
        message.row_step = message.point_step * message.width
        data = bytearray(message.row_step)
        for index, (x, y, z, tsdf, rgb) in enumerate(rows):
            struct.pack_into("<ffffI", data, index * message.point_step, x, y, z, tsdf, rgb)
        message.data = bytes(data)
        return message

    def _candidate_markers(self, candidates: Sequence[Tuple[RigidTransform, float]]) -> MarkerArray:
        result = MarkerArray()
        marker = self._gripper_marker(
            "grasp_candidates", 0, None, 0.04, (0.15, 0.90, 0.75, 0.70)
        )
        marker.scale.x = 0.003
        for pose, width_m in candidates:
            marker.points.extend(self._gripper_lines(pose, width_m))
        result.markers.append(marker)
        return result

    def _gripper_marker(
        self,
        namespace: str,
        marker_id: int,
        pose: Optional[RigidTransform],
        width_m: float,
        color: Tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._volume_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.006
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        if pose is not None:
            marker.points = self._gripper_lines(pose, width_m)
        return marker

    def _gripper_lines(self, pose: RigidTransform, width_m: float) -> List[Point]:
        opening = max(0.005, min(width_m, 0.12))
        finger_length, finger_height = 0.055, 0.025
        segments = (
            ((0.0, -opening / 2.0, -finger_height), (finger_length, -opening / 2.0, -finger_height)),
            ((0.0, -opening / 2.0, finger_height), (finger_length, -opening / 2.0, finger_height)),
            ((finger_length, -opening / 2.0, -finger_height), (finger_length, -opening / 2.0, finger_height)),
            ((0.0, opening / 2.0, -finger_height), (finger_length, opening / 2.0, -finger_height)),
            ((0.0, opening / 2.0, finger_height), (finger_length, opening / 2.0, finger_height)),
            ((finger_length, opening / 2.0, -finger_height), (finger_length, opening / 2.0, finger_height)),
            ((0.0, -opening / 2.0, 0.0), (0.0, opening / 2.0, 0.0)),
        )
        points = []
        for first, second in segments:
            points.append(self._point(self._transform_point(pose, first)))
            points.append(self._point(self._transform_point(pose, second)))
        return points

    @staticmethod
    def _transform_point(pose: RigidTransform, point: Sequence[float]) -> Tuple[float, float, float]:
        rotated = rotate_vector(pose.rotation, tuple(point))
        return (
            pose.translation[0] + rotated[0],
            pose.translation[1] + rotated[1],
            pose.translation[2] + rotated[2],
        )

    def _pose_stamped(self, pose: RigidTransform) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._volume_frame
        message.pose = Pose()
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.translation
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = pose.rotation
        return message

    @staticmethod
    def _point(values: Sequence[float]) -> Point:
        point = Point()
        point.x, point.y, point.z = values
        return point

    @staticmethod
    def _field(name: str, offset: int, datatype: int) -> PointField:
        field = PointField()
        field.name, field.offset, field.datatype, field.count = name, offset, datatype, 1
        return field

    @staticmethod
    def _delete_all_marker() -> Marker:
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    @classmethod
    def _delete_all_marker_array(cls) -> MarkerArray:
        result = MarkerArray()
        result.markers.append(cls._delete_all_marker())
        return result

    def _publish(self) -> None:
        self._update_ply_cloud_from_current_tf()
        stamp = self.get_clock().now().to_msg()
        self._cloud.header.stamp = stamp
        self._cloud_publisher.publish(self._cloud)
        self._refresh_marker_stamps(self._candidates.markers)
        self._candidate_publisher.publish(self._candidates)
        if self._selected_pose is not None:
            self._selected_pose.header.stamp = stamp
            self._selected_pose_publisher.publish(self._selected_pose)
        self._selected_marker.header.stamp = stamp
        self._selected_marker_publisher.publish(self._selected_marker)

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)
        self.get_logger().info(f"NeuGrasp replay: {text}")

    def _refresh_marker_stamps(self, markers: Iterable[Marker]) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in markers:
            marker.header.stamp = stamp

    @staticmethod
    def _frame(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError(f"{name} must be a non-empty relative TF frame")
        return value.strip()

    def _finite_float(self, name: str) -> float:
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _nonnegative_float(self, name: str) -> float:
        value = self._finite_float(name)
        if value < 0.0:
            raise ValueError(f"{name} must be non-negative")
        return value

    def _positive_int(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            raise TypeError(f"{name} must be an integer")
        try:
            normalized = int(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be an integer") from error
        if normalized <= 0:
            raise ValueError(f"{name} must be positive")
        return normalized


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = NeugraspReplayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
