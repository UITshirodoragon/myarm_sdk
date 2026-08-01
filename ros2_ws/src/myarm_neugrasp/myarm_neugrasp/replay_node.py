"""Publish visualization-friendly NeuGrasp artifacts from an old run.

The node deliberately replays rendered PLY/JSON products instead of loading a
model or opening a camera.  This makes legacy run validation independent from
CUDA, sensor transport and the robot driver.
"""

from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .math3d import RigidTransform, finite_vector, normalize_quaternion, rotate_vector


_CLOUD_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
_MARKER_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class NeugraspReplayNode(Node):
    """Replay a completed run into the canonical NeuGrasp visualization topics."""

    def __init__(self) -> None:
        super().__init__("neugrasp_replay")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("allow_legacy_frame_relabel", False)
        self.declare_parameter("top_k", 50)
        self.declare_parameter("republish_period_s", 2.0)
        self.declare_parameter("publish_on_start", True)
        self._base_frame = self._frame(self.get_parameter("base_frame").value)
        if (
            self._base_frame != "base_link"
            and not bool(self.get_parameter("allow_legacy_frame_relabel").value)
        ):
            raise ValueError(
                "legacy replay artifacts are already expressed in base_link; "
                "set allow_legacy_frame_relabel:=true only when an external TF "
                "contract proves the relabeling is correct"
            )
        self._top_k = self._positive_int(self.get_parameter("top_k").value, "top_k")
        run_dir_value = str(self.get_parameter("run_dir").value).strip()
        self._run_dir = Path(run_dir_value).expanduser() if run_dir_value else None

        self._cloud_publisher = self.create_publisher(
            PointCloud2, "/neugrasp/tsdf_cloud", _CLOUD_QOS
        )
        self._candidate_publisher = self.create_publisher(
            MarkerArray, "/neugrasp/grasp_candidates", _MARKER_QOS
        )
        self._legacy_wireframe_publisher = self.create_publisher(
            MarkerArray, "/neugrasp/legacy_grasp_wireframes", _MARKER_QOS
        )
        self._selected_pose_publisher = self.create_publisher(
            PoseStamped, "/neugrasp/selected_grasp", _MARKER_QOS
        )
        self._selected_marker_publisher = self.create_publisher(
            Marker, "/neugrasp/selected_grasp_marker", _MARKER_QOS
        )
        self._status_publisher = self.create_publisher(String, "/neugrasp/replay/status", _MARKER_QOS)

        self._cloud = self._point_cloud(())
        self._candidates = self._delete_all_marker_array()
        self._legacy_wireframe = self._delete_all_marker_array()
        self._selected_pose: Optional[PoseStamped] = None
        self._selected_marker = self._delete_all_marker()
        self._load_run()
        if bool(self.get_parameter("publish_on_start").value):
            self._publish()
        period_s = self._nonnegative_float(
            self.get_parameter("republish_period_s").value, "republish_period_s"
        )
        if period_s > 0.0:
            self.create_timer(period_s, self._publish)

    @staticmethod
    def _frame(value: Any) -> str:
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError("base_frame must be a non-empty relative TF frame")
        return value.strip()

    @staticmethod
    def _positive_int(value: Any, name: str) -> int:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be an integer")
        try:
            normalized = int(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be an integer") from error
        if normalized <= 0:
            raise ValueError(f"{name} must be positive")
        return normalized

    @staticmethod
    def _nonnegative_float(value: Any, name: str) -> float:
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or normalized < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return normalized

    def _load_run(self) -> None:
        # Keep a new/missing run from leaving retained RViz entities from a
        # previous replay phase.  The messages are published by _publish().
        self._cloud = self._point_cloud(())
        self._candidates = self._delete_all_marker_array()
        self._legacy_wireframe = self._delete_all_marker_array()
        self._selected_pose = None
        self._selected_marker = self._delete_all_marker()
        if self._run_dir is None:
            self._publish_status("idle: run_dir is empty")
            return
        if not self._run_dir.is_dir():
            self._publish_status(f"error: run_dir does not exist: {self._run_dir}")
            self.get_logger().error(f"Replay run directory does not exist: {self._run_dir}")
            return
        cloud_path = self._run_dir / "visualizations" / "tsdf_near_surface_base.ply"
        candidates_path = self._run_dir / "inference" / "candidates.json"
        selected_path = self._run_dir / "inference" / "selected_grasp.json"
        wireframe_path = self._run_dir / "visualizations" / "grasp_candidates_wireframes_base.ply"
        statuses = []
        if cloud_path.is_file():
            try:
                vertices, _ = self._read_ascii_ply(cloud_path)
                self._cloud = self._point_cloud(vertices)
                statuses.append(f"cloud={len(vertices)}")
            except (OSError, ValueError, IndexError, OverflowError) as error:
                statuses.append(f"cloud_error={error}")
        else:
            statuses.append("cloud=missing")
        parsed_candidates = []
        if candidates_path.is_file():
            try:
                parsed_candidates = self._read_candidates(candidates_path)
                if parsed_candidates:
                    self._candidates = self._candidate_markers(parsed_candidates)
                statuses.append(f"candidates={len(parsed_candidates)}")
            except (OSError, ValueError, TypeError, IndexError, OverflowError) as error:
                statuses.append(f"candidates_error={error}")
        else:
            statuses.append("candidates=missing")
        if not parsed_candidates and wireframe_path.is_file():
            try:
                vertices, edges = self._read_ascii_ply(wireframe_path)
                if edges:
                    self._legacy_wireframe = self._legacy_tcp_wireframe_marker(vertices, edges)
                statuses.append(f"legacy_tcp_wireframe={len(vertices)}")
            except (OSError, ValueError, IndexError, OverflowError) as error:
                statuses.append(f"wireframe_error={error}")
        elif not parsed_candidates:
            statuses.append("legacy_tcp_wireframe=missing")
        if selected_path.is_file():
            try:
                selected = self._read_selected(selected_path)
                if selected is not None:
                    self._selected_pose = self._pose_stamped(selected[0])
                    self._selected_marker = self._gripper_marker(
                        "selected_grasp", 0, selected[0], selected[1], (1.0, 0.80, 0.10, 1.0)
                    )
                    statuses.append("selected=1")
            except (OSError, ValueError, TypeError, IndexError, OverflowError) as error:
                statuses.append(f"selected_error={error}")
        if self._selected_pose is None and parsed_candidates:
            pose, width = parsed_candidates[0]
            self._selected_pose = self._pose_stamped(pose)
            self._selected_marker = self._gripper_marker(
                "selected_grasp", 0, pose, width, (1.0, 0.80, 0.10, 1.0)
            )
            statuses.append("selected=first_candidate")
        elif self._selected_pose is None and not any(
            status.startswith("selected_error=") for status in statuses
        ):
            statuses.append("selected=missing")
        self._publish_status("; ".join(statuses))

    def _publish(self) -> None:
        self._cloud.header.stamp = self.get_clock().now().to_msg()
        self._cloud_publisher.publish(self._cloud)
        self._refresh_marker_stamps(self._candidates.markers)
        self._candidate_publisher.publish(self._candidates)
        self._refresh_marker_stamps(self._legacy_wireframe.markers)
        self._legacy_wireframe_publisher.publish(self._legacy_wireframe)
        if self._selected_pose is not None:
            self._selected_pose.header.stamp = self.get_clock().now().to_msg()
            self._selected_pose_publisher.publish(self._selected_pose)
        self._selected_marker.header.stamp = self.get_clock().now().to_msg()
        self._selected_marker_publisher.publish(self._selected_marker)

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_publisher.publish(message)
        self.get_logger().info(f"NeuGrasp replay: {text}")

    def _read_ascii_ply(
        self, path: Path
    ) -> Tuple[List[Tuple[float, float, float, int]], List[Tuple[int, int]]]:
        with path.open("r", encoding="utf-8") as stream:
            lines = stream.readlines()
        if not lines or lines[0].strip() != "ply":
            raise ValueError(f"not an ASCII PLY file: {path}")
        vertex_count = None
        edge_count = 0
        properties = []
        in_vertex = False
        header_end = None
        format_seen = False
        for index, line in enumerate(lines[1:], start=1):
            fields = line.strip().split()
            if not fields:
                continue
            if fields[0] == "format":
                if len(fields) != 3 or fields[1:] != ["ascii", "1.0"]:
                    raise ValueError(f"only ASCII PLY is supported: {path}")
                format_seen = True
                continue
            if fields[:2] == ["element", "vertex"]:
                if len(fields) != 3:
                    raise ValueError(f"invalid PLY vertex element declaration: {path}")
                try:
                    vertex_count = int(fields[2])
                except ValueError as error:
                    raise ValueError(f"invalid PLY vertex count: {path}") from error
                if vertex_count < 0:
                    raise ValueError(f"PLY vertex count must be non-negative: {path}")
                in_vertex = True
                continue
            if fields[:2] == ["element", "edge"]:
                if len(fields) != 3:
                    raise ValueError(f"invalid PLY edge element declaration: {path}")
                try:
                    edge_count = int(fields[2])
                except ValueError as error:
                    raise ValueError(f"invalid PLY edge count: {path}") from error
                if edge_count < 0:
                    raise ValueError(f"PLY edge count must be non-negative: {path}")
                in_vertex = False
                continue
            if fields[0] == "element":
                in_vertex = False
            if fields[0] == "property" and in_vertex:
                properties.append(fields[-1])
            if fields[0] == "end_header":
                header_end = index + 1
                break
        if not format_seen or vertex_count is None or header_end is None:
            raise ValueError(f"PLY header is missing format, vertex, or end_header: {path}")
        required = {"x", "y", "z"}
        if not required.issubset(properties):
            raise ValueError(f"PLY vertex has no x/y/z fields: {path}")
        property_index = {name: index for index, name in enumerate(properties)}
        vertices = []
        if len(lines) < header_end + vertex_count:
            raise ValueError(f"PLY vertex data is truncated: {path}")
        for line in lines[header_end:header_end + vertex_count]:
            fields = line.strip().split()
            if len(fields) < len(properties):
                raise ValueError(f"invalid PLY vertex row: {path}")
            x = float(fields[property_index["x"]])
            y = float(fields[property_index["y"]])
            z = float(fields[property_index["z"]])
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise ValueError(f"non-finite PLY point: {path}")
            red = int(fields[property_index["red"]]) if "red" in property_index else 180
            green = int(fields[property_index["green"]]) if "green" in property_index else 180
            blue = int(fields[property_index["blue"]]) if "blue" in property_index else 180
            rgb = (max(0, min(red, 255)) << 16) | (max(0, min(green, 255)) << 8) | max(0, min(blue, 255))
            vertices.append((x, y, z, rgb))
        edges = []
        edge_start = header_end + vertex_count
        if len(lines) < edge_start + edge_count:
            raise ValueError(f"PLY edge data is truncated: {path}")
        for line in lines[edge_start:edge_start + edge_count]:
            fields = line.strip().split()
            if len(fields) >= 2:
                first, second = int(fields[-2]), int(fields[-1])
                if 0 <= first < len(vertices) and 0 <= second < len(vertices):
                    edges.append((first, second))
        return vertices, edges

    def _point_cloud(self, vertices: Sequence[Tuple[float, float, float, int]]) -> PointCloud2:
        message = PointCloud2()
        message.header.frame_id = self._base_frame
        message.height = 1
        message.width = len(vertices)
        message.is_bigendian = False
        message.is_dense = True
        message.fields = [
            self._field("x", 0, PointField.FLOAT32),
            self._field("y", 4, PointField.FLOAT32),
            self._field("z", 8, PointField.FLOAT32),
            self._field("rgb", 12, PointField.UINT32),
        ]
        message.point_step = 16
        message.row_step = message.point_step * message.width
        data = bytearray(message.row_step)
        for index, (x, y, z, rgb) in enumerate(vertices):
            struct.pack_into("<fffI", data, index * message.point_step, x, y, z, rgb)
        message.data = bytes(data)
        return message

    @staticmethod
    def _field(name: str, offset: int, datatype: int) -> PointField:
        field = PointField()
        field.name = name
        field.offset = offset
        field.datatype = datatype
        field.count = 1
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

    def _read_candidates(self, path: Path) -> List[Tuple[RigidTransform, float]]:
        with path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        if isinstance(document, dict):
            document = document.get("candidates", [])
        if not isinstance(document, list):
            raise ValueError("candidates.json must contain a list")
        result = []
        for candidate in document[:self._top_k]:
            if not isinstance(candidate, dict):
                continue
            pose_document = candidate.get("pose_base_grasp") or candidate.get("pose")
            if pose_document is None:
                continue
            try:
                pose = self._rigid_from_document(pose_document)
                width = self._nonnegative_float(candidate.get("width_m", 0.04), "width_m")
            except (TypeError, ValueError):
                continue
            result.append((pose, width))
        return result

    def _read_selected(self, path: Path) -> Optional[Tuple[RigidTransform, float]]:
        with path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        if isinstance(document, dict) and "selected_grasp" in document:
            document = document["selected_grasp"]
        if not isinstance(document, dict):
            raise ValueError("selected_grasp.json must contain a mapping")
        pose_document = document.get("pose_base_grasp") or document.get("pose")
        if pose_document is None:
            return None
        return (
            self._rigid_from_document(pose_document),
            self._nonnegative_float(document.get("width_m", 0.04), "width_m"),
        )

    @staticmethod
    def _rigid_from_document(document: Any) -> RigidTransform:
        if not isinstance(document, dict):
            raise TypeError("grasp pose must be a mapping")
        translation = finite_vector(document.get("translation_m"), "translation_m", 3)
        rotation = normalize_quaternion(document.get("quat_xyzw"))
        return RigidTransform(translation, rotation)

    def _candidate_markers(self, candidates: Sequence[Tuple[RigidTransform, float]]) -> MarkerArray:
        result = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self._base_frame
        marker.ns = "grasp_candidates"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.003
        marker.color.r = 0.15
        marker.color.g = 0.9
        marker.color.b = 0.75
        marker.color.a = 0.70
        for pose, width in candidates:
            marker.points.extend(self._gripper_lines(pose, width))
        result.markers.append(marker)
        return result

    def _legacy_tcp_wireframe_marker(
        self,
        vertices: Sequence[Tuple[float, float, float, int]],
        edges: Sequence[Tuple[int, int]],
    ) -> MarkerArray:
        """Keep the raw legacy TCP wireframe separate from grasp-frame JSON."""
        marker = Marker()
        marker.header.frame_id = self._base_frame
        marker.ns = "legacy_tcp_wireframe"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.003
        marker.color.r = 0.35
        marker.color.g = 0.80
        marker.color.b = 1.0
        marker.color.a = 0.85
        for first, second in edges:
            marker.points.append(self._point(vertices[first][:3]))
            marker.points.append(self._point(vertices[second][:3]))
        result = MarkerArray()
        result.markers.append(marker)
        return result

    def _gripper_marker(
        self,
        namespace: str,
        marker_id: int,
        pose: RigidTransform,
        width_m: float,
        color: Tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._base_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.006
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = self._gripper_lines(pose, width_m)
        return marker

    def _gripper_lines(self, pose: RigidTransform, width_m: float) -> List[Point]:
        opening = max(0.005, min(width_m, 0.12))
        finger_length = 0.055
        finger_height = 0.025
        local_segments = (
            ((0.0, -opening / 2.0, -finger_height), (finger_length, -opening / 2.0, -finger_height)),
            ((0.0, -opening / 2.0, finger_height), (finger_length, -opening / 2.0, finger_height)),
            ((finger_length, -opening / 2.0, -finger_height), (finger_length, -opening / 2.0, finger_height)),
            ((0.0, opening / 2.0, -finger_height), (finger_length, opening / 2.0, -finger_height)),
            ((0.0, opening / 2.0, finger_height), (finger_length, opening / 2.0, finger_height)),
            ((finger_length, opening / 2.0, -finger_height), (finger_length, opening / 2.0, finger_height)),
            ((0.0, -opening / 2.0, 0.0), (0.0, opening / 2.0, 0.0)),
        )
        points = []
        for first, second in local_segments:
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
        message.header.frame_id = self._base_frame
        message.pose = self._pose(pose)
        return message

    @staticmethod
    def _pose(transform: RigidTransform) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = transform.translation
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = transform.rotation
        return pose

    @staticmethod
    def _point(values: Sequence[float]) -> Point:
        point = Point()
        point.x, point.y, point.z = values
        return point

    def _refresh_marker_stamps(self, markers: Iterable[Marker]) -> None:
        stamp = self.get_clock().now().to_msg()
        for marker in markers:
            marker.header.stamp = stamp


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
