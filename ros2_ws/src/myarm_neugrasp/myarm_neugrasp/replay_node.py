"""Replay exactly two legacy PLY products through the current TF tree."""

from __future__ import annotations

import math
import struct
from pathlib import Path
from typing import Any, Iterable, List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from .math3d import RigidTransform, rotate_vector


_REPLAY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class NeugraspReplayNode(Node):
    """Publish TSDF PLY and grasp-wireframe PLY in one current target frame."""

    def __init__(self) -> None:
        super().__init__("neugrasp_replay")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("source_frame", "base_link")
        self.declare_parameter("target_frame", "neugrasp_volume")
        self.declare_parameter("republish_period_s", 1.0)

        run_dir_value = str(self.get_parameter("run_dir").value).strip()
        if not run_dir_value:
            raise ValueError("run_dir must contain the two PLY visualization files")
        self._run_dir = Path(run_dir_value).expanduser()
        if not self._run_dir.is_dir():
            raise ValueError(f"run_dir does not exist: {self._run_dir}")
        self._source_frame = self._frame(self.get_parameter("source_frame").value, "source_frame")
        self._target_frame = self._frame(self.get_parameter("target_frame").value, "target_frame")
        period_s = self._positive_float(self.get_parameter("republish_period_s").value, "republish_period_s")

        tsdf_path = self._run_dir / "visualizations" / "tsdf_near_surface_base.ply"
        grasp_path = self._run_dir / "visualizations" / "grasp_candidates_wireframes_base.ply"
        self._tsdf_vertices, _ = self._read_ascii_ply(tsdf_path)
        self._grasp_vertices, self._grasp_edges = self._read_ascii_ply(grasp_path)
        self.get_logger().info(
            "Loaded PLY replay artifacts: "
            f"tsdf_points={len(self._tsdf_vertices)} "
            f"grasp_vertices={len(self._grasp_vertices)} "
            f"grasp_edges={len(self._grasp_edges)}"
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._cloud_publisher = self.create_publisher(PointCloud2, "/neugrasp/tsdf_cloud", _REPLAY_QOS)
        self._grasp_publisher = self.create_publisher(
            Marker, "/neugrasp/grasp_wireframes", _REPLAY_QOS
        )
        self._cloud = self._cloud_message(())
        self._grasp_marker = self._empty_grasp_marker()
        self._converted = False
        self._publish_timer = self.create_timer(period_s, self._publish)

    def _publish(self) -> None:
        if not self._converted:
            self._convert_with_current_tf()
        if not self._converted:
            return
        stamp = self.get_clock().now().to_msg()
        self._cloud.header.stamp = stamp
        self._grasp_marker.header.stamp = stamp
        self._cloud_publisher.publish(self._cloud)
        self._grasp_publisher.publish(self._grasp_marker)

    def _convert_with_current_tf(self) -> None:
        try:
            message = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return
        translation = message.transform.translation
        rotation = message.transform.rotation
        target_from_source = RigidTransform(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        self._cloud = self._cloud_message(
            (*self._transform_point(target_from_source, (x, y, z)), rgb)
            for x, y, z, rgb in self._tsdf_vertices
        )
        self._grasp_marker = self._grasp_marker_message(target_from_source)
        self._converted = True
        self.get_logger().info(
            f"Converted both PLY files with current TF "
            f"{self._target_frame} <- {self._source_frame}"
        )

    def _cloud_message(self, rows: Iterable[Tuple[float, float, float, int]]) -> PointCloud2:
        rows = list(rows)
        message = PointCloud2()
        message.header.frame_id = self._target_frame
        message.height, message.width = 1, len(rows)
        message.is_bigendian, message.is_dense = False, True
        message.fields = [
            self._field("x", 0, PointField.FLOAT32),
            self._field("y", 4, PointField.FLOAT32),
            self._field("z", 8, PointField.FLOAT32),
            # The FLOAT32 bit pattern is the PCL/RViz convention for packed RGB.
            self._field("rgb", 12, PointField.FLOAT32),
        ]
        message.point_step = 16
        message.row_step = message.point_step * message.width
        data = bytearray(message.row_step)
        for index, (x, y, z, rgb) in enumerate(rows):
            rgb_float = struct.unpack("<f", struct.pack("<I", rgb))[0]
            struct.pack_into("<ffff", data, index * message.point_step, x, y, z, rgb_float)
        message.data = bytes(data)
        return message

    def _empty_grasp_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._target_frame
        marker.ns, marker.id = "grasp_wireframes", 0
        marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.004
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.55, 0.0, 1.0
        return marker

    def _grasp_marker_message(self, target_from_source: RigidTransform) -> Marker:
        marker = self._empty_grasp_marker()
        for first, second in self._grasp_edges:
            for vertex_index in (first, second):
                x, y, z, _ = self._grasp_vertices[vertex_index]
                marker.points.append(self._point(self._transform_point(target_from_source, (x, y, z))))
        return marker

    def _read_ascii_ply(
        self, path: Path
    ) -> Tuple[List[Tuple[float, float, float, int]], List[Tuple[int, int]]]:
        if not path.is_file():
            raise FileNotFoundError(f"required PLY is missing: {path}")
        with path.open("r", encoding="utf-8") as stream:
            lines = stream.readlines()
        if not lines or lines[0].strip() != "ply":
            raise ValueError(f"not an ASCII PLY: {path}")
        vertex_count, edge_count, header_end = 0, 0, None
        for index, line in enumerate(lines):
            fields = line.strip().split()
            if fields[:2] == ["element", "vertex"]:
                vertex_count = int(fields[-1])
            elif fields[:2] == ["element", "edge"]:
                edge_count = int(fields[-1])
            elif fields == ["end_header"]:
                header_end = index + 1
                break
        if header_end is None or vertex_count < 0 or edge_count < 0:
            raise ValueError(f"invalid PLY header: {path}")
        if len(lines) < header_end + vertex_count + edge_count:
            raise ValueError(f"truncated PLY data: {path}")
        vertices = []
        for line in lines[header_end:header_end + vertex_count]:
            fields = line.strip().split()
            if len(fields) < 3:
                raise ValueError(f"invalid PLY vertex row: {path}")
            x, y, z = (float(value) for value in fields[:3])
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise ValueError(f"non-finite PLY vertex: {path}")
            red, green, blue = (int(value) for value in fields[3:6]) if len(fields) >= 6 else (180, 180, 180)
            rgb = (max(0, min(red, 255)) << 16) | (max(0, min(green, 255)) << 8) | max(0, min(blue, 255))
            vertices.append((x, y, z, rgb))
        edges = []
        for line in lines[header_end + vertex_count:header_end + vertex_count + edge_count]:
            fields = line.strip().split()
            if len(fields) >= 2:
                first, second = int(fields[0]), int(fields[1])
                if 0 <= first < len(vertices) and 0 <= second < len(vertices):
                    edges.append((first, second))
        return vertices, edges

    @staticmethod
    def _transform_point(transform: RigidTransform, point: Sequence[float]) -> Tuple[float, float, float]:
        rotated = rotate_vector(transform.rotation, tuple(point))
        return (
            transform.translation[0] + rotated[0],
            transform.translation[1] + rotated[1],
            transform.translation[2] + rotated[2],
        )

    @staticmethod
    def _field(name: str, offset: int, datatype: int) -> PointField:
        field = PointField()
        field.name, field.offset, field.datatype, field.count = name, offset, datatype, 1
        return field

    @staticmethod
    def _point(values: Sequence[float]) -> Point:
        point = Point()
        point.x, point.y, point.z = values
        return point

    @staticmethod
    def _frame(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError(f"{name} must be a non-empty relative TF frame")
        return value.strip()

    @staticmethod
    def _positive_float(value: Any, name: str) -> float:
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
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
