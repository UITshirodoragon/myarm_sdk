"""Visualize NeuGrasp inference products in the current volume frame."""

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
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .math3d import (
    RigidTransform,
    compose,
    finite_vector,
    normalize_quaternion,
    rotate_vector,
)


_VISUALIZATION_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class NeugraspReplayNode(Node):
    """Replay TSDF, quality and grasp visualizations without historical TF."""

    def __init__(self) -> None:
        super().__init__("neugrasp_replay")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("scene_config", "")
        self.declare_parameter("grasp_visualization_config", "")
        self.declare_parameter("volume_frame", "neugrasp_volume")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tsdf_threshold_low", -0.2)
        self.declare_parameter("tsdf_threshold_high", 0.2)
        self.declare_parameter("quality_threshold", 0.1)
        self.declare_parameter("voxel_resolution", 40)
        self.declare_parameter("max_cloud_points", 50000)
        self.declare_parameter("max_quality_points", 10000)
        self.declare_parameter("top_k", 50)
        self.declare_parameter("republish_period_s", 2.0)
        self.declare_parameter("publish_on_start", True)

        run_dir_value = str(self.get_parameter("run_dir").value).strip()
        self._run_dir = Path(run_dir_value).expanduser() if run_dir_value else None
        self._volume_frame = self._frame(self.get_parameter("volume_frame").value, "volume_frame")
        self._base_frame = self._frame(self.get_parameter("base_frame").value, "base_frame")
        self._threshold_low = self._finite_float("tsdf_threshold_low")
        self._threshold_high = self._finite_float("tsdf_threshold_high")
        if self._threshold_low >= self._threshold_high:
            raise ValueError("tsdf_threshold_low must be below tsdf_threshold_high")
        self._quality_threshold = self._finite_float("quality_threshold")
        if not 0.0 <= self._quality_threshold <= 1.0:
            raise ValueError("quality_threshold must be in [0, 1]")
        self._voxel_resolution = self._positive_int("voxel_resolution")
        self._max_cloud_points = self._positive_int("max_cloud_points")
        self._max_quality_points = self._positive_int("max_quality_points")
        self._top_k = self._positive_int("top_k")
        self._volume_extent_m = self._load_volume_extent()
        self._legacy_voxel_size_m = tuple(
            extent / self._voxel_resolution for extent in self._volume_extent_m
        )
        self._grasp_tcp, self._gripper_geometry = self._load_grasp_visualization()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._cloud_publisher = self.create_publisher(
            PointCloud2, "/neugrasp/tsdf_cloud", _VISUALIZATION_QOS
        )
        self._quality_publisher = self.create_publisher(
            PointCloud2, "/neugrasp/grasp_quality_raw_cloud", _VISUALIZATION_QOS
        )
        self._legacy_cloud_publisher = self.create_publisher(
            PointCloud2, "/neugrasp/legacy_tsdf_ply_cloud", _VISUALIZATION_QOS
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
        self._legacy_wireframe_publisher = self.create_publisher(
            MarkerArray, "/neugrasp/legacy_grasp_wireframes", _VISUALIZATION_QOS
        )
        self._status_publisher = self.create_publisher(
            String, "/neugrasp/replay/status", _VISUALIZATION_QOS
        )

        self._tsdf_cloud = self._tsdf_cloud_message(())
        self._quality_cloud = self._quality_cloud_message(())
        self._legacy_cloud = self._legacy_cloud_message(())
        self._legacy_tsdf_vertices: Optional[List[Tuple[float, float, float, int]]] = None
        self._legacy_wire_vertices: Optional[List[Tuple[float, float, float, int]]] = None
        self._legacy_wire_edges: Optional[List[Tuple[int, int]]] = None
        self._legacy_waiting_for_tf = False
        self._candidates = self._delete_all_marker_array()
        self._selected_pose: Optional[PoseStamped] = None
        self._selected_marker = self._delete_all_marker()
        self._legacy_wireframes = self._delete_all_marker_array()
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
        if not all(component > 0.0 for component in extent):
            raise ValueError("scene_config workspace.bbox_m must have positive extents")
        if max(extent) - min(extent) > 1e-9:
            raise ValueError("workspace.bbox_m must be cubic for RViz PointCloud2 Boxes")
        return extent

    def _load_grasp_visualization(self) -> Tuple[RigidTransform, Dict[str, float]]:
        value = str(self.get_parameter("grasp_visualization_config").value).strip()
        if not value:
            raise ValueError("grasp_visualization_config is required")
        path = Path(value).expanduser()
        try:
            with path.open("r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream)
            if config.get("schema_version") != 1:
                raise ValueError("schema_version must be 1")
            tcp = config["grasp_tcp"]
            transform = RigidTransform(
                finite_vector(tcp["translation_m"], "grasp_tcp.translation_m", 3),
                normalize_quaternion(tcp["rotation_xyzw"]),
            )
            wireframe = config["wireframe"]
            geometry = {
                key: float(wireframe[key])
                for key in ("finger_length_m", "finger_height_m", "min_opening_m", "max_opening_m")
            }
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            raise ValueError(f"invalid grasp visualization config {path}: {error}") from error
        if (
            geometry["finger_length_m"] <= 0.0
            or geometry["finger_height_m"] <= 0.0
            or geometry["min_opening_m"] <= 0.0
            or geometry["min_opening_m"] > geometry["max_opening_m"]
        ):
            raise ValueError("wireframe dimensions/opening range must be positive and ordered")
        return transform, geometry

    def _load_run(self) -> None:
        self._tsdf_cloud = self._tsdf_cloud_message(())
        self._quality_cloud = self._quality_cloud_message(())
        self._legacy_cloud = self._legacy_cloud_message(())
        self._legacy_tsdf_vertices = None
        self._legacy_wire_vertices = None
        self._legacy_wire_edges = None
        self._legacy_waiting_for_tf = False
        self._candidates = self._delete_all_marker_array()
        self._selected_pose = None
        self._selected_marker = self._delete_all_marker()
        self._legacy_wireframes = self._delete_all_marker_array()
        if self._run_dir is None:
            self._publish_status("idle: run_dir is empty")
            return
        if not self._run_dir.is_dir():
            self._publish_status(f"error: run_dir does not exist: {self._run_dir}")
            return
        statuses = [
            self._load_tsdf_and_quality(),
            self._load_candidates(),
            self._load_selected(),
            self._load_legacy_ply(),
            "rotation_raw=not_visualized_untrusted_channel_order",
        ]
        self._publish_status("; ".join(statuses))

    def _load_tsdf_and_quality(self) -> str:
        assert self._run_dir is not None
        tsdf_path = self._run_dir / "inference" / "tsdf_vol.npy"
        quality_path = self._run_dir / "inference" / "qual_vol_raw.npy"
        width_path = self._run_dir / "inference" / "width_vol_raw.npy"
        if not tsdf_path.is_file():
            return "tsdf=missing"
        try:
            tsdf = self._volume_from_npy(tsdf_path, "tsdf")
            self._tsdf_cloud = self._tsdf_cloud_from_volume(tsdf)
            if quality_path.is_file() and width_path.is_file():
                quality = self._volume_from_npy(quality_path, "quality")
                width = self._volume_from_npy(width_path, "width")
                if quality.shape != tsdf.shape or width.shape != tsdf.shape:
                    raise ValueError("tsdf, quality and width tensors must have the same shape")
                self._quality_cloud = self._quality_cloud_from_volumes(tsdf, quality, width)
                return f"tsdf_voxels={self._tsdf_cloud.width}; quality_raw_voxels={self._quality_cloud.width}"
            return f"tsdf_voxels={self._tsdf_cloud.width}; quality_raw=missing"
        except (OSError, ValueError, TypeError) as error:
            return f"tsdf_or_quality_error={error}"

    def _volume_from_npy(self, path: Path, name: str) -> np.ndarray:
        volume = np.asarray(np.load(path, allow_pickle=False)).squeeze()
        if volume.ndim != 3 or min(volume.shape) <= 0 or len(set(volume.shape)) != 1:
            raise ValueError(f"{name} must reduce to a non-empty cubic 3D tensor, got {volume.shape}")
        if not bool(np.isfinite(volume).all()):
            raise ValueError(f"{name} contains non-finite values")
        return volume

    def _voxel_steps(self, volume: np.ndarray) -> Tuple[float, float, float]:
        return tuple(self._volume_extent_m[index] / volume.shape[index] for index in range(3))

    def _tsdf_cloud_from_volume(self, tsdf: np.ndarray) -> PointCloud2:
        indices = np.argwhere((tsdf > self._threshold_low) & (tsdf < self._threshold_high))
        indices = self._downsample_indices(indices, self._max_cloud_points)
        steps = self._voxel_steps(tsdf)
        rows = []
        for index in indices:
            i, j, k = (int(value) for value in index)
            rows.append(((i + 0.5) * steps[0], (j + 0.5) * steps[1], (k + 0.5) * steps[2],
                         float(tsdf[i, j, k])))
        return self._tsdf_cloud_message(rows)

    def _quality_cloud_from_volumes(
        self, tsdf: np.ndarray, quality: np.ndarray, width: np.ndarray
    ) -> PointCloud2:
        mask = (
            (tsdf > self._threshold_low)
            & (tsdf < self._threshold_high)
            & (quality >= self._quality_threshold)
        )
        indices = self._downsample_indices(np.argwhere(mask), self._max_quality_points)
        steps = self._voxel_steps(tsdf)
        rows = []
        for index in indices:
            i, j, k = (int(value) for value in index)
            rows.append(((i + 0.5) * steps[0], (j + 0.5) * steps[1], (k + 0.5) * steps[2],
                         float(tsdf[i, j, k]), float(quality[i, j, k]), float(width[i, j, k])))
        return self._quality_cloud_message(rows)

    @staticmethod
    def _downsample_indices(indices: np.ndarray, maximum: int) -> np.ndarray:
        if len(indices) <= maximum:
            return indices
        return indices[::int(math.ceil(len(indices) / maximum))]

    def _load_candidates(self) -> str:
        assert self._run_dir is not None
        path = self._run_dir / "inference" / "candidates.json"
        if not path.is_file():
            return "candidates=missing"
        try:
            candidates = self._read_candidates(path)
            self._candidates = self._candidate_markers(candidates)
            return f"candidates=current_volume_tcp:{len(candidates)}"
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
            pose, width_m, _ = selected
            tcp = compose(pose, self._grasp_tcp)
            self._selected_pose = self._pose_stamped(tcp)
            self._selected_marker = self._gripper_marker(
                "selected_grasp", 0, tcp, width_m, (1.0, 0.65, 0.05, 1.0), 0.006
            )
            return "selected=current_volume_tcp:1"
        except (OSError, ValueError, TypeError, KeyError) as error:
            return f"selected=error:{error}"

    def _read_candidates(self, path: Path) -> List[Tuple[RigidTransform, float, float]]:
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

    def _read_candidate(self, path: Path) -> Optional[Tuple[RigidTransform, float, float]]:
        with path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        if isinstance(document, dict) and "selected_grasp" in document:
            document = document["selected_grasp"]
        return self._candidate_from_document(document)

    def _candidate_from_document(self, document: Any) -> Optional[Tuple[RigidTransform, float, float]]:
        if not isinstance(document, dict):
            return None
        # Only the cube/volume pose survives replay.  base/table/TCP poses in
        # a run depend on historical calibration and are never used here.
        pose_document = document.get("pose_cube_grasp")
        if not isinstance(pose_document, dict):
            return None
        pose = RigidTransform(
            finite_vector(pose_document["translation_m"], "pose_cube_grasp.translation_m", 3),
            normalize_quaternion(pose_document["quat_xyzw"]),
        )
        width_m = float(document.get("width_m", 0.04))
        score = float(document.get("score", 0.0))
        if not math.isfinite(width_m) or width_m < 0.0:
            raise ValueError("width_m must be finite and non-negative")
        if not math.isfinite(score):
            raise ValueError("score must be finite")
        return pose, width_m, score

    def _load_legacy_ply(self) -> str:
        assert self._run_dir is not None
        tsdf_path = self._run_dir / "visualizations" / "tsdf_near_surface_base.ply"
        wire_path = self._run_dir / "visualizations" / "grasp_candidates_wireframes_base.ply"
        statuses = []
        try:
            if tsdf_path.is_file():
                self._legacy_tsdf_vertices, _ = self._read_ascii_ply(tsdf_path)
                statuses.append(f"legacy_tsdf_ply={len(self._legacy_tsdf_vertices)}_pending_current_tf")
            else:
                statuses.append("legacy_tsdf_ply=missing")
            if wire_path.is_file():
                self._legacy_wire_vertices, self._legacy_wire_edges = self._read_ascii_ply(wire_path)
                statuses.append(f"legacy_gripper_ply={len(self._legacy_wire_edges)}_edges_pending_current_tf")
            else:
                statuses.append("legacy_gripper_ply=missing")
            self._legacy_waiting_for_tf = (
                self._legacy_tsdf_vertices is not None or self._legacy_wire_vertices is not None
            )
            return ",".join(statuses)
        except (OSError, ValueError, IndexError, OverflowError) as error:
            return f"legacy_ply=error:{error}"

    def _update_legacy_products_from_current_tf(self) -> None:
        if not self._legacy_waiting_for_tf:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._volume_frame, self._base_frame, Time(), timeout=Duration(seconds=0.0)
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            base_to_volume = RigidTransform(
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            if self._legacy_tsdf_vertices is not None:
                rows = []
                for x, y, z, rgb in self._legacy_tsdf_vertices:
                    point = self._transform_point(base_to_volume, (x, y, z))
                    rows.append((
                        point[0] + self._legacy_voxel_size_m[0] / 2.0,
                        point[1] + self._legacy_voxel_size_m[1] / 2.0,
                        point[2] + self._legacy_voxel_size_m[2] / 2.0,
                        rgb,
                    ))
                self._legacy_cloud = self._legacy_cloud_message(rows)
            if self._legacy_wire_vertices is not None and self._legacy_wire_edges is not None:
                self._legacy_wireframes = self._legacy_wireframe_markers(
                    base_to_volume, self._legacy_wire_vertices, self._legacy_wire_edges
                )
            self._legacy_waiting_for_tf = False
            self._publish_status("legacy_ply=current_base_to_volume_transform_applied")
        except TransformException:
            # Scene TF may appear after replay starts. Never relabel unconverted
            # base points as volume points; retry at the next low-rate publish.
            return

    def _read_ascii_ply(
        self, path: Path
    ) -> Tuple[List[Tuple[float, float, float, int]], List[Tuple[int, int]]]:
        with path.open("r", encoding="utf-8") as stream:
            lines = stream.readlines()
        if not lines or lines[0].strip() != "ply":
            raise ValueError(f"not an ASCII PLY: {path}")
        vertex_count, edge_count, header_end = None, 0, None
        properties: List[str] = []
        in_vertex = False
        for index, line in enumerate(lines[1:], start=1):
            fields = line.strip().split()
            if fields[:2] == ["element", "vertex"]:
                vertex_count, in_vertex = int(fields[2]), True
            elif fields[:2] == ["element", "edge"]:
                edge_count, in_vertex = int(fields[2]), False
            elif fields and fields[0] == "element":
                in_vertex = False
            elif fields and fields[0] == "property" and in_vertex:
                properties.append(fields[-1])
            elif fields and fields[0] == "end_header":
                header_end = index + 1
                break
        if vertex_count is None or vertex_count < 0 or edge_count < 0 or header_end is None:
            raise ValueError("PLY header has no valid vertex section")
        if not {"x", "y", "z"}.issubset(properties) or len(lines) < header_end + vertex_count + edge_count:
            raise ValueError("PLY vertex/edge section is invalid or truncated")
        fields_by_name = {name: index for index, name in enumerate(properties)}
        vertices = []
        for line in lines[header_end:header_end + vertex_count]:
            values = line.strip().split()
            x, y, z = (float(values[fields_by_name[name]]) for name in ("x", "y", "z"))
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise ValueError("PLY contains non-finite point")
            red = int(values[fields_by_name["red"]]) if "red" in fields_by_name else 180
            green = int(values[fields_by_name["green"]]) if "green" in fields_by_name else 180
            blue = int(values[fields_by_name["blue"]]) if "blue" in fields_by_name else 180
            vertices.append((x, y, z, (max(0, min(red, 255)) << 16) |
                             (max(0, min(green, 255)) << 8) | max(0, min(blue, 255))))
        edges = []
        for line in lines[header_end + vertex_count:header_end + vertex_count + edge_count]:
            fields = line.strip().split()
            first, second = int(fields[-2]), int(fields[-1])
            if 0 <= first < len(vertices) and 0 <= second < len(vertices):
                edges.append((first, second))
        return vertices, edges

    def _tsdf_cloud_message(self, rows: Iterable[Tuple[float, float, float, float]]) -> PointCloud2:
        return self._point_cloud(rows, (("tsdf", PointField.FLOAT32),))

    def _quality_cloud_message(
        self, rows: Iterable[Tuple[float, float, float, float, float, float]]
    ) -> PointCloud2:
        return self._point_cloud(rows, (
            ("tsdf", PointField.FLOAT32),
            ("quality", PointField.FLOAT32),
            ("width_vox", PointField.FLOAT32),
        ))

    def _legacy_cloud_message(self, rows: Iterable[Tuple[float, float, float, int]]) -> PointCloud2:
        return self._point_cloud(rows, (("rgb", PointField.UINT32),))

    def _point_cloud(self, rows: Iterable[tuple], extra_fields: Sequence[Tuple[str, int]]) -> PointCloud2:
        rows = list(rows)
        message = PointCloud2()
        message.header.frame_id = self._volume_frame
        message.height, message.width, message.is_bigendian, message.is_dense = 1, len(rows), False, True
        fields = [
            self._field("x", 0, PointField.FLOAT32),
            self._field("y", 4, PointField.FLOAT32),
            self._field("z", 8, PointField.FLOAT32),
        ]
        offset = 12
        pack_format = "<fff"
        for name, datatype in extra_fields:
            fields.append(self._field(name, offset, datatype))
            if datatype == PointField.FLOAT32:
                pack_format += "f"
                offset += 4
            elif datatype == PointField.UINT32:
                pack_format += "I"
                offset += 4
            else:
                raise ValueError(f"unsupported PointCloud2 field datatype for {name}")
        message.fields, message.point_step = fields, offset
        message.row_step = message.point_step * message.width
        data = bytearray(message.row_step)
        for index, row in enumerate(rows):
            struct.pack_into(pack_format, data, index * message.point_step, *row)
        message.data = bytes(data)
        return message

    def _candidate_markers(self, candidates: Sequence[Tuple[RigidTransform, float, float]]) -> MarkerArray:
        result = self._delete_all_marker_array()
        for index, (grasp, width_m, score) in enumerate(candidates):
            tcp = compose(grasp, self._grasp_tcp)
            # Blue-to-green is a score cue; selected remains orange and is
            # published separately so it is visually unambiguous.
            normalized = max(0.0, min(1.0, score))
            color = (0.10, 0.35 + 0.55 * normalized, 1.0 - 0.55 * normalized, 0.72)
            result.markers.append(
                self._gripper_marker("grasp_candidates", index, tcp, width_m, color, 0.003)
            )
        return result

    def _legacy_wireframe_markers(
        self,
        base_to_volume: RigidTransform,
        vertices: Sequence[Tuple[float, float, float, int]],
        edges: Sequence[Tuple[int, int]],
    ) -> MarkerArray:
        result = self._delete_all_marker_array()
        marker = Marker()
        marker.header.frame_id, marker.ns, marker.id = self._volume_frame, "legacy_grasp_wireframes", 0
        marker.type, marker.action, marker.scale.x = Marker.LINE_LIST, Marker.ADD, 0.0025
        for first, second in edges:
            for vertex_index in (first, second):
                x, y, z, rgb = vertices[vertex_index]
                marker.points.append(self._point(self._transform_point(base_to_volume, (x, y, z))))
                marker.colors.append(self._color_from_rgb(rgb, 0.85))
        result.markers.append(marker)
        return result

    def _gripper_marker(
        self,
        namespace: str,
        marker_id: int,
        tcp: RigidTransform,
        width_m: float,
        color: Tuple[float, float, float, float],
        line_width_m: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id, marker.ns, marker.id = self._volume_frame, namespace, marker_id
        marker.type, marker.action, marker.scale.x = Marker.LINE_LIST, Marker.ADD, line_width_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = self._gripper_lines(tcp, width_m)
        return marker

    def _gripper_lines(self, tcp: RigidTransform, width_m: float) -> List[Point]:
        opening = max(
            self._gripper_geometry["min_opening_m"],
            min(width_m, self._gripper_geometry["max_opening_m"]),
        )
        finger_length = self._gripper_geometry["finger_length_m"]
        finger_height = self._gripper_geometry["finger_height_m"]
        segments = (
            ((0.0, -opening / 2.0, -finger_height), (finger_length, -opening / 2.0, -finger_height)),
            ((0.0, -opening / 2.0, finger_height), (finger_length, -opening / 2.0, finger_height)),
            ((finger_length, -opening / 2.0, -finger_height), (finger_length, -opening / 2.0, finger_height)),
            ((0.0, opening / 2.0, -finger_height), (finger_length, opening / 2.0, -finger_height)),
            ((0.0, opening / 2.0, finger_height), (finger_length, opening / 2.0, finger_height)),
            ((finger_length, opening / 2.0, -finger_height), (finger_length, opening / 2.0, finger_height)),
            ((0.0, -opening / 2.0, 0.0), (0.0, opening / 2.0, 0.0)),
        )
        return [
            self._point(self._transform_point(tcp, endpoint))
            for segment in segments for endpoint in segment
        ]

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
    def _color_from_rgb(rgb: int, alpha: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r, color.g, color.b, color.a = (
            ((rgb >> 16) & 0xFF) / 255.0,
            ((rgb >> 8) & 0xFF) / 255.0,
            (rgb & 0xFF) / 255.0,
            alpha,
        )
        return color

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
        self._update_legacy_products_from_current_tf()
        stamp = self.get_clock().now().to_msg()
        for publisher, message in (
            (self._cloud_publisher, self._tsdf_cloud),
            (self._quality_publisher, self._quality_cloud),
            (self._legacy_cloud_publisher, self._legacy_cloud),
        ):
            message.header.stamp = stamp
            publisher.publish(message)
        self._refresh_marker_stamps(self._candidates.markers)
        self._candidate_publisher.publish(self._candidates)
        self._refresh_marker_stamps(self._legacy_wireframes.markers)
        self._legacy_wireframe_publisher.publish(self._legacy_wireframes)
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
