"""ROS messages for the current-frame NeuGrasp artifact snapshot.

This ROS boundary deliberately consumes the pure :mod:`artifact_pipeline`
bundle.  It is shared by standalone replay and the phase-gated fake trial so
both publish identical local-volume geometry without duplicating processing.
"""

from __future__ import annotations

import struct
from typing import Sequence

import numpy as np
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from .artifact_pipeline import (
    GRIPPER_EDGES,
    ArtifactBundle,
    ArtifactSettings,
    gripper_wireframe_points,
    quality_color,
    voxel_center_position,
)


SNAPSHOT_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ArtifactVisualizationPublisher:
    """Publish one retained TSDF/candidate snapshot in a current volume frame."""

    def __init__(
        self,
        node: Node,
        target_frame: str,
        settings: ArtifactSettings,
        tsdf_z_index_range: Sequence[int] = (5, 30),
        grasp_wireframe_top_k: int = 50,
        finger_length_m: float = 0.05,
        palm_depth_m: float = 0.025,
    ) -> None:
        if not target_frame or target_frame.startswith("/"):
            raise ValueError("target_frame must be a non-empty relative TF frame")
        if grasp_wireframe_top_k < 0:
            raise ValueError("grasp_wireframe_top_k must be non-negative")
        z_index_range = tuple(int(value) for value in tsdf_z_index_range)
        if len(z_index_range) != 2:
            raise ValueError("tsdf_z_index_range must contain [min, max]")
        z_min, z_max = z_index_range
        if z_min < 0 or z_max < z_min or z_max >= settings.volume_resolution:
            raise ValueError("tsdf_z_index_range must be a valid inclusive volume Z range")
        self._node = node
        self._target_frame = target_frame
        self._settings = settings
        self._tsdf_z_index_range = z_index_range
        self._top_k = int(grasp_wireframe_top_k)
        self._finger_length_m = float(finger_length_m)
        self._palm_depth_m = float(palm_depth_m)
        self._cloud_publisher = node.create_publisher(
            PointCloud2, "/neugrasp/tsdf_cloud", SNAPSHOT_QOS
        )
        self._grasp_publisher = node.create_publisher(
            Marker, "/neugrasp/grasp_wireframes", SNAPSHOT_QOS
        )

    def publish_snapshot(self, bundle: ArtifactBundle) -> int:
        """Build and publish exactly one retained visualization snapshot."""
        cloud = self._cloud_message(bundle)
        grasp = self._grasp_marker_message(bundle)
        stamp = self._node.get_clock().now().to_msg()
        cloud.header.stamp = stamp
        grasp.header.stamp = stamp
        self._cloud_publisher.publish(cloud)
        self._grasp_publisher.publish(grasp)
        return cloud.width

    def _surface_indices(self, bundle: ArtifactBundle) -> np.ndarray:
        # The current NeuGrasp volume starts 0.0503 m below the workspace.
        # Render the configured inclusive Z-index band only.  The full tensor
        # remains untouched for candidate extraction and motion planning.
        z_min, z_max = self._tsdf_z_index_range
        z_indices = bundle.surface_indices[:, 2]
        return bundle.surface_indices[(z_indices >= z_min) & (z_indices <= z_max)]

    def _cloud_message(self, bundle: ArtifactBundle) -> PointCloud2:
        rows = []
        span = self._settings.tsdf_surface_threshold_high - self._settings.tsdf_surface_threshold_low
        for index_array in self._surface_indices(bundle):
            index = tuple(int(value) for value in index_array)
            i, j, k = index
            point = voxel_center_position(index, bundle.voxel_size_m)
            normalized = min(1.0, max(0.0, (
                float(bundle.tsdf[i, j, k]) - self._settings.tsdf_surface_threshold_low
            ) / span))
            rgb = (int(255 * normalized) << 16) | (80 << 8) | int(255 * (1.0 - normalized))
            rows.append((point[0], point[1], point[2], rgb))
        message = PointCloud2()
        message.header.frame_id = self._target_frame
        message.height, message.width = 1, len(rows)
        message.is_bigendian, message.is_dense = False, True
        message.fields = [
            self._field("x", 0, PointField.FLOAT32),
            self._field("y", 4, PointField.FLOAT32),
            self._field("z", 8, PointField.FLOAT32),
            self._field("rgb", 12, PointField.FLOAT32),
        ]
        message.point_step = 16
        message.row_step = message.point_step * message.width
        data = bytearray(message.row_step)
        for row_index, (x, y, z, rgb) in enumerate(rows):
            rgb_float = struct.unpack("<f", struct.pack("<I", rgb))[0]
            struct.pack_into("<ffff", data, row_index * message.point_step, x, y, z, rgb_float)
        message.data = bytes(data)
        return message

    def _grasp_marker_message(self, bundle: ArtifactBundle) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._target_frame
        marker.ns, marker.id = "grasp_wireframes", 0
        marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.004
        marker.color.a = 1.0
        for candidate in bundle.candidates[:self._top_k]:
            local_points = gripper_wireframe_points(
                candidate.width_m,
                finger_length_m=self._finger_length_m,
                palm_depth_m=self._palm_depth_m,
            )
            color = self._color(quality_color(
                candidate.score, minimum_score=self._settings.grasp_score_threshold
            ))
            for first, second in GRIPPER_EDGES:
                for local_point in (local_points[first], local_points[second]):
                    marker.points.append(self._point(candidate.rotation_matrix.dot(local_point) + candidate.position_m))
                    marker.colors.append(color)
        return marker

    @staticmethod
    def _field(name: str, offset: int, datatype: int) -> PointField:
        field = PointField()
        field.name, field.offset, field.datatype, field.count = name, offset, datatype, 1
        return field

    @staticmethod
    def _point(values: Sequence[float]) -> Point:
        point = Point()
        point.x, point.y, point.z = (float(value) for value in values)
        return point

    @staticmethod
    def _color(values: Sequence[float]) -> ColorRGBA:
        color = ColorRGBA()
        color.r, color.g, color.b, color.a = (float(value) for value in values)
        return color
