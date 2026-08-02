"""Retained, artifact-only NeuGrasp NPY replay."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node

from .artifact_pipeline import ArtifactSettings, load_artifacts
from .artifact_visualization import ArtifactVisualizationPublisher


_SETTING_PARAMETERS = (
    "volume_size_m", "volume_resolution", "tsdf_surface_threshold_low",
    "tsdf_surface_threshold_high", "gaussian_filter_sigma", "min_width_vox",
    "max_width_vox", "grasp_score_threshold", "max_filter_size", "gripper_max_width_m",
)


class NeugraspReplayNode(Node):
    """Publish one retained current-volume snapshot, optionally at a low rate."""

    def __init__(self) -> None:
        super().__init__("neugrasp_replay")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("source_frame", "base_link")  # compatibility only; ignored
        self.declare_parameter("target_frame", "neugrasp_volume")
        # Zero means one retained snapshot.  A positive rate is only for
        # legacy volatile RViz consumers and is intentionally opt-in.
        self.declare_parameter("republish_period_s", 0.0)
        self.declare_parameter("tsdf_z_index_range", [5, 30])
        self.declare_parameter("grasp_wireframe_top_k", 50)
        self.declare_parameter("gripper_finger_length_m", 0.05)
        self.declare_parameter("gripper_palm_depth_m", 0.025)
        self.declare_parameter("voxel_reference", "center")
        for name, value in (("volume_size_m", 0.30), ("volume_resolution", 40),
                            ("tsdf_surface_threshold_low", -0.85), ("tsdf_surface_threshold_high", 0.0),
                            ("gaussian_filter_sigma", 1.0), ("min_width_vox", 1.33),
                            ("max_width_vox", 9.33), ("grasp_score_threshold", 0.70),
                            ("max_filter_size", 4), ("gripper_max_width_m", 0.08)):
            self.declare_parameter(name, value)
        run_dir_value = str(self.get_parameter("run_dir").value).strip()
        if not run_dir_value:
            raise ValueError("run_dir must name an existing completed run")
        run_dir = Path(run_dir_value).expanduser()
        if not run_dir.is_dir():
            raise ValueError("run_dir must name an existing completed run")
        target = self._frame(self.get_parameter("target_frame").value, "target_frame")
        if str(self.get_parameter("voxel_reference").value).strip().lower() != "center":
            raise ValueError("voxel_reference is compatibility-only; center is required")
        settings = ArtifactSettings.from_mapping({
            name: self.get_parameter(name).value for name in _SETTING_PARAMETERS
        })
        self._bundle = load_artifacts(run_dir, settings)
        self._tsdf_z_index_range = self._z_index_range(
            self.get_parameter("tsdf_z_index_range").value, settings.volume_resolution
        )
        self._visualizer = ArtifactVisualizationPublisher(
            self, target, settings,
            tsdf_z_index_range=self._tsdf_z_index_range,
            grasp_wireframe_top_k=self._nonnegative_int(self.get_parameter("grasp_wireframe_top_k").value, "grasp_wireframe_top_k"),
            finger_length_m=self._positive_float(self.get_parameter("gripper_finger_length_m").value, "gripper_finger_length_m"),
            palm_depth_m=self._positive_float(self.get_parameter("gripper_palm_depth_m").value, "gripper_palm_depth_m"),
        )
        rendered_points = self._publish()
        period = self._nonnegative_float(self.get_parameter("republish_period_s").value, "republish_period_s")
        if period > 0.0:
            self.create_timer(period, self._publish)
        self.get_logger().info(
            "Published retained NeuGrasp artifact snapshot in {}: tsdf_points={} candidates={} rendered_tsdf_points={}".format(
                target, len(self._bundle.surface_indices), len(self._bundle.candidates),
                rendered_points,
            )
        )

    def _publish(self) -> int:
        return self._visualizer.publish_snapshot(self._bundle)

    @staticmethod
    def _frame(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise ValueError(f"{name} must be a non-empty relative TF frame")
        return value.strip()

    @staticmethod
    def _nonnegative_float(value: Any, name: str) -> float:
        result = float(value)
        if not math.isfinite(result) or result < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return result

    @classmethod
    def _positive_float(cls, value: Any, name: str) -> float:
        result = cls._nonnegative_float(value, name)
        if result <= 0.0:
            raise ValueError(f"{name} must be positive")
        return result

    @staticmethod
    def _nonnegative_int(value: Any, name: str) -> int:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be an integer")
        result = int(value)
        if result < 0 or result != value:
            raise ValueError(f"{name} must be a non-negative integer")
        return result

    @staticmethod
    def _z_index_range(value: Any, resolution: int):
        if not isinstance(value, (list, tuple)):
            raise TypeError("tsdf_z_index_range must be an integer array")
        z_index_range = tuple(int(item) for item in value)
        if len(z_index_range) != 2:
            raise ValueError("tsdf_z_index_range must contain [min, max]")
        z_min, z_max = z_index_range
        if z_min < 0 or z_max < z_min or z_max >= resolution:
            raise ValueError("tsdf_z_index_range must be a valid inclusive volume Z range")
        return z_index_range


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
