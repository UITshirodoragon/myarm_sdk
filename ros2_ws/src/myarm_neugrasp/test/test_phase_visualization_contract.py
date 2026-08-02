"""Small unit contracts for fake-trial timing and retained visualization."""

from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from builtin_interfaces.msg import Time

from myarm_neugrasp.artifact_visualization import ArtifactVisualizationPublisher
from myarm_neugrasp.math3d import RigidTransform
from myarm_neugrasp.scan_node import NeugraspScanNode
from myarm_neugrasp.trial_node import NeugraspTrialNode


class PhaseVisualizationContractTest(unittest.TestCase):
    def test_fake_trial_config_has_timing_and_grasp_to_tool0_contract(self):
        root = Path(__file__).resolve().parents[2]
        config = root / "neugrasp_bringup" / "config" / "neugrasp_fake_trial.yaml"
        parsed = NeugraspTrialNode._load_trial_config(config)
        self.assertEqual(parsed.grasp_to_tool.translation, (0.0, 0.0, 0.05))
        self.assertEqual(parsed.grasp_to_tool.rotation, (-1.0, 0.0, 0.0, 0.0))
        self.assertEqual(parsed.timing.scan_view_settle_s, 2.0)
        self.assertEqual(
            parsed.timing.after_phase_s,
            {
                "init_home": 3.0,
                "scan": 3.0,
                "predict_artifact": 3.0,
                "select_preflight": 3.0,
                "pregrasp": 3.0,
                "grasp": 3.0,
                "close": 3.0,
                "lift": 3.0,
            },
        )

    def test_wlan_surface_filter_keeps_the_configured_inclusive_z_band(self):
        visualizer = object.__new__(ArtifactVisualizationPublisher)
        visualizer._tsdf_z_index_range = (5, 7)
        surface = np.asarray(((1, 2, 4), (3, 4, 5), (5, 6, 6), (7, 8, 7), (9, 10, 8)))
        bundle = SimpleNamespace(surface_indices=surface)
        sampled = visualizer._surface_indices(bundle)
        np.testing.assert_array_equal(sampled, surface[1:4])

    def test_neugrasp_wireframe_frames_stay_distinct_from_tool0_motion_frames(self):
        """A grasp-frame wireframe must not inherit the tool0 TCP offset."""
        node = object.__new__(NeugraspTrialNode)
        node._config = SimpleNamespace(
            grasp_to_tool=RigidTransform(
                translation=(0.0, 0.0, 0.05), rotation=(-1.0, 0.0, 0.0, 0.0)
            ),
            primitive=SimpleNamespace(
                pregrasp_distance_m=0.05,
                lift_distance_m=0.10,
                top_down_max_angle_rad=1.0471975512,
            ),
        )
        candidate = SimpleNamespace(
            position_m=(0.10, 0.20, 0.30), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)
        )
        targets = node._candidate_targets(
            candidate,
            RigidTransform(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)),
        )
        self.assertEqual(targets.grasp_frame.translation, candidate.position_m)
        self.assertNotEqual(targets.grasp_frame.translation, targets.grasp_tool.translation)

    def test_scan_axis_marker_has_rep103_rgb_axes(self):
        node = object.__new__(NeugraspScanNode)
        node._frames = SimpleNamespace(workspace="neugrasp_workspace")
        node._marker = NeugraspScanNode._marker.__get__(node)
        node._point = NeugraspScanNode._point
        node._color = NeugraspScanNode._color
        marker = node._view_axes_marker(
            0,
            RigidTransform(translation=(1.0, 2.0, 3.0), rotation=(0.0, 0.0, 0.0, 1.0)),
            Time(),
        )
        self.assertEqual(len(marker.points), 6)
        self.assertEqual(len(marker.colors), 6)
        self.assertEqual((marker.colors[0].r, marker.colors[0].g, marker.colors[0].b), (1.0, 0.0, 0.0))
        self.assertEqual((marker.colors[2].r, marker.colors[2].g, marker.colors[2].b), (0.0, 1.0, 0.0))
        self.assertEqual((marker.colors[4].r, marker.colors[4].g, marker.colors[4].b), (0.0, 0.35, 1.0))


if __name__ == "__main__":
    unittest.main()
