"""ROS-independent contract tests for the NeuGrasp NPY artifact pipeline."""

import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

from myarm_neugrasp.artifact_pipeline import ArtifactSettings, load_artifacts


class ArtifactPipelineTest(unittest.TestCase):
    def test_missing_tensor_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            inference = Path(temporary_directory) / "run" / "inference"
            inference.mkdir(parents=True)
            np.save(inference / "tsdf_vol.npy", np.zeros((1, 1, 4, 4, 4)))
            with self.assertRaisesRegex(FileNotFoundError, "qual_vol_raw.npy"):
                load_artifacts(
                    inference.parent,
                    ArtifactSettings(volume_size_m=0.04, volume_resolution=4),
                )

    def test_no_valid_local_maximum_returns_no_motion_candidate(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            inference = Path(temporary_directory) / "run" / "inference"
            inference.mkdir(parents=True)
            shape = (1, 1, 4, 4, 4)
            np.save(inference / "tsdf_vol.npy", np.ones(shape, dtype=np.float32))
            np.save(inference / "qual_vol_raw.npy", np.zeros(shape, dtype=np.float32))
            np.save(inference / "width_vol_raw.npy", np.ones(shape, dtype=np.float32))
            rotation = np.zeros((1, 4, 4, 4, 4), dtype=np.float32)
            rotation[:, 3, :, :, :] = 1.0
            np.save(inference / "rot_vol_raw.npy", rotation)
            bundle = load_artifacts(
                inference.parent,
                ArtifactSettings(volume_size_m=0.04, volume_resolution=4),
            )
        self.assertEqual(bundle.candidates, ())

    def test_candidate_uses_voxel_center_and_normalized_xyzw_rotation(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            inference = Path(temporary_directory) / "run" / "inference"
            inference.mkdir(parents=True)
            shape = (1, 1, 4, 4, 4)
            tsdf = np.ones(shape, dtype=np.float32)
            # This is a visualization surface point, separate from the grasp
            # candidate below, so the two contracts are independently tested.
            tsdf[0, 0, 0, 0, 0] = -0.5
            quality = np.zeros(shape, dtype=np.float32)
            quality[0, 0, 1, 2, 3] = 0.9
            width = np.full(shape, 2.0, dtype=np.float32)
            rotation = np.zeros((1, 4, 4, 4, 4), dtype=np.float32)
            rotation[:, 3, :, :, :] = 2.0  # identity XYZW after normalization
            np.save(inference / "tsdf_vol.npy", tsdf)
            np.save(inference / "qual_vol_raw.npy", quality)
            np.save(inference / "rot_vol_raw.npy", rotation)
            np.save(inference / "width_vol_raw.npy", width)

            bundle = load_artifacts(
                inference.parent,
                ArtifactSettings(
                    volume_size_m=0.04,
                    volume_resolution=4,
                    gaussian_filter_sigma=0.0,
                    min_width_vox=1.0,
                    max_width_vox=3.0,
                    grasp_score_threshold=0.8,
                    max_filter_size=1,
                ),
            )

        self.assertEqual(len(bundle.surface_indices), 1)
        self.assertEqual(len(bundle.candidates), 1)
        candidate = bundle.candidates[0]
        self.assertEqual(candidate.index, (1, 2, 3))
        self.assertEqual(candidate.position_m, (0.015, 0.025, 0.035))
        self.assertAlmostEqual(candidate.width_m, 0.02)
        self.assertEqual(candidate.rotation_xyzw, (0.0, 0.0, 0.0, 1.0))
        np.testing.assert_allclose(candidate.rotation_matrix, np.eye(3))

    def test_archived_run_contract_when_sample_is_available(self):
        """Check the agreed archived-run regression contract when opted in.

        The run archive is intentionally outside the package, so CI and a
        clean checkout skip this test.  Developers can opt in with
        ``NEUGRASP_SAMPLE_RUN_DIR=/path/to/neugrasp_real_20260603_194238``.
        """
        run_dir = os.environ.get("NEUGRASP_SAMPLE_RUN_DIR")
        if not run_dir:
            self.skipTest("NEUGRASP_SAMPLE_RUN_DIR is not set")
        bundle = load_artifacts(Path(run_dir), ArtifactSettings())
        self.assertEqual(len(bundle.surface_indices), 7735)
        selected = next(
            candidate for candidate in bundle.candidates if candidate.index == (6, 25, 20)
        )
        self.assertAlmostEqual(selected.score, 0.7974, places=4)
        self.assertAlmostEqual(selected.width_m, 0.0382264, places=6)
        self.assertEqual(selected.position_m, (0.04875, 0.19125, 0.15375))


if __name__ == "__main__":
    unittest.main()
