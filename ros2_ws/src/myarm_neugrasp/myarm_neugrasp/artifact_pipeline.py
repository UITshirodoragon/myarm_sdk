"""Pure NeuGrasp tensor-artifact loading and post-processing helpers.

The artifacts written by the legacy NeuGrasp runtime are model-local volumes.
They deliberately contain no trustworthy ROS frame history.  This module keeps
that boundary explicit: it reads only the four NPY tensors, turns lattice
indices into *voxel centres* in ``neugrasp_volume``, and does not import ROS.

Both the visualization replay node and the fake one-trial coordinator use this
module so their cloud, grasp markers, and motion targets have one identical
voxel/frame contract.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import math
from pathlib import Path
from typing import Any, Mapping, Sequence, Tuple

import numpy as np
from scipy import ndimage


# The simple gripper wireframe is expressed in the predicted grasp frame.  Its
# local +Z direction is the finger direction; callers transform it with the
# candidate rotation before publishing it in their chosen frame.
GRIPPER_EDGES: Tuple[Tuple[int, int], ...] = ((0, 2), (0, 1), (2, 3), (4, 5))


@dataclass(frozen=True)
class ArtifactSettings:
    """Parameters shared by artifact replay and fake trial selection.

    Defaults intentionally match the existing ROS replay configuration for the
    archived run artifacts.  ``voxel_size_m`` is derived rather than supplied
    separately, which prevents an inconsistent resolution/metric mapping.
    """

    volume_size_m: float = 0.30
    volume_resolution: int = 40
    tsdf_surface_threshold_low: float = -0.85
    tsdf_surface_threshold_high: float = 0.0
    gaussian_filter_sigma: float = 1.0
    min_width_vox: float = 1.33
    max_width_vox: float = 9.33
    grasp_score_threshold: float = 0.70
    max_filter_size: int = 4
    gripper_max_width_m: float = 0.08

    def __post_init__(self) -> None:
        volume_size_m = _positive_float(self.volume_size_m, "volume_size_m")
        volume_resolution = _positive_int(self.volume_resolution, "volume_resolution")
        surface_low = _finite_float(
            self.tsdf_surface_threshold_low, "tsdf_surface_threshold_low"
        )
        surface_high = _finite_float(
            self.tsdf_surface_threshold_high, "tsdf_surface_threshold_high"
        )
        if surface_low >= surface_high:
            raise ValueError(
                "tsdf_surface_threshold_low must be less than tsdf_surface_threshold_high"
            )
        gaussian_sigma = _nonnegative_float(
            self.gaussian_filter_sigma, "gaussian_filter_sigma"
        )
        min_width_vox = _nonnegative_float(self.min_width_vox, "min_width_vox")
        max_width_vox = _positive_float(self.max_width_vox, "max_width_vox")
        if min_width_vox > max_width_vox:
            raise ValueError("min_width_vox must not exceed max_width_vox")
        grasp_score_threshold = _finite_float(
            self.grasp_score_threshold, "grasp_score_threshold"
        )
        max_filter_size = _positive_int(self.max_filter_size, "max_filter_size")
        gripper_max_width_m = _positive_float(
            self.gripper_max_width_m, "gripper_max_width_m"
        )

        object.__setattr__(self, "volume_size_m", volume_size_m)
        object.__setattr__(self, "volume_resolution", volume_resolution)
        object.__setattr__(self, "tsdf_surface_threshold_low", surface_low)
        object.__setattr__(self, "tsdf_surface_threshold_high", surface_high)
        object.__setattr__(self, "gaussian_filter_sigma", gaussian_sigma)
        object.__setattr__(self, "min_width_vox", min_width_vox)
        object.__setattr__(self, "max_width_vox", max_width_vox)
        object.__setattr__(self, "grasp_score_threshold", grasp_score_threshold)
        object.__setattr__(self, "max_filter_size", max_filter_size)
        object.__setattr__(self, "gripper_max_width_m", gripper_max_width_m)

    @property
    def voxel_size_m(self) -> float:
        """Metric width of one model voxel."""
        return self.volume_size_m / self.volume_resolution

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "ArtifactSettings":
        """Build settings from a ROS/YAML-like mapping with strict key checks."""
        valid_names = {item.name for item in fields(cls)}
        unexpected = sorted(set(values) - valid_names)
        if unexpected:
            raise KeyError("unknown NeuGrasp artifact settings: " + ", ".join(unexpected))
        return cls(**{name: values[name] for name in valid_names if name in values})


@dataclass(frozen=True)
class ArtifactCandidate:
    """One post-processed grasp prediction in the local NeuGrasp volume frame."""

    score: float
    index: Tuple[int, int, int]
    position_m: Tuple[float, float, float]
    rotation_xyzw: Tuple[float, float, float, float]
    rotation_matrix: np.ndarray
    width_m: float


@dataclass(frozen=True)
class ArtifactBundle:
    """The model-local tensors and candidates reconstructed from one run."""

    voxel_size_m: float
    surface_indices: np.ndarray
    tsdf: np.ndarray
    candidates: Tuple[ArtifactCandidate, ...]


def load_artifacts(run_dir: Path, settings: ArtifactSettings) -> ArtifactBundle:
    """Load and post-process one ``run_dir/inference`` tensor set.

    Only ``tsdf_vol.npy``, ``qual_vol_raw.npy``, ``rot_vol_raw.npy`` and
    ``width_vol_raw.npy`` are read.  Historical PLY/JSON/calibration artifacts
    are intentionally excluded because they encode a previous scene setup.
    """
    run_path = Path(run_dir).expanduser()
    if not run_path.is_dir():
        raise ValueError("run_dir does not exist: {}".format(run_path))

    tensors = _load_raw_tensors(run_path, settings.volume_resolution)
    tsdf, raw_quality, rotation, width = tensors
    processed_quality = postprocess_quality(tsdf, raw_quality, width, settings)
    surface_indices = np.argwhere(
        (tsdf > settings.tsdf_surface_threshold_low)
        & (tsdf < settings.tsdf_surface_threshold_high)
    )
    candidates = select_candidates(
        processed_quality, rotation, width, settings=settings
    )

    # A bundle is an immutable snapshot by convention.  This protects the raw
    # metric lattice from accidental mutation after replay and trial initialize.
    tsdf.setflags(write=False)
    surface_indices.setflags(write=False)
    return ArtifactBundle(
        voxel_size_m=settings.voxel_size_m,
        surface_indices=surface_indices,
        tsdf=tsdf,
        candidates=tuple(candidates),
    )


def postprocess_quality(
    tsdf: np.ndarray,
    quality: np.ndarray,
    width: np.ndarray,
    settings: ArtifactSettings,
) -> np.ndarray:
    """Apply the legacy NeuGrasp Gaussian/TSDF/width quality filtering.

    This is intentionally equivalent to the old runtime's
    ``process_volumes`` behaviour.  Local-maxima extraction is kept separate
    in :func:`select_candidates` so callers can inspect the filtered field in
    tests without a ROS dependency.
    """
    processed = np.asarray(quality, dtype=np.float64).copy()
    if settings.gaussian_filter_sigma > 0.0:
        processed = ndimage.gaussian_filter(
            processed, sigma=settings.gaussian_filter_sigma, mode="nearest"
        )

    outside_voxels = tsdf > settings.tsdf_surface_threshold_high
    inside_voxels = np.logical_and(
        -1.0 < tsdf, tsdf < settings.tsdf_surface_threshold_low
    )
    valid_voxels = ndimage.binary_dilation(
        outside_voxels, iterations=2, mask=np.logical_not(inside_voxels)
    )
    processed[np.logical_not(valid_voxels)] = 0.0
    invalid_width = np.logical_or(
        width < settings.min_width_vox, width > settings.max_width_vox
    )
    processed[invalid_width] = 0.0
    return processed


def select_candidates(
    quality: np.ndarray,
    rotation: np.ndarray,
    width: np.ndarray,
    *,
    settings: ArtifactSettings,
) -> Tuple[ArtifactCandidate, ...]:
    """Extract sorted local maxima using the historical NeuGrasp policy.

    Every returned position is a voxel centre, i.e. ``(index + 0.5) * voxel``.
    The network rotation convention is normalized XYZW quaternion order.
    """
    filtered = np.asarray(quality, dtype=np.float64).copy()
    filtered[filtered < settings.grasp_score_threshold] = 0.0
    local_maxima = ndimage.maximum_filter(filtered, size=settings.max_filter_size)
    indices = np.argwhere(np.logical_and(filtered == local_maxima, filtered > 0.0))

    candidates = []
    for index_array in indices:
        index = tuple(int(value) for value in index_array)
        i, j, k = index
        width_m = float(width[i, j, k]) * settings.voxel_size_m
        if width_m > settings.gripper_max_width_m:
            continue
        try:
            rotation_xyzw, rotation_matrix = _normalise_quaternion_xyzw(
                rotation[:, i, j, k]
            )
        except ValueError:
            # A malformed learned rotation is not a motion candidate.  Keep
            # other local maxima available rather than failing visualization.
            continue
        rotation_matrix.setflags(write=False)
        candidates.append(
            ArtifactCandidate(
                score=float(filtered[i, j, k]),
                index=index,
                position_m=voxel_center_position(index, settings.voxel_size_m),
                rotation_xyzw=rotation_xyzw,
                rotation_matrix=rotation_matrix,
                width_m=width_m,
            )
        )

    # np.argwhere is lexicographically ordered, but state the tie-break
    # explicitly so ranking remains deterministic across NumPy versions.
    candidates.sort(key=lambda item: (-item.score, item.index))
    return tuple(candidates)


def voxel_center_position(
    index: Sequence[int], voxel_size_m: float
) -> Tuple[float, float, float]:
    """Return an artifact lattice index as a metric voxel centre."""
    if len(index) != 3:
        raise ValueError("a NeuGrasp voxel index must contain exactly three values")
    voxel_size = _positive_float(voxel_size_m, "voxel_size_m")
    return tuple((float(value) + 0.5) * voxel_size for value in index)  # type: ignore[return-value]


def gripper_wireframe_points(
    width_m: float,
    finger_length_m: float = 0.05,
    palm_depth_m: float = 0.025,
) -> np.ndarray:
    """Return a six-vertex grasp-frame wireframe for an opening width.

    The very small lower clamp makes a closed candidate visible without
    inventing a wider physical gripper state.
    """
    width = _nonnegative_float(width_m, "width_m")
    finger_length = _positive_float(finger_length_m, "finger_length_m")
    palm_depth = _positive_float(palm_depth_m, "palm_depth_m")
    half_width = max(width, 0.002) / 2.0
    return np.asarray(
        [
            [0.0, -half_width, 0.0],
            [0.0, -half_width, finger_length],
            [0.0, half_width, 0.0],
            [0.0, half_width, finger_length],
            [0.0, 0.0, -palm_depth],
            [0.0, 0.0, 0.0],
        ],
        dtype=np.float64,
    )


def quality_color(
    score: float,
    *,
    minimum_score: float = 0.70,
    maximum_score: float = 1.0,
) -> Tuple[float, float, float, float]:
    """Map a grasp quality to a high-contrast, RViz-friendly RGBA colour.

    The mapping is pure and intentionally does not encode a selected grasp;
    selected/pregrasp/lift colours are owned by the trial visualizer.
    """
    score_value = _finite_float(score, "score")
    low = _finite_float(minimum_score, "minimum_score")
    high = _finite_float(maximum_score, "maximum_score")
    if high <= low:
        raise ValueError("maximum_score must be greater than minimum_score")
    fraction = min(1.0, max(0.0, (score_value - low) / (high - low)))
    # Compact approximation of viridis: dark purple -> teal -> yellow.  It is
    # readable on RViz's dark background and clearly distinguishes rankings.
    stops = (
        (0.267, 0.005, 0.329),
        (0.230, 0.322, 0.545),
        (0.128, 0.567, 0.551),
        (0.369, 0.789, 0.383),
        (0.993, 0.906, 0.144),
    )
    scaled = fraction * (len(stops) - 1)
    lower = min(int(math.floor(scaled)), len(stops) - 1)
    upper = min(lower + 1, len(stops) - 1)
    blend = scaled - lower
    rgb = tuple(
        (1.0 - blend) * stops[lower][axis] + blend * stops[upper][axis]
        for axis in range(3)
    )
    return rgb[0], rgb[1], rgb[2], 1.0


def _load_raw_tensors(
    run_dir: Path, volume_resolution: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    inference = run_dir / "inference"
    paths = {
        "tsdf": inference / "tsdf_vol.npy",
        "quality": inference / "qual_vol_raw.npy",
        "rotation": inference / "rot_vol_raw.npy",
        "width": inference / "width_vol_raw.npy",
    }
    missing = [str(path) for path in paths.values() if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "required NeuGrasp tensors are missing:\n" + "\n".join(missing)
        )

    expected = (volume_resolution,) * 3
    tsdf = _scalar_volume(np.load(paths["tsdf"], allow_pickle=False), "tsdf_vol.npy")
    quality = _scalar_volume(
        np.load(paths["quality"], allow_pickle=False), "qual_vol_raw.npy"
    )
    width = _scalar_volume(
        np.load(paths["width"], allow_pickle=False), "width_vol_raw.npy"
    )
    rotation = _rotation_volume(
        np.load(paths["rotation"], allow_pickle=False), "rot_vol_raw.npy"
    )
    for name, volume in (
        ("tsdf", tsdf),
        ("quality", quality),
        ("rotation", rotation),
        ("width", width),
    ):
        if not np.all(np.isfinite(volume)):
            raise ValueError("{} tensor contains non-finite values".format(name))
    if tsdf.shape != expected or quality.shape != expected or width.shape != expected:
        raise ValueError("scalar tensor shape must be {}".format(expected))
    if rotation.shape != (4,) + expected:
        raise ValueError(
            "rot_vol_raw.npy shape must be (1, 4, {}, {}, {})".format(
                volume_resolution, volume_resolution, volume_resolution
            )
        )
    return tsdf, quality, rotation, width


def _scalar_volume(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.ndim == 5 and result.shape[:2] == (1, 1):
        result = result[0, 0]
    if result.ndim != 3:
        raise ValueError("{} must have shape (1, 1, R, R, R)".format(name))
    return result


def _rotation_volume(value: np.ndarray, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.ndim == 5 and result.shape[0] == 1 and result.shape[1] == 4:
        result = result[0]
    if result.ndim != 4 or result.shape[0] != 4:
        raise ValueError("{} must have shape (1, 4, R, R, R)".format(name))
    return result


def _normalise_quaternion_xyzw(
    quaternion: Sequence[float],
) -> Tuple[Tuple[float, float, float, float], np.ndarray]:
    if len(quaternion) != 4:
        raise ValueError("grasp quaternion must have four XYZW values")
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("grasp quaternion must be finite and non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return (x, y, z, w), matrix


def _finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise TypeError("{} must be numeric".format(name)) from error
    if not math.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def _positive_float(value: Any, name: str) -> float:
    result = _finite_float(value, name)
    if result <= 0.0:
        raise ValueError("{} must be positive".format(name))
    return result


def _nonnegative_float(value: Any, name: str) -> float:
    result = _finite_float(value, name)
    if result < 0.0:
        raise ValueError("{} must be non-negative".format(name))
    return result


def _positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool):
        raise TypeError("{} must be an integer".format(name))
    try:
        result = int(value)
    except (TypeError, ValueError) as error:
        raise TypeError("{} must be an integer".format(name)) from error
    if result <= 0 or result != value:
        raise ValueError("{} must be a positive integer".format(name))
    return result
