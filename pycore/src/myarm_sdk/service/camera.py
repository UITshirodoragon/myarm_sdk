"""Lifecycle and calibration policy for one configured camera instance."""

from __future__ import annotations

import time
from typing import Any, Mapping, Optional, Sequence, Tuple

from myarm_sdk.core import (
    CameraCalibration,
    CameraFrame,
    CameraStatus,
    load_sdk_json,
    load_sdk_yaml,
)
from myarm_sdk.core.validation import require_enabled
from myarm_sdk.plugin_adapter.camera import FakeCameraAdapter, OpenCVCameraAdapter
from myarm_sdk.port_interface import CameraInterface


class CameraServiceError(RuntimeError):
    """A camera lifecycle, configuration, or calibration policy failed."""


class CameraService:
    """Own one camera port and enforce its configured capture contract."""

    def __init__(
        self,
        camera: CameraInterface,
        instance_id: str,
        optical_frame: str,
        calibration: Optional[CameraCalibration],
        requested_width: int,
        requested_height: int,
        requested_fps: float,
        requested_pixel_format: str,
        require_exact_resolution: bool = True,
    ) -> None:
        self._camera = camera
        self.instance_id = str(instance_id)
        self.optical_frame = str(optical_frame)
        self._calibration = calibration
        self._requested_width = int(requested_width)
        self._requested_height = int(requested_height)
        self._requested_fps = float(requested_fps)
        self._requested_pixel_format = str(requested_pixel_format).upper()
        self._require_exact_resolution = bool(require_exact_resolution)
        self._state = "closed"
        self._frame_count = 0
        self._capture_error_count = 0
        self._last_frame_timestamp_s: Optional[float] = None
        self._last_error: Optional[str] = None
        self._retry_after_monotonic_s: Optional[float] = None

    @classmethod
    def from_config(cls, instance_config: Mapping[str, Any]) -> CameraService:
        """Create one configured backend without opening the camera device."""
        require_enabled(instance_config, "camera instance")
        plugin_adapter = cls._required_string(
            instance_config.get("plugin_adapter"), "camera plugin_adapter"
        )
        if plugin_adapter not in ("opencv", "fake"):
            raise ValueError("camera plugin_adapter must be opencv or fake")
        adapter_config = load_sdk_yaml(
            cls._required_string(instance_config.get("plugin_config"), "plugin_config")
        )
        if adapter_config.get("plugin_adapter") != plugin_adapter:
            raise ValueError(
                "camera plugin_config plugin_adapter must match service plugin_adapter"
            )
        capture = cls._mapping(adapter_config.get("capture"), "camera capture")
        width = cls._positive_int(capture.get("width"), "camera capture.width")
        height = cls._positive_int(capture.get("height"), "camera capture.height")
        fps = cls._positive_float(capture.get("fps"), "camera capture.fps")
        pixel_format = cls._required_string(
            capture.get("pixel_format"), "camera capture.pixel_format"
        ).upper()
        encoding = cls._required_string(capture.get("encoding"), "camera capture.encoding")
        buffer_size = cls._positive_int(
            capture.get("buffer_size", 1), "camera capture.buffer_size"
        )
        calibration = cls._calibration_from_config(adapter_config)
        calibration_config = cls._mapping(
            adapter_config.get("intrinsic_calibration"), "camera intrinsic_calibration"
        )
        require_exact_resolution = cls._boolean(
            calibration_config.get("require_exact_resolution", True),
            "camera intrinsic_calibration.require_exact_resolution",
        )
        frames = cls._mapping(adapter_config.get("frames"), "camera frames")
        instance_id = cls._required_string(adapter_config.get("instance_id"), "camera instance_id")
        if plugin_adapter == "fake":
            camera: CameraInterface = FakeCameraAdapter(
                width=width,
                height=height,
                fps=fps,
                pixel_format=pixel_format,
                encoding=encoding,
            )
        else:
            device = cls._mapping(adapter_config.get("device"), "camera device")
            device_path = cls._required_string(
                device.get("device_path"), "camera device.device_path"
            )
            if device_path.startswith("REPLACE_"):
                raise CameraServiceError(
                    "camera device_path is a commissioning placeholder; configure "
                    "the exact /dev/v4l/by-id/*-video-index0 path first"
                )
            if cls._boolean(device.get("allow_fallback_index", False), "camera allow_fallback_index"):
                raise CameraServiceError(
                    "fallback camera indices are forbidden for production camera instances"
                )
            camera = OpenCVCameraAdapter(
                device_path=device_path,
                allow_fallback_index=False,
                width=width,
                height=height,
                fps=fps,
                pixel_format=pixel_format,
                buffer_size=buffer_size,
                encoding=encoding,
            )
        return cls(
            camera=camera,
            instance_id=instance_id,
            optical_frame=cls._required_string(frames.get("optical_frame"), "camera optical_frame"),
            calibration=calibration,
            requested_width=width,
            requested_height=height,
            requested_fps=fps,
            requested_pixel_format=pixel_format,
            require_exact_resolution=require_exact_resolution,
        )

    def open(self) -> None:
        """Open once and transition to backoff rather than leaking an adapter error."""
        if self._state == "streaming":
            return
        self._state = "opening"
        try:
            self._camera.open()
            self._validate_calibration_mode()
        except Exception as error:
            self._record_failure(error)
            raise CameraServiceError(str(error)) from error
        self._state = "streaming"
        self._last_error = None
        self._retry_after_monotonic_s = None

    def capture(self) -> CameraFrame:
        """Capture one frame while recording failures for reconnect policy."""
        if self._state != "streaming":
            raise CameraServiceError("camera is not streaming")
        try:
            frame = self._camera.capture()
        except Exception as error:
            self._record_failure(error)
            raise CameraServiceError(str(error)) from error
        if frame.width != self._requested_width or frame.height != self._requested_height:
            error = CameraServiceError(
                f"captured frame resolution mismatch: expected {self._requested_width}x{self._requested_height}, got {frame.width}x{frame.height}"
            )
            self._record_failure(error)
            raise error
        self._frame_count += 1
        self._last_frame_timestamp_s = frame.timestamp_s
        return CameraFrame(
            data=frame.data,
            timestamp_s=frame.timestamp_s,
            encoding=frame.encoding,
            sequence=frame.sequence,
            width=frame.width,
            height=frame.height,
            optical_frame=self.optical_frame,
        )

    def capture_once(self) -> CameraFrame:
        """Compatibility alias for the previous public capture API."""
        return self.capture()

    def calibration(self) -> CameraCalibration:
        if self._calibration is None:
            raise CameraServiceError("camera has no configured intrinsic calibration")
        return self._calibration

    def status(self) -> CameraStatus:
        adapter_status = self._camera.status()
        return CameraStatus(
            instance_id=self.instance_id,
            state=self._state,
            requested_width=self._requested_width,
            requested_height=self._requested_height,
            requested_fps=self._requested_fps,
            requested_pixel_format=self._requested_pixel_format,
            actual_width=adapter_status.get("actual_width"),
            actual_height=adapter_status.get("actual_height"),
            actual_fps=adapter_status.get("actual_fps"),
            actual_pixel_format=adapter_status.get("actual_pixel_format"),
            frame_count=self._frame_count,
            capture_error_count=self._capture_error_count,
            last_frame_timestamp_s=self._last_frame_timestamp_s,
            last_error=self._last_error,
            retry_after_monotonic_s=self._retry_after_monotonic_s,
        )

    def reconnect(self, now_monotonic_s: Optional[float] = None) -> bool:
        """Attempt one backoff-aware reconnect; return whether streaming resumed."""
        if self._state == "streaming":
            return True
        now = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        if self._retry_after_monotonic_s is not None and now < self._retry_after_monotonic_s:
            return False
        self.close()
        try:
            self.open()
        except CameraServiceError:
            return False
        return True

    def close(self) -> None:
        self._camera.close()
        self._state = "closed"

    def _record_failure(self, error: Exception) -> None:
        self._camera.close()
        self._capture_error_count += 1
        self._last_error = str(error)
        retry_delay_s = min(30.0, float(2 ** min(self._capture_error_count - 1, 5)))
        self._retry_after_monotonic_s = time.monotonic() + retry_delay_s
        self._state = "backoff"

    def _validate_calibration_mode(self) -> None:
        if self._calibration is None:
            return
        if self._require_exact_resolution and (
            self._calibration.width != self._requested_width
            or self._calibration.height != self._requested_height
        ):
            raise CameraServiceError(
                f"camera calibration resolution {self._calibration.width}x{self._calibration.height} does not match capture {self._requested_width}x{self._requested_height}"
            )

    @classmethod
    def _calibration_from_config(
        cls, adapter_config: Mapping[str, Any]
    ) -> Optional[CameraCalibration]:
        calibration_config = cls._mapping(
            adapter_config.get("intrinsic_calibration"), "camera intrinsic_calibration"
        )
        path = calibration_config.get("path")
        if path is None:
            if cls._boolean(calibration_config.get("allow_uncalibrated", False), "camera allow_uncalibrated"):
                return None
            raise ValueError("camera intrinsic_calibration.path is required")
        document = load_sdk_json(cls._required_string(path, "camera calibration path"))
        matrix = document.get("camera_matrix", document.get("K"))
        k = cls._matrix_values(matrix, "camera calibration camera_matrix", 3, 3)
        d = cls._vector_values(
            document.get("distortion_coefficients", document.get("dist_coeffs")),
            "camera calibration distortion_coefficients",
        )
        return CameraCalibration(
            calibration_id=cls._required_string(
                document.get("calibration_id"), "camera calibration calibration_id"
            ),
            source_sha256=cls._required_string(
                document.get("source_sha256"), "camera calibration source_sha256"
            ),
            width=cls._positive_int(document.get("width"), "camera calibration width"),
            height=cls._positive_int(document.get("height"), "camera calibration height"),
            distortion_model=cls._required_string(
                document.get("distortion_model"), "camera calibration distortion_model"
            ),
            k=k,
            d=d,
        )

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    @staticmethod
    def _required_string(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise TypeError(f"{name} must be a non-empty string")
        return value.strip()

    @staticmethod
    def _positive_int(value: Any, name: str) -> int:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be a positive integer")
        try:
            converted = int(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be a positive integer") from error
        if converted <= 0:
            raise ValueError(f"{name} must be positive")
        return converted

    @staticmethod
    def _positive_float(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be positive")
        try:
            converted = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be positive") from error
        if converted <= 0.0:
            raise ValueError(f"{name} must be positive")
        return converted

    @staticmethod
    def _boolean(value: Any, name: str) -> bool:
        if not isinstance(value, bool):
            raise TypeError(f"{name} must be boolean")
        return value

    @classmethod
    def _matrix_values(
        cls, value: Any, name: str, rows: int, columns: int
    ) -> Tuple[float, ...]:
        if not isinstance(value, list) or len(value) != rows:
            raise TypeError(f"{name} must contain {rows} rows")
        flattened = []
        for row in value:
            if not isinstance(row, list) or len(row) != columns:
                raise TypeError(f"{name} rows must contain {columns} values")
            flattened.extend(cls._numeric(item, name) for item in row)
        return tuple(flattened)

    @classmethod
    def _vector_values(cls, value: Any, name: str) -> Tuple[float, ...]:
        if not isinstance(value, list):
            raise TypeError(f"{name} must be a list")
        flattened: Sequence[Any] = value[0] if len(value) == 1 and isinstance(value[0], list) else value
        if not flattened:
            raise ValueError(f"{name} must not be empty")
        return tuple(cls._numeric(item, name) for item in flattened)

    @staticmethod
    def _numeric(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} values must be numeric")
        try:
            return float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} values must be numeric") from error
