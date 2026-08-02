"""OpenCV/V4L2 implementation of the camera adapter contract."""

from __future__ import annotations

import time
from typing import Any, Mapping, Optional, Union

from myarm_sdk.core import CameraFrame


class OpenCVCameraAdapter:
    """Capture one V4L2 camera after explicit mode negotiation."""

    def __init__(
        self,
        device_path: Optional[str] = None,
        device_index: Optional[int] = None,
        allow_fallback_index: bool = False,
        width: int = 1280,
        height: int = 720,
        fps: float = 30.0,
        pixel_format: str = "MJPG",
        buffer_size: int = 1,
        encoding: str = "bgr8",
        cv2_module: Optional[Any] = None,
    ) -> None:
        if cv2_module is None:
            try:
                import cv2
            except ImportError as error:
                raise RuntimeError(
                    "Install camera support with pip install myarm-sdk[camera]."
                ) from error
            cv2_module = cv2
        self._cv2: Any = cv2_module
        self._encoding = str(encoding)
        self._device_path = device_path
        self._device_index = device_index
        self._allow_fallback_index = bool(allow_fallback_index)
        self._width = int(width)
        self._height = int(height)
        self._fps = float(fps)
        self._pixel_format = str(pixel_format).upper()
        self._buffer_size = int(buffer_size)
        self._capture: Optional[Any] = None
        self._sequence = 0
        self._actual: Mapping[str, Any] = {}

    def open(self) -> None:
        """Open V4L2 lazily, negotiate the requested mode, then verify it."""
        if self._capture is not None and self._capture.isOpened():
            return
        source = self._source()
        capture = self._cv2.VideoCapture(source, self._cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Unable to open V4L2 camera device {source}")
        try:
            if self._buffer_size > 0:
                capture.set(self._cv2.CAP_PROP_BUFFERSIZE, self._buffer_size)
            capture.set(
                self._cv2.CAP_PROP_FOURCC,
                self._cv2.VideoWriter_fourcc(*self._pixel_format),
            )
            capture.set(self._cv2.CAP_PROP_FRAME_WIDTH, self._width)
            capture.set(self._cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            capture.set(self._cv2.CAP_PROP_FPS, self._fps)
            actual = {
                "actual_width": round(capture.get(self._cv2.CAP_PROP_FRAME_WIDTH)),
                "actual_height": round(capture.get(self._cv2.CAP_PROP_FRAME_HEIGHT)),
                "actual_fps": float(capture.get(self._cv2.CAP_PROP_FPS)),
                "actual_pixel_format": self._fourcc(
                    round(capture.get(self._cv2.CAP_PROP_FOURCC))
                ),
            }
            self._validate_actual(actual)
        except Exception:
            capture.release()
            raise
        self._capture = capture
        self._actual = actual

    def capture(self) -> CameraFrame:
        if self._capture is None or not self._capture.isOpened():
            raise RuntimeError("Camera is not open")
        ok, image = self._capture.read()
        if not ok or image is None:
            raise RuntimeError("Unable to capture a camera frame")
        height, width = image.shape[:2]
        frame = CameraFrame(
            data=image,
            timestamp_s=time.time(),
            encoding=self._encoding,
            sequence=self._sequence,
            width=int(width),
            height=int(height),
        )
        self._sequence += 1
        return frame

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        self._actual = {}

    def status(self) -> Mapping[str, Any]:
        opened = self._capture is not None and self._capture.isOpened()
        return {
            "opened": opened,
            "actual_width": self._actual.get("actual_width"),
            "actual_height": self._actual.get("actual_height"),
            "actual_fps": self._actual.get("actual_fps"),
            "actual_pixel_format": self._actual.get("actual_pixel_format"),
        }

    def _source(self) -> Union[int, str]:
        if self._device_path:
            return self._device_path
        if self._allow_fallback_index and self._device_index is not None:
            return self._device_index
        raise RuntimeError(
            "Camera requires device_path; fallback device indices are disabled"
        )

    def _validate_actual(self, actual: Mapping[str, Any]) -> None:
        if actual["actual_width"] != self._width or actual["actual_height"] != self._height:
            raise RuntimeError(
                "Camera resolution mismatch: requested {}x{}, got {}x{}".format(
                    self._width,
                    self._height,
                    actual["actual_width"],
                    actual["actual_height"],
                )
            )
        if abs(float(actual["actual_fps"]) - self._fps) > 0.5:
            raise RuntimeError(
                "Camera FPS mismatch: requested {}, got {}".format(
                    self._fps, actual["actual_fps"]
                )
            )
        if actual["actual_pixel_format"] != self._pixel_format:
            raise RuntimeError(
                "Camera pixel format mismatch: requested {}, got {}".format(
                    self._pixel_format, actual["actual_pixel_format"]
                )
            )

    @staticmethod
    def _fourcc(value: int) -> str:
        return "".join(chr((value >> (8 * index)) & 0xFF) for index in range(4))
