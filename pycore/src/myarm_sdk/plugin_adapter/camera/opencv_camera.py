"""Optional OpenCV implementation of :class:`CameraInterface`."""

from __future__ import annotations

import time
from typing import Any, Optional, Union

from myarm_sdk.core import CameraFrame


class OpenCVCameraAdapter:
    """Capture frames from one OpenCV video device."""

    def __init__(
        self,
        device_index: int = 0,
        device_path: Optional[str] = None,
        encoding: str = "bgr8",
    ) -> None:
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError(
                "Install camera support with `pip install myarm-sdk[camera]`."
            ) from error
        self._cv2: Any = cv2
        self._encoding = encoding
        source: Union[int, str] = device_path if device_path else device_index
        self._capture: Any = cv2.VideoCapture(source)
        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError(f"Unable to open camera device {source}")

    def capture(self) -> CameraFrame:
        ok, image = self._capture.read()
        if not ok:
            raise RuntimeError("Unable to capture a camera frame")
        return CameraFrame(data=image, timestamp_s=time.time(), encoding=self._encoding)

    def close(self) -> None:
        self._capture.release()
