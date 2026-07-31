"""Optional OpenCV-backed implementation of :class:`myarm_sdk.ports.Camera`."""

import time
from typing import Any

from myarm_sdk.model import CameraFrame


class OpenCVCamera:
    def __init__(self, device_index: int = 0) -> None:
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError("Install camera support with `pip install myarm-sdk[camera]`") from error
        self._cv2: Any = cv2
        self._capture: Any = cv2.VideoCapture(device_index)
        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError("Unable to open camera device {0}".format(device_index))

    def capture(self) -> CameraFrame:
        ok, image = self._capture.read()
        if not ok:
            raise RuntimeError("Unable to capture a camera frame")
        return CameraFrame(data=image, timestamp_s=time.time())

    def close(self) -> None:
        self._capture.release()
