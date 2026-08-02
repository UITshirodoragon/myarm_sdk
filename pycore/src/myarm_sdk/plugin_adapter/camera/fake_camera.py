"""Deterministic in-memory camera implementation for tests and fake launches."""

from __future__ import annotations

import time
from typing import Any, Mapping

import numpy as np

from myarm_sdk.core import CameraFrame


class FakeCameraAdapter:
    """Return synthetic BGR frames without opening a hardware device."""

    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        fps: float = 30.0,
        pixel_format: str = "MJPG",
        encoding: str = "bgr8",
    ) -> None:
        self._width = int(width)
        self._height = int(height)
        self._fps = float(fps)
        self._pixel_format = str(pixel_format)
        self._encoding = str(encoding)
        self._opened = False
        self._sequence = 0

    def open(self) -> None:
        self._opened = True

    def capture(self) -> CameraFrame:
        if not self._opened:
            raise RuntimeError("Fake camera is not open")
        image = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        frame = CameraFrame(
            data=image,
            timestamp_s=time.time(),
            encoding=self._encoding,
            sequence=self._sequence,
            width=self._width,
            height=self._height,
        )
        self._sequence += 1
        return frame

    def close(self) -> None:
        self._opened = False

    def status(self) -> Mapping[str, Any]:
        return {
            "opened": self._opened,
            "actual_width": self._width if self._opened else None,
            "actual_height": self._height if self._opened else None,
            "actual_fps": self._fps if self._opened else None,
            "actual_pixel_format": self._pixel_format if self._opened else None,
        }
