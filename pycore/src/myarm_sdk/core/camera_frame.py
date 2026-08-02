"""Camera frame value type without image-library coupling."""

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CameraFrame:
    """An image payload owned by a camera adapter."""

    data: Any
    timestamp_s: float
    encoding: str = "bgr8"
    sequence: int = 0
    width: int = 0
    height: int = 0
    optical_frame: str = ""
