from typing import Protocol

from myarm_sdk.model import CameraFrame


class Camera(Protocol):
    """Capture a single image from a camera source."""

    def capture(self) -> CameraFrame:
        ...

    def close(self) -> None:
        ...
