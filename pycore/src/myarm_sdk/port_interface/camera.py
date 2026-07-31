"""Single-camera contract."""

from typing import Protocol

from myarm_sdk.core import CameraFrame


class CameraInterface(Protocol):
    """Capture frames from one physical camera instance."""

    def capture(self) -> CameraFrame:
        ...

    def close(self) -> None:
        ...
