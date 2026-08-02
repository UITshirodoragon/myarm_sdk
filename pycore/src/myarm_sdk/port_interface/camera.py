"""Single-camera contract."""

from typing import Any, Mapping, Protocol

from myarm_sdk.core import CameraFrame


class CameraInterface(Protocol):
    """Capture frames from one physical camera instance."""

    def open(self) -> None:
        ...

    def capture(self) -> CameraFrame:
        ...

    def close(self) -> None:
        ...

    def status(self) -> Mapping[str, Any]:
        ...
