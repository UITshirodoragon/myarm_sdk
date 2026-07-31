"""Camera capability placeholder for future ROS camera workers."""

from typing import Any, Mapping

from myarm_sdk.core import CameraFrame, load_sdk_yaml
from myarm_sdk.core.validation import require_enabled
from myarm_sdk.plugin_adapter.camera import OpenCVCameraAdapter
from myarm_sdk.port_interface import CameraInterface


class CameraService:
    """Own one configured camera instance through the CameraInterface contract."""

    def __init__(
        self,
        camera: CameraInterface,
        instance_id: str,
        optical_frame: str,
    ) -> None:
        self._camera = camera
        self.instance_id = instance_id
        self.optical_frame = optical_frame

    @classmethod
    def from_config(cls, instance_config: Mapping[str, Any]) -> "CameraService":
        """Create the currently supported OpenCV adapter from one camera instance."""
        require_enabled(instance_config, "camera instance")
        if instance_config.get("plugin_adapter") != "opencv":
            raise ValueError("Only the opencv camera plugin adapter is available")

        adapter_config = load_sdk_yaml(str(instance_config["plugin_config"]))
        device = adapter_config["device"]
        capture = adapter_config["capture"]
        frames = adapter_config["frames"]
        camera = OpenCVCameraAdapter(
            device_index=int(device["fallback_index"]),
            device_path=device.get("device_path"),
            encoding=str(capture["encoding"]),
        )
        return cls(
            camera=camera,
            instance_id=str(adapter_config["instance_id"]),
            optical_frame=str(frames["optical_frame"]),
        )

    def capture_once(self) -> CameraFrame:
        return self._camera.capture()

    def close(self) -> None:
        self._camera.close()
