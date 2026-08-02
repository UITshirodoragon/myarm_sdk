"""Camera plugin adapters."""

from .fake_camera import FakeCameraAdapter
from .opencv_camera import OpenCVCameraAdapter

__all__ = ["FakeCameraAdapter", "OpenCVCameraAdapter"]
