"""Convert SDK intrinsic calibration to ROS CameraInfo."""

from typing import Any

from myarm_sdk.core import CameraCalibration
from sensor_msgs.msg import CameraInfo


def camera_info_message(
    calibration: CameraCalibration, stamp: Any, optical_frame: str
) -> CameraInfo:
    """Create an unrectified plumb_bob CameraInfo for the raw C925e stream."""
    if calibration.distortion_model != "opencv_radtan5":
        raise ValueError("only opencv_radtan5 calibration is supported")
    if len(calibration.k) != 9:
        raise ValueError("camera calibration K must contain nine values")
    message = CameraInfo()
    message.header.stamp = stamp
    message.header.frame_id = optical_frame
    message.width = calibration.width
    message.height = calibration.height
    message.distortion_model = "plumb_bob"
    message.d = list(calibration.d)
    message.k = list(calibration.k)
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [
        calibration.k[0], 0.0, calibration.k[2], 0.0,
        0.0, calibration.k[4], calibration.k[5], 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    message.binning_x = 0
    message.binning_y = 0
    return message
