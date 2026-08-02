"""Convert SDK camera frames to ROS Image messages without cv_bridge."""

from typing import Any

from myarm_sdk.core import CameraFrame
from sensor_msgs.msg import Image


def image_message(frame: CameraFrame, stamp: Any) -> Image:
    """Create a contiguous bgr8 Image with the supplied ROS stamp."""
    image = frame.data
    if frame.encoding != "bgr8":
        raise ValueError("only bgr8 camera frames are supported")
    if getattr(image, "dtype", None) is None or str(image.dtype) != "uint8":
        raise TypeError("camera frame data must be a uint8 image")
    if len(image.shape) != 3 or image.shape[2] != 3:
        raise ValueError("bgr8 camera frame must have shape height x width x 3")
    height, width = image.shape[:2]
    if int(width) != frame.width or int(height) != frame.height:
        raise ValueError("camera frame dimensions do not match image payload")
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame.optical_frame
    message.height = int(height)
    message.width = int(width)
    message.encoding = "bgr8"
    message.is_bigendian = False
    message.step = int(width) * 3
    message.data = image.tobytes(order="C")
    return message
