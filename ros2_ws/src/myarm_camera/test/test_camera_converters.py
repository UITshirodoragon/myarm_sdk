"""ROS boundary tests for raw C925e image and CameraInfo conversion."""

import unittest

import numpy as np
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus
from myarm_camera.camera_diagnostics import diagnostic_message
from myarm_camera.ros_camera_info_converter import camera_info_message
from myarm_camera.ros_image_converter import image_message
from myarm_sdk.core import CameraCalibration, CameraFrame, CameraStatus


class CameraConverterTests(unittest.TestCase):
    def setUp(self):
        self.stamp = Time(sec=123, nanosec=456)
        self.frame = CameraFrame(
            data=np.zeros((2, 3, 3), dtype=np.uint8),
            timestamp_s=1.0,
            encoding="bgr8",
            sequence=7,
            width=3,
            height=2,
            optical_frame="wrist_camera_optical_frame",
        )
        self.calibration = CameraCalibration(
            calibration_id="test",
            source_sha256="sha256:test",
            width=3,
            height=2,
            distortion_model="opencv_radtan5",
            k=(10.0, 0.0, 1.0, 0.0, 11.0, 0.5, 0.0, 0.0, 1.0),
            d=(0.1, -0.2, 0.0, 0.0, 0.03),
        )

    def test_image_and_camera_info_share_stamp_and_optical_frame(self):
        image = image_message(self.frame, self.stamp)
        info = camera_info_message(
            self.calibration, self.stamp, "wrist_camera_optical_frame"
        )

        self.assertEqual(image.header.stamp, info.header.stamp)
        self.assertEqual(image.header.frame_id, "wrist_camera_optical_frame")
        self.assertEqual(info.header.frame_id, "wrist_camera_optical_frame")
        self.assertEqual(image.encoding, "bgr8")
        self.assertEqual((image.width, image.height, image.step), (3, 2, 9))
        self.assertEqual(info.distortion_model, "plumb_bob")
        self.assertEqual(list(info.r), [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(list(info.p), [10.0, 0.0, 1.0, 0.0, 0.0, 11.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0])

    def test_converter_rejects_non_bgr8_frame(self):
        bad_frame = CameraFrame(
            data=np.zeros((2, 3, 3), dtype=np.uint8),
            timestamp_s=1.0,
            encoding="rgb8",
            width=3,
            height=2,
        )
        with self.assertRaises(ValueError):
            image_message(bad_frame, self.stamp)

    def test_diagnostics_reports_offline_and_recovered_state(self):
        offline = CameraStatus(
            instance_id="cam01",
            state="backoff",
            requested_width=1280,
            requested_height=720,
            requested_fps=30.0,
            requested_pixel_format="MJPG",
            actual_width=None,
            actual_height=None,
            actual_fps=None,
            actual_pixel_format=None,
            frame_count=0,
            capture_error_count=1,
            last_frame_timestamp_s=None,
            last_error="camera unplugged",
            retry_after_monotonic_s=2.0,
        )
        recovered = CameraStatus(
            instance_id="cam01",
            state="streaming",
            requested_width=1280,
            requested_height=720,
            requested_fps=30.0,
            requested_pixel_format="MJPG",
            actual_width=1280,
            actual_height=720,
            actual_fps=30.0,
            actual_pixel_format="MJPG",
            frame_count=1,
            capture_error_count=1,
            last_frame_timestamp_s=1.0,
            last_error=None,
            retry_after_monotonic_s=None,
        )

        offline_message = diagnostic_message(offline, self.calibration, self.stamp)
        recovered_message = diagnostic_message(recovered, self.calibration, self.stamp)

        self.assertEqual(offline_message.status[0].level, DiagnosticStatus.ERROR)
        self.assertEqual(offline_message.status[0].message, "camera unplugged")
        self.assertEqual(recovered_message.status[0].level, DiagnosticStatus.OK)
        self.assertEqual(recovered_message.status[0].message, "streaming")


if __name__ == "__main__":
    unittest.main()
