"""One ROS node/process per configured MyArm camera instance."""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from myarm_sdk.core import CameraFrame
from myarm_sdk.service import CameraService, CameraServiceError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .camera_config import camera_instance, camera_ros_config
from .camera_diagnostics import diagnostic_message
from .ros_camera_info_converter import camera_info_message
from .ros_image_converter import image_message


class MyArmCameraNode(Node):
    """Publish raw image, CameraInfo, and health from one CameraService."""

    _SERVICES_CONFIG = "service/config/services.yaml"

    def __init__(self) -> None:
        super().__init__("myarm_camera")
        self.declare_parameter("services_config", self._SERVICES_CONFIG)
        self.declare_parameter("camera_instance", "")
        services_config = str(self.get_parameter("services_config").value)
        instance_id = str(self.get_parameter("camera_instance").value).strip()
        if not instance_id:
            raise ValueError("camera_instance ROS parameter is required")
        instance = camera_instance(services_config, instance_id)
        ros_config = camera_ros_config(instance)
        topics = self._mapping(ros_config.get("topics"), "camera ros topics")
        self._publish_rate_hz = self._positive_float(
            ros_config.get("publish_rate_hz"), "camera ros publish_rate_hz"
        )
        self._service = CameraService.from_config(instance)
        self._calibration = self._service.calibration()
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        diagnostics_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_publisher = self.create_publisher(
            Image, self._required_string(topics.get("image_raw"), "image_raw topic"), sensor_qos
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo,
            self._required_string(topics.get("camera_info"), "camera_info topic"),
            sensor_qos,
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            self._required_string(topics.get("diagnostics"), "diagnostics topic"),
            diagnostics_qos,
        )
        self._latest_lock = threading.Lock()
        self._latest_frame: Optional[CameraFrame] = None
        self._stop_event = threading.Event()
        self._worker = threading.Thread(
            target=self._capture_loop,
            name="camera-" + instance_id,
            daemon=True,
        )
        self._worker.start()
        self._publish_timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish_latest)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, "_worker"):
            self._worker.join(timeout=2.0)
        if hasattr(self, "_service"):
            self._service.close()
        return super().destroy_node()

    def _capture_loop(self) -> None:
        interval_s = 1.0 / self._publish_rate_hz
        while not self._stop_event.is_set():
            if self._service.status().state != "streaming":
                self._service.reconnect()
                self._stop_event.wait(min(interval_s, 0.1))
                continue
            try:
                frame = self._service.capture()
            except CameraServiceError:
                self._stop_event.wait(min(interval_s, 0.1))
                continue
            with self._latest_lock:
                self._latest_frame = frame
            self._stop_event.wait(interval_s)

    def _publish_latest(self) -> None:
        with self._latest_lock:
            frame = self._latest_frame
            self._latest_frame = None
        if frame is None:
            return
        stamp = self.get_clock().now().to_msg()
        image = image_message(frame, stamp)
        info = camera_info_message(self._calibration, stamp, frame.optical_frame)
        self._image_publisher.publish(image)
        self._camera_info_publisher.publish(info)

    def _publish_diagnostics(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._diagnostics_publisher.publish(
            diagnostic_message(self._service.status(), self._calibration, stamp)
        )

    @staticmethod
    def _mapping(value, name):
        if not isinstance(value, dict):
            raise TypeError(name + " must be a mapping")
        return value

    @staticmethod
    def _required_string(value, name):
        if not isinstance(value, str) or not value.strip():
            raise TypeError(name + " must be a non-empty string")
        return value.strip()

    @staticmethod
    def _positive_float(value, name):
        if isinstance(value, bool):
            raise TypeError(name + " must be positive")
        try:
            converted = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(name + " must be positive") from error
        if converted <= 0.0:
            raise ValueError(name + " must be positive")
        return converted


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MyArmCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
