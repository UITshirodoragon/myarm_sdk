"""Diagnostic conversion for the ROS camera boundary."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from myarm_sdk.core import CameraCalibration, CameraStatus


def diagnostic_message(
    status: CameraStatus, calibration: CameraCalibration, stamp
) -> DiagnosticArray:
    """Publish camera health even while the device is disconnected."""
    message = DiagnosticArray()
    message.header.stamp = stamp
    item = DiagnosticStatus()
    item.name = "myarm/camera/" + status.instance_id
    item.hardware_id = status.instance_id
    item.level = DiagnosticStatus.OK if status.state == "streaming" else DiagnosticStatus.ERROR
    item.message = status.state if status.last_error is None else status.last_error
    values = {
        "calibration_id": calibration.calibration_id,
        "calibration_source_sha256": calibration.source_sha256,
        "requested_mode": f"{status.requested_width}x{status.requested_height}@{status.requested_fps}/{status.requested_pixel_format}",
        "actual_mode": f"{status.actual_width}x{status.actual_height}@{status.actual_fps}/{status.actual_pixel_format}",
        "frame_count": str(status.frame_count),
        "capture_error_count": str(status.capture_error_count),
        "retry_after_monotonic_s": str(status.retry_after_monotonic_s),
    }
    item.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    message.status = [item]
    return message
