import pytest
from myarm_sdk.core import CameraCalibration, CameraFrame, load_sdk_yaml
from myarm_sdk.plugin_adapter.camera import FakeCameraAdapter, OpenCVCameraAdapter
from myarm_sdk.service.camera import CameraService, CameraServiceError


def test_fake_camera_service_loads_calibration_and_publishes_optical_frame():
    service = CameraService.from_config({
        "enabled": True,
        "plugin_adapter": "fake",
        "plugin_config": "plugin_adapter/camera/config/fake.yaml",
    })

    service.open()
    frame = service.capture_once()

    assert frame.width == 1280
    assert frame.height == 720
    assert frame.encoding == "bgr8"
    assert frame.optical_frame == "wrist_camera_optical_frame"
    assert service.calibration().calibration_id == "logitech_c925e_cam01_1280x720_mjpg_20260602"
    assert service.status().state == "streaming"


def test_production_camera_config_rejects_commissioning_placeholder():
    services = load_sdk_yaml("service/config/services.yaml")
    cam01 = services["services"]["camera"]["instances"]["cam01"]

    with pytest.raises(CameraServiceError, match="commissioning placeholder"):
        CameraService.from_config(cam01)


def test_camera_service_fails_closed_on_calibration_resolution_mismatch():
    calibration = CameraCalibration(
        calibration_id="wrong_resolution",
        source_sha256="test",
        width=640,
        height=480,
        distortion_model="opencv_radtan5",
        k=(1.0,) * 9,
        d=(0.0,) * 5,
    )
    service = CameraService(
        camera=FakeCameraAdapter(width=1280, height=720),
        instance_id="cam01",
        optical_frame="wrist_camera_optical_frame",
        calibration=calibration,
        requested_width=1280,
        requested_height=720,
        requested_fps=30.0,
        requested_pixel_format="MJPG",
    )

    with pytest.raises(CameraServiceError, match="calibration resolution"):
        service.open()

    assert service.status().state == "backoff"


class _FailingCamera:
    def open(self):
        pass

    def capture(self):
        raise RuntimeError("capture failed")

    def close(self):
        pass

    def status(self):
        return {}


def test_capture_error_enters_backoff_instead_of_switching_device():
    service = CameraService(
        camera=_FailingCamera(),
        instance_id="cam01",
        optical_frame="wrist_camera_optical_frame",
        calibration=None,
        requested_width=1280,
        requested_height=720,
        requested_fps=30.0,
        requested_pixel_format="MJPG",
    )
    service.open()

    with pytest.raises(CameraServiceError, match="capture failed"):
        service.capture()

    status = service.status()
    assert status.state == "backoff"
    assert status.capture_error_count == 1
    assert status.retry_after_monotonic_s is not None


def test_camera_frame_keeps_existing_positional_constructor_compatibility():
    frame = CameraFrame(data="image", timestamp_s=1.0, encoding="bgr8")
    assert frame.sequence == 0
    assert frame.width == 0


class _ModeMismatchCapture:
    def isOpened(self):
        return True

    def set(self, *_args):
        return True

    def get(self, property_id):
        values = {
            _ModeMismatchCV2.CAP_PROP_FRAME_WIDTH: 640.0,
            _ModeMismatchCV2.CAP_PROP_FRAME_HEIGHT: 480.0,
            _ModeMismatchCV2.CAP_PROP_FPS: 30.0,
            _ModeMismatchCV2.CAP_PROP_FOURCC: float(
                _ModeMismatchCV2.VideoWriter_fourcc(*"MJPG")
            ),
        }
        return values[property_id]

    def release(self):
        pass


class _ModeMismatchCV2:
    CAP_V4L2 = 200
    CAP_PROP_BUFFERSIZE = 38
    CAP_PROP_FOURCC = 6
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_FPS = 5

    @staticmethod
    def VideoCapture(_source, _backend):
        return _ModeMismatchCapture()

    @staticmethod
    def VideoWriter_fourcc(*characters):
        return sum(ord(character) << (8 * index) for index, character in enumerate(characters))


def test_opencv_adapter_rejects_negotiated_mode_mismatch():
    adapter = OpenCVCameraAdapter(
        device_path="/dev/v4l/by-id/test-video-index0",
        width=1280,
        height=720,
        fps=30.0,
        pixel_format="MJPG",
        cv2_module=_ModeMismatchCV2,
    )

    with pytest.raises(RuntimeError, match="resolution mismatch"):
        adapter.open()
