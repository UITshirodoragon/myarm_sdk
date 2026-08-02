> Tài liệu này là ghi chú thiết kế lịch sử. Runtime C925e v1 hiện hành được
> chốt tại [camera_c925e.md](camera_c925e.md); các ví dụ C922 bên dưới không
> phải cấu hình để triển khai.

Bạn nói đúng. Với kiến trúc hiện tại của SDK, **ROS camera node nên sử dụng `CameraService` làm tầng giao tiếp duy nhất với camera**. Việc tôi đề xuất dùng trực tiếp `usb_cam` làm production node đã làm mất vai trò của service và không còn nhất quán với kiến trúc:

```text
ROS node → service → port interface → plugin adapter
```

Repository của bạn đã định nghĩa rõ `CameraService` là thành phần sở hữu một camera instance thông qua `CameraInterface`; `OpenCVCameraAdapter` chỉ là implementation hiện có tạm thời.

# 1. Kiến trúc camera nên chốt lại

```text
myarm_camera_node
        │
        │ gọi API Python
        ▼
CameraService
        │
        ▼
CameraInterface
        │
        ├── OpenCVCameraAdapter       # placeholder hiện tại
        ├── V4L2CameraAdapter         # hướng production Linux USB
        ├── GStreamerCameraAdapter    # hướng tối ưu Jetson
        └── FakeCameraAdapter         # test / replay
```

ROS node không được trực tiếp:

* gọi `cv2.VideoCapture`;
* mở `/dev/video*`;
* đọc YAML hardware;
* biết camera đang dùng OpenCV, V4L2 hay GStreamer;
* xử lý reconnect ở mức thiết bị.

Những trách nhiệm đó thuộc `CameraService` và adapter.

ROS node chỉ làm nhiệm vụ chuyển đổi giữa domain model và ROS:

```text
CameraFrame
    → sensor_msgs/Image

CameraCalibration
    → sensor_msgs/CameraInfo

CameraStatus
    → diagnostic_msgs/DiagnosticArray
```

`CameraInterface` hiện đã được thiết kế như single-camera contract với `capture()` và `close()`, đúng với hướng một service quản lý một camera vật lý.

---

# 2. Phân biệt “CameraService” và “ROS service”

Tên `CameraService` trong `pycore` là **application service**, không phải ROS 2 service server.

```text
CameraService
    Python application/domain service
    Điều phối camera adapter và policy.

ROS service
    Giao tiếp request/response qua DDS,
    ví dụ /camera/restart hoặc /camera/capture_once.
```

Luồng ảnh liên tục vẫn phải publish bằng topic:

```text
sensor_msgs/msg/Image
sensor_msgs/msg/CameraInfo
```

Không dùng ROS service để gửi từng frame vì service là request/response và không phù hợp với stream camera.

Do đó:

```text
ROS node dùng CameraService
```

không có nghĩa là:

```text
ảnh được truyền bằng ROS service
```

---

# 3. Vai trò chính xác của từng lớp

## 3.1. `CameraInterface`

Contract ổn định, độc lập ROS và độc lập thư viện capture:

```python
class CameraInterface(Protocol):
    def open(self) -> None:
        ...

    def capture(self) -> CameraFrame:
        ...

    def close(self) -> None:
        ...

    def get_status(self) -> CameraDeviceStatus:
        ...
```

Contract hiện tại mới có `capture()` và `close()`.

Nên mở rộng có kiểm soát, không đưa ROS type vào đây.

## 3.2. `CameraService`

Đây là tầng ROS node phải sử dụng.

Service chịu trách nhiệm:

* tạo adapter từ cấu hình instance;
* sở hữu duy nhất adapter;
* mở và đóng camera;
* xác thực capture resolution;
* kiểm tra resolution khớp calibration;
* giới hạn publish/capture rate;
* quản lý reconnect;
* trả trạng thái camera;
* trả calibration tương ứng;
* tạo timestamp/capture metadata;
* bảo đảm camera chưa bị một owner khác chiếm dụng.

Code hiện tại đã đi đúng hướng: `CameraService.from_config()` đọc plugin config, tạo adapter và lưu `instance_id`, `optical_frame`.

Điểm thiếu là service hiện còn quá mỏng và chỉ hỗ trợ OpenCV.

## 3.3. Plugin adapter

Adapter chỉ xử lý chi tiết backend:

```text
OpenCVCameraAdapter
    cv2.VideoCapture

V4L2CameraAdapter
    Linux V4L2 API

GStreamerCameraAdapter
    GStreamer pipeline

FakeCameraAdapter
    generated image / replay image
```

`OpenCVCameraAdapter` hiện chỉ mở nguồn video và trả `CameraFrame`; nó chưa áp dụng width, height, FPS hay reconnect policy.

Điều đó không có nghĩa bỏ service; nó có nghĩa phải phát triển adapter và service đầy đủ hơn.

## 3.4. ROS camera node

Mỗi ROS node sở hữu đúng một `CameraService`:

```text
logitech_c922_01_camera_node
    └── CameraService(instance=logitech_c922_01)

logitech_c922_02_camera_node
    └── CameraService(instance=logitech_c922_02)
```

Node chịu trách nhiệm:

* khai báo ROS parameters;
* tạo publisher;
* timer hoặc worker loop;
* gọi `service.capture()`;
* chuyển NumPy frame thành `Image`;
* publish `CameraInfo`;
* publish diagnostics;
* shutdown sạch khi ROS dừng.

---

# 4. Vì sao không nên dùng trực tiếp `usb_cam` trong kiến trúc chính

`usb_cam` tự nó đã là một ROS driver node. Nếu dùng trực tiếp:

```text
usb_cam node → Image
```

thì luồng không đi qua:

```text
CameraService → CameraInterface → adapter
```

Khi đó:

* service không sở hữu camera;
* config instance trong pycore không còn là nguồn chuẩn;
* standalone SDK và ROS runtime dùng hai đường khác nhau;
* policy reconnect, diagnostics và calibration dễ bị phân tán;
* NeuGrasp hoặc application khó dùng chung abstraction camera ngoài ROS.

Vì mục tiêu của bạn là xây SDK có core độc lập ROS, phương án nhất quán hơn là:

> `CameraService` là stable application API; ROS node chỉ là một delivery adapter của service.

`usb_cam` vẫn có thể tồn tại như một phương án thử nghiệm hoặc fallback, nhưng không nên là backend mặc định của kiến trúc này.

---

# 5. Backend production nên phát triển thế nào

## Giai đoạn 1 — dùng OpenCV qua CameraService

```text
CameraNode
    → CameraService
        → OpenCVCameraAdapter
```

Đây là phương án phù hợp để hoàn thiện node, topic, launch và config trước.

Cần nâng cấp `OpenCVCameraAdapter` để nhận đầy đủ:

```python
OpenCVCameraAdapter(
    device_path=...,
    fallback_index=...,
    width=1280,
    height=720,
    fps=30.0,
    pixel_format="MJPG",
    encoding="bgr8",
)
```

Sau khi set thông số, adapter phải đọc lại giá trị thực tế từ thiết bị:

```text
requested width  = 1280
actual width     = 1280

requested height = 720
actual height    = 720
```

Nếu không khớp calibration và `require_exact_resolution=true`, service phải từ chối chuyển sang trạng thái streaming.

## Giai đoạn 2 — thêm GStreamer hoặc V4L2 adapter

```text
CameraNode
    → CameraService
        → GStreamerCameraAdapter
```

Không thay đổi ROS node, topic hay application API.

Chỉ đổi config:

```yaml
plugin_adapter: gstreamer
```

Đây chính là lợi ích của service và port interface.

## Giai đoạn 3 — C++ nếu benchmark yêu cầu

Nếu Python/OpenCV không đáp ứng latency hoặc CPU:

```text
Python ROS node
    không còn tối ưu
```

Có thể chuyển camera worker sang C++, nhưng vẫn giữ cùng contract logic:

```text
ROS adapter → camera application service → hardware adapter
```

Không nên tối ưu sớm khi chưa benchmark.

---

# 6. `CameraService` nên được mở rộng

Đề xuất API:

```python
class CameraService:
    @classmethod
    def from_config(
        cls,
        instance_config: Mapping[str, Any],
    ) -> "CameraService":
        ...

    def open(self) -> None:
        ...

    def capture(self) -> CameraFrame:
        ...

    def calibration(self) -> CameraCalibration:
        ...

    def status(self) -> CameraStatus:
        ...

    def reconnect(self) -> None:
        ...

    def close(self) -> None:
        ...
```

## `CameraFrame`

Type hiện tại:

```python
@dataclass(frozen=True)
class CameraFrame:
    data: Any
    timestamp_s: float
    encoding: str = "bgr8"
```

Nên mở rộng thành:

```python
@dataclass(frozen=True)
class CameraFrame:
    data: Any
    timestamp_s: float
    sequence: int
    width: int
    height: int
    encoding: str
    optical_frame: str
```

Không đưa `sensor_msgs/Image` vào core.

## `CameraCalibration`

Thêm stable core type:

```python
@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    distortion_model: str
    k: tuple[float, ...]
    d: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]
```

ROS node chuyển type này sang `sensor_msgs/CameraInfo`.

## `CameraStatus`

```python
@dataclass(frozen=True)
class CameraStatus:
    instance_id: str
    state: CameraState
    requested_width: int
    requested_height: int
    actual_width: int
    actual_height: int
    requested_fps: float
    measured_fps: float
    frame_count: int
    capture_error_count: int
    last_frame_timestamp_s: float | None
    last_error: str | None
```

---

# 7. Node ROS đề xuất

Package:

```text
myarm_camera/
├── myarm_camera/
│   ├── camera_node.py
│   ├── ros_image_converter.py
│   ├── ros_camera_info_converter.py
│   └── diagnostics_publisher.py
├── launch/
│   └── camera_system.launch.py
└── test/
```

## Một executable dùng cho mọi instance

Không cần viết:

```text
camera_01_node.py
camera_02_node.py
```

Chỉ cần:

```text
myarm_camera_node
```

Khởi tạo nhiều lần với config khác nhau:

```text
/myarm/cameras/logitech_c922_01/camera_node
/myarm/cameras/logitech_c922_02/camera_node
```

Node pseudoflow:

```python
class MyArmCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera")

        instance_config = load_instance_config(...)
        self._service = CameraService.from_config(instance_config)
        self._service.open()

        self._image_pub = ...
        self._info_pub = ...
        self._diagnostic_pub = ...

        self._timer = self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_frame,
        )

    def _publish_frame(self) -> None:
        frame = self._service.capture()
        calibration = self._service.calibration()

        stamp = self.get_clock().now().to_msg()

        image_msg = convert_frame(frame, stamp)
        info_msg = convert_calibration(calibration, stamp)

        self._image_pub.publish(image_msg)
        self._info_pub.publish(info_msg)
```

Một lưu ý timestamp:

* adapter có thể lưu capture timestamp monotonic/system time để diagnostics;
* ROS message nên dùng ROS clock tại boundary;
* `Image` và `CameraInfo` phải dùng cùng một `stamp`.

---

# 8. Config sau khi chốt dùng `CameraService`

## `services.yaml`

Deployment vẫn gộp trong manifest như bạn yêu cầu:

```yaml
services:
  camera:
    enabled: false
    default_profile: none

    profiles:
      none:
        instances: []

      camera_01:
        instances: [logitech_c922_01]

      camera_02:
        instances: [logitech_c922_02]

      dual:
        instances:
          - logitech_c922_01
          - logitech_c922_02

    instances:
      logitech_c922_01:
        enabled: true
        role: wrist

        service:
          factory: camera_service

        plugin_adapter: opencv
        plugin_config: plugin_adapter/camera/config/logitech_c922_01.yaml

        ros:
          node_name: logitech_c922_01_camera
          namespace: /myarm/cameras/logitech_c922_01
          publish_rate_hz: 30.0

          topics:
            image_raw: image_raw
            camera_info: camera_info
            diagnostics: diagnostics

          frames:
            camera_link: wrist_camera_link
            optical_frame: wrist_camera_optical_frame

        mount:
          owner: urdf_xacro
          parent_frame: gripper_base_link
          translation_m: [0.0, 0.0, 0.0]
          rotation_rpy_rad: [0.0, 0.0, 0.0]

      logitech_c922_02:
        enabled: true
        role: shoulder

        service:
          factory: camera_service

        plugin_adapter: opencv
        plugin_config: plugin_adapter/camera/config/logitech_c922_02.yaml

        ros:
          node_name: logitech_c922_02_camera
          namespace: /myarm/cameras/logitech_c922_02
          publish_rate_hz: 30.0

          topics:
            image_raw: image_raw
            camera_info: camera_info
            diagnostics: diagnostics

          frames:
            camera_link: shoulder_camera_link
            optical_frame: shoulder_camera_optical_frame

        mount:
          owner: urdf_xacro
          parent_frame: shoulder_link
          translation_m: [0.0, 0.0, 0.0]
          rotation_rpy_rad: [0.0, 0.0, 0.0]
```

Manifest hiện đã có capability camera và hai instance, nhưng mới đặt `plugin_adapter: opencv`, namespace và mount cơ bản.

## Instance config

```yaml
schema_version: 1

instance_id: logitech_c922_01
plugin_adapter: opencv

device:
  device_path: /dev/v4l/by-id/REPLACE_CAMERA_01
  fallback_index: 0
  allow_fallback_index: false

capture:
  width: 1280
  height: 720
  fps: 30.0
  pixel_format: MJPG
  encoding: bgr8

intrinsic_calibration:
  camera_info_url: package://myarm_camera/config/calibration/logitech_c922_01_1280x720.yaml
  require_exact_resolution: true
  allow_uncalibrated: false

frames:
  optical_frame: wrist_camera_optical_frame
```

Khi thay backend:

```yaml
plugin_adapter: gstreamer
```

ROS node không thay đổi.

---

# 9. Launch chính xác

`camera_system.launch.py`:

1. đọc `services.yaml`;
2. resolve `camera_profile`;
3. lấy danh sách instance;
4. tạo một `myarm_camera_node` cho mỗi instance;
5. truyền config path hoặc instance ID cho node;
6. không trực tiếp tạo OpenCV adapter trong launch.

Luồng:

```text
camera_profile:=dual
        │
        ▼
profile resolver
        │
        ├── logitech_c922_01
        └── logitech_c922_02
               │
               ▼
        2 ROS camera nodes
               │
               ▼
        mỗi node tạo 1 CameraService
```

Bốn profile vẫn giữ:

```text
none
camera_01
camera_02
dual
```

---

# 10. Có nên giữ `usb_cam` làm option không?

Có thể, nhưng phải phân biệt hai chế độ.

## Chế độ khuyến nghị

```text
backend: sdk_service
```

```text
ROS CameraNode
    → CameraService
        → OpenCV/GStreamer/V4L2 adapter
```

Đây là kiến trúc chính.

## Chế độ compatibility

```text
backend: external_ros_driver
```

```text
usb_cam
    → ROS Image topics
```

Chế độ này có thể dùng để:

* so sánh benchmark;
* kiểm tra camera nhanh;
* fallback khi SDK adapter chưa hoàn thiện.

Nhưng nó không đi qua `CameraService`, vì vậy không nên là production mặc định của SDK.

Không nên tạo adapter kiểu:

```text
CameraService
    → UsbCamAdapter
        → khởi chạy một ROS node khác
```

vì adapter core sẽ phụ thuộc ROS và quản lý process bên ngoài, phá vỡ ranh giới kiến trúc.

---

# 11. Quyết định cuối cùng

Kiến trúc camera của dự án nên chốt:

```text
1. CameraService tiếp tục được sử dụng.
2. Mỗi ROS camera node sở hữu một CameraService.
3. CameraService là API duy nhất để node giao tiếp với camera.
4. CameraInterface giữ độc lập ROS.
5. OpenCVCameraAdapter là placeholder/backend ban đầu.
6. GStreamerCameraAdapter hoặc V4L2CameraAdapter là hướng production.
7. ROS node chỉ chuyển CameraFrame/Calibration sang ROS messages.
8. Một camera vật lý tương ứng một service instance và một node instance.
9. Deployment, role, namespace và mount nằm trong services.yaml.
10. Device/capture/calibration nằm trong plugin config riêng.
11. Ảnh capture ở 1280×720 để khớp intrinsic đã calibration.
12. usb_cam chỉ là compatibility/benchmark option, không phải kiến trúc chính.
```

Vì vậy sơ đồ chính xác của package là:

```text
myarm_camera_node
    ├── ROS parameters
    ├── Image publisher
    ├── CameraInfo publisher
    ├── Diagnostics publisher
    │
    └── CameraService
            ├── lifecycle
            ├── calibration validation
            ├── reconnect policy
            ├── capture policy
            │
            └── CameraInterface
                    └── OpenCV / GStreamer / V4L2 adapter
```

Đây mới là phương án nhất quán với mục tiêu ban đầu của `CameraService` và toàn bộ kiến trúc ports/adapters hiện tại của `myarm_sdk`.
