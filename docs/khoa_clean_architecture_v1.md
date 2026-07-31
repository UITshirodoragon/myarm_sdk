Được. Với quyết định mới này, tôi chốt theo hướng đơn giản hơn:

- Bỏ `runtime_assembly/` trong giai đoạn hiện tại.
- Không có `application/` hay `api/`.
- ROS 2 node làm việc trực tiếp với đúng service nó cần.
- `service/config/` chỉ có đúng một file: `services.yaml`.
- Camera được nhận diện bằng instance ID `cam01`, `cam02`; không cần khái niệm role `wrist` hay `shoulder`.
- Cờ `enabled` quyết định service hoặc camera instance nào được chạy.

Lưu ý kiến trúc: khi bỏ `RuntimeAssembler`, việc tạo adapter từ config vẫn phải diễn ra ở đâu đó. Ở phiên bản đơn giản này, mỗi service sẽ có factory classmethod `from_config()`. ROS node gọi service đó trực tiếp.

```text
ROS node
  ↓
KinematicsService.from_config(...)
  ↓
KinematicsInterface
  ↓
PinocchioKinematicsAdapter
```

## Cấu trúc pycore chốt

```text
myarm_sdk/
├── core/
│   ├── __init__.py
│   ├── pose.py
│   ├── joint_positions.py
│   ├── camera_frame.py
│   ├── spatial.py
│   ├── configuration.py
│   └── validation.py
│
├── port_interface/
│   ├── __init__.py
│   ├── camera.py
│   ├── controller.py
│   ├── kinematics.py
│   ├── robot_arm.py
│   └── trajectory.py
│
├── plugin_adapter/
│   ├── __init__.py
│   ├── camera/
│   │   ├── __init__.py
│   │   ├── opencv_camera.py
│   │   └── config/
│   │       ├── default.yaml
│   │       ├── cam01.yaml
│   │       └── cam02.yaml
│   │
│   ├── controller/
│   │   ├── __init__.py
│   │   ├── memory_controller.py
│   │   └── config/
│   │       └── default.yaml
│   │
│   ├── kinematics/
│   │   ├── __init__.py
│   │   ├── identity_kinematics.py
│   │   ├── pinocchio_kinematics.py
│   │   └── config/
│   │       ├── default.yaml
│   │       └── pinocchio_m750_poe.yaml
│   │
│   ├── robot_arm/
│   │   ├── __init__.py
│   │   ├── fake_robot_arm.py
│   │   ├── myarm_m750_robot_arm.py
│   │   └── config/
│   │       └── default.yaml
│   │
│   └── trajectory/
│       ├── __init__.py
│       ├── linear_trajectory.py
│       └── config/
│           └── default.yaml
│
└── service/
    ├── __init__.py
    ├── camera.py
    ├── controller.py
    ├── kinematics.py
    ├── trajectory.py
    └── config/
        └── services.yaml
```

Không có config riêng trong `port_interface/`. Interface chỉ là contract.

## Tên class chốt

```text
port_interface/
    CameraInterface
    ControllerInterface
    KinematicsInterface
    RobotArmInterface
    TrajectoryInterface

plugin_adapter/
    OpenCVCameraAdapter
    MemoryControllerAdapter
    PinocchioKinematicsAdapter
    MyArmM750RobotArmAdapter
    LinearTrajectoryAdapter

service/
    CameraService
    ControllerService
    KinematicsService
    TrajectoryService
```

## Hai cấp config còn lại

Do bỏ runtime assembly và gom service config thành một file, kiến trúc thực tế còn hai cấp config:

```text
1. Plugin adapter config
   → profile/backend/hardware instance cụ thể

2. service/config/services.yaml
   → config cao nhất hiện tại:
     bật/tắt service, chọn plugin adapter,
     chọn camera instance, tốc độ, named pose...
```

Sau này nếu deployment bắt đầu có nhiều Jetson/lab/simulation khác nhau, bạn có thể thêm cấp runtime lại. Hiện tại chưa cần.

## Adapter config camera

`plugin_adapter/camera/config/cam01.yaml`

```yaml
instance_id: cam01
plugin_adapter: opencv

device:
  device_path: /dev/video-by-id/usb-MyCamera_cam01-video-index0
  fallback_index: 0

capture:
  width: 640
  height: 480
  fps: 5
  encoding: bgr8

intrinsic_calibration:
  camera_info_url: package://myarm_calibration/cam01_intrinsics.yaml

frames:
  optical_frame: cam01_optical_frame
```

`plugin_adapter/camera/config/cam02.yaml`

```yaml
instance_id: cam02
plugin_adapter: opencv

device:
  device_path: /dev/video-by-id/usb-MyCamera_cam02-video-index0
  fallback_index: 1

capture:
  width: 640
  height: 480
  fps: 5
  encoding: bgr8

intrinsic_calibration:
  camera_info_url: package://myarm_calibration/cam02_intrinsics.yaml

frames:
  optical_frame: cam02_optical_frame
```

`cam01` và `cam02` là identity thực. Không cần `wrist` hoặc `shoulder`.

Nếu một camera gắn ở wrist, thông tin mount/extrinsic thuộc deployment/service config, không thuộc intrinsic calibration của adapter camera.

## Một service config duy nhất

`service/config/services.yaml`

```yaml
schema_version: 1

defaults:
  update_rate_hz: 5.0

services:
  camera:
    enabled: true

    instances:
      cam01:
        enabled: true
        plugin_adapter: opencv
        plugin_config: plugin_adapter/camera/config/cam01.yaml

        topic_namespace: /myarm/cameras/cam01
        publish_rate_hz: 5.0

        mount:
          parent_frame: wrist_link
          child_frame: cam01_link
          translation_m: [0.030, 0.000, 0.040]
          rotation_xyzw: [0.0, 0.0, 0.0, 1.0]

      cam02:
        enabled: false
        plugin_adapter: opencv
        plugin_config: plugin_adapter/camera/config/cam02.yaml

        topic_namespace: /myarm/cameras/cam02
        publish_rate_hz: 5.0

        mount:
          parent_frame: shoulder_link
          child_frame: cam02_link
          translation_m: [0.040, 0.000, 0.030]
          rotation_xyzw: [0.0, 0.0, 0.0, 1.0]

  kinematics:
    enabled: true
    plugin_adapter: pinocchio
    plugin_config: plugin_adapter/kinematics/config/pinocchio_m750_poe.yaml
    update_rate_hz: 5.0

    initial_named_pose: zero

  trajectory:
    enabled: false
    plugin_adapter: linear
    plugin_config: plugin_adapter/trajectory/config/default.yaml
    update_rate_hz: 5.0

  controller:
    enabled: false
    plugin_adapter: memory
    plugin_config: plugin_adapter/controller/config/default.yaml
    update_rate_hz: 5.0
```

Chỉ dùng `cam01`:

```yaml
services:
  camera:
    enabled: true
    instances:
      cam01:
        enabled: true
      cam02:
        enabled: false
```

Dùng cả hai:

```yaml
services:
  camera:
    enabled: true
    instances:
      cam01:
        enabled: true
      cam02:
        enabled: true
```

Tắt toàn bộ camera:

```yaml
services:
  camera:
    enabled: false
```

## Service factory trực tiếp

Ví dụ `service/camera.py`:

```python
from myarm_sdk.plugin_adapter.camera.opencv_camera import OpenCVCameraAdapter


class CameraService:
    def __init__(self, camera, camera_id, optical_frame):
        self._camera = camera
        self._camera_id = camera_id
        self._optical_frame = optical_frame

    @classmethod
    def from_config(cls, camera_config):
        if camera_config.plugin_adapter != "opencv":
            raise ValueError(
                f"Unsupported camera adapter: {camera_config.plugin_adapter}"
            )

        adapter_config = load_adapter_config(
            camera_config.plugin_config
        )

        camera = OpenCVCameraAdapter(
            device_path=adapter_config.device.device_path,
            fallback_index=adapter_config.device.fallback_index,
        )

        return cls(
            camera=camera,
            camera_id=adapter_config.instance_id,
            optical_frame=adapter_config.frames.optical_frame,
        )

    def capture_once(self):
        return self._camera.capture()

    def close(self):
        self._camera.close()
```

Trong kiến trúc đơn giản này, `CameraService.from_config()` là nơi chọn plugin adapter. Sau này nếu số plugin phức tạp hơn, logic này mới cần tách ra thành registry hoặc assembler.

Ví dụ `service/kinematics.py`:

```python
from myarm_sdk.plugin_adapter.kinematics.pinocchio_kinematics import (
    PinocchioKinematicsAdapter,
)


class KinematicsService:
    def __init__(self, kinematics, initial_joints):
        self._kinematics = kinematics
        self._last_solution = initial_joints
        self._target_pose = None

    @classmethod
    def from_config(cls, config):
        if config.plugin_adapter != "pinocchio":
            raise ValueError(
                f"Unsupported kinematics adapter: {config.plugin_adapter}"
            )

        adapter_config = load_adapter_config(config.plugin_config)

        kinematics = PinocchioKinematicsAdapter(
            urdf_path=resolve_robot_description(
                package_name=adapter_config.robot_description.package,
                relative_path=adapter_config.robot_description.relative_path,
            ),
            tool_frame=adapter_config.tool_frame,
        )

        return cls(
            kinematics=kinematics,
            initial_joints=load_named_pose(config.initial_named_pose),
        )

    def set_target_pose(self, pose):
        self._target_pose = pose

    def step(self):
        solution = self._kinematics.inverse(
            self._target_pose,
            self._last_solution,
        )
        tcp_pose = self._kinematics.forward(solution)
        self._last_solution = solution

        return solution, tcp_pose
```

## ROS node gọi trực tiếp service

Camera node:

```python
class CameraNode(Node):
    def __init__(self, service, camera_config):
        super().__init__(f"myarm_camera_{camera_config.instance_id}")

        self._service = service
        self._publisher = self.create_publisher(
            Image,
            f"{camera_config.topic_namespace}/image_raw",
            10,
        )
        self._timer = self.create_timer(
            1.0 / camera_config.publish_rate_hz,
            self._publish_frame,
        )

    def _publish_frame(self):
        frame = self._service.capture_once()
        message = to_ros_image(frame.data, frame.encoding)
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)
```

Entrypoint camera nhận instance ID:

```python
def main(camera_id: str):
    config = load_services_config(
        "service/config/services.yaml"
    )

    camera_config = config.services.camera.instances[camera_id]

    if not config.services.camera.enabled:
        raise RuntimeError("Camera service is disabled")

    if not camera_config.enabled:
        raise RuntimeError(f"Camera instance {camera_id} is disabled")

    service = CameraService.from_config(camera_config)

    rclpy.init()
    node = CameraNode(service, camera_config)
    rclpy.spin(node)
```

Kinematics node:

```python
def main():
    config = load_services_config(
        "service/config/services.yaml"
    )

    if not config.services.kinematics.enabled:
        raise RuntimeError("Kinematics service is disabled")

    service = KinematicsService.from_config(
        config.services.kinematics
    )

    rclpy.init()
    node = CartesianCommandNode(service)
    rclpy.spin(node)
```

Như vậy:

```text
CameraNode
→ CameraService
→ CameraInterface
→ OpenCVCameraAdapter

CartesianCommandNode
→ KinematicsService
→ KinematicsInterface
→ PinocchioKinematicsAdapter

TrajectoryNode
→ TrajectoryService
→ TrajectoryInterface
→ LinearTrajectoryAdapter
```

## Quy ước tên file config

```text
default.yaml
    Cấu hình mặc định an toàn cho một plugin adapter.

<instance_id>.yaml
    Config của hardware instance.
    Ví dụ: cam01.yaml, cam02.yaml

<adapter>_<profile>.yaml
    Config một backend/model profile.
    Ví dụ: pinocchio_m750_poe.yaml

services.yaml
    File config duy nhất của toàn bộ service.
```

Tất cả dùng:

```text
lowercase
snake_case
.yaml
```

Điểm chốt cuối: `services.yaml` là source of truth bật/tắt service và instance. Các YAML ở `plugin_adapter/*/config/` chỉ là profile/config chi tiết cho backend hoặc hardware instance.