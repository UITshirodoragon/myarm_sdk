# MyArm Python core

`pycore` là Python SDK độc lập với ROS. Nó chứa stable core types,
`port_interface`, `plugin_adapter` và các service capability dùng trực tiếp bởi
ROS 2 node.

```text
ROS node → service → port_interface → plugin_adapter
```

`service/config/services.yaml` là service manifest duy nhất: nó bật/tắt
camera, kinematics, trajectory và controller. Cấu hình chi tiết của hardware
instance hoặc backend nằm cạnh từng plugin adapter.

Package có thể được cài trực tiếp từ thư mục `pycore`:

```bash
cd pycore
../myarm_venv/bin/python -m pip install -e '.[kinematics]'
```

Hoặc, để tương thích với workflow ở thư mục gốc project:

```bash
cd ..
./install.sh --kinematics
```

`install.sh` mặc định dùng `../myarm_venv`, là cùng interpreter nên dùng để
build/chạy ROS 2. Có thể override bằng biến môi trường `VENV_DIR`.

Namespace import là `myarm_sdk`.

## Kinematics contract

`PinocchioKinematicsAdapter` lấy URDF làm source of truth cho thứ tự joint,
axis dương, hard limit và transform `base_link -> tool0`. YAML chỉ chọn URDF,
xác nhận `joint_order`, chọn base/TCP frame và đặt solver/safety policy.

- Pose luôn là metres + quaternion `xyzw`; solver sử dụng SE(3), không dùng
  Euler interpolation.
- API đầy đủ là `IKRequest(target_pose, seed, policy)` và trả `IKResult` gồm
  `q_solution`, residual position/orientation, iteration, SVD singularity và
  failure reason.
- `POSITION_ONLY` là mode riêng; orientation residual vẫn được report nhưng
  không dùng làm điều kiện hội tụ.
- `home` là initial pose an toàn; zero pose không được dùng làm seed mặc định
  vì wrist singularity q5≈0.

Feedback thật phải ở canonical model-space trước khi vào kinematics. Với
MyArm M750 PoE URDF hiện tại, robot adapter quy đổi q2/q3 firmware ±10° trước
khi trả `JointPositions` cho Pinocchio.

## Interfaces, adapters và services

Các contract là `CameraInterface`, `ControllerInterface`,
`KinematicsInterface`, `RobotArmInterface` và `TrajectoryInterface` trong
`myarm_sdk.port_interface`. Pinocchio, OpenCV và pymycobot nằm trong
`myarm_sdk.plugin_adapter`. Node chỉ dùng service phù hợp, ví dụ
`KinematicsService`.

```python
from myarm_sdk.core import JointPositions
from myarm_sdk.plugin_adapter.robot_arm import FakeRobotArmAdapter

arm = FakeRobotArmAdapter()
arm.move_joints(JointPositions((0, 0, 0, 0, 0, 0)))
```
