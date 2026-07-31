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
