# ROS 2 support core

`pycore` là thư viện Python nhỏ dùng bởi các ROS 2 node của MyArm M750. Nó
không phải ROS 2 node, không chứa `rclpy`, message ROS, action server hay
business workflow.

Mục tiêu v0.x là cung cấp ba thứ:

* model Python độc lập với ROS (`JointPositions`);
* port `Arm` để node dùng cùng contract cho robot thật và fake;
* adapter mỏng cho `pymycobot`, fake arm và Pinocchio model.

ROS 2 node chịu trách nhiệm đọc parameters, tạo adapter, chuyển kiểu ROS sang
model và sở hữu luồng điều khiển. Camera, trajectory execution, controller,
API facade và application use case sẽ chỉ được thêm khi có ROS use case cụ thể.

Package có thể được cài trực tiếp từ thư mục `pycore`:

```bash
cd pycore
python3 -m pip install -e .
```

Hoặc, để tương thích với workflow ở thư mục gốc project:

```bash
python3 -m pip install -e '.[pycore]'
```

Namespace import là `myarm_sdk`.

## Ports và adapters

Mỗi nhóm adapter có port tương ứng trong `myarm_sdk.ports`: `Camera`,
`JointPositionController`, `Kinematics`, `RobotArm` và `TrajectoryPlanner`.
Mỗi adapter có thư mục `config/` riêng; các YAML trong đó là cấu hình mẫu,
an toàn để commit và được đóng gói cùng package.

```python
from myarm_sdk.adapters.robot_arm import FakeRobotArm
from myarm_sdk.model import JointPositions

arm = FakeRobotArm()
arm.move_joints(JointPositions((0, 0, 0, 0, 0, 0)))
```
