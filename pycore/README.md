# MyArm Python core

`pycore` là Python SDK độc lập với ROS. Nó chứa stable core types,
`port_interface`, `plugin_adapter` và các service capability dùng trực tiếp bởi
ROS 2 node.

```text
ROS node → service → port_interface → plugin_adapter
```

`service/config/services.yaml` là service manifest duy nhất: nó bật/tắt
camera, robot arm, kinematics, trajectory planner và motion execution. Cấu
hình chi tiết của hardware instance hoặc backend nằm cạnh từng plugin adapter.

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

Các contract là `CameraInterface`, `KinematicsInterface`, `RobotArmInterface`,
`TrajectoryPlannerInterface` và `MotionExecutionInterface` trong
`myarm_sdk.port_interface`. Pinocchio, OpenCV, pymycobot, minimum-jerk planner
và monotonic-time executor nằm trong `myarm_sdk.plugin_adapter`. Node chỉ dùng
service phù hợp, ví dụ `KinematicsService`, `TrajectoryPlannerService` hoặc
`MotionExecutionService`.

```python
from myarm_sdk.core import JointPositions
from myarm_sdk.plugin_adapter.robot_arm import FakeRobotArm

arm = FakeRobotArm()
arm.write_joint_positions(JointPositions((0, 0, 0, 0, 0, 0)))
```

Robot-arm implementations are stateful. ``arm.state`` is an immutable cached
snapshot with measured state and last accepted command; ``read_state()`` is the
explicit feedback transaction. ``FakeRobotArm`` applies each accepted target
immediately to measured state, while ``MyArmM750RobotArm`` only changes its
measured state after `MyArmMControl.get_angles()` returns feedback.

`FakeRobotArm` starts at the configured safe `home` pose unless an explicit
initial pose is supplied. `MyArmM750RobotArm` requires the six
`JointMetadata` entries loaded from the canonical URDF, reads feedback during
`connect()`, and refuses physical commands until that measurement is valid.

`RobotArmService` is the only service that owns a `RobotArmInterface`. It
loads the same shared `robot` config as kinematics and exposes only lifecycle,
`read_feedback()` and `send_joint_setpoint()`. Nó không xếp hàng target, không
nội suy trajectory và không arbitrate goal; các trách nhiệm đó thuộc
`MotionExecutionService`. The checked-in configuration selects `FakeRobotArm`.
With a physical profile, feedback/RViz still work by default; actual motion
requires the explicit `transport.allow_physical_motion: true` opt-in, and the
driver accepts only the executor's internal setpoint stream.

## Joint trajectory và motion execution

`MinimumJerkJointTrajectoryAdapter` implements a synchronized quintic
minimum-jerk point-to-point profile. Its output is a validated
`JointTrajectory` with `q`, `qdot`, `qddot` and strictly increasing timestamps.
Position/velocity limits come from URDF; per-joint acceleration limits belong
to `plugin_adapter/trajectory/config/minimum_jerk_joint_trajectory.yaml`.

`TimeScalingPolicy` supports `auto_limited`, `requested_duration_stretch`,
`requested_duration_strict` and `speed_scale`. A profile that cannot satisfy
the configured limits never returns a command trajectory. For speed scale
`r`, duration becomes `T/r`, velocity scales by `r` and acceleration by `r²`.

`MotionExecutionService` is transport-free. It samples an already validated
trajectory using a monotonic clock, returns desired setpoints and reports
execution state, timing lag, tracking error and terminal timeout. The ROS node
publishes those setpoints to the robot driver; it does not create a second
robot adapter or serial connection.
