# MyArm SDK architecture

Tài liệu này mô tả cấu trúc hiện tại của `myarm_sdk`. Mục tiêu là giữ pycore
độc lập ROS và độc lập hardware transport, trong khi node ROS 2 chỉ là boundary
map message/topic/action sang service phù hợp.

```text
ROS 2 node → service → port_interface → plugin_adapter
```

Không có `application/`, `api/` hoặc `runtime_assembly/` ở giai đoạn này. Mỗi
node gọi trực tiếp đúng service mà nó cần. Factory `from_config()` của service
là composition point nhỏ, chọn plugin adapter từ manifest.

## Cấu trúc pycore

```text
myarm_sdk/
├── core/
│   ├── pose.py, joint_positions.py, joint_metadata.py
│   ├── joint_trajectory.py, trajectory_point.py, trajectory_planning.py
│   ├── motion_execution.py
│   ├── robot_arm.py, urdf.py
│   └── spatial.py, configuration.py, validation.py
├── port_interface/
│   ├── camera.py                 # CameraInterface
│   ├── kinematics.py             # KinematicsInterface
│   ├── robot_arm.py              # RobotArmInterface
│   ├── trajectory.py             # TrajectoryPlannerInterface
│   └── motion_execution.py       # MotionExecutionInterface
├── plugin_adapter/
│   ├── camera/
│   ├── kinematics/
│   ├── robot_arm/
│   ├── trajectory/
│   │   └── minimum_jerk_joint_trajectory.py
│   └── motion_execution/
│       └── monotonic_time_motion_execution.py
└── service/
    ├── camera.py
    ├── kinematics.py
    ├── robot_arm.py
    ├── trajectory.py             # TrajectoryPlannerService
    ├── motion_execution.py       # MotionExecutionService
    └── config/services.yaml
```

Tên `controller` đã được bỏ. Trong kiến trúc này nó quá mơ hồ: phần lập kế
hoạch là `TrajectoryPlanner`, còn phần tiến hành một trajectory theo clock là
`MotionExecution`.

## Config

Có hai cấp config.

1. `plugin_adapter/<module>/config/*.yaml` là profile cụ thể cho adapter.
   Ví dụ camera instance, serial connection, Pinocchio solver, acceleration
   limit hoặc policy executor.
2. `service/config/services.yaml` là manifest cấp cao nhất hiện tại. Nó chọn
   plugin, bật/tắt capability, topic ROS, rate, URDF và named pose chung.

`port_interface` không có config vì nó chỉ định nghĩa contract.

### Camera theo instance

`cam01` và `cam02` là identity của camera, không phải role. Một deployment có
thể bật chỉ `cam01`, chỉ `cam02`, hoặc cả hai. Mỗi instance dùng profile riêng
để calibration intrinsic không bị lẫn:

```yaml
services:
  camera:
    enabled: true
    instances:
      cam01:
        enabled: true
        plugin_adapter: opencv
        plugin_config: plugin_adapter/camera/config/cam01.yaml
      cam02:
        enabled: false
        plugin_adapter: opencv
        plugin_config: plugin_adapter/camera/config/cam02.yaml
```

Thông tin mount/extrinsic (parent frame, child frame, pose) nằm trong manifest
deployment/service. Vì vậy cùng `cam01` có thể gắn wrist ở deployment này và
gắn shoulder ở deployment khác mà không sửa calibration intrinsic.

## Kinematics

`PinocchioKinematicsAdapter` dùng URDF làm source of truth cho:

- joint order canonical;
- axis/chiều dương theo quy tắc bàn tay phải của URDF;
- hard position limit và velocity limit;
- `base_link` và `tool0` transform tree.

YAML chỉ chọn URDF/frame và đặt solver policy. Pose dùng metre + quaternion
`xyzw`/SE(3) xuyên suốt; không nội suy Euler. IK nhận target pose, seed và
policy; kết quả có solution, residual position/orientation, iteration,
singularity metric và failure reason. `home` là seed/initial pose mặc định;
`zero` không dùng mặc định vì wrist singularity gần `q5 ≈ 0`.

`MyArmKinematicsNode` luôn lấy feedback canonical model-space từ
`/myarm/state/joint_state`. Khi IK thành công, nó chỉ publish endpoint an toàn
vào `/myarm/command/joint_goal`; nó không chạm robot driver trực tiếp.

## Trajectory planner

`TrajectoryPlannerInterface` được cài bởi
`MinimumJerkJointTrajectoryAdapter` và được expose bởi
`TrajectoryPlannerService`.

```text
q_start measured + q_goal + limits + time-scaling policy
  → validated JointTrajectory(time, q, qdot, qddot)
```

Planner dùng quintic minimum-jerk. `q`, `qdot`, `qddot` liên tục; qdot/qddot
bằng 0 ở đầu và cuối. Mọi point được validate theo hard position limit, URDF
velocity limit và acceleration limit trong profile YAML. Timestamp luôn bắt
đầu `t=0` và tăng nghiêm ngặt.

Các mode `TimeScalingPolicy`:

- `auto_limited`: chọn duration tối thiểu an toàn.
- `requested_duration_stretch`: tôn trọng duration nếu đủ; thiếu thì kéo dài.
- `requested_duration_strict`: reject nếu duration không đạt limit.
- `speed_scale`: với `0 < r ≤ 1`, `T = T_base/r`; qdot scale `r`, qddot scale
  `r²`.

Không có trường hợp trả trajectory lỗi để rồi gửi xuống robot.

## Motion execution và robot arm

`MotionExecutionInterface` được cài bởi
`MonotonicTimeMotionExecutionAdapter`; `MotionExecutionService` giữ lifecycle
`idle/executing/holding/canceled/succeeded/fault` và sample trajectory theo
monotonic clock. Nó không import ROS, không mở serial và không sở hữu robot.

`RobotArmService` ngược lại chỉ sở hữu một `RobotArmInterface`, có các trách
nhiệm nhỏ và rõ:

- lifecycle `connect`, `disconnect`, `power_on/off`, `stop`;
- `read_feedback()` không làm timer chết khi adapter lỗi;
- `send_joint_setpoint()` gửi một q đã được executor authorize.

Nó không queue goal, không plan, không preempt và không nội suy trajectory.
`FakeRobotArm` là memory robot: nó lưu đúng setpoint đã chấp nhận. Với
`MyArmM750RobotArm`, command và measured feedback luôn khác biệt cho tới khi
firmware trả feedback mới.

Physical motion phải opt-in rõ ràng:

```yaml
services:
  robot_arm:
    transport:
      accept_internal_setpoints: true
      allow_physical_motion: false
```

`allow_physical_motion: false` vẫn cho phép feedback và RViz, nhưng driver
không tạo subscription nhận setpoint execution cho robot thật.

## ROS runtime

```text
/myarm/command/tcp_pose
  → MyArmKinematicsNode
  → /myarm/command/joint_goal
  → MyArmMotionExecutionNode
      + /myarm/state/joint_state fresh feedback
  → TrajectoryPlannerService
  → MotionExecutionService
  → /myarm/internal/driver_joint_setpoint
  → MyArmRobotDriverNode
  → FakeRobotArm | MyArmM750RobotArm
  → /myarm/state/joint_state and /joint_states
  → robot_state_publisher → /tf, /tf_static → RViz2
```

`myarm_robot_driver` là serial owner duy nhất. Nó không nhận public direct
joint target; input của nó là private setpoint stream từ executor. Điều này
tránh hai node cùng điều khiển robot.

`myarm_motion_execution` còn cung cấp:

- `/myarm/trajectory/preview` với full `q/qdot/qddot`;
- action `/myarm/follow_joint_trajectory` cho full trajectory từ client;
- diagnostic, cancel và reset cho topic-driven motion.

Action chỉ nhận trajectory canonical đủ q/qdot/qddot, `t=0`, tất cả limit hợp
lệ và q0 gần feedback thật. Nó không dùng `time.sleep`; timer 5 Hz độc lập
tiến hành executor trong multi-threaded ROS executor.

## Deployment modes

`myarm_bringup` chạy trên Jetson và có launch flag độc lập:

- `enable_driver` để chọn fake/physical feedback driver;
- `enable_kinematics` để bật IK/FK;
- `enable_motion_execution` để bật joint planning/execution;
- `enable_robot_state_publisher` để phát TF cho RViz.

Do đó có thể:

- fake robot + RViz: bật tất cả (profile default);
- physical robot không RViz: tắt `enable_robot_state_publisher`;
- RViz chỉ với state ngoài: tắt driver nhưng cần một publisher `/joint_states`;
- host PC chỉ chạy RViz, còn `robot_state_publisher` và TF chạy Jetson qua DDS.

Mọi rate mặc định của driver, kinematics, planner/executor là 5 Hz. Đây là
profile bảo thủ cho demo/lab; acceleration/feedback/tracking policy cần được
tune và xác nhận trên hardware trước khi production motion được bật.
