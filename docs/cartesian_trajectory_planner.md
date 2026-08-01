````````````````````````Có thể tích hợp tốt, nhưng không phải chỉ thêm YAML là xong. Kiến trúc MyArm hiện đã có đúng các điểm ghép cần thiết:

- Pinocchio đã cung cấp FK/IK theo quaternion/SE(3), limit, singularity và diagnostics. [PinocchioKinematicsAdapter](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/plugin_adapter/kinematics/pinocchio_kinematics.py:189)
- `JointTrajectory(q, qdot, qddot)` và `MotionExecutionService` đã là đầu ra thực thi phù hợp.
- `myarm_motion_execution` đã có `FollowJointTrajectory`, kiểm tra state đo thực và là nơi duy nhất stream setpoint xuống driver. [MyArmMotionExecutionNode](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/ros2_ws/src/myarm_motion_execution/myarm_motion_execution/motion_execution_node.py:45)

Điểm chưa đủ là planner hiện tại chỉ nhận `q_start + q_goal`, và factory chỉ cho phép `minimum_jerk_joint`. [Contract hiện tại](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/port_interface/trajectory.py:8) [request joint-only](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/core/trajectory_planning.py:198)

Kiến trúc tôi khuyến nghị chốt:

```text
PoseStamped / Cartesian action
  → MyArmCartesianTrajectoryNode
  → CartesianTrajectoryPlannerService
  → CartesianSequentialClikTrajectoryPlannerAdapter
  → validated JointTrajectory(q, qdot, qddot)
  → /myarm/follow_joint_trajectory
  → MyArmMotionExecutionNode
  → /myarm/internal/driver_joint_setpoint
  → MyArmRobotDriverNode
```

Không cần thêm driver robot, không để planner publish thẳng private driver topic, và không dùng `KinematicsService` stateful như một planner ẩn.

Về Python SDK, nên có bộ riêng vì semantic đầu vào khác planner joint point-to-point:

```text
core/cartesian_trajectory_planning.py
port_interface/cartesian_trajectory.py
service/cartesian_trajectory.py
plugin_adapter/trajectory/cartesian_sequential_clik_trajectory.py
```

Tên lớp phù hợp:

```text
CartesianTrajectoryPlannerInterface
CartesianTrajectoryPlannerService
CartesianSequentialClikTrajectoryPlannerAdapter
```

Tôi không khuyên ép adapter Cartesian vào `TrajectoryPlannerInterface` hiện tại ngay lúc này: interface đó thực chất là joint-goal planner. Hai service cùng nằm trong một `services.yaml` vẫn đúng với thiết kế config một file của bạn:

```yaml
services:
  trajectory_planner:
    enabled: true
    plugin_adapter: minimum_jerk_joint

  cartesian_trajectory_planner:
    enabled: false
    plugin_adapter: cartesian_sequential_clik
    plugin_config: plugin_adapter/trajectory/config/cartesian_sequential_clik.yaml
```

Luồng thuật toán an toàn nên là:

```text
fresh measured q
  → FK(q) lấy TCP start thực
  → tạo Cartesian reference path
  → waypoint i: IK(target_i, seed=q(i-1))
  → FK/limit/singularity validation mọi waypoint
  → retime theo joint velocity/acceleration limit
  → dense validation q/qdot/qddot
  → JointTrajectory
```

Các nguyên tắc quan trọng:

- Seed đầu tiên luôn là `q` đo thực, không dùng last-commanded.
- Waypoint kế tiếp dùng nghiệm waypoint trước để giữ branch continuity.
- Một waypoint IK fail, joint-limit blocked, near-singular bị policy reject, timeout, hoặc residual vượt ngưỡng thì hủy toàn plan — không gửi trajectory một phần.
- Adapter inject `KinematicsInterface`, không gọi trực tiếp ROS và không phụ thuộc concrete `PinocchioKinematicsAdapter`.
- Không share một instance Pinocchio giữa planner workers/thread nếu không có lock, vì nó giữ mutable `Data`.

Từ reBot Python, nên lấy ý tưởng SE(3) geodesic và minimum-jerk:

\[
s(u)=10u^3-15u^4+6u^5
\]

\[
T(s)=T_0\exp(\log(T_0^{-1}T_1)s)
\]

[reBot sampler](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/trajectory/sampler.py:55) và warm-start waypoint tuần tự của [reBot CLIK tracker](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/trajectory/clik_tracker.py:62).

Nhưng không nên copy trực tiếp reBot vì tracker của họ chỉ trả `q + success flag`, không có `qdot/qddot`, không retime theo limit, vẫn có thể gửi waypoint IK lỗi, và trapezoid profile có lỗi normalization. ROS2 reBot cũng trộn high-level Cartesian motion với hardware manager; MyArm nên giữ separation hiện tại. [reBot ROS actions](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArmController_ROS2/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py)

Một chi tiết rất quan trọng: `se3_geodesic` là screw motion, không bảo đảm TCP XYZ đi thẳng. Config nên phân biệt rõ:

- `se3_geodesic`: giống reBot.
- `linear_translation_slerp`: translation đi thẳng, orientation quaternion SLERP.

Với ROS2, giữ `/myarm/command/tcp_pose` cho one-shot IK hiện tại. Planner mới nên có endpoint riêng, ví dụ `/myarm/command/cartesian_pose`, dùng `PoseStamped`; tốt nhất production dùng custom action để có cancel, feedback, duration/policy. Node phải transform pose về `base_link` hoặc reject frame khác rõ ràng.

Cuối cùng, 5 Hz đủ cho preview, RViz và demo fake. Nhưng 5 Hz không đủ để đảm bảo Cartesian path vật lý mượt/chính xác; execution setpoint rate cần được tách cấu hình và kiểm chứng với firmware/serial trước khi chạy robot thật, thường tối thiểu khoảng 20–50 Hz. Đồng thời phải dense-validate chính nội suy quintic mà executor sử dụng, không chỉ kiểm tra các waypoint.

Kết luận: tích hợp hoàn toàn khả thi và rất hợp với kiến trúc hiện tại. Nên làm theo hướng planner Cartesian độc lập → `FollowJointTrajectory` hiện có, giữ nguyên MotionExecution và RobotDriver. Tôi không thay đổi tệp nào trong lượt phân tích này

Khả thi và kiến trúc hiện tại đã có đúng điểm ghép. Tôi khuyến nghị giữ nguyên `RobotArm`/driver và `MotionExecution`; bổ sung một planner Cartesian độc lập, sau đó đưa `JointTrajectory` đã kiểm chứng vào executor hiện có.

Hiện tại planner chỉ là joint point-to-point: `q_start → q_goal`, nên tên `TrajectoryPlanner` là quá chung. [Contract hiện tại](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/port_interface/trajectory.py:8) và [request](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/core/trajectory_planning.py:199). Ngược lại, `myarm_motion_execution` đã là executor đúng vai trò: nhận trajectory, kiểm tra feedback, chạy FSM và là nơi duy nhất gửi setpoint private xuống driver. [Node hiện tại](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/ros2_ws/src/myarm_motion_execution/myarm_motion_execution/motion_execution_node.py:47)

```text
fresh measured q
  │
  ├─ FK(q) → TCP start thực
  │
  └─ Cartesian reference path
        │
        ├─ waypoint 0: seed = measured q
        └─ waypoint i: seed = q(i - 1)
                 │
                 ▼
        sequential DLS/CLIK IK + limit/singularity checks
                 │
                 ▼
        retime + dense validation q/qdot/qddot
                 │
                 ▼
        JointTrajectory
                 │
                 ▼
        /myarm/follow_joint_trajectory
                 │
                 ▼
        MyArmMotionExecutionNode → driver
```

## 1. Chuẩn hóa tên joint planner

Đây nên là một lần đổi tên atomic, không để alias `TrajectoryPlanner` mơ hồ.

| Hiện tại | Tên chốt |
|---|---|
| `TrajectoryPlannerInterface` | `JointTrajectoryPlannerInterface` |
| `TrajectoryPlannerService` | `JointTrajectoryPlannerService` |
| `TrajectoryPlanningRequest` | `JointTrajectoryPlanningRequest` |
| `TrajectoryPlanningResult` | `JointTrajectoryPlanningResult` |
| `TrajectoryPlanningFailureReason` | `JointTrajectoryPlanningFailureReason` |
| `core/trajectory_planning.py` | `core/joint_trajectory_planning.py` |
| `port_interface/trajectory.py` | `port_interface/joint_trajectory.py` |
| `service/trajectory.py` | `service/joint_trajectory.py` |
| `MinimumJerkJointTrajectoryAdapter` | `MinimumJerkJointTrajectoryPlannerAdapter` |
| `services.trajectory_planner` | `services.joint_trajectory_planner` |

`myarm_motion_execution` không nên đổi tên thành `myarm_joint_trajectory_planner`: package này thực sự vừa nhận `FollowJointTrajectory`, vừa chạy FSM/cancel/safety và stream setpoint. Tên `motion_execution` đang đúng. Bên trong node chỉ đổi field/config/diagnostic sang `joint_trajectory_planner`.

`plugin_adapter/trajectory/` vẫn có thể giữ tên chung vì nó sẽ chứa hai loại planner:

```text
trajectory/
  minimum_jerk_joint_trajectory_planner.py
  cartesian_sequential_clik_trajectory_planner.py
  config/
    minimum_jerk_joint.yaml
    cartesian_sequential_clik.yaml
```

## 2. Thành phần Cartesian mới trong pycore

Không nên nhét nó vào `KinematicsService`: service đó stateful, có pending target và semantics one-shot IK. Cartesian planner cần một lời gọi thuần, xác định rõ `q_start` và toàn bộ trajectory.

```text
core/
  cartesian_trajectory_planning.py
  joint_trajectory_interpolation.py

port_interface/
  cartesian_trajectory.py

service/
  cartesian_trajectory.py

plugin_adapter/trajectory/
  cartesian_sequential_clik_trajectory_planner.py
```

Các lớp chính:

```python
CartesianTrajectoryPlannerInterface
CartesianTrajectoryPlannerService
CartesianSequentialCLIKTrajectoryPlannerAdapter
```

Request nên chỉ nhận `q_start`, `target_pose`, `JointMotionLimits` và policy. TCP start luôn được suy ra bằng `FK(q_start)`, không nhận từ caller để tránh mismatch.

Result cần có:

```text
trajectory | None
succeeded
failure_reason
detail
failed_waypoint_index
resolved_duration_s
minimum_joint_limit_margin_rad
minimum_singular_value
maximum_position_residual_m
maximum_orientation_residual_rad
```

Planner inject `KinematicsInterface`, vốn đã đủ `forward()` và `solve_ik()`. [Interface](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/port_interface/kinematics.py:8) Node Cartesian sẽ sở hữu một instance Pinocchio riêng, không dùng chung instance mutable với `myarm_kinematics`.

Tên “sequential CLIK” ở đây có nghĩa là offline DLS/CLIK theo từng waypoint, seed của waypoint sau là nghiệm waypoint trước. Nó không phải online velocity controller 500 Hz.

## 3. Thuật toán và validation bắt buộc

Hai path mode nên được hỗ trợ rõ:

- `linear_translation_slerp` — mặc định: TCP XYZ đi thẳng, orientation quaternion SLERP.
- `se3_geodesic` — giống reBot: screw motion, endpoint đúng nhưng XYZ không nhất thiết đi thẳng.

Không dùng Euler ở bất kỳ bước nào. `POSITION_ONLY` là IK policy riêng; orientation của goal không được âm thầm thay bằng identity.

Trình tự:

1. Kiểm tra fresh measured `q`, hard limit và canonical joint order.
2. `FK(q_start)` lấy TCP start thực.
3. Sinh Cartesian reference theo minimum-jerk.
4. Giải IK tuần tự; lỗi tại một waypoint thì trả failure, không tạo trajectory một phần.
5. Kiểm tra residual, hard limit, software margin, singularity và branch discontinuity ở mọi waypoint.
6. Sinh `qdot`, `qddot`, rồi auto-stretch duration theo velocity/acceleration limits.
7. Dense-validate chính nội suy quintic mà executor sử dụng.

Bước 7 là rất quan trọng: executor hiện nội suy quintic giữa các point khi có đủ `qdot/qddot`. [Implementation](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/plugin_adapter/motion_execution/monotonic_time_motion_execution.py:609) `JointMotionLimits.trajectory_violations()` hiện chỉ kiểm tra các point đã xuất ra, nên trước Cartesian cần tách interpolation kernel dùng chung vào `core/joint_trajectory_interpolation.py`. Planner và executor phải dùng cùng công thức; nếu nội suy giữa point vượt limit hoặc lệch Cartesian reference thì reject plan.

Các failure reason tối thiểu:

```text
invalid_request
stale_measured_state
start_out_of_limit
unreachable
joint_limit_blocked
singular
near_singular
ik_timeout
ik_max_iterations
branch_discontinuity
retiming_failed
continuous_validation_failed
```

## 4. Config

Giữ đúng mô hình một runtime config hiện tại:

```yaml
services:
  joint_trajectory_planner:
    enabled: true
    plugin_adapter: minimum_jerk_joint
    plugin_config: plugin_adapter/trajectory/config/minimum_jerk_joint.yaml

  cartesian_trajectory_planner:
    enabled: false
    plugin_adapter: cartesian_sequential_clik
    plugin_config: plugin_adapter/trajectory/config/cartesian_sequential_clik.yaml
```

Cartesian adapter config chỉ chứa path, sampling, retiming, validation và policy override. Solver Pinocchio, URDF, base frame, tool frame, joint order vẫn lấy từ config kinematics/robot hiện hữu — không tạo URDF thứ hai hoặc copy limit sang YAML.

URDF baseline tiếp tục là nguồn chân lý cho joint order, position/velocity limit, `base_link` và `tool0`; acceleration limit vẫn ở trajectory config. Việc sau này thêm camera/workspace Xacro không làm ảnh hưởng Pinocchio, miễn là chain `base_link → tool0` và six arm joints baseline không đổi.

## 5. ROS 2 interface và package

Thêm package mới:

```text
myarm_cartesian_trajectory/
  myarm_cartesian_trajectory/
    cartesian_trajectory_node.py
    trajectory_preview_player_node.py
  launch/
    cartesian_trajectory.launch.py
  README.md
```

Node chính: `MyArmCartesianTrajectoryNode`.

Không dùng lại `/myarm/command/tcp_pose`; topic đó giữ cho one-shot IK của `myarm_kinematics`.

API production nên là custom action trong `myarm_interfaces`:

```text
/myarm/plan_cartesian_trajectory
```

Goal gồm `PoseStamped`, requested duration, path mode, task mode và time-scaling policy tối giản. Result chứa `JointTrajectory`, diagnostics và failure reason. Feedback báo planning progress/waypoint.

Sau khi preview ổn định mới thêm action thứ hai:

```text
/myarm/follow_cartesian_trajectory
```

Action này mới được phép gọi planner rồi gửi kết quả sang `/myarm/follow_joint_trajectory`. Không để Cartesian node publish trực tiếp `/myarm/internal/driver_joint_setpoint`.

Topic output đề xuất:

```text
/myarm/cartesian_trajectory/reference_path      nav_msgs/Path
/myarm/cartesian_trajectory/joint_preview       trajectory_msgs/JointTrajectory
/myarm/cartesian_trajectory/diagnostics         DiagnosticArray
```

Node phải transform `PoseStamped` về `base_link` bằng TF2 trước khi gọi pycore; core chỉ làm việc với `Pose` trong base frame. Nếu TF thiếu hoặc frame không hợp lệ, action fail rõ ràng.

## 6. Launch theo use case

| Launch | Thành phần | Mục đích |
|---|---|---|
| `myarm_joint_motion.launch.py` | driver + kinematics + motion execution + RSP | Luồng joint/IK hiện tại |
| `myarm_cartesian_preview.launch.py` | fake feedback + Cartesian planner + preview player + RSP | Xem path trong RViz, không chuyển động thật |
| `myarm_cartesian_fake_execution.launch.py` | fake driver + planner + motion execution + RSP | End-to-end không phần cứng |
| `myarm_cartesian_headless.launch.py` | driver + planner + executor, không RViz | Jetson/hardware workflow |
| `neugrasp_cartesian_preview.launch.py` | Neugrasp TF/camera + Cartesian preview | Scan/inference/visualization |
| `myarm_rviz2` remote launch | RViz duy nhất ở Host PC | Không sinh node điều khiển |

`myarm_system.launch.py` có thể nhận thêm `enable_cartesian_trajectory`; các launch trên là wrapper để tránh một launch quá nhiều cờ. Không được chạy hai `robot_state_publisher` hoặc hai publisher cùng `/joint_states`. Preview player phải dùng topic preview riêng khi driver thật đang publish actual state.

## 7. Điều nên lấy và không lấy từ reBot

Nên lấy:

- Minimum-jerk và SE(3) geodesic từ [sampler.py](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/trajectory/sampler.py:87).
- Seed tuần tự, DLS và tư duy null-space limit avoidance từ [clik_tracker.py](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/trajectory/clik_tracker.py:62).

Không copy trực tiếp:

- reBot chỉ xuất `q + ik_success`, không có `qdot/qddot`, retiming và all-or-nothing failure.
- Hàm của reBot đặt tên joint-space nhưng thực chất chuyển `q_end` thành pose rồi CLIK; không bảo đảm `q_final == q_goal`.
- ROS executor reBot nội suy tuyến tính nội bộ và không dùng đầy đủ derivative từ trajectory. [ros_actions.py](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArmController_ROS2/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py:134)

## 8. Thứ tự triển khai tôi đề xuất

1. Rename toàn bộ joint planner, config, test và README — không đổi hành vi.
2. Tách shared quintic interpolation/dense validation.
3. Thêm core types, port, service và Cartesian sequential CLIK adapter.
4. Unit test với fake kinematics, rồi test Pinocchio FK→Cartesian plan→FK.
5. Thêm `myarm_interfaces` action và `myarm_cartesian_trajectory`.
6. Thêm preview/RViz/bringup launch.
7. Chỉ sau fake integration mới cho phép handoff sang `FollowJointTrajectory`.

Điều kiện an toàn: planner action có thể chạy ở 5 Hz cho status/RViz, nhưng physical Cartesian execution không nên bật ở 5 Hz. Executor và driver cần một execution rate đã commission riêng, dense validation, tracking-error policy thực sự dừng robot, và vẫn chưa có collision checking. Vì vậy phase đầu nên là preview/fake/scan-replay; không bật `allow_physical_motion: true` chỉ vì Cartesian planner đã tồn tại.

Tôi chưa thay đổi tệp nào trong lượt rà soát này.