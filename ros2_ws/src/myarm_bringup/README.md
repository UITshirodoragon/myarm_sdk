# MyArm bringup: launch và lệnh ROS 2

README này dùng runtime config mặc định
`service/config/services.yaml`: `FakeRobotArm`, `home = [0.0, -0.35, 0.70,
0.0, -0.35, 0.0]`, và không mở serial hardware. Nếu thay bằng profile vật lý,
hãy chỉ thực hiện lệnh chuyển động sau khi đã commission robot, bật quyền
motion một cách chủ động và re-arm driver.

Từ `ros2_ws/`, sau khi build workspace:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

Các ví dụ joint bên dưới luôn dùng thứ tự URDF canonical:

```text
shoulder_pan_joint, shoulder_lift_joint, elbow_flex_joint,
forearm_roll_joint, wrist_flex_joint, wrist_roll_joint
```

Mục tiêu nhỏ dùng trong ví dụ là `q = [0.02, -0.35, 0.70, 0.0, -0.35, 0.0]`.
Với URDF hiện tại, TCP tương ứng xấp xỉ `base_link` pose `(0.454585,
0.009093, 0.365635)` và quaternion `(0.007071, -0.707071, 0.007071,
0.707071)`.

## Quy tắc an toàn

- Không bao giờ publish trực tiếp vào
  `/myarm/internal/driver_joint_setpoint`; đó là private boundary giữa
  `myarm_motion_execution` và driver.
- Cartesian planner chỉ plan/preview. Nó không tự gửi kết quả sang
  `/myarm/follow_joint_trajectory` và không thể tự tạo chuyển động vật lý.
- Chưa có collision checking. Một kế hoạch IK thành công không phải là xác
  nhận an toàn va chạm.
- Với adapter vật lý, kiểm tra trước:

```bash
ros2 topic echo /myarm/robot/safety_state
ros2 topic echo /myarm/state/joint_state
```

Các lệnh `topic pub` chuyển động trong tài liệu này nên được thử trước với
`FakeRobotArm` mặc định.

## `myarm_joint_motion.launch.py`

Khởi động driver + one-shot kinematics + joint motion execution +
`robot_state_publisher`:

```bash
ros2 launch myarm_bringup myarm_joint_motion.launch.py
```

Gửi joint endpoint. Node sẽ lấy feedback mới nhất, lập minimum-jerk trajectory
và chỉ executor mới stream setpoint xuống driver:

```bash
ros2 topic pub --once /myarm/command/joint_goal sensor_msgs/msg/JointState \
"{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint], position: [0.02, -0.35, 0.70, 0.0, -0.35, 0.0]}"
```

Gửi TCP target cho one-shot IK. Target phải ở `base_link`; nghiệm IK sau đó đi
qua cùng `/myarm/command/joint_goal` và executor:

```bash
ros2 topic pub --once /myarm/command/tcp_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: base_link}, pose: {position: {x: 0.454585, y: 0.009093, z: 0.365635}, orientation: {x: 0.007071, y: -0.707071, z: 0.007071, w: 0.707071}}}"
```

Điều khiển gripper bằng độ mở **tổng giữa hai đầu gắp**, đơn vị mét:

```bash
ros2 topic pub --once /myarm/gripper/command std_msgs/msg/Float64 "{data: 0.08}"
ros2 topic pub --once /myarm/gripper/command std_msgs/msg/Float64 "{data: 0.0}"
```

## `myarm_cartesian_preview.launch.py`

Khởi động fake feedback + Cartesian planner + synthetic preview player +
`robot_state_publisher`; không có motion executor:

```bash
ros2 launch myarm_bringup myarm_cartesian_preview.launch.py
```

Cartesian input là ROS action, không phải topic. Điều này giữ result,
failure-reason và trajectory nguyên tử; vì vậy không có lệnh `ros2 topic pub`
để kích hoạt planner này. Gửi goal mặc định-policy và xem feedback:

```bash
ros2 action send_goal --feedback /myarm/plan_cartesian_trajectory \
  myarm_interfaces/action/PlanCartesianTrajectory \
"{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.454585, y: 0.009093, z: 0.365635}, orientation: {x: 0.007071, y: -0.707071, z: 0.007071, w: 0.707071}}}, requested_duration: {sec: 0, nanosec: 0}, path_mode: 0, task_mode: 0, time_scaling_mode: 0, speed_scale: 0.0, max_translation_step_m: 0.0, max_rotation_step_rad: 0.0}"
```

`path_mode: 0`, `task_mode: 0`, `time_scaling_mode: 0` lần lượt là
`PATH_DEFAULT`, `TASK_DEFAULT`, `TIME_DEFAULT`, tức dùng policy YAML. Kết quả
chỉ publish path và `/myarm/cartesian_trajectory/joint_preview`; preview player
là publisher duy nhất của `/joint_states` trong launch này.

Driver vẫn là fake, nên có thể kiểm thử transport gripper:

```bash
ros2 topic pub --once /myarm/gripper/command std_msgs/msg/Float64 "{data: 0.04}"
```

Lệnh `/myarm/command/joint_goal` không có consumer trong launch preview và sẽ
không tạo chuyển động.

## `myarm_cartesian_fake_execution.launch.py`

Khởi động fake driver + Cartesian planner + `FollowJointTrajectory` executor +
`robot_state_publisher`:

```bash
ros2 launch myarm_bringup myarm_cartesian_fake_execution.launch.py
```

Kiểm tra end-to-end joint execution trên fake robot:

```bash
ros2 topic pub --once /myarm/command/joint_goal sensor_msgs/msg/JointState \
"{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint], position: [0.02, -0.35, 0.70, 0.0, -0.35, 0.0]}"
```

Lập Cartesian plan bằng cùng action ở phần preview:

```bash
ros2 action send_goal --feedback /myarm/plan_cartesian_trajectory \
  myarm_interfaces/action/PlanCartesianTrajectory \
"{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.454585, y: 0.009093, z: 0.365635}, orientation: {x: 0.007071, y: -0.707071, z: 0.007071, w: 0.707071}}}, requested_duration: {sec: 0, nanosec: 0}, path_mode: 0, task_mode: 0, time_scaling_mode: 0, speed_scale: 0.0, max_translation_step_m: 0.0, max_rotation_step_rad: 0.0}"
```

Action plan ở trên vẫn **không** handoff tự động. Để test pose → fake execution
trong đúng tầng executor, gọi action riêng sau:

```bash
ros2 action send_goal --feedback /myarm/follow_cartesian_trajectory \
  myarm_interfaces/action/FollowCartesianTrajectory \
"{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.454585, y: 0.009093, z: 0.365635}, orientation: {x: 0.007071, y: -0.707071, z: 0.007071, w: 0.707071}}}, requested_duration: {sec: 0, nanosec: 0}, path_mode: 0, task_mode: 0, time_scaling_mode: 0, speed_scale: 0.0, max_translation_step_m: 0.0, max_rotation_step_rad: 0.0}"
```

`FollowCartesianTrajectory` plan, preflight và execute qua
`MotionExecutionService`; nó không publish vào private driver topic từ CLI.
Action này chỉ tồn tại trong fake-execution launch ở phase hiện tại.

## `myarm_cartesian_headless.launch.py`

Khởi động driver + Cartesian planner + executor + `robot_state_publisher`,
không chạy RViz cục bộ. TF vẫn có để remote RViz hoặc transform pose:

```bash
ros2 launch myarm_bringup myarm_cartesian_headless.launch.py
```

Với config fake mặc định, có thể dùng cùng joint command:

```bash
ros2 topic pub --once /myarm/command/joint_goal sensor_msgs/msg/JointState \
"{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint], position: [0.02, -0.35, 0.70, 0.0, -0.35, 0.0]}"
```

Hoặc lập plan Cartesian bằng action ở phần preview. Không publish
`/myarm/command/tcp_pose` ở launch này: one-shot `myarm_kinematics` đã được cố
ý tắt, để Cartesian planner là đường TCP planning duy nhất.

Nếu dùng profile vật lý, launch này không tự arm robot và Cartesian action vẫn
không tự execute. Chỉ gửi goal sau khi operator xác nhận state, driver đã
re-arm và policy transport cho phép physical motion.

## `myarm_system.launch.py`

Đây là composition tổng quát. Mặc định nó tương đương luồng joint: driver +
kinematics + executor + TF; Cartesian node mặc định tắt:

```bash
ros2 launch myarm_bringup myarm_system.launch.py
```

Do đó có thể dùng cả hai topic command của `myarm_joint_motion`:

```bash
ros2 topic pub --once /myarm/command/joint_goal sensor_msgs/msg/JointState \
"{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint], position: [0.02, -0.35, 0.70, 0.0, -0.35, 0.0]}"
```

```bash
ros2 topic pub --once /myarm/gripper/command std_msgs/msg/Float64 "{data: 0.04}"
```

Để mở thêm Cartesian planner trong composition này:

```bash
ros2 launch myarm_bringup myarm_system.launch.py \
  enable_cartesian_trajectory:=true
```

Sau đó dùng `ros2 action send_goal /myarm/plan_cartesian_trajectory ...` như
phần preview. Không cần và không nên chạy thêm một `robot_state_publisher`.

`FollowCartesianTrajectory` có cờ riêng và chỉ được phép với fake adapter ở
phase này:

```bash
ros2 launch myarm_bringup myarm_system.launch.py \
  enable_cartesian_execution:=true
```

Không dùng cờ này với profile `myarm_m750_robot_arm`; node sẽ fail-closed.

## Dừng, reset và quan sát

Các lệnh này không gửi target mới:

```bash
ros2 service call /myarm/motion_execution/cancel std_srvs/srv/Trigger "{}"
ros2 service call /myarm/motion_execution/reset std_srvs/srv/Trigger "{}"
ros2 service call /myarm/robot/stop std_srvs/srv/Trigger "{}"
ros2 topic echo /myarm/motion_execution/diagnostics
ros2 topic echo /myarm/cartesian_trajectory/diagnostics
```

`cancel`/`reset` chỉ có khi motion execution được khởi động. Cartesian preview
launch không có executor; `/myarm/robot/stop` chỉ tồn tại khi driver được bật.
