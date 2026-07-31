## Kết luận

reBot là một prototype điều khiển robot thực dụng hơn `myarm_sdk`: có đường lệnh xuống motor, các mode MIT/POS_VEL/VEL, vòng lặp điều khiển 500 Hz, IK/trajectory Cartesian và bù trọng lực. Tuy nhiên, nó chưa có kiến trúc an toàn đủ chặt cho robot thật: không có FSM chính thức, đồng bộ luồng yếu, xử lý lỗi/feedback không đáng tin cậy, và trajectory chưa được kiểm chứng ràng buộc động học.

Ngược lại, `myarm_sdk` có nền kiến trúc phần mềm sạch hơn — core/port/plugin/service, kiểu dữ liệu rõ ràng, Pinocchio backend và ROS validation tốt — nhưng hiện chủ yếu là stack kinematics/RViz, chưa phải motion-control stack nối robot thật.

Tôi chỉ đọc và phân tích mã; không thay đổi repo nào.

## 1. Chuỗi điều khiển thực tế của reBot

```text
Application / example
  → RebotArmEndPoseController
    → FK / IK Pinocchio
    → SE(3) trajectory sampler
    → sequential CLIK tracker
    → shared joint target q_target
    → control loop 500 Hz
    → MIT / POS_VEL / VEL command
    → motorbridge → actuator

feedback motor
  → JointGroup / cached state
  → q_now cho IK, gravity compensation, trajectory start
```

Điểm đáng chú ý: “CLIK tracking” trong reBot chủ yếu diễn ra lúc lập kế hoạch. Nó giải IK tuần tự cho từng waypoint Cartesian để sinh danh sách `q`. Khi chạy thực tế, hệ thống chủ yếu stream `q_target` xuống motor; không có vòng Cartesian closed-loop online so sánh pose thực tế với pose tham chiếu tại mỗi tick.

Nguồn chính: [EndPose controller](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/controllers/rebotarm_endpose_controller.py:270), [CLIK tracker](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/trajectory/clik_tracker.py:97), [control loop](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py:705).

## 2. Kinematics: reBot làm gì tốt và còn yếu ở đâu

Cả hai repo đều dùng URDF + Pinocchio, FK theo cây URDF và IK số dạng damped least squares; không phải analytic IK hay PoE solver viết tay.

reBot có pipeline IK thực dụng:

- Tạo sai số pose trong SE(3) bằng `log6`.
- Lấy Jacobian đầu công tác.
- Giải DLS để tính bước `dq`.
- Lặp nhiều bước, giới hạn joint và dùng nghiệm trước làm seed.
- Khi đi trajectory, dùng nghiệm waypoint trước làm seed waypoint sau để hạn chế nhảy branch IK.

Đây là ý tưởng tốt để mang sang `myarm_sdk`, nhất là interpolation trên SE(3) và seed liên tục.

Tuy vậy, implementation reBot có các rủi ro đáng kể:

- `target_rot=None` không thật sự là IK chỉ vị trí: code thay orientation bằng identity, nên robot vẫn bị ép về một hướng tool cụ thể. Xem [IK API](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/kinematics/inverse_kinematics.py:275).
- IK không dừng sớm ngay sau khi hội tụ trong loop, làm thống kê iteration và chi phí giải không chính xác.
- Hàm đọc joint limit index sai khi joint ID không trùng `idx_q`; mô hình DM có thể lỗi ở joint cuối. [Robot model](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/kinematics/robot_model.py:74).
- Mô hình RS có 8 DOF trong URDF gồm hai jaw prismatic, nhưng lớp arm/hardware chủ yếu coi robot là 6 joint + gripper; mapping tên/DOF chưa chặt.
- Non-finite limit bị thay bằng zero, không phù hợp nếu tương lai có continuous joint.

`myarm_sdk` hiện làm tốt hơn ở phần contract kinematics:

- Ép đúng 6 joint arm bằng reduced model.
- Có `Jlog6` correction trong IK, tách tolerance position/orientation.
- Có kiểu `JointPositions` radians và `Pose` metres + quaternion ROS.
- ROS node reject frame sai, NaN/Inf và quaternion không hợp lệ.

Nhưng SDK có ba điểm P0 cần chốt trước robot thật:

1. URDF active mô tả calibration q2/q3, nhưng adapter hardware hiện chỉ đổi radian–degree; nếu contract URDF đúng, mapping model ↔ robot thật đang thiếu offset. Sai lệch TCP có thể cỡ 54 mm ở một tư thế zero.  
2. Seed mặc định `zero` nằm tại wrist singularity rank-5; `home` an toàn hơn.  
3. TCP transform trong comment URDF và transform thật không thống nhất orientation.

Nguồn: [Pinocchio adapter SDK](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/plugin_adapter/kinematics/pinocchio_kinematics.py:99), [URDF calibration](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/ros2_ws/src/myarm_description/urdf/myarm_m750_poe_v3_2.urdf:9), [hardware adapter](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/plugin_adapter/robot_arm/myarm_m750_robot_arm.py:21).

## 3. Controllers của reBot

reBot có ba mode cấp thấp:

- `MIT`: gửi position target, velocity target, `kp`, `kd`, torque feed-forward.
- `POS_VEL`: position với giới hạn tốc độ.
- `VEL`: vận tốc.

Lớp `RebotArmEndPoseController` nối kinematics với hardware, duy trì target joint và gọi callback điều khiển định kỳ. Đây là giá trị lớn nhất của reBot: nó không chỉ “tính IK” mà đã nối từ TCP target tới actuator.

Một điểm đáng học là bù trọng lực:

```text
tau_command = gravity(q_measured) + torque feed-forward
```

Trong MIT mode, reBot dùng Pinocchio để tính generalized gravity. Đây là nền tảng tốt cho holding, gravity compensation và sau này impedance/compliance. [Gravity computation](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/dynamics/inverse_dynamics.py:89).

Nhưng không nên sao chép implementation nguyên trạng:

- `start()` khởi tạo `_q_target = 0` rồi enable và chạy loop mà không đọc pose hiện tại để “hold at current pose”. Robot đang ở tư thế khác zero có thể nhận lệnh quay về zero ngay sau enable. Đây là lỗi an toàn mức cao. [Controller initialization/start](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/controllers/rebotarm_endpose_controller.py:112).
- Feedback lỗi hoặc thiếu có thể bị thay bằng zero, khiến zero measurement không phân biệt được với communication failure. [State feedback](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py:615).
- Callback exception làm thread điều khiển chết, nhưng không có chuyển trạng thái `FAULT`, disable motor hay diagnostic đáng tin cậy.
- Trong MIT loop, `qdot_target` gần như luôn zero; minimum-jerk velocity không được đưa xuống low-level controller.
- Gravity torque có hệ số hard-code `1.55` cho q2/q3. Điều này cho thấy mô hình chưa khớp robot thật; nên biến nó thành calibration có đo đạc, không copy literal.
- `Pinocchio.Data` mutable dùng chung giữa foreground IK và background gravity loop, không lock.
- `JointGroup` có mode logic nhưng kết quả chuyển mode không được kiểm tra nghiêm ngặt trước khi tiếp tục enable/command.

Bài học cho SDK: cần có controller thật, nhưng controller phải lấy measured state có timestamp, có fault state, command ownership rõ ràng và không được nuốt lỗi.

## 4. Trajectory: điểm mạnh thực sự của reBot

Pipeline trajectory mặc định của reBot khá đúng hướng:

```text
q_start đo được
  → solve IK q_goal
  → FK tạo T_start, T_goal
  → nội suy geodesic trong SE(3)
  → minimum-jerk time profile
  → IK tuần tự từng pose
  → joint trajectory theo thời gian
```

Điểm nên học:

- Nội suy Cartesian bằng SE(3), không nội suy Euler angle.
- Minimum jerk là default tốt cho chuyển động point-to-point.
- Seed IK liên tiếp giúp tránh flip nghiệm.
- Tách tốc độ planner khỏi motor loop là hợp lý về kiến trúc.

Nhưng executor của reBot chưa đạt mức trajectory controller an toàn:

- Planner mặc định khoảng 100 Hz (`dt=0.01`), motor loop 500 Hz; waypoint bị phát lặp nhiều lần.
- Sender không dùng chính xác `point.time`; gửi theo `T/n`, nên timing có sai lệch.
- Trajectory chỉ chứa `q` + timestamp, không có `qdot`, `qddot`, jerk.
- Không kiểm tra joint velocity, acceleration, jerk, current, collision hay workspace constraints trước khi execute.
- Waypoint IK thất bại vẫn có thể được giữ trong trajectory nếu còn các waypoint khác.
- Trapezoid sampler có công thức normalization không đúng, có nguy cơ overshoot/discontinuity. [Sampler](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/trajectory/sampler.py:66).
- Khi trajectory cũ đang chạy, `move_to_traj()` lại lập kế hoạch mới trước rồi mới cancel sender cũ. Robot có thể đã di chuyển, trong khi plan mới dựa trên `q_start` cũ.
- `end()` gọi safe-home trong lúc sender trajectory vẫn có thể ghi đè `_q_target`.

`myarm_sdk` hiện chỉ có `LinearTrajectoryAdapter`: nội suy tuyến tính joint-space, service đang disable, không gắn IK/hardware/executor. [Linear trajectory](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/plugin_adapter/trajectory/linear_trajectory.py:10), [service config](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/pycore/src/myarm_sdk/service/config/services.yaml:47).

## 5. FSM: reBot thực tế không có FSM chính thức

Không có class FSM, enum state, transition guard, event table hay fault latch trong reBot. Nó dùng các cờ phân tán:

- `RebotArm`: `_connected`, `_running`
- `JointGroup`: `_mode`
- End-pose controller: `_running`, `_moving`, `_stop_send`, `_send_thread`

Có thể suy ra flow sau từ code, nhưng đây không phải FSM được implement:

```text
CONSTRUCTED
  → CONNECTED / ENABLED
  → HOLDING
  → PLANNING
  → EXECUTING
  → SAFE_HOME / DISCONNECT
```

Các vấn đề do thiếu FSM:

- `estop()` chỉ disable motor; không cancel trajectory, không clear target, không dừng loop, không latch trạng thái emergency. Khi enable lại, stale target có thể còn tồn tại. [E-stop](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py:672).
- Không có `FAULT` khi mất feedback, timeout, driver error hoặc thread control chết.
- Nhiều thread có thể cùng sửa `_q_target`, `_traj`, `_moving`.
- Không có policy preemption: lệnh IK mới, trajectory cũ, safe-home và stop có thể tranh quyền command.
- Không có command queue, goal ID, cancel acknowledgement hay feedback/result lifecycle.

Cả reBot và `myarm_sdk` đều thiếu FSM. Khác biệt là reBot đã có “state phân tán”, còn SDK gần như chỉ có service placeholder.

FSM nên có tối thiểu:

```text
DISCONNECTED → CONNECTING → DISABLED → ENABLING → HOLDING/READY
HOLDING → PLANNING → EXECUTING → HOLDING
bất kỳ lỗi nào → FAULT
E-stop → ESTOP_LATCHED
FAULT / ESTOP_LATCHED → chỉ reset rõ ràng mới được recovery
```

Nguyên tắc quan trọng: chỉ FSM/executor được quyền ghi command cuối cùng xuống robot; planner, UI, ROS callback không trực tiếp tranh nhau ghi `q_target`.

## 6. So sánh trực tiếp

| Hạng mục | reBot | myarm_sdk hiện tại | Nên mang sang SDK |
|---|---|---|---|
| Kinematics | Pinocchio, DLS IK, FK, Jacobian, SE(3) | Pinocchio, DLS IK tốt hơn về contract | Giữ backend SDK, thêm singularity/limit diagnostics |
| Hardware path | Có motor modes và loop thực | Adapter basic, ROS node chưa điều khiển robot thật | Nối feedback + command thật qua port rõ ràng |
| Controller | MIT/POS_VEL/VEL, gravity FF | `MemoryControllerAdapter`, service placeholder | Controller execution layer có measured state |
| Cartesian motion | SE(3) + min-jerk + sequential IK | Chỉ linear joint-space, disabled | Cartesian planner + time parameterization |
| FSM/lifecycle | Có state ngầm, không an toàn | Hầu như chưa có | FSM explicit, fault/e-stop latch, preemption |
| Safety | Enable/disable nhưng lỗi handling yếu | Chưa có API safety | Watchdog, fault model, stale feedback timeout |
| Dynamics | Có gravity compensation | Chưa có | Chỉ thêm sau calibration thực tế |
| Kiến trúc code | Gắn sát hardware/global config | Core/port/plugin/service sạch | Giữ kiến trúc SDK, không copy global/thread model |
| Test | Hầu như examples/manual | Test fake service là chính | Golden FK/IK, limits, singularity, calibration, hardware-in-loop |

## 7. Những gì `myarm_sdk` nên học từ reBot — và không nên học

Nên học ý tưởng:

1. Chuỗi hoàn chỉnh từ pose TCP đến actuator, không dừng ở việc publish RViz.
2. Cartesian SE(3) trajectory thay vì chỉ nội suy `q`.
3. Seed IK theo measured joint state hoặc waypoint trước.
4. Minimum-jerk/time scaling và validation trước execution.
5. Gravity feed-forward, nhưng chỉ sau khi mô hình mass/COM/friction và mapping joint đã được đo thực.
6. Tách planner rate, executor rate và low-level motor control.

Không nên copy:

1. Cờ trạng thái phân tán thay cho FSM.
2. Nhiều thread cùng ghi target.
3. Nuốt communication error rồi thay bằng zero.
4. Hard-code gain/torque scale theo joint.
5. Global config khiến model URDF và hardware config có thể lệch nhau.
6. “First N joints” hoặc padding joint vector thay vì mapping theo tên.
7. E-stop không latch và không có recovery procedure.

## 8. Những phần `myarm_sdk` còn thiếu

Ưu tiên hợp lý là:

1. P0 — Physical truth  
   Chốt mapping encoder/hardware ↔ `q_ros`, offset q2/q3, joint direction, limit thật, TCP transform và base frame. State phải có timestamp/freshness.

2. P0 — Robot-control port đầy đủ  
   Không chỉ `read_joints()`/`move_joints()`, mà cần state gồm `q`, `qdot`, effort/fault/temperature/timestamp; command gồm position, velocity, optional torque FF; kèm `enable`, `disable`, `stop`, `estop`, `reset_fault`, `set_mode`.

3. P1 — FSM và command ownership  
   Một executor duy nhất nhận goal, cancel, preemption, timeout và fault transition.

4. P1 — Trajectory executor  
   Kiểm tra limit q/qd/qdd/jerk, monotonic timing, actual feedback tracking, stop policy. Với ROS 2, dùng Action cho goal/feedback/result/cancel sẽ phù hợp hơn timer callback tự phát; tài liệu SDK cũng đã nêu hướng này. [ROS Action note](/home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk/docs/rclpy_foxy_cheatsheet_vi.md:511).

5. P2 — Cartesian planning  
   SE(3) interpolation + seeded IK từng waypoint + collision/workspace checking nếu có collision model.

6. P2 — Dynamics/control nâng cao  
   Gravity compensation, friction/coulomb compensation, impedance/admittance chỉ sau calibration và hardware safety.

7. P1 xuyên suốt — Test thực  
   Golden FK, FK→IK→FK, unreachable target, gần singularity, near-limit, tool transform, q2/q3 calibration, stale feedback, e-stop, cancel/preemption và hardware-in-loop.

Tóm lại: hãy lấy reBot làm tài liệu tham khảo tốt cho “robot thật cần gì ngoài IK”, nhưng xây phần đó theo kiến trúc sạch của `myarm_sdk`: typed contracts, plugin adapter, explicit FSM, một command executor duy nhất và test/calibration trước khi cấp quyền điều khiển motor.


Nên đi theo thứ tự: **làm kinematics đáng tin trước, rồi làm trajectory joint-space an toàn, sau đó mới đưa Cartesian trajectory/CLIK của reBot vào.** Không nên port nguyên các module reBot.

## Giai đoạn 1 — Bắt buộc làm trước

### 1. Chốt “sự thật vật lý” của kinematics

Giữ `PinocchioKinematicsAdapter` của `myarm_sdk`, không thay bằng `inverse_kinematics.py` của reBot.

Cần hoàn thiện trước:

- Mapping encoder/hardware ↔ joint model, đặc biệt offset q2/q3 trong URDF.
- Joint order, chiều dương, joint limit, base frame và TCP/tool frame.
- Dùng `q` đo thực từ robot làm seed IK, không dùng “last commanded q” hoặc luôn dùng zero.
- Đổi initial seed từ `zero` sang `home` hoặc current feedback vì zero đang ở wrist singularity.
- Test FK/IK thật: FK→IK→FK, gần limit, target không tới được, singularity q5≈0.

Đây là điều kiện tiên quyết. Nếu model sai 1–2 cm hoặc joint offset sai, trajectory dù đẹp vẫn đi sai ngoài đời.

### 2. Làm IK SDK đủ dùng cho motion planning

Bản IK hiện tại của SDK đã là nền tốt hơn reBot. Chỉ cần bổ sung contract rõ hơn:

```text
IK input:
  target pose + measured seed + policy

IK output:
  q solution
  converged / failure reason
  position residual
  orientation residual
  iteration count
  near-singularity metric
```

Nên thêm ngay:

- Kiểm tra limit trước/sau giải.
- Báo lỗi rõ: unreachable, joint-limit blocked, singular/near-singular, timeout.
- Position-only IK phải là mode riêng hoặc mask orientation; không được thay orientation bằng identity như reBot.
- Cấu hình damping, tolerance, max iterations qua YAML.
- Dùng quaternion/SE(3) xuyên suốt; không nội suy Euler.

Chưa cần làm ngay:

- Analytic IK.
- Null-space optimization phức tạp.
- Collision avoidance.
- Dynamics/gravity compensation.
- Online CLIK 500 Hz.

## Giai đoạn 2 — Trajectory đầu tiên nên là joint-space

Trước khi làm Cartesian trajectory, hãy nâng `LinearTrajectoryAdapter` thành trajectory joint-space có giới hạn động học.

Nên xây:

```text
JointTrajectoryPlanner
  input: q_start, q_goal, v_limit, a_limit, duration/policy
  output: time, q, qdot, qddot
```

Dùng quintic/minimum-jerk trước. Nó đơn giản, mượt và phù hợp cho move point-to-point:

- `q(t)` liên tục.
- `qdot(t)` bắt đầu/kết thúc bằng 0.
- `qddot(t)` bắt đầu/kết thúc bằng 0.
- Kiểm tra mọi điểm không vượt joint limit, velocity và acceleration limit.
- Timestamp phải tăng đơn điệu.
- Nếu không thỏa limit, tự kéo dài duration; không được vẫn gửi trajectory lỗi.

Đây là phần nên lấy ý tưởng từ reBot: minimum-jerk profile. Nhưng không copy trực tiếp `sampler.py`, vì trapezoid profile ở đó có lỗi normalization/timing và trajectory của reBot không mang đầy đủ `qdot`, `qddot`.

Mục tiêu giai đoạn này là:

```text
Current measured q
  → joint minimum-jerk planner
  → validated trajectory
  → preview / simulation
  → executor sau này
```

Chưa cần Cartesian IK theo từng điểm. Điều này giúp bạn kiểm chứng hardware mapping, limits và execution trước.

## Giai đoạn 3 — Sau khi joint trajectory ổn, thêm Cartesian trajectory

Đây là phần đáng học nhất từ reBot:

```text
T_start → T_goal
  → SE(3) geodesic interpolation
  → minimum-jerk timing
  → IK tuần tự từng waypoint
  → joint trajectory q, qdot, qddot
```

Các thành phần nên đưa vào SDK:

1. `CartesianTrajectoryPlanner`
2. Nội suy translation + rotation trên SE(3), không Euler.
3. Lấy nghiệm IK waypoint trước làm seed waypoint sau.
4. Nếu một waypoint IK thất bại: fail toàn bộ plan, không execute partial path.
5. Kiểm tra q/qd/qddot sau khi IK.
6. Có chế độ preview: publish trajectory/RViz trước, chưa điều khiển robot.

Pseudo-API nên hướng đến:

```python
plan_joint(start_q, goal_q, limits) -> JointTrajectory

plan_cartesian(
    start_q,
    target_pose,
    duration,
    orientation_policy,
    limits,
) -> JointTrajectory
```

`start_q` phải lấy từ feedback robot thật, không phải từ target cũ.

## Chưa nên đưa vào ngay từ reBot

| Thành phần reBot | Quyết định | Lý do |
|---|---|---|
| Pinocchio FK/IK | Giữ SDK hiện tại | SDK đã có backend sạch hơn |
| SE(3) Cartesian interpolation | Đưa vào giai đoạn 3 | Rất đáng dùng |
| Minimum-jerk | Đưa vào giai đoạn 2 | Phù hợp joint motion trước |
| Sequential seeded IK | Đưa vào giai đoạn 3 | Giúp trajectory không nhảy branch |
| `clik_tracker.py` nguyên trạng | Không port | Thiếu validation và có thể giữ waypoint lỗi |
| `sampler.py` nguyên trạng | Không port | Có vấn đề trapezoid/timing |
| Background sender thread | Không port | Dễ race target/preemption |
| Gravity compensation | Để sau | Cần calibration dynamics thật |
| Online CLIK servo | Để sau nữa | Cần feedback tốt, controller/FSM/watchdog |

## Roadmap ngắn gọn

```text
P0: Calibrate model ↔ hardware
P1: Harden FK/IK + test thật
P2: Joint minimum-jerk trajectory + q/qd/qdd limits
P3: Cartesian SE(3) trajectory + seeded IK
P4: Trajectory executor có feedback/cancel
P5: Online CLIK, collision, dynamics, gravity compensation
```

Nếu chỉ chọn một việc để bắt đầu ngay: **sửa/chốt mapping kinematics và thêm test FK→IK→FK thực**, sau đó triển khai **joint-space minimum-jerk trajectory**. Cartesian trajectory của reBot chỉ nên vào sau khi hai phần này đã xác minh ổn định.