# ROS 2 packages

Mỗi thư mục con là một ROS 2 package độc lập. Build từ `ros2_ws/` bằng
`colcon build --symlink-install`.

`myarm_cartesian_trajectory` là boundary plan/preview: action
`/myarm/plan_cartesian_trajectory` nhận TCP pose, yêu cầu `/myarm/state/joint_state`
mới, rồi trả/publish một `trajectory_msgs/JointTrajectory` đã được validate.
Nó không có driver publisher và không thể tự chuyển động robot. Các launch là:

Lệnh khởi động và các ví dụ `ros2 topic pub`/action cho toàn bộ launch trong
`myarm_bringup` nằm tại [myarm_bringup/README.md](myarm_bringup/README.md).

- `myarm_bringup myarm_joint_motion.launch.py`: driver + kinematics + joint
  motion execution, không RViz.
- `myarm_bringup myarm_cartesian_headless.launch.py`: driver + Cartesian
  planning + joint executor + `robot_state_publisher`, không RViz cục bộ. TCP
  plan vẫn không tự handoff sang executor; ứng dụng phải gọi explicit
  `FollowJointTrajectory`.
- `myarm_bringup myarm_cartesian_preview.launch.py`: fake feedback + Cartesian
  planning + synthetic `/joint_states` + robot_state_publisher.
- `myarm_bringup myarm_cartesian_fake_execution.launch.py`: fake driver +
  Cartesian planner + existing joint executor + robot_state_publisher. Một
  client có thể gọi explicit executor-owned
  `FollowCartesianTrajectory`; action plan-only vẫn không tự execute.
- `neugrasp_bringup neugrasp_cartesian_preview.launch.py`: cùng preview với
  Neugrasp scene frames.
- `myarm_rviz2 cartesian_trajectory_rviz_remote.launch.py`: chỉ RViz ở máy
  remote; TF, state và path vẫn do robot-side host xuất qua DDS.
