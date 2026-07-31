# MyArm M750 kinematics

This package runs the ROS 2 boundary for `myarm_sdk.service.KinematicsService`.
The service configuration is packaged with pycore at
`myarm_sdk/service/config/services.yaml`.

By default IK uses fresh canonical model-space feedback from
`/myarm/state/joint_state` as its seed. It does **not** use `/joint_states`,
because that topic is produced by the RViz visualization bridge from a command,
not from the physical arm.

Run the complete Jetson-side visualization chain:

```bash
ros2 launch myarm_kinematics ik_rviz_remote.launch.py
```

Send a TCP target in `base_link`:

```bash
# A real robot driver should publish this topic continuously. This one-shot
# message is only useful for an offline visualization check.
ros2 topic pub --once /myarm/state/joint_state sensor_msgs/msg/JointState \
"{name: [shoulder_pan_joint, shoulder_lift_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint], position: [0.0, -0.35, 0.70, 0.0, -0.35, 0.0]}"

ros2 topic pub --once /myarm/command/tcp_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.05, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Outputs:

- `/myarm/command/joint_target`: only a new, validated IK solution.
- `/myarm/state/tcp_pose`: FK of measured feedback.
- `/myarm/kinematics/commanded_tcp_pose`: FK of the last safe command.
- `/myarm/kinematics/ik_status` (`DiagnosticArray`): convergence, reason,
  residuals, iterations, singularity metrics and active joint limits.

For a simulation-only loop, change `services.yaml` seed source to
`last_commanded`. Production operation should keep `measured_joint_state` and
make the driver convert hardware-space q to canonical URDF q before publishing.
