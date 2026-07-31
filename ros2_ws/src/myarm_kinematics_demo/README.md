# MyArm M750 IK/FK remote-RViz demo

Run this launch file on the Jetson. It loads the sole canonical URDF from
`myarm_description`, receives a target in `base_link`, computes Pinocchio IK,
checks the result with FK, and publishes `/joint_states` for RViz over DDS.

```bash
ros2 launch myarm_kinematics_demo ik_rviz_remote.launch.py
```

Run RViz2 on the remote host as usual, then send a reachable target from the
Jetson:

```bash
ros2 topic pub --once /myarm/command/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.05, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

The bridge in `myarm_joint_state_publisher` is demo-only. Replace it with
hardware feedback when a real robot driver is added.
