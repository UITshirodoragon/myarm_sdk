# MyArm M750 kinematics

This package runs the ROS 2 boundary for `myarm_sdk.service.KinematicsService`.
The service configuration is packaged with pycore at
`myarm_sdk/service/config/services.yaml`.

Run the complete Jetson-side visualization chain:

```bash
ros2 launch myarm_kinematics ik_rviz_remote.launch.py
```

Send a TCP target in `base_link`:

```bash
ros2 topic pub --once /myarm/command/tcp_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.05, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
