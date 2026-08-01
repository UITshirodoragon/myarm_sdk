# neugrasp_bringup

Neugrasp application composition for MyArm M750. It owns application scene
frames and uses an Xacro wrapper only for optional robot-mounted fixed hardware.
The canonical URDF used by pycore and Pinocchio remains unchanged.

Robot-side launch (Jetson):

```bash
ros2 launch neugrasp_bringup neugrasp_system.launch.py
```

Start a wrist-camera TF subtree only after supplying verified mount calibration:

```bash
ros2 launch neugrasp_bringup neugrasp_system.launch.py \
  use_wrist_camera:=true \
  wrist_camera_mount_xyz:="x y z" \
  wrist_camera_mount_rpy:="roll pitch yaw"
```

Remote viewer only (Host PC):

```bash
ros2 launch neugrasp_bringup neugrasp_rviz_remote.launch.py
```

The remote launch intentionally starts no `robot_state_publisher`; it consumes
`/robot_description`, `/tf` and `/tf_static` from the robot-side host.

`config/neugrasp_scene.yaml` owns only static non-robot frames. Do not add
robot links, wrist-camera frames created by Xacro, or dynamic `selected_grasp`
to that file.
