# myarm_joint_state_demo

Demo không cần robot thật: publisher phát `joint_states`,
`robot_state_publisher` phát TF từ URDF và RViz2 hiển thị robot.

```bash
cd ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch myarm_joint_state_demo demo.launch.py
```
