# myarm_joint_state_publisher

Bridge demo này nhận `/myarm/command/joint_target` từ kinematics service và
phát `/joint_states` ở 5 Hz. `robot_state_publisher` dùng topic chuẩn này để
phát TF cho RViz2.

```bash
cd ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch myarm_joint_state_publisher command_publisher.launch.py
```
