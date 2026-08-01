# myarm_joint_state_publisher

Bridge demo legacy này nhận `/myarm/command/joint_goal` từ kinematics service
và phát `/joint_states` ở 5 Hz. `robot_state_publisher` dùng topic chuẩn này
để phát TF cho RViz2.

`/joint_states` ở package này là visualization state từ **command**; không
được nối lại làm measured feedback/IK seed. Robot driver thật phải publish
canonical feedback riêng ở `/myarm/state/joint_state`. Không chạy package này
cùng `myarm_robot_driver`, vì driver production đã là publisher duy nhất của
actual-feedback `/joint_states`.

```bash
cd ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch myarm_joint_state_publisher command_publisher.launch.py
```
