# ROS 2 workspace

Đặt các package ROS 2 trong `src/`.

## Dùng Python core

Trước khi build/chạy node ROS 2 cần cài `pycore` vào đúng Python environment.
Trong project này đó là `../myarm_venv`; venv này cần thấy các package ROS 2
(ví dụ được tạo với `--system-site-packages`).

```bash
source /opt/ros/<distro>/setup.bash
cd ..
./install.sh --robot-arm-kinematics
../myarm_venv/bin/python -c "import myarm_sdk, rclpy; print('SDK and ROS Python are ready')"
cd ros2_ws
```

Sau bước này, các node ROS 2 có thể import `myarm_sdk` như một Python package
bình thường.

## Cài dependency

ROS 2 và `rosdep` được cài theo bản phân phối Linux/ROS 2 đang sử dụng, không
cài `rclpy` bằng pip.

```bash
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
source /opt/ros/<distro>/setup.bash
../myarm_venv/bin/python -m colcon build --symlink-install
source install/setup.bash
```

Dùng `../myarm_venv/bin/python -m colcon` bảo đảm console script của
`myarm_kinematics`, `myarm_motion_execution` và `myarm_robot_driver` được tạo
với cùng interpreter đã cài `myarm_sdk`.

Các thư mục `build/`, `install/` và `log/` là output tạm, không commit.

## Setup DDS WLAN

```bash
source /opt/ros/foxy/setup.bash

export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Runtime profiles

`myarm_bringup` là launch production trên Jetson. Nó sở hữu đúng một
`robot_state_publisher`, có thể bật/tắt độc lập driver, kinematics, motion
execution và TF publisher, và không khởi chạy RViz hay demo bridge cũ.

### Jetson: fake robot + kinematics + motion execution + TF

Đây là profile mặc định an toàn để kiểm tra ROS/DDS và RViz mà không mở serial
hay gửi lệnh tới robot thật. `FakeRobotArm` lưu đúng executor setpoint đã được
chấp nhận rồi publish nó thành feedback canonical.

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch myarm_bringup myarm_system.launch.py
```

`/myarm/state/joint_state` là feedback authoritative canonical model-space.
Driver đồng thời publish `/joint_states` chỉ cho `robot_state_publisher` và
RViz. Không chạy thêm `myarm_joint_state_publisher` cùng profile này vì sẽ tạo
hai publisher cho `/joint_states`.

### Host PC: RViz2 từ xa

Sau khi Jetson đang chạy TF qua DDS cùng `ROS_DOMAIN_ID`, chạy trên host:

```bash
source /opt/ros/<distro>/setup.bash
source install/setup.bash
ros2 launch myarm_rviz2 myarm_rviz2.launch.py
```

Host chỉ chạy RViz2; `robot_state_publisher` vẫn ở Jetson nên TF tồn tại ngay
cả khi RViz khởi động muộn.

### Các tổ hợp không cần RViz hoặc robot thật

```bash
# Robot driver + kinematics + executor, không publish TF/RViz
ros2 launch myarm_bringup myarm_system.launch.py enable_robot_state_publisher:=false

# Chỉ kinematics + TF; model vẫn xuất hiện nhưng không có feedback/animation
ros2 launch myarm_bringup myarm_system.launch.py enable_driver:=false
```

Để sử dụng MyArm M750 thật, đổi `services.robot_arm.plugin_adapter` sang
`myarm_m750_robot_arm` và chọn profile serial tương ứng trong
`myarm_sdk/service/config/services.yaml`. Driver chỉ đọc/publish feedback mặc
định; nó không subscribe public direct-joint target. Production motion goes
through `myarm_motion_execution`, and physical setpoints remain disabled until
the operator explicitly enables them.

```yaml
services:
  robot_arm:
    plugin_adapter: myarm_m750_robot_arm
    plugin_config: plugin_adapter/robot_arm/config/myarm_m750_robot_arm.yaml
    transport:
      accept_internal_setpoints: true
      allow_physical_motion: false  # feedback-only, vẫn an toàn
```
