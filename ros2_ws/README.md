# ROS 2 workspace

Đặt các package ROS 2 trong `src/`.

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
colcon build --symlink-install
source install/setup.bash
```

Các thư mục `build/`, `install/` và `log/` là output tạm, không commit.

## Demo RViz2

Sau khi build, chạy:

```bash
source install/setup.bash
ros2 launch myarm_joint_state_demo demo.launch.py
```

Đây là demo mô phỏng joint state, không gửi lệnh đến robot thật.
