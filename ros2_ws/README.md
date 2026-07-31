# ROS 2 workspace

Đặt các package ROS 2 trong `src/`.

## Dùng Python core

Trước khi chạy node ROS 2 cần cài `pycore` vào đúng Python environment:

```bash
cd ..
python3 -m pip install -e '.[pycore]'
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
colcon build --symlink-install
source install/setup.bash
```

Các thư mục `build/`, `install/` và `log/` là output tạm, không commit.

## Setup DDS WLAN

```bash
source /opt/ros/foxy/setup.bash

export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Demo RViz2

Sau khi build, chạy:

```bash
source install/setup.bash
ros2 launch myarm_joint_state_demo demo.launch.py
```

Đây là demo mô phỏng joint state, không gửi lệnh đến robot thật.
