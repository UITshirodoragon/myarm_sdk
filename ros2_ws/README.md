# ROS 2 workspace

Đặt các package ROS 2 trong `src/`.

## Dùng Python core

Trước khi build/chạy node ROS 2 cần cài `pycore` vào đúng Python environment.
Trong project này đó là `../myarm_venv`; venv này cần thấy các package ROS 2
(ví dụ được tạo với `--system-site-packages`).

```bash
source /opt/ros/<distro>/setup.bash
cd ..
./install.sh --kinematics
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
`myarm_kinematics` được tạo với cùng interpreter đã cài `myarm_sdk`.

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
ros2 launch myarm_kinematics ik_rviz_remote.launch.py
```

Kinematics chạy qua `KinematicsService` ở 5 Hz. Bridge state hiện tại vẫn là
mô phỏng, không gửi lệnh đến robot thật.
