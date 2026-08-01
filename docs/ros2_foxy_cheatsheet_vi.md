# ROS 2 Foxy Cheatsheet

> **Phạm vi:** ROS 2 Foxy Fitzroy trên Ubuntu 20.04, dùng cho phát triển node Python/C++, workspace `colcon`, giao tiếp DDS, TF2, URDF, RViz2 và robot arm.
>
> **Lưu ý:** Foxy đã **End-of-Life (EOL)** và không còn được ROS 2 hỗ trợ chính thức. Với Jetson chạy Ubuntu 20.04, nên khóa dependency, lưu image/container tái lập được và tránh nâng cấp package tùy tiện.

---

## 1. Mô hình tư duy nhanh

```text
Workspace
└── packages
    ├── executables
    │   └── nodes
    ├── launch files
    ├── config YAML
    ├── interfaces: msg / srv / action
    └── libraries

ROS graph
├── nodes
├── topics       publish/subscribe, dữ liệu liên tục
├── services     request/response, tác vụ ngắn
├── actions      goal/feedback/result, tác vụ dài và có thể hủy
├── parameters   cấu hình của node
└── TF tree      quan hệ giữa các coordinate frame
```

### Chọn cơ chế giao tiếp

| Nhu cầu | Nên dùng |
|---|---|
| Joint state, camera, IMU, diagnostics liên tục | Topic |
| Power on/off, reset, clear error | Service |
| Chạy trajectory, homing, pick-and-place | Action |
| Port, baudrate, rate, frame ID, offset | Parameter |
| Quan hệ không gian giữa base, tool, camera | TF2 |
| Ghi lại dữ liệu để debug/benchmark | rosbag2 |

---

## 2. Environment

### Source ROS 2 Foxy

```bash
source /opt/ros/foxy/setup.bash
```

Source workspace sau khi build:

```bash
cd ~/your_ws
source install/setup.bash
```

Kiểm tra:

```bash
echo "$ROS_DISTRO"
echo "$ROS_VERSION"
echo "$AMENT_PREFIX_PATH"
printenv | grep -E 'ROS|RMW|AMENT|COLCON'
```

Kết quả mong đợi:

```text
ROS_DISTRO=foxy
ROS_VERSION=2
```

### Tự động source Foxy

```bash
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

Không nên tự động source nhiều workspace không liên quan trong `.bashrc`, vì dễ tạo overlay lẫn nhau.

### Alias hữu ích

```bash
alias sfoxy='source /opt/ros/foxy/setup.bash'
alias sws='source install/setup.bash'
alias cb='colcon build --symlink-install'
alias cbt='colcon build --symlink-install --event-handlers console_direct+'
```

---

## 3. Workspace và overlay

Cấu trúc chuẩn:

```text
my_ws/
├── src/
├── build/
├── install/
└── log/
```

Tạo workspace:

```bash
mkdir -p ~/my_ws/src
cd ~/my_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

- `/opt/ros/foxy` là **underlay**.
- `~/my_ws/install` là **overlay**.
- Source underlay trước, overlay sau.

---

## 4. Package

### Tạo package Python

```bash
cd ~/my_ws/src

ros2 pkg create my_robot_driver \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs diagnostic_msgs
```

### Tạo package C++

```bash
cd ~/my_ws/src

ros2 pkg create my_robot_driver_cpp \
  --build-type ament_cmake \
  --dependencies rclcpp sensor_msgs diagnostic_msgs
```

### Tìm package

```bash
ros2 pkg list
ros2 pkg list | grep myarm
ros2 pkg prefix myarm_m750_driver
ros2 pkg executables myarm_m750_driver
ros2 pkg xml myarm_m750_driver
```

### Chạy executable

```bash
ros2 run <package_name> <executable_name>
```

Ví dụ:

```bash
ros2 run myarm_m750_driver driver_node
```

---

## 5. Dependency với rosdep

Khởi tạo một lần trên máy:

```bash
sudo rosdep init
rosdep update
```

Cài dependency của toàn workspace:

```bash
cd ~/my_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro foxy \
  -r \
  -y
```

Kiểm tra dependency nhưng không cài:

```bash
rosdep check --from-paths src --ignore-src --rosdistro foxy
```

Giải thích:

- `--from-paths src`: đọc package trong `src`.
- `--ignore-src`: không cài dependency đã có dưới dạng source package.
- `--rosdistro foxy`: resolve theo Foxy.
- `-r`: tiếp tục với package còn lại nếu có lỗi.
- `-y`: tự động xác nhận.

---

## 6. Build với colcon

### Build toàn workspace

```bash
cd ~/my_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Build một package

```bash
colcon build \
  --symlink-install \
  --packages-select myarm_m750_driver
```

### Build package và dependency của nó

```bash
colcon build \
  --symlink-install \
  --packages-up-to myarm_m750_bringup
```

### Hiện output trực tiếp

```bash
colcon build \
  --symlink-install \
  --event-handlers console_direct+
```

### Build CMake với compile commands

```bash
colcon build \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Xóa build cũ

> Kiểm tra chắc chắn đang đứng đúng workspace trước khi chạy.

```bash
pwd
rm -rf build install log
colcon build --symlink-install
```

### Liệt kê package được colcon nhận diện

```bash
colcon list
colcon list | grep myarm
colcon info myarm_m750_driver
```

---

## 7. Test

```bash
colcon test
colcon test-result
colcon test-result --verbose
```

Test một package:

```bash
colcon test --packages-select myarm_m750_driver
colcon test-result --verbose
```

Chạy lại test đã fail:

```bash
colcon test --packages-select myarm_m750_driver
```

---

## 8. ROS 2 CLI tổng quát

```bash
ros2 --help
ros2 <command> --help
```

Các command quan trọng:

```text
ros2 bag
ros2 daemon
ros2 doctor
ros2 interface
ros2 launch
ros2 lifecycle
ros2 node
ros2 param
ros2 pkg
ros2 run
ros2 service
ros2 action
ros2 topic
```

---

## 9. Node

```bash
ros2 node list
ros2 node info /node_name
```

Ví dụ:

```bash
ros2 node info /myarm_driver
ros2 node info /robot_state_publisher
```

Đổi tên node khi chạy:

```bash
ros2 run my_package my_node \
  --ros-args \
  -r __node:=new_node_name
```

Đặt namespace:

```bash
ros2 run my_package my_node \
  --ros-args \
  -r __ns:=/myarm_01
```

Đổi cả tên và namespace:

```bash
ros2 run my_package my_node \
  --ros-args \
  -r __node:=driver \
  -r __ns:=/myarm_01
```

---

## 10. Topic

### Liệt kê và kiểm tra

```bash
ros2 topic list
ros2 topic list -t
ros2 topic type /joint_states
ros2 topic info /joint_states
ros2 topic echo /joint_states
```

Tìm topic theo kiểu:

```bash
ros2 topic find sensor_msgs/msg/JointState
```

### Đo tần số và bandwidth

```bash
ros2 topic hz /joint_states
ros2 topic bw /camera/image_raw
```

### Publish một lần

```bash
ros2 topic pub --once \
  /test_message \
  std_msgs/msg/String \
  "{data: 'hello'}"
```

### Publish định kỳ

```bash
ros2 topic pub \
  --rate 5 \
  /test_message \
  std_msgs/msg/String \
  "{data: 'hello at 5 Hz'}"
```

### Ví dụ JointState chỉ dùng khi test offline

```bash
ros2 topic pub --once \
  /test_joint_states \
  sensor_msgs/msg/JointState \
  "{
    name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    position: [0.0, 0.2, -0.1, 0.0, 0.3, 0.0]
  }"
```

Không publish giả vào `/joint_states` khi driver thật đang chạy, vì sẽ tạo nhiều nguồn state cho cùng robot.

---

## 11. Service

### Liệt kê và kiểm tra

```bash
ros2 service list
ros2 service list -t
ros2 service type /myarm/power_on
ros2 service find std_srvs/srv/Trigger
```

Xem cấu trúc service:

```bash
ros2 interface show std_srvs/srv/Trigger
```

Gọi service:

```bash
ros2 service call \
  /myarm/power_on \
  std_srvs/srv/Trigger \
  "{}"
```

Ví dụ SetBool:

```bash
ros2 service call \
  /feature_enable \
  std_srvs/srv/SetBool \
  "{data: true}"
```

Service phù hợp với thao tác ngắn; không dùng service như control loop liên tục.

---

## 12. Action

### Liệt kê và kiểm tra

```bash
ros2 action list
ros2 action list -t
ros2 action info /arm_controller/follow_joint_trajectory
```

Xem interface:

```bash
ros2 interface show \
  control_msgs/action/FollowJointTrajectory
```

Gửi goal và xem feedback:

```bash
ros2 action send_goal \
  /action_name \
  package_name/action/ActionType \
  "{goal_field: value}" \
  --feedback
```

Action phù hợp với trajectory, homing hoặc tác vụ kéo dài cần:

- Goal.
- Feedback.
- Result.
- Cancel/preemption.

---

## 13. Interface: msg, srv, action

```bash
ros2 interface list
ros2 interface packages
ros2 interface package sensor_msgs
ros2 interface show sensor_msgs/msg/JointState
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show std_srvs/srv/Trigger
ros2 interface show control_msgs/action/FollowJointTrajectory
```

Quy tắc:

```text
package/msg/Type
package/srv/Type
package/action/Type
```

Ưu tiên interface chuẩn trước khi tạo custom interface.

Interface thường dùng cho robot arm:

```text
sensor_msgs/msg/JointState
trajectory_msgs/msg/JointTrajectory
geometry_msgs/msg/PoseStamped
geometry_msgs/msg/TransformStamped
diagnostic_msgs/msg/DiagnosticArray
std_srvs/srv/Trigger
control_msgs/action/FollowJointTrajectory
```

---

## 14. Parameter

### Liệt kê, đọc và sửa

```bash
ros2 param list
ros2 param list /myarm_driver
ros2 param get /myarm_driver read_rate_hz
ros2 param set /myarm_driver read_rate_hz 5.0
ros2 param describe /myarm_driver read_rate_hz
```

### Dump và load YAML

```bash
ros2 param dump /myarm_driver
ros2 param dump /myarm_driver > myarm_driver.yaml
ros2 param load /myarm_driver myarm_driver.yaml
```

### Chạy node với parameter trực tiếp

```bash
ros2 run my_package my_node \
  --ros-args \
  -p read_rate_hz:=5.0 \
  -p port:=/dev/ttyUSB0
```

### Chạy node với file YAML

```bash
ros2 run my_package my_node \
  --ros-args \
  --params-file config/robot.yaml
```

Mẫu YAML Foxy:

```yaml
myarm_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 1000000
    read_rate_hz: 5.0
    command_rate_hz: 5.0
    timeout_sec: 0.1
    publish_joint_states: true
```

Parameter là cấu hình; không nên dùng để gửi joint command liên tục.

---

## 15. Remapping

Remap topic:

```bash
ros2 run my_package my_node \
  --ros-args \
  -r /input:=/joint_states
```

Remap nhiều tên:

```bash
ros2 run my_package my_node \
  --ros-args \
  -r /input:=/joint_states \
  -r /output:=/filtered_joint_states
```

Tên node và namespace:

```bash
-r __node:=driver
-r __ns:=/myarm_01
```

---

## 16. Launch

### Chạy launch file

```bash
ros2 launch <package> <launch_file>
```

Ví dụ:

```bash
ros2 launch myarm_m750_bringup robot.launch.py
```

Liệt kê launch argument:

```bash
ros2 launch myarm_m750_bringup robot.launch.py --show-args
```

Truyền launch argument:

```bash
ros2 launch myarm_m750_bringup robot.launch.py \
  use_rviz:=true \
  use_camera:=false
```

Mẫu launch Python:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="myarm_m750_driver",
            executable="driver_node",
            name="myarm_driver",
            output="screen",
            parameters=["config/robot.yaml"],
        ),
    ])
```

---

## 17. Logging

### Log level từ CLI

```bash
ros2 run my_package my_node \
  --ros-args \
  --log-level debug
```

Đặt level cho logger cụ thể:

```bash
ros2 run my_package my_node \
  --ros-args \
  --log-level myarm_driver:=debug
```

Mức log:

```text
DEBUG < INFO < WARN < ERROR < FATAL
```

Python:

```python
self.get_logger().debug("Raw packet received")
self.get_logger().info("Robot connected")
self.get_logger().warning("Joint state is stale")
self.get_logger().error("Serial communication failed")
self.get_logger().fatal("Driver cannot continue")
```

GUI:

```bash
ros2 run rqt_console rqt_console
```

Log thường nằm dưới:

```bash
~/.ros/log/
```

---

## 18. ROS graph và GUI debug

```bash
rqt_graph
ros2 run rqt_graph rqt_graph

ros2 run rqt_console rqt_console
ros2 run rqt_topic rqt_topic
```

`rqt_graph` giúp xem:

- Node nào publish topic nào.
- Node nào subscribe topic nào.
- Có namespace hoặc remapping sai hay không.
- Có publisher/subscriber ngoài dự kiến hay không.

---

## 19. rosbag2

### Ghi một số topic

```bash
ros2 bag record \
  -o my_test_bag \
  /joint_states \
  /tf \
  /tf_static \
  /diagnostics
```

### Ghi toàn bộ topic

```bash
ros2 bag record -a
```

Chỉ nên dùng `-a` khi biết rõ lượng dữ liệu, đặc biệt với camera.

### Xem thông tin bag

```bash
ros2 bag info my_test_bag
```

### Phát lại

```bash
ros2 bag play my_test_bag
```

### Benchmark MyArm gợi ý

```bash
ros2 bag record \
  -o myarm_t1_joint_waypoint \
  /joint_states \
  /myarm/command/joint_goal \
  /myarm/state/joint_state \
  /myarm/trajectory/preview \
  /myarm/motion_execution/diagnostics \
  /tf \
  /tf_static
```

Không ghi camera nếu benchmark chỉ cần joint và timing, vì camera làm bag lớn nhanh.

---

## 20. TF2

### Kiểm tra transform

```bash
ros2 run tf2_ros tf2_echo base_link tool0
```

### Xuất cây TF

```bash
ros2 run tf2_tools view_frames.py
```

Kết quả thường tạo `frames.pdf`.

### Xem topic TF

```bash
ros2 topic echo /tf
ros2 topic echo /tf_static
ros2 topic hz /tf
```

### Publish static transform

Euler:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.1 0.0 0.2 \
  0.0 0.0 0.0 \
  parent_frame child_frame
```

Cần duy trì một cây TF hợp lệ:

- Không có vòng lặp.
- Mỗi child frame chỉ có một parent authoritative.
- Frame name nhất quán.
- Dynamic transform có timestamp hợp lệ.
- Sensor pose cố định nên dùng `/tf_static`.

---

## 21. URDF, Xacro và robot_state_publisher

### Kiểm tra URDF

```bash
check_urdf robot.urdf
```

### Chuyển Xacro sang URDF

```bash
xacro robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

### Xem robot description

```bash
ros2 param get /robot_state_publisher robot_description
```

### Kiểm tra joint state

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

Luồng cơ bản:

```text
URDF/Xacro + /joint_states
            |
            v
robot_state_publisher
            |
            +--> /tf
            +--> /tf_static
            |
            v
           RViz2
```

Chạy RViz2:

```bash
rviz2
```

---

## 22. Lifecycle node

```bash
ros2 lifecycle nodes
ros2 lifecycle get /node_name
ros2 lifecycle list /node_name
```

Chuyển state:

```bash
ros2 lifecycle set /node_name configure
ros2 lifecycle set /node_name activate
ros2 lifecycle set /node_name deactivate
ros2 lifecycle set /node_name cleanup
ros2 lifecycle set /node_name shutdown
```

Lifecycle phù hợp cho driver cần trình tự:

```text
unconfigured
    -> configure: mở serial, đọc config
inactive
    -> activate: bắt đầu publish/control
active
    -> deactivate: dừng control an toàn
inactive
    -> cleanup: đóng tài nguyên
```

---

## 23. Component và composition

Liệt kê component type:

```bash
ros2 component types
```

Chạy component container:

```bash
ros2 run rclcpp_components component_container
```

Liệt kê container:

```bash
ros2 component list
```

Load component:

```bash
ros2 component load \
  /ComponentManager \
  package_name \
  plugin_name
```

Composition chủ yếu dành cho C++ component, giảm số process và chi phí truyền dữ liệu nội bộ. Chỉ nên áp dụng sau khi kiến trúc node riêng đã ổn định.

---

## 24. DDS, RMW và QoS

### Xem RMW hiện tại

```bash
echo "$RMW_IMPLEMENTATION"
```

Nếu biến rỗng, ROS 2 dùng implementation mặc định đã cài.

Chọn Fast DDS:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Cài và chọn Cyclone DDS:

```bash
sudo apt install ros-foxy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Sau khi đổi RMW hoặc `ROS_DOMAIN_ID`:

```bash
ros2 daemon stop
ros2 daemon start
```

### QoS cần nhớ

| Policy | Giá trị thường gặp | Ý nghĩa |
|---|---|---|
| Reliability | `reliable` | Cố gắng giao đủ dữ liệu |
| Reliability | `best_effort` | Ưu tiên dữ liệu mới, có thể mất gói |
| History | `keep_last` | Chỉ giữ N message gần nhất |
| Depth | `1`, `5`, `10` | Kích thước queue |
| Durability | `volatile` | Subscriber mới chỉ nhận message mới |
| Durability | `transient_local` | Publisher giữ dữ liệu cho subscriber đến sau |

Gợi ý khởi đầu:

| Dữ liệu | QoS |
|---|---|
| Camera RGB/depth | Best effort, depth nhỏ |
| Joint state | Reliable, keep last 5–10 |
| Joint command | Reliable, depth 1–5 |
| Diagnostics | Reliable |
| `/tf` | Volatile |
| `/tf_static` | Transient local |

Publisher và subscriber phải có QoS tương thích mới giao tiếp được.

---

## 25. ROS 2 nhiều máy qua LAN/WLAN

Thiết lập giống nhau trên Jetson và host:

```bash
source /opt/ros/foxy/setup.bash

export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Kiểm tra:

```bash
printenv | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION'
```

Restart daemon:

```bash
ros2 daemon stop
ros2 daemon start
```

### Test tối thiểu

Máy A:

```bash
ros2 run demo_nodes_cpp talker
```

Máy B:

```bash
ros2 run demo_nodes_py listener
```

Sau đó:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
```

### Checklist khi không discover được nhau

```text
[ ] Ping được hai chiều.
[ ] Hai máy cùng subnet hoặc có route phù hợp.
[ ] Cùng ROS_DOMAIN_ID.
[ ] ROS_LOCALHOST_ONLY không phải 1.
[ ] Cùng RMW implementation khi có thể.
[ ] Firewall không chặn UDP/multicast.
[ ] Wi-Fi AP không bật client isolation.
[ ] Container dùng network mode phù hợp.
[ ] QoS publisher/subscriber tương thích.
[ ] Không source nhầm distro hoặc workspace.
```

Docker trên Linux thường cần host networking để DDS multicast hoạt động đơn giản:

```bash
docker run --rm -it \
  --net=host \
  <image_name>
```

---

## 26. ROS_DOMAIN_ID

```bash
export ROS_DOMAIN_ID=30
```

- Node cùng domain có thể discover nhau.
- Node khác domain được tách thành mạng ROS logic khác.
- Dùng một domain cố định cho toàn dự án.
- Tránh dùng domain mặc định `0` trong môi trường có nhiều nhóm ROS.

Kiểm tra:

```bash
echo "$ROS_DOMAIN_ID"
```

---

## 27. ROS 2 daemon

```bash
ros2 daemon status
ros2 daemon stop
ros2 daemon start
```

Restart daemon khi:

- Đổi `ROS_DOMAIN_ID`.
- Đổi `RMW_IMPLEMENTATION`.
- CLI vẫn hiển thị graph cũ.
- `ros2 node list` không khớp trạng thái thực tế.

Daemon chỉ hỗ trợ introspection CLI; nó không phải ROS Master và không điều phối communication giữa các node.

---

## 28. ros2doctor

```bash
ros2 doctor
ros2 doctor --report
```

Dùng để kiểm tra:

- Distro.
- Network.
- RMW.
- Platform.
- Package và environment.
- Một số lỗi cấu hình phổ biến.

---

## 29. Diagnostics hệ thống

### Process và tài nguyên

```bash
htop
ps aux | grep ros
free -h
df -h
```

### Network

```bash
ip addr
ip route
ping <peer_ip>
```

### Serial

```bash
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*
dmesg --follow
```

### Camera

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

### USB

```bash
lsusb
```

---

## 30. Lệnh debug MyArm M750

### Kiểm tra graph

```bash
ros2 node list
ros2 topic list -t
rqt_graph
```

### Kiểm tra driver

```bash
ros2 node info /myarm_driver
ros2 param list /myarm_driver
ros2 topic echo /myarm/status
ros2 topic echo /diagnostics
```

### Kiểm tra joint

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
ros2 topic info /joint_states
```

### Kiểm tra TF và model

```bash
ros2 run tf2_ros tf2_echo base_link tool0
ros2 run tf2_tools view_frames.py
ros2 node info /robot_state_publisher
rviz2
```

### Kiểm tra trajectory action

```bash
ros2 action list -t
ros2 action info /arm_controller/follow_joint_trajectory
ros2 interface show control_msgs/action/FollowJointTrajectory
```

### Ghi benchmark

```bash
ros2 bag record \
  -o myarm_debug_$(date +%Y%m%d_%H%M%S) \
  /joint_states \
  /myarm/status \
  /diagnostics \
  /tf \
  /tf_static
```

---

## 31. Kiến trúc package gợi ý cho MyArm M750

```text
ros2/src/
├── myarm_m750_msgs/
├── myarm_m750_description/
├── myarm_m750_driver/
├── myarm_m750_bringup/
├── myarm_m750_visualization/
├── myarm_m750_camera/
├── myarm_m750_moveit_config/
└── myarm_m750_gazebo/
```

| Package | Vai trò |
|---|---|
| `msgs` | Custom msg/srv/action |
| `description` | URDF, Xacro, meshes |
| `driver` | Serial, protocol, hardware adapter |
| `bringup` | Launch toàn robot |
| `visualization` | RViz config và host launch |
| `camera` | Camera node và calibration |
| `moveit_config` | Planning và execution |
| `gazebo` | Gazebo Classic integration |

---

## 32. Quy trình làm việc hằng ngày

```bash
# 1. Mở terminal
source /opt/ros/foxy/setup.bash

# 2. Vào workspace
cd ~/my_ws

# 3. Cài dependency nếu package.xml thay đổi
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro foxy \
  -r -y

# 4. Build package đang làm
colcon build \
  --symlink-install \
  --packages-select myarm_m750_driver

# 5. Source overlay
source install/setup.bash

# 6. Chạy
ros2 run myarm_m750_driver driver_node

# Hoặc
ros2 launch myarm_m750_bringup robot.launch.py
```

Terminal debug khác cũng phải source:

```bash
source /opt/ros/foxy/setup.bash
source ~/my_ws/install/setup.bash
```

---

## 33. Lỗi thường gặp

### `Package '<name>' not found`

```bash
source /opt/ros/foxy/setup.bash
source ~/my_ws/install/setup.bash
ros2 pkg list | grep <name>
```

Nếu vẫn không thấy:

```bash
colcon list
colcon build --packages-select <name> --event-handlers console_direct+
```

### `No executable found`

Kiểm tra:

```bash
ros2 pkg executables <package>
```

Python:

- Kiểm tra `setup.py`.
- Kiểm tra `console_scripts`.
- Build lại và source lại.

C++:

- Kiểm tra `add_executable(...)`.
- Kiểm tra `install(TARGETS ...)`.

### Sửa Python nhưng node vẫn chạy code cũ

```bash
colcon build --symlink-install --packages-select <package>
source install/setup.bash
```

### Topic tồn tại nhưng không nhận dữ liệu

Kiểm tra:

```bash
ros2 topic info /topic_name
ros2 node info /publisher_node
ros2 node info /subscriber_node
```

Nguyên nhân thường gặp:

- QoS không tương thích.
- Namespace/remapping sai.
- Publisher chưa thực sự publish.
- Subscriber callback bị block.
- Khác domain hoặc lỗi discovery.

### `ros2 node list` hiển thị node đã tắt

```bash
ros2 daemon stop
ros2 daemon start
```

### TF lỗi hoặc RViz không hiện robot

Kiểm tra:

```bash
ros2 topic echo /joint_states
ros2 topic echo /tf_static
ros2 run tf2_ros tf2_echo base_link tool0
ros2 run tf2_tools view_frames.py
```

Kiểm tra thêm:

- `Fixed Frame` trong RViz.
- Joint name có khớp URDF.
- `robot_description` đã được load.
- Timestamp của `/joint_states`.
- TF tree có nhiều publisher cho cùng child frame hay không.

### Build bị lỗi khó hiểu sau khi đổi dependency

```bash
pwd
rm -rf build install log
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y
colcon build --symlink-install --event-handlers console_direct+
```

---

## 34. Nguyên tắc thực dụng

1. Mỗi node có một trách nhiệm chính.
2. Một process/node duy nhất sở hữu serial port.
3. Dùng interface chuẩn trước khi tạo custom interface.
4. Topic cho stream; service cho thao tác ngắn; action cho tác vụ dài.
5. Parameter chỉ dùng cho cấu hình.
6. Luôn ghi frame ID và timestamp đúng.
7. Tránh nhiều publisher authoritative cho `/joint_states` hoặc cùng một TF child.
8. Callback không được block vô hạn.
9. Serial read phải có timeout.
10. Log có level; tránh spam mỗi chu kỳ.
11. Dùng rosbag2 để tái hiện lỗi.
12. Đo `hz`, `bw`, latency và packet loss thay vì phỏng đoán.
13. Build từng package khi phát triển.
14. Source lại overlay sau mỗi build.
15. Với Foxy EOL, khóa môi trường và dependency.

---

## 35. Tài liệu tham khảo chính thức

### Foxy

- [ROS 2 Foxy Tutorials](https://docs.ros.org/en/foxy/Tutorials.html)
- [ROS 2 Foxy Concepts](https://docs.ros.org/en/foxy/Concepts.html)
- [Foxy Fitzroy release information](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html)
- [Configuring the ROS 2 environment](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [ROS 2 command-line introspection](https://docs.ros.org/en/foxy/Concepts/About-Command-Line-Tools.html)
- [Understanding nodes](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [Understanding topics](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Understanding services](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [Understanding actions](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
- [Creating a package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Using colcon](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Managing dependencies with rosdep](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Rosdep.html)
- [Creating launch files](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 QoS](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)
- [Working with multiple RMW implementations](https://docs.ros.org/en/foxy/How-To-Guides/Working-with-multiple-RMW-implementations.html)
- [URDF tutorials](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [URDF with robot_state_publisher](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html)
- [TF2 tutorials](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Recording and playing back data](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

### Công cụ build

- [colcon documentation](https://colcon.readthedocs.io/en/released/)
- [colcon quick start](https://colcon.readthedocs.io/en/main/user/quick-start.html)
- [colcon build reference](https://colcon.readthedocs.io/en/main/reference/verb/build.html)

---

## 36. Quick command block

```bash
# Environment
source /opt/ros/foxy/setup.bash
source install/setup.bash
printenv | grep -E 'ROS|RMW'

# Build
rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y
colcon build --symlink-install
colcon test
colcon test-result --verbose

# Package/node
ros2 pkg list
ros2 pkg executables <pkg>
ros2 run <pkg> <exe>
ros2 node list
ros2 node info <node>

# Topic
ros2 topic list -t
ros2 topic info <topic>
ros2 topic echo <topic>
ros2 topic hz <topic>
ros2 topic bw <topic>

# Service/action
ros2 service list -t
ros2 service call <service> <type> "<request>"
ros2 action list -t
ros2 action info <action>
ros2 action send_goal <action> <type> "<goal>" --feedback

# Parameter/interface
ros2 param list <node>
ros2 param get <node> <param>
ros2 param set <node> <param> <value>
ros2 interface show <interface>

# Launch/TF/bag
ros2 launch <pkg> <launch_file>
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
ros2 bag record -o <bag_name> <topics...>
ros2 bag info <bag_name>
ros2 bag play <bag_name>

# Debug
rqt_graph
ros2 run rqt_console rqt_console
ros2 doctor --report
ros2 daemon stop
ros2 daemon start
```
