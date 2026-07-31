# rclpy Cheatsheet — ROS 2 Foxy

> **Phạm vi:** `rclpy` trên ROS 2 Foxy, Ubuntu 20.04, Python 3.8.
>
> **Mục tiêu:** mẫu code và quy tắc thực dụng để viết node Python cho robot: publisher/subscriber, timer, service, action, parameter, QoS, executor, callback group, time, logging, packaging và debug.
>
> **Lưu ý:** Foxy đã End-of-Life. Luôn kiểm tra API theo tài liệu Foxy trước khi copy code từ Humble/Jazzy/Rolling.

---

## 1. `rclpy` là gì?

`rclpy` là Python client library chính thức của ROS 2. Nó cung cấp API để tạo:

```text
Node
├── Publisher / Subscription
├── Service server / client
├── Action server / client
├── Timer
├── Parameter
├── Clock / Time / Duration
├── Executor
└── Callback group
```

Luồng cơ bản:

```text
rclpy.init()
    ↓
tạo Node và ROS entities
    ↓
rclpy.spin(node)
    ↓
executor thực thi callback
    ↓
destroy_node()
    ↓
rclpy.shutdown()
```

Tài liệu chính:

- [rclpy Foxy API](https://docs.ros2.org/foxy/api/rclpy/)
- [Node API](https://docs.ros2.org/foxy/api/rclpy/api/node.html)
- [Initialization, shutdown and spinning](https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html)
- [Execution and callbacks](https://docs.ros2.org/foxy/api/rclpy/api/execution_and_callbacks.html)

---

## 2. Environment

```bash
source /opt/ros/foxy/setup.bash
```

Workspace overlay:

```bash
cd ~/my_ws
source install/setup.bash
```

Kiểm tra:

```bash
python3 --version
python3 -c "import rclpy; print(rclpy.__file__)"
```

### Không cài `rclpy` bằng pip

Không nên:

```bash
pip install rclpy
```

Dùng package ROS 2 và source đúng environment:

```bash
sudo apt install ros-foxy-rclpy
source /opt/ros/foxy/setup.bash
```

---

## 3. Node tối thiểu

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    def __init__(self) -> None:
        super().__init__('minimal_node')
        self.get_logger().info('Node started')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MinimalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Ý nghĩa:

```python
rclpy.init(args=args)      # Khởi tạo ROS context
rclpy.spin(node)           # Executor chờ và chạy callback
node.destroy_node()        # Giải phóng entity của node
rclpy.shutdown()           # Shutdown context
```

---

## 4. Tên node và namespace

```python
self.get_name()
self.get_namespace()
self.get_fully_qualified_name()
```

Đổi tên node:

```bash
ros2 run my_package my_node \
  --ros-args \
  -r __node:=myarm_driver
```

Đặt namespace:

```bash
ros2 run my_package my_node \
  --ros-args \
  -r __ns:=/myarm_01
```

---

## 5. Publisher

Tạo publisher:

```python
from std_msgs.msg import String

self.publisher = self.create_publisher(
    String,
    '/status',
    10,
)
```

Publish:

```python
msg = String()
msg.data = 'ready'
self.publisher.publish(msg)
```

Node hoàn chỉnh:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusPublisher(Node):
    def __init__(self) -> None:
        super().__init__('status_publisher')

        self.publisher = self.create_publisher(
            String,
            '/status',
            10,
        )

        self.counter = 0
        self.timer = self.create_timer(
            1.0,
            self.publish_status,
        )

    def publish_status(self) -> None:
        msg = String()
        msg.data = f'heartbeat={self.counter}'
        self.publisher.publish(msg)
        self.counter += 1
```

Subscriber count:

```python
count = self.publisher.get_subscription_count()
```

Tài liệu: [Topics API](https://docs.ros2.org/foxy/api/rclpy/api/topics.html)

---

## 6. Subscription

```python
from std_msgs.msg import String

self.subscription = self.create_subscription(
    String,
    '/status',
    self.status_callback,
    10,
)
```

```python
def status_callback(self, msg: String) -> None:
    self.get_logger().info(f'Received: {msg.data}')
```

### Quy tắc callback

Callback nên:

```text
- kết thúc nhanh;
- không có while True;
- không chờ vô hạn;
- I/O phải có timeout;
- không giữ lock lâu;
- không chạy inference nặng trong driver callback;
- không gọi synchronous service/action một cách tùy tiện.
```

Sai:

```python
def callback(self, msg):
    while True:
        do_work()
```

Tốt hơn:

```python
def callback(self, msg):
    self.latest_msg = msg
```

Sau đó xử lý bằng timer hoặc worker riêng.

Tutorial: [Python publisher/subscriber](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

## 7. Message có `Header`

```python
from geometry_msgs.msg import PoseStamped

msg = PoseStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'base_link'

msg.pose.position.x = 0.30
msg.pose.position.y = 0.00
msg.pose.position.z = 0.25
msg.pose.orientation.w = 1.0
```

Nguyên tắc:

- `stamp`: thời điểm dữ liệu được tạo/đo.
- `frame_id`: frame biểu diễn dữ liệu.
- Identity quaternion là `w = 1.0`, không phải toàn số 0.

---

## 8. `sensor_msgs/msg/JointState`

```python
from sensor_msgs.msg import JointState

self.joint_pub = self.create_publisher(
    JointState,
    '/joint_states',
    10,
)
```

```python
msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.name = [
    'joint1', 'joint2', 'joint3',
    'joint4', 'joint5', 'joint6',
]
msg.position = [q1, q2, q3, q4, q5, q6]

self.joint_pub.publish(msg)
```

Lưu ý:

```text
position: radian với revolute joint
velocity: rad/s
effort: tùy hardware/driver
name: phải khớp URDF
```

Chỉ nên có một nguồn authoritative publish `/joint_states` cho cùng robot.

---

## 9. Timer

5 Hz:

```python
rate_hz = 5.0
period_sec = 1.0 / rate_hz

self.timer = self.create_timer(
    period_sec,
    self.timer_callback,
)
```

```python
def timer_callback(self) -> None:
    self.get_logger().debug('Timer tick')
```

Điều khiển timer:

```python
self.timer.cancel()
self.timer.reset()
self.timer.is_canceled()
```

### Timer không phải hard real-time loop

Timer `rclpy` phụ thuộc Linux scheduler, Python, GIL, executor, callback khác và I/O. Với driver MyArm, 5–20 Hz phù hợp cho orchestration/feedback nhẹ; control loop có deadline nghiêm ngặt nên đặt ở firmware, C++ hoặc `ros2_control` phù hợp.

Tài liệu: [Timer API](https://docs.ros2.org/foxy/api/rclpy/api/timers.html)

---

## 10. `spin`, `spin_once`, `spin_until_future_complete`

Chạy liên tục:

```python
rclpy.spin(node)
```

Xử lý một work item:

```python
rclpy.spin_once(
    node,
    timeout_sec=0.1,
)
```

Tích hợp vòng lặp ngoài:

```python
while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.05)
    do_non_ros_work()
```

Chờ future:

```python
rclpy.spin_until_future_complete(
    node,
    future,
    timeout_sec=2.0,
)
```

### Cảnh báo deadlock

Không gọi `spin_until_future_complete()` hoặc synchronous service/action bên trong callback thuộc mutually-exclusive group nếu callback response cần chính executor đó xử lý.

---

## 11. Service server

```python
from std_srvs.srv import Trigger

self.service = self.create_service(
    Trigger,
    '/myarm/power_on',
    self.power_on_callback,
)
```

```python
def power_on_callback(
    self,
    request: Trigger.Request,
    response: Trigger.Response,
) -> Trigger.Response:
    del request

    try:
        self.hardware.power_on()
        response.success = True
        response.message = 'Robot powered on'
    except Exception as exc:
        response.success = False
        response.message = str(exc)

    return response
```

Service phù hợp tác vụ ngắn. Tác vụ dài có progress/cancel nên dùng action.

---

## 12. Service client bất đồng bộ

```python
from std_srvs.srv import Trigger

self.client = self.create_client(
    Trigger,
    '/myarm/power_on',
)
```

Chờ service:

```python
if not self.client.wait_for_service(timeout_sec=2.0):
    raise RuntimeError('Service unavailable')
```

Gọi:

```python
request = Trigger.Request()
future = self.client.call_async(request)
```

Chờ từ `main`:

```python
rclpy.spin_until_future_complete(
    node,
    future,
    timeout_sec=3.0,
)

if future.done():
    response = future.result()
```

Future callback:

```python
future.add_done_callback(self.response_callback)
```

```python
def response_callback(self, future) -> None:
    try:
        response = future.result()
    except Exception as exc:
        self.get_logger().error(
            f'Service failed: {exc}'
        )
        return

    self.get_logger().info(
        f'success={response.success}'
    )
```

Tài liệu:

- [Services API](https://docs.ros2.org/foxy/api/rclpy/api/services.html)
- [Python service/client tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

---

## 13. Action server

Action phù hợp trajectory, homing, scan sequence hoặc tác vụ dài.

```python
import asyncio

import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
from rclpy.node import Node


class FibonacciServer(Node):
    def __init__(self) -> None:
        super().__init__('fibonacci_server')

        self.action_server = ActionServer(
            self,
            Fibonacci,
            '/fibonacci',
            self.execute_callback,
        )

    async def execute_callback(self, goal_handle):
        feedback = Fibonacci.Feedback()
        feedback.sequence = [0, 1]

        for index in range(1, goal_handle.request.order):
            feedback.sequence.append(
                feedback.sequence[index]
                + feedback.sequence[index - 1]
            )

            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback.sequence
        return result
```

### Goal và cancel callback

```python
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
```

```python
def goal_callback(self, goal_request):
    if goal_request.order <= 0:
        return GoalResponse.REJECT
    return GoalResponse.ACCEPT


def cancel_callback(self, goal_handle):
    del goal_handle
    return CancelResponse.ACCEPT
```

Trong execute callback:

```python
if goal_handle.is_cancel_requested:
    goal_handle.canceled()
    result = Fibonacci.Result()
    result.sequence = feedback.sequence
    return result
```

---

## 14. Action client

```python
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient

self.action_client = ActionClient(
    self,
    Fibonacci,
    '/fibonacci',
)
```

Gửi goal:

```python
def send_goal(self, order: int) -> None:
    if not self.action_client.wait_for_server(
        timeout_sec=2.0
    ):
        self.get_logger().error('Action unavailable')
        return

    goal = Fibonacci.Goal()
    goal.order = order

    future = self.action_client.send_goal_async(
        goal,
        feedback_callback=self.feedback_callback,
    )

    future.add_done_callback(
        self.goal_response_callback
    )
```

Goal response:

```python
def goal_response_callback(self, future) -> None:
    goal_handle = future.result()

    if not goal_handle.accepted:
        self.get_logger().warning('Goal rejected')
        return

    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(
        self.result_callback
    )
```

Feedback/result:

```python
def feedback_callback(self, feedback_msg) -> None:
    feedback = feedback_msg.feedback
    self.get_logger().info(
        f'Feedback: {feedback.sequence}'
    )


def result_callback(self, future) -> None:
    result_response = future.result()
    result = result_response.result
    self.get_logger().info(
        f'Result: {result.sequence}'
    )
```

Cancel:

```python
cancel_future = goal_handle.cancel_goal_async()
```

Tài liệu:

- [Actions API](https://docs.ros2.org/foxy/api/rclpy/api/actions.html)
- [Python action tutorial](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [ROS 2 action design](https://design.ros2.org/articles/actions.html)

---

## 15. Parameter

Khai báo:

```python
self.declare_parameter('read_rate_hz', 5.0)
```

Đọc:

```python
rate_hz = self.get_parameter('read_rate_hz').value
```

Nhiều parameter:

```python
self.declare_parameters(
    namespace='',
    parameters=[
        ('port', '/dev/ttyUSB0'),
        ('baudrate', 1000000),
        ('read_rate_hz', 5.0),
        ('timeout_sec', 0.1),
    ],
)
```

YAML:

```yaml
myarm_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 1000000
    read_rate_hz: 5.0
    timeout_sec: 0.1
```

Chạy:

```bash
ros2 run myarm_m750_driver driver_node \
  --ros-args \
  --params-file config/robot.yaml
```

---

## 16. Validate parameter update

```python
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
```

```python
self.add_on_set_parameters_callback(
    self.on_set_parameters
)
```

```python
def on_set_parameters(self, parameters):
    for parameter in parameters:
        if parameter.name == 'read_rate_hz':
            if parameter.type_ not in (
                Parameter.Type.INTEGER,
                Parameter.Type.DOUBLE,
            ):
                return SetParametersResult(
                    successful=False,
                    reason='read_rate_hz must be numeric',
                )

            if float(parameter.value) <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason='read_rate_hz must be > 0',
                )

    return SetParametersResult(successful=True)
```

### Foxy-specific caution

Parameter behavior đã thay đổi qua các distro. Với Foxy, nên tự validate type và range, không giả định runtime có toàn bộ static-typing behavior của distro mới.

---

## 17. QoS

Depth đơn giản:

```python
self.publisher = self.create_publisher(
    String,
    '/status',
    10,
)
```

Custom QoS:

```python
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
```

Sensor preset:

```python
from rclpy.qos import qos_profile_sensor_data

self.subscription = self.create_subscription(
    Image,
    '/camera/image_raw',
    self.image_callback,
    qos_profile_sensor_data,
)
```

Gợi ý:

| Dữ liệu | QoS khởi đầu |
|---|---|
| Joint state | reliable, keep-last 5–10 |
| Joint command | reliable, depth 1–5 |
| Camera | sensor-data preset, queue nhỏ |
| Diagnostics | reliable, depth 10 |
| `/tf` | volatile |
| `/tf_static` | transient local |

Topic có thể xuất hiện nhưng không truyền dữ liệu nếu QoS không tương thích.

```bash
ros2 topic info /topic_name --verbose
```

Tài liệu:

- [rclpy QoS API](https://docs.ros2.org/foxy/api/rclpy/api/qos.html)
- [Foxy QoS concepts](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)

---

## 18. Clock, Time và Duration

```python
now = self.get_clock().now()
stamp = now.to_msg()
now_ns = now.nanoseconds
```

Khoảng thời gian:

```python
current = self.get_clock().now()
elapsed = current - self.last_time
elapsed_sec = elapsed.nanoseconds / 1e9
self.last_time = current
```

```python
from rclpy.duration import Duration

timeout = Duration(seconds=1)
```

Dùng ROS clock khi dữ liệu cần tương thích simulation và `use_sim_time`.

Dùng monotonic clock cho timeout nội bộ:

```python
import time

deadline = time.monotonic() + 1.0
```

---

## 19. Logging

```python
self.get_logger().debug('Debug information')
self.get_logger().info('Robot connected')
self.get_logger().warning('State is stale')
self.get_logger().error('Serial read failed')
self.get_logger().fatal('Driver cannot continue')
```

CLI:

```bash
ros2 run my_package my_node \
  --ros-args \
  --log-level debug
```

Logger cụ thể:

```bash
ros2 run my_package my_node \
  --ros-args \
  --log-level myarm_driver:=debug
```

Manual throttle tương thích, dễ kiểm soát:

```python
import time

self.last_warning_time = 0.0
```

```python
def warn_throttled(self, message: str) -> None:
    now = time.monotonic()

    if now - self.last_warning_time >= 2.0:
        self.get_logger().warning(message)
        self.last_warning_time = now
```

Không dùng `print()` cho hệ thống log runtime chính.

---

## 20. Executor

### Mặc định single-threaded

```python
rclpy.spin(node)
```

Explicit:

```python
from rclpy.executors import SingleThreadedExecutor

executor = SingleThreadedExecutor()
executor.add_node(node)

try:
    executor.spin()
finally:
    executor.shutdown()
```

Multi-threaded:

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)

try:
    executor.spin()
finally:
    executor.shutdown()
```

MultiThreadedExecutor không tự làm mọi callback chạy song song; callback groups vẫn quyết định concurrency.

Tài liệu:

- [Execution and callbacks](https://docs.ros2.org/foxy/api/rclpy/api/execution_and_callbacks.html)
- [Foxy executors](https://docs.ros.org/en/foxy/Concepts/About-Executors.html)

---

## 21. Callback group

```python
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
```

Mutually exclusive:

```python
self.hardware_group = MutuallyExclusiveCallbackGroup()
```

Phù hợp serial/hardware không thread-safe.

Reentrant:

```python
self.compute_group = ReentrantCallbackGroup()
```

Chỉ dùng khi code thực sự thread-safe.

Gán group:

```python
self.timer = self.create_timer(
    0.2,
    self.read_robot,
    callback_group=self.hardware_group,
)
```

```python
self.subscription = self.create_subscription(
    JointState,
    '/command',
    self.command_callback,
    10,
    callback_group=self.hardware_group,
)
```

Nếu mọi entity nằm trong default mutually-exclusive group, MultiThreadedExecutor có thể không tạo parallelism như mong đợi.

Tài liệu: [Using callback groups](https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html)

---

## 22. Shared state và lock

```python
from threading import Lock

self.state_lock = Lock()
self.latest_state = None
```

Writer:

```python
with self.state_lock:
    self.latest_state = new_state
```

Reader:

```python
with self.state_lock:
    state = self.latest_state
```

Không giữ lock trong lúc:

```text
- serial blocking;
- publish;
- service/action call;
- inference;
- logging nặng.
```

---

## 23. Worker thread cho blocking I/O

```python
from queue import Empty, Queue
from threading import Event, Thread

self.stop_event = Event()
self.state_queue = Queue(maxsize=1)

self.worker = Thread(
    target=self.hardware_worker,
    daemon=True,
)
self.worker.start()
```

```python
def hardware_worker(self) -> None:
    while not self.stop_event.is_set():
        try:
            state = self.hardware.read_state(
                timeout_sec=0.1
            )
        except Exception as exc:
            self.get_logger().error(
                f'Hardware read failed: {exc}'
            )
            continue

        if state is None:
            continue

        try:
            self.state_queue.get_nowait()
        except Empty:
            pass

        self.state_queue.put_nowait(state)
```

Timer publish:

```python
def publish_latest_state(self) -> None:
    try:
        state = self.state_queue.get_nowait()
    except Empty:
        return

    self.publisher.publish(
        self.to_joint_state(state)
    )
```

Cleanup:

```python
def destroy_node(self) -> None:
    self.stop_event.set()

    if self.worker.is_alive():
        self.worker.join(timeout=1.0)

    self.hardware.close()
    super().destroy_node()
```

Nguyên tắc:

```text
- một worker sở hữu hardware;
- queue có giới hạn;
- chỉ giữ state mới nhất;
- có timeout;
- có shutdown path.
```

---

## 24. Graph introspection trong code

```python
self.get_node_names()
self.get_node_names_and_namespaces()
self.get_topic_names_and_types()
self.get_service_names_and_types()
```

Đếm endpoint:

```python
self.count_publishers('/joint_states')
self.count_subscribers('/joint_states')
```

Discovery là phân tán và thay đổi theo thời gian; không dùng introspection như safety guarantee tuyệt đối.

---

## 25. Exception handling

```python
def timer_callback(self) -> None:
    try:
        state = self.hardware.read_state()
    except TimeoutError:
        self.get_logger().warning(
            'Robot response timeout'
        )
        return
    except Exception as exc:
        self.get_logger().error(
            f'Unexpected hardware error: {exc}'
        )
        return

    self.publish_state(state)
```

Không dùng:

```python
except Exception:
    pass
```

---

## 26. Tách pure Python core khỏi ROS adapter

Domain model:

```python
from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class RobotState:
    positions_rad: List[float]
    moving: bool
    error_code: int
```

ROS mapper:

```python
def to_joint_state(self, state: RobotState) -> JointState:
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = list(self.joint_names)
    msg.position = list(state.positions_rad)
    return msg
```

Kiến trúc:

```text
pycore/
├── domain
├── application
├── ports
└── adapters

ros2 package/
├── driver_node.py
├── ros_mapper.py
└── main.py
```

`rclpy` nên là integration boundary; kinematics, mapping, protocol parsing và business logic nên test được mà không cần ROS graph.

---

## 27. Package `ament_python`

Cấu trúc:

```text
myarm_m750_driver/
├── package.xml
├── resource/
│   └── myarm_m750_driver
├── setup.cfg
├── setup.py
├── myarm_m750_driver/
│   ├── __init__.py
│   ├── driver_node.py
│   └── main.py
└── test/
```

`setup.py`:

```python
from setuptools import setup

package_name = 'myarm_m750_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='MyArm M750 ROS 2 driver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = myarm_m750_driver.main:main',
        ],
    },
)
```

`setup.cfg`:

```ini
[develop]
script-dir=$base/lib/myarm_m750_driver

[install]
install-scripts=$base/lib/myarm_m750_driver
```

Build:

```bash
colcon build \
  --symlink-install \
  --packages-select myarm_m750_driver

source install/setup.bash
ros2 run myarm_m750_driver driver_node
```

---

## 28. Install config và launch file

Trong `setup.py`:

```python
import os
from glob import glob
```

```python
(
    os.path.join('share', package_name, 'config'),
    glob('config/*.yaml'),
),
(
    os.path.join('share', package_name, 'launch'),
    glob('launch/*.launch.py'),
),
```

Tìm share directory:

```python
from ament_index_python.packages import (
    get_package_share_directory,
)

share_dir = get_package_share_directory(
    'myarm_m750_driver'
)
```

Không phụ thuộc current working directory trong node production.

---

## 29. Driver pattern cho MyArm M750 ở 5 Hz

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MyArmDriverNode(Node):
    JOINT_NAMES = [
        'joint1', 'joint2', 'joint3',
        'joint4', 'joint5', 'joint6',
    ]

    def __init__(self, hardware) -> None:
        super().__init__('myarm_driver')
        self.hardware = hardware

        self.declare_parameters(
            namespace='',
            parameters=[
                ('read_rate_hz', 5.0),
                ('timeout_sec', 0.1),
            ],
        )

        read_rate_hz = float(
            self.get_parameter('read_rate_hz').value
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )

        self.read_timer = self.create_timer(
            1.0 / read_rate_hz,
            self.read_and_publish,
        )

        self.hardware.connect()

    def read_and_publish(self) -> None:
        try:
            positions = self.hardware.read_joint_positions(
                timeout_sec=0.1
            )
        except TimeoutError:
            return
        except Exception as exc:
            self.get_logger().error(
                f'Read failed: {exc}'
            )
            return

        if len(positions) != len(self.JOINT_NAMES):
            self.get_logger().error(
                'Invalid joint state length'
            )
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = list(positions)
        self.joint_pub.publish(msg)

    def destroy_node(self) -> None:
        try:
            self.hardware.close()
        finally:
            super().destroy_node()
```

Nguyên tắc:

```text
- hardware adapter được inject;
- read timeout hữu hạn;
- validate số joint;
- publish radian;
- một owner của serial;
- bắt đầu ở 5 Hz để debug ổn định.
```

---

## 30. Watchdog stale state

```python
import time

self.last_state_time = None
self.watchdog_timer = self.create_timer(
    0.5,
    self.watchdog_callback,
)
```

Khi có state:

```python
self.last_state_time = time.monotonic()
```

```python
def watchdog_callback(self) -> None:
    if self.last_state_time is None:
        return

    age_sec = time.monotonic() - self.last_state_time

    if age_sec > 1.0:
        self.get_logger().error(
            f'Robot state stale: {age_sec:.3f}s'
        )
```

---

## 31. Chỉ giữ command mới nhất

```python
from threading import Lock

self.command_lock = Lock()
self.latest_command = None
```

```python
def command_callback(self, msg) -> None:
    command = self.parse_command(msg)

    with self.command_lock:
        self.latest_command = command
```

```python
def send_latest_command(self) -> None:
    with self.command_lock:
        command = self.latest_command
        self.latest_command = None

    if command is not None:
        self.hardware.send_command(command)
```

QoS depth 1:

```python
command_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
```

---

## 32. Không block executor bằng inference

Không nên:

```python
def image_callback(self, image):
    result = huge_model(image)
    self.publish_result(result)
```

Pattern tốt hơn:

```text
image subscription
    ↓
latest-frame queue depth 1
    ↓
inference worker/process
    ↓
result queue
    ↓
ROS timer/guard publish result
```

Tách inference thành node/process riêng giúp tránh GIL contention, executor starvation và driver bị trễ.

---

## 33. Test pure logic

```python
def map_joint_angles(raw_angles, offsets):
    if len(raw_angles) != len(offsets):
        raise ValueError('length mismatch')

    return [
        raw + offset
        for raw, offset in zip(raw_angles, offsets)
    ]
```

```python
def test_map_joint_angles():
    actual = map_joint_angles(
        [0.0, 1.0],
        [0.1, -0.2],
    )

    assert actual == [0.1, 0.8]
```

Giữ mapping, validation, kinematics và protocol parsing ngoài `Node` để unit test dễ hơn.

---

## 34. Common imports

```python
import rclpy

from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.duration import Duration
from rclpy.executors import (
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from rcl_interfaces.msg import SetParametersResult
```

Messages thường dùng:

```python
from diagnostic_msgs.msg import (
    DiagnosticArray,
    DiagnosticStatus,
    KeyValue,
)
from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
    Twist,
)
from sensor_msgs.msg import (
    CameraInfo,
    Image,
    JointState,
)
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)
```

---

## 35. CLI debug

```bash
# Node
ros2 node list
ros2 node info /myarm_driver

# Topic
ros2 topic list -t
ros2 topic info /joint_states
ros2 topic echo /joint_states
ros2 topic hz /joint_states
ros2 topic bw /camera/image_raw

# Service
ros2 service list -t
ros2 service call \
  /myarm/power_on \
  std_srvs/srv/Trigger \
  "{}"

# Action
ros2 action list -t
ros2 action info \
  /arm_controller/follow_joint_trajectory

# Parameter
ros2 param list /myarm_driver
ros2 param get /myarm_driver read_rate_hz
ros2 param set /myarm_driver read_rate_hz 5.0

# Logging/graph
rqt_graph
ros2 run rqt_console rqt_console
```

---

## 36. Lỗi thường gặp

### `ModuleNotFoundError: No module named 'rclpy'`

```bash
source /opt/ros/foxy/setup.bash
python3 -c "import rclpy; print(rclpy.__file__)"
```

Nguyên nhân:

```text
- chưa source Foxy;
- interpreter Python sai;
- virtual environment không thấy system packages;
- IDE chọn interpreter khác;
- chạy shell không có ROS environment.
```

Không sửa bằng `pip install rclpy`.

### `Could not import rosidl_typesupport_c`

Kiểm tra:

```text
- interface package đã build;
- đã source overlay;
- Python version đúng;
- package.xml dependency đúng;
- custom interface generation đúng.
```

Build sạch:

```bash
rm -rf build install log
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### `No executable found`

Kiểm tra `setup.py` entry point:

```python
entry_points={
    'console_scripts': [
        'driver_node = myarm_m750_driver.main:main',
    ],
}
```

```bash
colcon build --symlink-install \
  --packages-select myarm_m750_driver
source install/setup.bash
ros2 pkg executables myarm_m750_driver
```

### Callback không được gọi

Kiểm tra:

```bash
ros2 node info /node_name
ros2 topic info /topic_name
ros2 topic echo /topic_name
```

Nguyên nhân thường gặp:

```text
- sai topic/namespace/remap;
- QoS mismatch;
- node không spin;
- callback khác block executor;
- publisher không publish;
- exception trong callback;
- lỗi discovery/network.
```

### Service/action deadlock

Nguyên nhân:

```text
- synchronous call trong callback;
- cùng mutually-exclusive group;
- single-thread executor không xử lý được response;
- spin_until_future_complete ở vị trí không phù hợp.
```

Giải pháp:

```text
- call_async();
- add_done_callback();
- thiết kế callback group;
- dùng MultiThreadedExecutor khi cần;
- không block callback.
```

### Serial protocol lỗi do nhiều callback cùng truy cập

Sai:

```text
timer read_state ─┐
service reset ────┼── cùng gọi serial
command callback ─┘
```

Tốt hơn:

```text
ROS callbacks
    ↓
command queue
    ↓
một hardware worker
    ↓
serial port
```

Hoặc đặt hardware entities vào cùng `MutuallyExclusiveCallbackGroup`.

---

## 37. Code review checklist

### Lifecycle

```text
[ ] rclpy.init() một lần.
[ ] Node được destroy.
[ ] rclpy.shutdown() trong cleanup.
[ ] Thread/serial/file được đóng.
```

### Callback

```text
[ ] Không while True.
[ ] Blocking I/O có timeout.
[ ] Exception được log.
[ ] Không giữ lock lâu.
[ ] Không synchronous service/action trong callback.
```

### Topic

```text
[ ] Message type đúng.
[ ] QoS tương thích.
[ ] Timestamp đúng.
[ ] frame_id rõ ràng.
[ ] Queue depth hợp lý.
```

### Parameter

```text
[ ] Được declare.
[ ] Type/range được validate.
[ ] YAML đúng node name.
[ ] Runtime update không phá resource.
```

### Hardware

```text
[ ] Một owner của serial/device.
[ ] Có watchdog.
[ ] Có reconnect policy.
[ ] Command cũ không tích lũy.
[ ] Joint unit chuẩn hóa.
```

---

## 38. Quick reference

```python
# Init
rclpy.init(args=args)

# Node
node = Node('node_name')

# Publisher
pub = node.create_publisher(MsgType, '/topic', 10)

# Subscription
sub = node.create_subscription(
    MsgType,
    '/topic',
    callback,
    10,
)

# Timer
timer = node.create_timer(0.2, callback)

# Service server
srv = node.create_service(
    SrvType,
    '/service',
    callback,
)

# Service client
client = node.create_client(
    SrvType,
    '/service',
)
future = client.call_async(SrvType.Request())

# Parameter
node.declare_parameter('rate_hz', 5.0)
value = node.get_parameter('rate_hz').value

# Time
stamp = node.get_clock().now().to_msg()

# Logging
node.get_logger().info('message')

# Spin
rclpy.spin(node)

# Cleanup
node.destroy_node()
rclpy.shutdown()
```

---

## 39. Tài liệu tham khảo

### rclpy Foxy API

- [rclpy documentation](https://docs.ros2.org/foxy/api/rclpy/)
- [API index](https://docs.ros2.org/foxy/api/rclpy/api.html)
- [Node](https://docs.ros2.org/foxy/api/rclpy/api/node.html)
- [Initialization, shutdown and spinning](https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html)
- [Topics](https://docs.ros2.org/foxy/api/rclpy/api/topics.html)
- [Services](https://docs.ros2.org/foxy/api/rclpy/api/services.html)
- [Actions](https://docs.ros2.org/foxy/api/rclpy/api/actions.html)
- [Timers](https://docs.ros2.org/foxy/api/rclpy/api/timers.html)
- [Execution and callbacks](https://docs.ros2.org/foxy/api/rclpy/api/execution_and_callbacks.html)
- [QoS](https://docs.ros2.org/foxy/api/rclpy/api/qos.html)
- [rclpy source](https://github.com/ros2/rclpy)
- [ROS 2 examples](https://github.com/ros2/examples)

### Foxy tutorials

- [Beginner client libraries](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries.html)
- [Python publisher/subscriber](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Python service/client](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Python action server/client](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Executors](https://docs.ros.org/en/foxy/Concepts/About-Executors.html)
- [Callback groups](https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html)
- [QoS concepts](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)
- [Action design](https://design.ros2.org/articles/actions.html)

---

## 40. 15 nguyên tắc cần nhớ

```text
1. rclpy là integration layer, không phải toàn bộ application.
2. Callback phải ngắn và không block vô hạn.
3. Ưu tiên async service/action client.
4. Một owner duy nhất cho serial/hardware không thread-safe.
5. Dùng mutually-exclusive group cho hardware.
6. MultiThreadedExecutor chỉ hiệu quả khi callback groups đúng.
7. Queue command/state phải có giới hạn.
8. Header stamp và frame_id phải đúng.
9. QoS publisher/subscriber phải tương thích.
10. Parameter phải được declare và validate.
11. Dùng monotonic clock cho timeout nội bộ.
12. Cleanup thread, serial và node trong finally.
13. Không pip install rclpy.
14. Đo tần số và latency thay vì đoán.
15. Với Foxy, luôn kiểm tra API theo tài liệu Foxy.
```
