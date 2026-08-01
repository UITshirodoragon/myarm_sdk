# Tài liệu lựa chọn API `MyArmMControl` cho ROS 2 Robot Driver MyArm M750

## 1. Mục tiêu và phạm vi

Tài liệu này xác định những API của `pymycobot.MyArmMControl` nên được sử dụng trong:

* lớp giao tiếp trung lập với phần cứng robot;
* `MyArmM750RobotArmAdapter`;
* ROS 2 Robot Driver Node;
* lộ trình tích hợp MoveIt2 và `ros2_control`.

Nguyên tắc thiết kế là:

> Firmware MyArm M750 chỉ được dùng như bộ điều khiển joint-position cấp thấp. FK, IK, mô hình tọa độ, kiểm tra giới hạn, lập kế hoạch và quản lý quỹ đạo được thực hiện ở Jetson bằng URDF, Pinocchio, MoveIt2 và ROS 2.

Việc phân loại bên dưới dựa trên catalog API bạn đã cung cấp, đối chiếu với source hiện tại của `pymycobot` và tài liệu chính thức của Elephant Robotics. 

---

## 2. Quy ước mức ưu tiên

| Mức      | Ý nghĩa                                                                                  | Thời điểm triển khai      |
| -------- | ---------------------------------------------------------------------------------------- | ------------------------- |
| **P0-A** | API quan trọng nhất, nằm trực tiếp trên đường đọc–ghi runtime                            | Bắt buộc trong v0.1.0     |
| **P0-B** | API lifecycle và cấu hình cần thiết để P0-A hoạt động an toàn                            | Bắt buộc trong v0.1.0     |
| **P1**   | Trạng thái và diagnostics có giá trị thực tế nhưng không nằm trong vòng điều khiển chính | Sau khi P0 ổn định        |
| **P2**   | API commissioning, thử nghiệm hoặc extension riêng                                       | Phát triển sau            |
| **P3**   | Không đưa vào Robot Arm Driver                                                           | Loại khỏi interface chính |

Trong đó, **P0-A chỉ gồm ba API vận động cốt lõi**:

1. `get_angles()`
2. `write_angles(angles, speed)`
3. `stop()`

Đây là bộ API nhỏ nhất nhưng hiệu quả nhất để triển khai robot driver theo joint space.

---

# 3. Cấu hình kết nối và baudrate

## 3.1 Cấu hình được chốt cho dự án

| Tham số                         |                              Giá trị đề xuất |
| ------------------------------- | -------------------------------------------: |
| Serial port                     |           `/dev/myarm_m750` qua udev symlink |
| Baudrate mặc định của dự án     |                           **1.000.000 baud** |
| Timeout khởi đầu                |                                      `0,1 s` |
| Debug SDK                       | Tắt trong runtime, chỉ bật khi commissioning |
| Số kết nối SDK                  |                        Một instance duy nhất |
| Số luồng được phép truy cập SDK |                    Một serialized I/O worker |

Constructor trong source `MyArmMControl` vẫn khai báo mặc định `115200`, nhưng baudrate là tham số đầu vào và serial port được mở bằng giá trị được cung cấp. Tài liệu ví dụ chính thức dành cho MyArm M750 cũng tạo `MyArmMControl` với baudrate `1.000.000`. Kết hợp với việc bạn đã thử nghiệm thành công ở mức này, dự án nên sử dụng **1.000.000 baud làm cấu hình mặc định chính thức**, còn `115200` chỉ giữ làm chế độ fallback khi kiểm tra firmware hoặc xử lý tương thích. 

Không nên tự động dò baudrate trong runtime. Nếu cấu hình sai, driver nên báo lỗi rõ ràng thay vì thử nhiều tốc độ và có nguy cơ làm serial buffer rơi vào trạng thái khó xác định.

## 3.2 Ý nghĩa của baudrate 1.000.000

Baudrate cao giúp giảm thời gian truyền frame serial, đặc biệt khi đọc state và gửi command liên tục. Tuy nhiên, nó không loại bỏ:

* thời gian firmware xử lý lệnh;
* thời gian chờ response;
* timeout của serial;
* độ trễ do Python và ROS executor;
* khả năng response cũ còn trong receive buffer.

Trong source hiện tại, `_mesg()` ép `has_reply=True`, gửi command rồi chờ đọc kết quả; khi không giải mã được response, API có thể trả `-1`. Vì vậy, dù sử dụng 1 Mbps, Robot Driver vẫn phải coi `MyArmMControl` là một API **blocking request–reply**, không phải streaming interface. 

---

# 4. Nhóm P0-A — API runtime quan trọng nhất

## 4.1 `get_angles()`

### Vai trò

Đây là API đọc trạng thái joint chính thức và là nguồn dữ liệu có thẩm quyền cho:

* `/joint_states`;
* `robot_state_publisher`;
* TF tree;
* MoveIt current state;
* kiểm tra sai số tracking;
* xác nhận robot đã đến đích;
* benchmark repeatability và stability.

### Tham số và kết quả

| Thành phần                | Nội dung                                             |
| ------------------------- | ---------------------------------------------------- |
| Tham số đầu vào           | Không có                                             |
| Kết quả mong đợi          | Danh sách góc joint                                  |
| Đơn vị vendor             | Độ                                                   |
| Số joint arm được sử dụng | 6 joint, tương ứng q1–q6                             |
| Giá trị lỗi cần xử lý     | `-1`, danh sách thiếu phần tử, dữ liệu không hữu hạn |

Source giải mã kết quả `GET_ANGLES` bằng phép chuyển đổi raw integer sang góc độ. Tài liệu sản phẩm của MyArm M750 cũng mô tả hệ joint q1–q6, do đó adapter không nên tự động đưa các channel servo thứ 7 hoặc thứ 8 vào arm state. 

### Xử lý bắt buộc trong adapter

Kết quả không được publish trực tiếp. Adapter phải lần lượt thực hiện:

1. xác nhận kết quả là danh sách;
2. xác nhận đúng sáu giá trị;
3. loại bỏ `-1`, `None`, NaN và infinity;
4. áp dụng mapping hardware → ROS;
5. đổi độ sang radian;
6. kiểm tra giới hạn hợp lý;
7. gắn timestamp sau khi transaction đọc hoàn tất.

Mapping đã thống nhất cho mô hình hiện tại:

[
q_{2,\mathrm{real}}=q_{2,\mathrm{ROS}}+10^\circ
]

[
q_{3,\mathrm{real}}=q_{3,\mathrm{ROS}}-10^\circ
]

Khi đọc trạng thái, adapter phải áp dụng phép biến đổi ngược.

### Tần số đề xuất

`get_angles()` là API duy nhất nên được gọi ở tần số state cao. Giai đoạn đầu nên giữ khoảng **5 Hz ổn định**, sau đó benchmark riêng tại 10 Hz, 20 Hz hoặc cao hơn. Không tăng tần số chỉ dựa trên baudrate; cần đo thêm timeout, stale response và jitter.

---

## 4.2 `write_angles(angles, speed)`

### Vai trò

Đây là API command chính và duy nhất nên dùng để gửi pose joint đầy đủ từ ROS 2 xuống firmware.

### Tham số

| Tham số  | Kiểu dữ liệu vendor  | Ý nghĩa                                         |
| -------- | -------------------- | ----------------------------------------------- |
| `angles` | Danh sách số thực    | Sáu góc joint theo quy ước phần cứng, đơn vị độ |
| `speed`  | Số nguyên `1–100`    | Mức tốc độ tương đối của firmware               |
| Return   | ACK hoặc mã response | Phải kiểm tra, không mặc định coi là thành công |

Source kiểm tra danh sách góc và `speed`, chuyển các góc sang raw integer rồi gửi một command `SEND_ANGLES`. 

### Vì sao đây là API hiệu quả nhất?

`write_angles()` gửi sáu target joint trong một transaction. Điều này tốt hơn đáng kể so với gọi `write_angle()` sáu lần vì:

* giảm số lượng transaction serial;
* tránh sáu lần chờ ACK;
* tránh robot đi qua trạng thái trung gian ngoài ý muốn;
* giữ target của toàn bộ arm nhất quán;
* đơn giản hóa cancel, retry và command sequencing.

### Quy ước của interface trung lập

Phía ROS và domain interface không nên nhận `speed` theo kiểu vendor. Nên sử dụng:

* `speed_scale` trong khoảng (0 < s \leq 1); hoặc
* `speed_percent` trong khoảng 1–100%.

Adapter mới chuyển giá trị này sang `speed` của firmware.

Không được gọi tham số này là `velocity`, vì `speed=50` không khẳng định robot đang chạy với một vận tốc vật lý cụ thể theo rad/s.

### Xử lý bắt buộc trước khi gửi

Adapter phải:

1. kiểm tra đủ sáu joint;
2. xác nhận đúng thứ tự joint;
3. kiểm tra URDF và safety limits;
4. đổi radian sang độ;
5. áp dụng mapping ROS → hardware;
6. chuyển speed scale sang `1–100`;
7. gửi một lần qua `write_angles()`;
8. kiểm tra ACK và lưu timestamp của command.

---

## 4.3 `stop()`

### Vai trò

Đây là API dừng vận động chính của driver.

### Tham số

Không có tham số.

### Trường hợp sử dụng bắt buộc

* operator yêu cầu dừng;
* hủy trajectory;
* timeout command;
* mất state feedback;
* lỗi liên tiếp từ serial;
* phát hiện target vượt giới hạn;
* ROS node chuyển sang trạng thái inactive;
* trajectory execution không còn hợp lệ;
* mất kết nối với controller cấp trên.

Source xếp `STOP` vào nhóm command trả single-value response, vì vậy driver vẫn phải kiểm tra response thay vì gửi theo kiểu fire-and-forget. 

`stop()` không đồng nghĩa với:

* power off;
* release torque;
* emergency stop đạt chuẩn an toàn công nghiệp.

Nó chỉ nên được mô tả là **software motion stop của firmware**.

---

# 5. Nhóm P0-B — lifecycle và cấu hình bắt buộc

## 5.1 Khởi tạo `MyArmMControl`

Các tham số cần được adapter quản lý:

| Tham số    | Giá trị dự án            |
| ---------- | ------------------------ |
| `port`     | Cấu hình bắt buộc        |
| `baudrate` | `1.000.000`              |
| `timeout`  | Mặc định ban đầu `0,1 s` |
| `debug`    | `false` trong runtime    |

Constructor mở serial ngay lập tức. Vì vậy, adapter nên tách rõ:

* tạo object adapter;
* connect;
* configure;
* activate;
* deactivate;
* disconnect.

Không nên để việc tạo domain object tự động mở phần cứng.

## 5.2 `set_fresh_mode(mode)`

### Tham số

| Giá trị | Ý nghĩa                                 |
| ------: | --------------------------------------- |
|     `1` | Luôn ưu tiên command mới nhất           |
|     `0` | Thực hiện command theo hàng đợi tuần tự |

Source hiện tại mô tả chính xác hai chế độ này. 

### Quyết định cho dự án

Mặc định sử dụng:

[
\texttt{fresh_mode}=1
]

Đây là lựa chọn phù hợp với external trajectory sampling, vì Jetson sẽ liên tục gửi target mới. Khi có jitter hoặc độ trễ, firmware không nên tiếp tục thực hiện một chuỗi dài các setpoint đã cũ.

`fresh_mode=0` chỉ phù hợp với trường hợp muốn firmware tự chạy tuần tự qua một hàng đợi waypoint. Điều đó đi ngược mục tiêu chuyển planning và trajectory management ra ngoài firmware.

## 5.3 `get_fresh_mode()`

Chỉ cần gọi:

* sau khi thiết lập fresh mode;
* sau reconnect;
* khi diagnostics phát hiện hành vi command bất thường.

Driver chỉ nên activate command path khi giá trị đọc lại đúng với cấu hình mong muốn.

## 5.4 `power_on()`, `power_off()` và `is_powered_on()`

### Vai trò

Ba API này tạo thành một nhóm, không nên sử dụng riêng lẻ.

| API               | Chức năng                        |
| ----------------- | -------------------------------- |
| `power_on()`      | Yêu cầu bật nguồn/enable arm     |
| `power_off()`     | Yêu cầu tắt nguồn                |
| `is_powered_on()` | Xác nhận trạng thái sau thao tác |

`is_powered_on()` trả:

* `1`: đã bật;
* `0`: đã tắt;
* `-1`: lỗi dữ liệu hoặc giao tiếp. 

Không được ép trực tiếp kết quả sang Boolean, vì trong Python `-1` cũng được coi là `True`.

`power_off()` không nên tự động chạy mỗi khi node shutdown. Việc disconnect ROS không nhất thiết đồng nghĩa với yêu cầu tắt nguồn robot. Chính sách power phải là một thao tác lifecycle rõ ràng hoặc yêu cầu operator.

---

# 6. Nhóm P1 — trạng thái và diagnostics nên tích hợp sau P0

## 6.1 `is_moving()`

### Kết quả

* `1`: robot đang chuyển động;
* `0`: robot không chuyển động;
* `-1`: lỗi.

API này hữu ích cho:

* feedback của action;
* kiểm tra `stop()` đã có hiệu lực;
* diagnostics;
* hỗ trợ xác định kết thúc trajectory.

Tuy nhiên, nó không nên là tiêu chí duy nhất để xác nhận robot đã đạt goal. Tiêu chí chính vẫn phải là sai số giữa `get_angles()` và joint goal. Source xác nhận `is_moving()` là một query riêng theo request–reply. 

Tần số hợp lý ban đầu: khoảng 1–2 Hz hoặc chỉ gọi khi cần xác nhận trạng thái.

## 6.2 `is_all_servo_enable()`

Dùng để xác định tất cả servo có ở trạng thái enable hay không.

Nên gọi:

* khi activate driver;
* sau power-on;
* sau command failure;
* trong diagnostics chậm.

Không nên gọi ở mỗi vòng state polling.

## 6.3 `get_robot_status()`

Dùng để thu thập trạng thái lỗi tổng thể từ firmware. Source hiện giải mã từng word trạng thái thành `0` hoặc danh sách vị trí bit đang bật, nhưng không cung cấp đầy đủ ý nghĩa nghiệp vụ của từng bit trong chính method. 

Vì vậy:

* lưu nguyên raw status;
* publish lên diagnostics;
* không tự đặt tên lỗi cụ thể nếu chưa có tài liệu firmware xác nhận;
* không sử dụng một bit chưa rõ nghĩa để tự động power-off.

## 6.4 Các API version

* `get_robot_modify_version()`
* `get_robot_system_version()`
* `get_robot_tool_modify_version()`
* `get_robot_tool_system_version()`

Chỉ cần gọi tại:

* startup;
* reconnect;
* quá trình ghi metadata benchmark.

Không poll định kỳ.

Các giá trị nên được ghi vào diagnostics và rosbag metadata để có thể xác định benchmark nào đã chạy với firmware nào. Source hiện cung cấp bốn API version riêng cho robot và tool. 

## 6.5 Servo diagnostics

| API                    | Mức ưu tiên | Mục đích                                   |
| ---------------------- | ----------- | ------------------------------------------ |
| `get_servo_status()`   | P1          | Trạng thái lỗi từng servo                  |
| `get_servo_temps()`    | P1          | Theo dõi nhiệt độ                          |
| `get_servo_currents()` | P1          | Quan sát current và xu hướng tải           |
| `get_servo_voltages()` | P1          | Quan sát nguồn cấp                         |
| `get_servo_speeds()`   | P2          | Dữ liệu speed theo step/s, dùng nghiên cứu |

Source mô tả servo status là danh sách mã `0–255`, temperature là danh sách giá trị, current là raw range và servo speed theo `step/s`. Voltage hiện còn được decode bằng helper `_int2coord()`, nên đơn vị thực tế cần được đối chứng trước khi công bố là volt chính xác. 

Các dữ liệu này:

* không được poll trong vòng 5 Hz của `get_angles()`;
* nên chạy khoảng 0,5–1 Hz;
* không được đưa trực tiếp vào `JointState.velocity` hoặc `JointState.effort`.

Current không phải torque nếu chưa biết torque constant, gear ratio, offset và friction. Servo speed theo `step/s` cũng không phải rad/s nếu chưa có conversion được xác minh.

---

# 7. Nhóm P2 — để phát triển sau hoặc dùng commissioning

## 7.1 `write_angle(joint_id, degree, speed)`

Chỉ nên giữ cho:

* kiểm tra từng joint;
* xác minh chiều quay;
* kiểm tra offset;
* commissioning sau bảo trì;
* debug mapping qROS ↔ qReal.

Không sử dụng trong trajectory execution vì phải gửi nhiều transaction và không cập nhật đồng thời sáu joint. Source xác định `joint_id` ở đây là 1–6 và `speed` nằm trong 1–100. 

## 7.2 `jog_angle()` và `jog_increment()`

Phù hợp với manual commissioning tool hoặc giao diện jog riêng.

Không đưa vào command path của MoveIt vì chúng:

* không biểu diễn trajectory có timestamp;
* khó đồng bộ cancel;
* dễ bypass validation;
* tạo state machine vận động khác với joint trajectory.

## 7.3 `pause()`, `resume()` và `is_paused()`

Có thể phát triển sau nếu thật sự cần pause/resume ở firmware level.

Không ưu tiên trong v0.1 vì có nguy cơ tạo hai state machine:

* action phía ROS vẫn active;
* firmware đang paused;
* trajectory manager không biết setpoint nào còn hợp lệ.

`stop()` đơn giản và rõ semantics hơn trong phiên bản đầu.

## 7.4 `get_encoder()` và `get_encoders()`

Giữ cho:

* benchmark;
* kiểm tra encoder raw;
* phân tích mapping encoder–angle;
* phát hiện backlash hoặc sai lệch.

Không dùng làm `/joint_states` chính cho đến khi xác định chính xác:

* số channel;
* zero offset;
* counts per revolution;
* chiều dương;
* wraparound;
* quan hệ với `get_angles()`.

## 7.5 `get_joint_min()` và `get_joint_max()`

Chỉ dùng startup audit hoặc tool bảo trì.

Không dùng làm joint limits authoritative vì source hiện giải mã hai giá trị angle bằng `_int2coord()`, tạo nghi vấn về scale và unit. 

Nguồn giới hạn chính phải là:

1. URDF;
2. YAML safety limits;
3. joint mapping;
4. giới hạn đã benchmark trên robot thật.

## 7.6 `is_in_position(data, mode)`

Có thể dùng như tín hiệu phụ để đối chiếu, nhưng không nên dùng làm tiêu chí chính.

Source có mô tả không nhất quán khi ghi rằng danh sách angle của MyArm có độ dài 7 trong khi command chính của arm dùng sáu joint. Firmware tolerance cũng không được method công bố. 

Tiêu chí goal nên dựa trên:

[
e_{\max}=\max_i\left|q_{i,\mathrm{measured}}-q_{i,\mathrm{goal}}\right|
]

và một khoảng settling time được cấu hình phía ROS.

---

# 8. Nhóm P3 — không đưa vào Robot Arm Driver

## 8.1 Cartesian API và firmware kinematics

Không đưa vào production arm driver:

* `get_coords()`
* `write_coord()`
* `write_coords()`
* `jog_coord()`
* `jog_rpy()`

Lý do:

* sử dụng FK/IK hoặc Cartesian interpretation bên trong firmware;
* dùng vector mm + Euler angle riêng của vendor;
* tạo nguồn pose thứ hai cạnh Pinocchio và URDF;
* khó đảm bảo khớp tool frame, world frame và joint mapping;
* firmware Cartesian còn phụ thuộc loại firmware riêng; tài liệu chính thức ghi rằng chức năng Cartesian cần firmware hỗ trợ và firmware đó có thể phải lấy riêng từ nhà sản xuất. 

`get_coords()` chỉ nên tồn tại trong benchmark so sánh:

[
T^\mathrm{firmware}*{base\rightarrow tool}
\quad\text{so với}\quad
T^\mathrm{Pinocchio}*{base\rightarrow tool}
]

Nó không được publish làm TF authoritative.

## 8.2 API frame và end type

Không đưa vào driver:

* `set_tool_reference()`
* `get_tool_reference()`
* `set_world_reference()`
* `get_world_reference()`
* `set_reference_frame()`
* `get_reference_frame()`
* `set_end_type()`
* `get_end_type()`

Tool, flange, base và world frame phải được quản lý bằng:

* URDF;
* TF2;
* calibration YAML;
* static transform;
* Pinocchio model.

Source cho thấy các API này lưu một hệ tọa độ riêng trong firmware, gồm translation theo mm và orientation theo angle. Điều đó có thể làm mô hình firmware và ROS không còn cùng một nguồn sự thật. 

## 8.3 Firmware movement và planner configuration

Không đưa vào driver:

* `set_movement_type()`
* `get_movement_type()`
* `set_plan_speed()`
* `get_plan_speed()`
* `set_plan_acceleration()`
* `get_plan_acceleration()`

Các API này điều khiển planner nội bộ theo MoveJ/MoveL và tham số tương đối `1–100`. Dự án đã chọn MoveIt2 và trajectory layer bên ngoài, do đó không nên để firmware planner tham gia thêm một lần nữa. 

## 8.4 API có khả năng thay đổi calibration hoặc cấu hình persistent

Không expose qua ROS 2 Robot Driver:

* `set_joint_min()`
* `set_joint_max()`
* `set_servo_data()`
* `set_servo_calibration()`
* `set_gripper_calibration()`

Những thao tác này phải thuộc maintenance application độc lập, có xác nhận operator và quy trình rollback.

## 8.5 Release/focus servo

Không đưa vào public Robot Arm Interface:

* `release_all_servos()`
* `release_servo()`
* `focus_servo()`

Release torque có thể làm arm tụt dưới tác dụng trọng lực. Nó không được coi là một biến thể của `stop()`.

## 8.6 `set_void_compensate(mode)`

Không bật trong runtime mặc định.

Tính năng này cho phép firmware tự bù vị trí sau chuyển động, tức robot có thể thay đổi vị trí sau khi ROS cho rằng command đã hoàn thành. Điều đó làm giảm khả năng giải thích kết quả benchmark và tạo hidden behavior.

Chỉ đánh giá API này trong một benchmark tách biệt.

## 8.7 Network, GPIO, LED và sensor ngoại vi

Không thuộc `RobotArmPort`:

* `set_ssid_pwd()`
* `get_ssid_pwd()`
* `set_server_port()`
* `set_digital_output()`
* `get_digital_input()`
* `set_basic_output()`
* `get_basic_input()`
* `set_led_color()`
* `is_tool_btn_clicked()`
* `get_tof_distance()`

Nếu cần, chúng nên nằm trong:

* provisioning tool;
* GPIO adapter;
* tool I/O node;
* sensor node riêng.

ROS 2 Control cũng phân biệt joint interface, sensor interface và GPIO interface thay vì trộn tất cả vào một robot-arm abstraction. ([Control ROS][1])

---

# 9. Phạm vi của `RobotArmPort`

Interface trung lập không nên sao chép toàn bộ bề mặt API của `MyArmMControl`. Nó chỉ cần các capability sau:

| Capability của interface  | API vendor phía adapter                 |
| ------------------------- | --------------------------------------- |
| Kết nối phần cứng         | Constructor `MyArmMControl`             |
| Ngắt kết nối              | Đóng serial tại adapter                 |
| Đọc joint position        | `get_angles()`                          |
| Gửi joint-position target | `write_angles()`                        |
| Dừng chuyển động          | `stop()`                                |
| Bật/tắt nguồn             | `power_on()`, `power_off()`             |
| Đọc power state           | `is_powered_on()`                       |
| Đọc moving state          | `is_moving()`                           |
| Đọc health                | `get_robot_status()`, servo diagnostics |
| Đọc identity              | Các API firmware version                |

Interface không chứa:

* degree;
* encoder raw;
* vendor speed;
* Cartesian coordinate;
* Euler RPY;
* `ProtocolCode`;
* servo register address;
* mã ACK đặc thù của `pymycobot`.

---

# 10. Trách nhiệm của `MyArmM750RobotArmAdapter`

Adapter là lớp duy nhất được phép biết `MyArmMControl`.

Nó phải chịu trách nhiệm về:

1. quản lý serial connection ở 1 Mbps;
2. sở hữu duy nhất một SDK instance;
3. serialize toàn bộ transaction;
4. chuyển radian ↔ degree;
5. áp dụng qROS ↔ qReal mapping;
6. chuyển speed scale ↔ vendor speed;
7. kiểm tra joint count và joint order;
8. kiểm tra limits;
9. chuẩn hóa `-1`, timeout và exception thành domain error;
10. lưu timestamp và consecutive error count;
11. không để API vendor rò rỉ lên ROS node.

Source hiện gửi frame trước rồi mới khóa phần đọc response. Vì vậy không nên giả định nhiều callback có thể gọi SDK đồng thời một cách an toàn; adapter nên khóa toàn bộ một lần gọi SDK, từ write đến khi nhận hoặc timeout response. Đây là hệ quả trực tiếp từ transaction implementation hiện tại. 

---

# 11. Phạm vi của ROS 2 Robot Driver Node

## Giai đoạn v0.1.0

Robot Driver Node chỉ cần:

* publish joint position;
* nhận joint-position command;
* stop;
* power lifecycle;
* publish diagnostics;
* quản lý timeout và stale state.

Không nên publish velocity hoặc effort giả. Trong `ros2_control`, nếu hardware không cung cấp một state interface thì `joint_state_broadcaster` có thể để trường tương ứng trống; dữ liệu bổ sung như temperature và voltage có thể nằm ở interface hoặc diagnostics riêng. ([Control ROS][2])

## Đích tích hợp

MyArm M750 phù hợp nhất với:

* `position` command interface;
* `position` state interface;
* velocity chỉ bổ sung sau khi có estimator được kiểm chứng;
* effort không công bố cho đến khi có mô hình chuyển current sang torque.

ROS 2 Control định nghĩa joint có command interface để đặt target và state interface để đọc trạng thái; state có thể được `joint_state_broadcaster` publish lên `/joint_states`. ([Control ROS][1])

`FollowJointTrajectory` và việc lấy mẫu trajectory nên nằm ở controller/node layer, không nằm trong `MyArmM750RobotArmAdapter`. MoveIt thường giao tiếp với low-level controller qua action `FollowJointTrajectory`; `ros2_control` là hướng triển khai phổ biến, nhưng một action server riêng vẫn có thể được sử dụng nếu chưa chuyển sang `ros2_control`. ([MoveIt][3])

---

# 12. Danh sách chốt cho từng giai đoạn

## v0.1.0 — bắt buộc

### Runtime hot path

* `get_angles()`
* `write_angles()`
* `stop()`

### Lifecycle/configuration

* constructor `MyArmMControl`
* `set_fresh_mode(1)`
* `get_fresh_mode()`
* `power_on()`
* `power_off()`
* `is_powered_on()`

## v0.1.1 — diagnostics và execution feedback

* `is_moving()`
* `is_all_servo_enable()`
* `get_robot_status()`
* bốn API firmware version
* `get_servo_status()`
* `get_servo_temps()`
* `get_servo_currents()`
* `get_servo_voltages()`

## Phát triển sau hoặc commissioning

* `write_angle()`
* `jog_angle()`
* `jog_increment()`
* `pause()`
* `resume()`
* `is_paused()`
* `get_encoder()`
* `get_encoders()`
* `get_joint_min()`
* `get_joint_max()`
* `is_in_position()`
* `get_servo_speeds()`

## Tách sang adapter/node riêng

* toàn bộ gripper API;
* GPIO và tool I/O;
* TOF sensor;
* LED và tool button;
* network provisioning.

## Không sử dụng trong production arm driver

* toàn bộ Cartesian command API;
* firmware FK/IK pose API;
* tool/world/reference frame API;
* MoveJ/MoveL firmware planner API;
* servo register write;
* servo calibration;
* runtime joint-limit write;
* release/focus servo công khai;
* void compensation mặc định.

---

## Kết luận

Thiết kế hiệu quả nhất không phải là bao bọc toàn bộ `MyArmMControl`, mà là **thu hẹp vendor SDK thành một position-controlled six-joint hardware adapter**.

Critical path của driver chỉ cần:

[
\boxed{
\texttt{get_angles}
;+;
\texttt{write_angles}
;+;
\texttt{stop}
}
]

Kết hợp với:

[
\boxed{
1,000,000\ \text{baud}
;+;
\texttt{fresh_mode}=1
;+;
\text{single serialized I/O worker}
}
]

Các API còn lại được bổ sung theo giá trị thực tế: lifecycle trước, diagnostics sau, commissioning tách riêng, còn Cartesian và planner firmware bị loại khỏi production path để URDF, Pinocchio, MoveIt2 và ROS 2 duy trì một nguồn động học duy nhất.

[1]: https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html "ros2_control hardware interface types — ROS2_Control: Humble Jul 2026 documentation"
[2]: https://control.ros.org/humble/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html "joint_state_broadcaster — ROS2_Control: Humble Jul 2026 documentation"
[3]: https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html "Low Level Controllers — MoveIt Documentation: Humble documentation"
