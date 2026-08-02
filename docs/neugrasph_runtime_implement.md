# Tài liệu tham chiếu kiến trúc NeuGrasp ROS 2 cho MyArm M750

Tài liệu này tổng hợp các kết luận đã thống nhất sau khi đối chiếu:

* [NeuGrasp real runtime v0.1.3](sandbox:/mnt/data/NeuGrasp_real_runtime_v0_1_3.zip)
* [URDF MyArm M750 PoE v3.2](sandbox:/mnt/data/myarm_m750_poe_v3_2%282%29.urdf)
* [Cấu hình RViz hiện tại](sandbox:/mnt/data/myarm_m750.rviz)
* Paper và repository chính thức của NeuGrasp.
* Quy ước ROS về TF, URDF, camera optical frame, Marker, PointCloud2 và Fast DDS.

Tài liệu phân biệt ba loại thông tin:

* **Đã xác nhận:** có trong paper, source code hoặc file bạn cung cấp.
* **Đã chốt cho SDK:** quyết định kiến trúc nên dùng lâu dài.
* **Cần hiệu chỉnh:** thông số phải đo lại trên robot thật.

---

# 1. Mục tiêu của NeuGrasp trong SDK

Giai đoạn đầu, NeuGrasp không nhất thiết phải chạy model. Hệ thống nên hỗ trợ ba chế độ độc lập:

```text
scan_debug
    Chỉ tạo và hiển thị workspace, camera frames và scan trajectory.

replay
    Đọc lại ảnh, .npy, .ply và candidates.json từ một run cũ.
    Không chạy inference.

full_runtime
    Scan ảnh thật → inference → postprocess → chọn grasp → thực thi.
```

NeuGrasp bringup nên là **package launch điều phối nhiều node**, không phải một node lớn chứa toàn bộ camera, robot, inference, visualization và replay.

Kiến trúc đề xuất:

```text
myarm_m750_neugrasp/
├── neugrasp_scene_node
├── neugrasp_scan_node
├── neugrasp_replay_node
├── neugrasp_visualization_node
└── neugrasp_model_node          # thêm sau

myarm_m750_neugrasp_bringup/
├── launch/
└── config/
```

Paper xác định đầu vào NeuGrasp gồm tập ảnh scene, ảnh background và camera parameters tương ứng; đầu ra trung gian là SDF/TSDF volume rồi mới sinh grasp pose. Vì vậy việc ổn định frame, camera pose và replay trước khi tích hợp model là đúng thứ tự phát triển. ([arXiv][1])

---

# 2. Quy ước tọa độ phải giữ cố định

## 2.1. Quy ước ROS

Tất cả TF và URDF phải giữ hệ tọa độ tay phải theo REP-103:

```text
+X: phía trước
+Y: phía trái
+Z: phía trên
```

Camera optical frame dùng:

```text
+X_camera: bên phải ảnh
+Y_camera: hướng xuống ảnh
+Z_camera: hướng nhìn của camera
```

Không được đổi `base_link` thành `+Y phải`, vì như vậy sẽ phá vỡ quy ước ROS và gây khó khăn khi sử dụng TF, MoveIt, RViz và các package chuẩn. ([ROS][2])

## 2.2. Quy ước azimuth của ứng dụng NeuGrasp

Bạn muốn sử dụng:

```text
φ = 0°   : +X workspace
φ = 90°  : phía bên phải robot
φ = 180° : -X workspace, phía robot hiện đang đứng
φ = 270° : phía bên trái robot
```

Đây là quy ước azimuth tăng theo chiều kim đồng hồ nhìn từ `+Z` xuống, trong khi ROS có `+Y` hướng trái.

Công thức phải dùng:

[
x=r\sin\theta\cos\phi
]

[
y=r\sin\theta\sin\phi
]

[
z=r\cos\theta
]

Vì workspace ROS là hệ tay phải (`+X` trước, `+Y` trái, `+Z` lên), góc
dương quanh `+Z` theo quy tắc bàn tay phải làm cho:

```text
φ = 90° → +Y_ROS → phía trái robot
```

Đây là quy ước duy nhất của SDK, không phải một phép đảo trục riêng của
NeuGrasp. Các vị trí cuối cùng được lưu dưới dạng pose trong frame ROS tay phải.

Config nên khai báo tường minh:

```yaml
spherical_convention:
  reference_frame: neugrasp_workspace
  azimuth_zero_axis: +x
  azimuth_positive_direction: counter_clockwise_about_positive_z
  polar_zero_axis: +z
  angle_unit: degree
```

Không được nhận một config clockwise trong khi code sinh pose theo ROS CCW; nó
sẽ mirror toàn bộ trajectory qua mặt phẳng XZ.

---

# 3. Quan hệ giữa `world`, `base_link` và azimuth 180°

Trong file RViz hiện tại:

```yaml
Fixed Frame: base_link
Target Frame: base_link

View:
  Class: Orbit
  Yaw: 0.65
  Pitch: 0.45
```

File không sử dụng `world`. RViz trực tiếp dùng `base_link` làm fixed frame.

Do đó, việc robot hoặc trục nào đó xuất hiện bên trái màn hình là kết quả của camera Orbit đang nhìn xiên, không phải do `base_link` bị xoay.

Sau này có thể bổ sung:

```text
world
└── base_link
```

với:

```text
T_world_base = identity
```

Điều này hoàn toàn hợp lệ. `world` và `base_link` trùng pose không làm thay đổi robot.

Giả sử workspace nằm trước robot:

```text
T_base_workspace.translation = [d, 0, z]
```

thì:

```text
Từ base nhìn workspace:
    workspace nằm ở +X
    azimuth = 0°

Từ workspace nhìn base:
    robot nằm ở -X
    azimuth = 180°
```

Vì vậy câu “robot nằm ở azimuth 180°” phải được hiểu là:

> Robot nằm ở hướng `φ = 180°` khi lấy `neugrasp_workspace` làm tâm quan sát.

Không được hiểu là `base_link` có yaw bằng 180° so với `world`.

Khi kiểm tra hướng trong RViz, nên dùng:

```text
TopDownOrtho
Target Frame = neugrasp_workspace
```

và bật trục TF. Không nên suy luận trái/phải dựa trên Orbit View.

---

# 4. Frame tree đã chốt

Cấu trúc lâu dài nên là:

```text
world                                  # tùy chọn
└── base_link
    ├── shoulder_link
    │   └── upper_arm_link
    │       └── lower_arm_link
    │           └── forearm_link
    │               └── wrist_link
    │                   └── flange_link       # sau q6
    │                       ├── tool0
    │                       └── gripper_base_link
    │                           ├── left_gripper_link
    │                           ├── right_gripper_link
    │                           └── wrist_camera_mount_link
    │                               └── wrist_camera_link
    │                                   └── wrist_camera_optical_frame
    │
    └── neugrasp_workspace
        ├── neugrasp_volume
        ├── scan_view_00_optical
        ├── scan_view_01_optical
        ├── scan_view_02_optical
        └── scan_view_03_optical
```

Các frame có chức năng riêng biệt:

| Frame                        | Ý nghĩa                                      |
| ---------------------------- | -------------------------------------------- |
| `flange_link`                | Đầu cuối chuỗi động học 6R sau q6            |
| `tool0`                      | TCP ứng dụng dùng cho IK hoặc command        |
| `gripper_base_link`          | Phần cứng gốc của gripper                    |
| `wrist_camera_link`          | Body frame của camera                        |
| `wrist_camera_optical_frame` | Frame dùng cho ảnh, intrinsic và projection  |
| `neugrasp_workspace`         | Tâm vùng thao tác                            |
| `neugrasp_volume`            | Min corner của voxel volume                  |
| `scan_view_xx_optical`       | Pose camera mục tiêu, không phải camera thật |
| `selected_grasp`             | Frame debug động, chỉ publish khi cần        |

TF2 quản lý các frame theo một cây, trong đó mỗi child frame chỉ có một parent. `robot_state_publisher` đọc URDF, dùng `joint_states` để publish movable transforms và publish fixed joints lên `/tf_static`. ([GitHub][3])

---

# 5. Camera sau q6: chốt parent và control frame

## 5.1. Điều đã xác nhận từ URDF

URDF hiện tại có:

```text
wrist_link
└── wrist_roll_joint q6
    └── flange_link
```

Sau `flange_link` có hai nhánh fixed độc lập:

```text
flange_link
├── tool0
└── gripper_base_link
```

Cụ thể:

```text
flange_link → tool0:
    xyz = [0.118, 0, 0]
    rpy = [0, -π/2, 0]

flange_link → gripper_base_link:
    xyz = [0.031, 0, 0]
    rpy = [0, +π/2, 0]
```

Do đó, trong URDF hiện tại:

> `tool0` và `gripper_base_link` không phải cùng một frame.

Chúng là hai child frame khác nhau của `flange_link`.

## 5.2. Quyết định đã chốt

Camera:

* nằm sau q6;
* gắn vật lý trên cụm gripper;
* nên có parent vật lý là `gripper_base_link`;
* không nên parent vào `wrist_link`;
* `tool0` vẫn giữ vai trò TCP điều khiển.

Cây camera:

```text
gripper_base_link
└── wrist_camera_mount_link
    └── wrist_camera_link
        └── wrist_camera_optical_frame
```

`wrist_camera_mount_link` là frame hữu ích để biểu diễn bracket hoặc mặt gá camera. Có thể bỏ frame này trong bản đầu nếu camera gắn trực tiếp và extrinsic đơn giản.

## 5.3. Camera parent và command frame không bắt buộc giống nhau

Camera có thể gắn vào `gripper_base_link`, trong khi MoveIt hoặc robot API vẫn điều khiển `tool0`.

Do hai frame cùng thuộc một rigid subtree, có thể suy ra:

[
{}^{tool0}T_{camera}
====================

\left({}^{flange}T_{tool0}\right)^{-1}
{}^{flange}T_{gripper_base}
{}^{gripper_base}T_{camera}
]

Khi biết camera target:

[
{}^{base}T_{tool0,target}
=========================

{}^{base}T_{camera,target}
\left({}^{tool0}T_{camera}\right)^{-1}
]

Như vậy:

* URDF phản ánh chính xác vị trí phần cứng camera;
* tool0 vẫn dùng cho IK và command;
* không cần giả định tool0 trùng gripper base.

## 5.4. Đổi tên calibration

File hiện tại dùng tên:

```text
T_eef_camera
```

Tên này quá mơ hồ vì không cho biết EEF là `flange_link`, `tool0` hay `gripper_base_link`.

Nên đổi thành:

```text
T_gripper_base_camera_optical
```

hoặc:

```text
T_camera_mount_camera_optical
```

Cần đo hoặc xác nhận lại transform này. Không nên trực tiếp tái sử dụng `T_eef_camera` cũ cho đến khi xác định rõ EEF mà runtime cũ sử dụng.

---

# 6. Tổ chức URDF gọn nhất mà không phá URDF hiện tại

## 6.1. Giai đoạn đầu

Giữ nguyên file URDF hiện tại.

Publish camera bằng static TF:

```text
gripper_base_link
→ wrist_camera_link
→ wrist_camera_optical_frame
```

Phương án này đủ cho:

* RViz;
* TF lookup;
* projection ảnh;
* scan trajectory;
* replay;
* kiểm tra extrinsic.

Nhưng MoveIt sẽ không biết kích thước camera và bracket để collision checking.

## 6.2. Phương án lâu dài

Không sao chép toàn bộ URDF thành một bản NeuGrasp riêng.

Nên refactor URDF hiện tại thành Xacro core, nhưng bảo đảm output baseline không đổi:

```text
myarm_m750_description/
├── urdf/
│   ├── myarm_m750_core.xacro
│   ├── myarm_m750.urdf.xacro
│   ├── components/
│   │   └── gripper.xacro
│   └── sensors/
│       └── wrist_camera.xacro
└── meshes/
```

Wrapper chính:

```text
myarm_m750.urdf.xacro
    = core + gripper
```

Wrapper NeuGrasp có thể dùng option:

```text
use_wrist_camera:=true
```

và sinh:

```text
core + gripper + wrist camera
```

Như vậy chỉ có một nguồn động học. Xacro được ROS khuyến nghị để giảm trùng lặp và giúp URDF dễ bảo trì hơn. ([ROS Documentation][4])

Nguyên tắc:

* `use_wrist_camera:=false` phải sinh robot tương đương URDF hiện tại.
* Không đổi joint origins, axes, limits hoặc tên frame của arm.
* Không chạy hai `robot_state_publisher` cùng xuất các frame robot giống nhau.
* Camera chỉ được đưa vào URDF khi cần visual/collision geometry.
* Workspace, scan views và voxel volume không đưa vào URDF.

---

# 7. Workspace và voxel volume

## 7.1. Workspace

Paper sử dụng workspace kích thước:

```text
30 × 30 × 30 cm³
```

trong simulation và real-world setup. Real setup dùng wrist-mounted RealSense, bốn ảnh và background được chuẩn bị trước để tái sử dụng. ([arXiv][1])

Frame:

```text
neugrasp_workspace
```

nên đặt tại tâm XY của vùng thao tác trên mặt bàn.

Current runtime có hai giá trị không thống nhất:

```text
Tài liệu runtime cũ:
    [0.45, 0.00, 0.005]

Template hiện tại:
    [0.55, 0.00, 0.000]
```

Đây là thông số **cần hiệu chỉnh lại**, không được chọn theo tài liệu cũ một cách tùy ý.

Nên chỉ có một source of truth:

```yaml
workspace:
  T_base_workspace:
    translation_m: [calibrated_x, calibrated_y, calibrated_z]
    quaternion_xyzw: [0, 0, 0, 1]
```

## 7.2. Volume frame

Current bbox:

```yaml
bbox_m:
  min: [-0.15, -0.15, -0.0503]
  max: [ 0.15,  0.15,  0.2497]
```

Kích thước đúng bằng:

```text
0.30 × 0.30 × 0.30 m
```

Frame `neugrasp_volume` phải đặt tại min corner:

```text
T_workspace_volume.translation = bbox.min
```

Tensor voxel được biểu diễn local trong `neugrasp_volume`.

## 7.3. Lỗi 5 cm cần sửa

Config hiện tại còn có:

```yaml
table_from_cube_translation_m:
  [-0.15, -0.15, -0.1003]
```

Trong khi bbox min là:

```text
[-0.15, -0.15, -0.0503]
```

Chênh lệch Z:

```text
0.05 m
```

Đây là lỗi có thể làm point cloud hoặc grasp marker thấp hơn đúng 5 cm.

Quyết định đã chốt:

> Xóa `table_from_cube_translation_m` khỏi source of truth và luôn suy ra `T_workspace_volume` trực tiếp từ `bbox.min`.

```yaml
derive_volume_origin_from_bbox_min: true
```

Cube visualization không phải là TF:

* TF xác định origin/orientation;
* Marker xác định kích thước cube;
* PointCloud2 chứa dữ liệu voxel.

---

# 8. Hai profile trajectory phải được giữ độc lập

Config phải hỗ trợ nhiều profile, ít nhất:

```text
paper_phi180
neugrasp_simulation_views_16_19
neugrasp_tranning_views_16_19
custom_fixed
```

Không được hardcode một trajectory duy nhất vào node.

---

## 8.1. Profile tái tạo theo paper: `paper_phi180`

Paper công bố:

```text
4 camera views
spiral trajectory
r ∈ [0.40, 0.50] m
θ ∈ Uniform(15°, 22.5°)
φ ∈ Uniform(0°, 60°)
```

Quỹ đạo bao phủ khoảng một phần sáu bán cầu. Paper không công bố chính xác bốn pose deterministic riêng lẻ. Vì vậy bất kỳ danh sách bốn góc cố định nào cũng là **cách tái tạo theo paper**, không phải pose chính thức của tác giả. ([arXiv][1])

Với setup của bạn, profile paper được xoay để tâm arc nằm tại:

```text
φ_center = 180°
φ_span = 60°
```

Giá trị canonical theo ROS CCW cho arc này là `150° → 210°`. Profile legacy
`paper_phi180` bên dưới giữ nguyên từng pose vật lý sau khi migrate từ parser
clockwise cũ, nên thứ tự azimuth là đảo ngược:

```text
view 0: θ=15.0°, φ=210°
view 1: θ=17.5°, φ=190°
view 2: θ=20.0°, φ=170°
view 3: θ=22.5°, φ=150°
r = 0.45 m
```

Các giá trị này là quyết định benchmark của SDK.

Config:

```yaml
paper_phi180:
  type: paper_spiral
  profile_version: 1

  num_views: 4

  radius:
    mode: fixed
    value_m: 0.45
    reference_range_m: [0.40, 0.50]

  polar_deg: [15.0, 17.5, 20.0, 22.5]
  azimuth_deg: [210.0, 190.0, 170.0, 150.0]

  capture_order: [0, 1, 2, 3]
  model_input_order: [0, 1, 2, 3]
  query_view_key: 1
```

Sau khi pipeline ổn định có thể thêm:

```yaml
radius:
  mode: uniform_seeded
  min_m: 0.40
  max_m: 0.50
  seed: 1001
```

Không dùng random không seed vì background và scene phải khớp camera poses.

---

## 8.2. Profile simulation-evaluation: `neugrasp_simulation_views_16_19`

`view_id` không đủ để suy ra pose nếu không chỉ rõ camera generator. Profile
này lấy đúng thứ tự view `[16,17,18,19]` từ simulation evaluation
`src/rd/render_utils.py`, không phải packed-training generator. Source đó có
grid `6 × 4`, `r=0.50 m`, beta `[45,35,25,15]°` và azimuth ROS-CCW
`[240,260,280,300]°`.

Robot dùng một arc xoay chung `-90°` để tâm azimuth `270°` của source nằm ở
azimuth `180°` đối diện robot. Vì thử reachability hiện tại dùng `r=0.40 m`,
profile được gọi `neugrasp_simulation_views_16_19`, không tự nhận là simulation
reproduction chính xác.

```yaml
neugrasp_simulation_views_16_19:
  type: paper_spiral
  profile_version: 2
  source: simulation_evaluation_render_utils
  source_view_ids: [16, 17, 18, 19]
  source_radius_m: 0.50
  source_azimuth_deg_ccw: [240.0, 260.0, 280.0, 300.0]
  alignment_yaw_deg_ccw: -90.0
  radius_m: [0.40, 0.40, 0.40, 0.40]
  polar_deg: [45.0, 35.0, 25.0, 15.0]
  azimuth_deg: [150.0, 170.0, 190.0, 210.0]
  look_at_m: [0.0, 0.0, 0.0]
  capture_order: [view_16, view_17, view_18, view_19]
  model_input_order: [view_16, view_17, view_18, view_19]
  query_view_key: view_17
```

Alignment là metadata/quy tắc sinh profile, không phải static TF hay URDF
joint. Không được chỉnh từng view độc lập vì sẽ làm thay đổi hình học tương
đối của trajectory.

## 8.3. Profile packed-training: `neugrasp_tranning_views_16_19`

Packed-training dùng camera bank `8 × 3`, khác simulation evaluation. Với
`view_id = [16,17,18,19]`, source nominal có azimuth ROS-CCW
`[270, 292.5, 292.5, 315]°` và beta `[mid, low, high, mid]`. Radius, beta và
pose đều có randomization theo từng scene, nên `camera_pose.npy` của scene đó
vẫn là source of truth nếu cần tái tạo chính xác.

Profile nominal dưới đây dùng `r=0.45 m`, beta midpoint/nominal
`[28.5,14,43,28.5]°` và alignment chung `-112.5°` để tâm cụm ở azimuth `180°`:

```yaml
neugrasp_tranning_views_16_19:
  type: paper_spiral
  source: packed_training_render_packed_std_rand
  source_view_ids: [16, 17, 18, 19]
  source_radius_range_m: [0.40, 0.50]
  source_azimuth_deg_ccw: [270.0, 292.5, 292.5, 315.0]
  alignment_yaw_deg_ccw: -112.5
  radius_m: [0.45, 0.45, 0.45, 0.45]
  polar_deg: [28.5, 14.0, 43.0, 28.5]
  azimuth_deg: [157.5, 180.0, 180.0, 202.5]
  look_at_m: [0.0, 0.0, 0.0]
  capture_order: [view_16, view_17, view_18, view_19]
  model_input_order: [view_16, view_17, view_18, view_19]
  query_view_key: view_17
```

Tên `tranning` được giữ theo public profile ID đã chốt; không sửa thành
`training` ở nội bộ để tránh làm sai lệnh action/config của người dùng.

---

# 9. Thứ tự view có ảnh hưởng inference không?

## 9.1. Phần gần như không phụ thuộc thứ tự

NeuGrasp tạo high-level volume bằng mean và variance của feature từ nhiều view tại mỗi voxel. Paper cũng mô tả mean/variance aggregation và View Transformer dùng learnable query token. ([arXiv][1])

Trong source code bạn cung cấp:

```python
mean = sum(feature * weight, dim=view)
var  = sum(weight * (feature - mean)^2, dim=view)
```

Các phép này không phụ thuộc thứ tự view nếu dữ liệu được hoán vị đồng bộ.

View Transformer cũng không có positional encoding riêng cho index view theo kiểu “view thứ nhất, view thứ hai”.

## 9.2. Nhưng runtime hiện tại có thành phần phụ thuộc index

Runtime đang có:

```yaml
query_view_id: 1
```

Trong `planner.py`, code lấy:

```python
extrinsics[query_view_id]
intrinsics[query_view_id]
depth_range[query_view_id]
```

để tạo `que_imgs_info`.

Vì vậy không thể kết luận rằng đổi thứ tự view luôn không ảnh hưởng.

Nếu hoán vị view, bắt buộc phải hoán vị đồng bộ:

```text
scene images
background images
intrinsics
extrinsics
depth ranges
source view IDs
```

và phải cập nhật query view tương ứng.

Quyết định đã chốt:

1. Giữ một `model_input_order` canonical.
2. Không lưu query view dưới dạng index thuần.
3. Lưu query dưới dạng `source_view_id` hoặc `view_key`.
4. Resolve index sau khi đã tạo batch.

Ví dụ:

```yaml
model_input_order: [16, 17, 18, 19]
query_view_key: 17
```

Node sẽ resolve:

```text
query_view_id = 1
```

Nếu đổi model order thành:

```text
[18, 16, 19, 17]
```

thì query view 17 tự chuyển thành index 3.

## 9.3. Capture order và model input order

Hai thứ tự phải tách riêng:

```yaml
capture_order: [...]
model_input_order: [...]
```

Robot có thể chọn capture order khác để giảm đường đi. Nhưng observation builder phải sắp xếp lại dữ liệu về canonical model order trước inference.

Capture order vẫn có thể ảnh hưởng dữ liệu thật do:

* rung robot;
* object dịch chuyển;
* auto exposure;
* ánh sáng thay đổi;
* thời gian giữa các ảnh.

Do đó, mặc định nên giữ:

```text
capture_order = model_input_order
```

cho đến khi có benchmark chứng minh một thứ tự khác tốt hơn.

Một regression test nên chạy cả `4! = 24` permutations trên cùng một run và so sánh:

```text
TSDF MAE
TSDF max difference
số grasp candidates
top score
translation của top grasp
rotation của top grasp
```

---

# 10. Replay tensor `.npy` không chạy model

SDK replay trực tiếp bốn tensor raw trong `inference/`, không dùng PLY hoặc
quan hệ frame/calibration đã lưu trong run. `neugrasp_volume` của scene hiện
tại là frame duy nhất của geometry replay.

Run hiện tại chứa:

| File                  |            Shape | Nội dung                 |
| --------------------- | ---------------: | ------------------------ |
| `tsdf_vol.npy`        | `(1,1,40,40,40)` | TSDF                     |
| `qual_vol_raw.npy`    | `(1,1,40,40,40)` | grasp quality            |
| `rot_vol_raw.npy`     | `(1,4,40,40,40)` | orientation tensor       |
| `width_vol_raw.npy`   | `(1,1,40,40,40)` | gripper width            |
| `candidates.json`     |        danh sách | score, width, pose, rank |
| `selected_grasp.json` |        một grasp | grasp được chọn          |

Với:

```text
workspace_size = 0.30 m
resolution = 40
```

voxel size:

```text
0.30 / 40 = 0.0075 m
```

Khi hiển thị bằng RViz `PointCloud2` với `Style: Boxes`, point phải là tâm
voxel, không phải min corner:

```text
p_volume = (index + 0.5) * 0.0075 m
RViz Size (m) = 0.0075
```

Như vậy 40 box liên tiếp phủ đúng đoạn `[0, 0.30]` của
`neugrasp_volume`, không lệch nửa voxel.

`rot_vol_raw.npy` được runtime cũ ghi theo thứ tự quaternion `xyzw`; replay
dùng đúng convention đó, kết hợp `qual` và `width` qua cùng pipeline
`process_volumes`/`select_candidates` của runtime cũ. Không dùng
`candidates.json`, vì pose trong JSON đã được đổi sang base/TCP bằng calibration
của run cũ.

---

# 11. Quy đổi dữ liệu sang ROS visualization

## 11.1. TSDF thành `PointCloud2`

Không publish toàn bộ tensor raw chỉ để RViz xem.

Nên lọc near-surface:

```text
tsdf_low < tsdf < tsdf_high
```

sau đó tạo:

```text
sensor_msgs/PointCloud2
```

Frame ưu tiên:

```text
header.frame_id = neugrasp_volume
```

Fields:

```text
x         float32
y         float32
z         float32
rgb       float32 packed RGB (màu sinh từ TSDF)
```

Replay hiện tại dùng field packed `rgb` để RViz `RGB8` hiển thị được ngay.
Màu được tạo từ giá trị TSDF near-surface (`-0.85` tới `0.0`), không phải RGB
của camera hay PLY. Đây là lựa chọn visualization; geometry vẫn là tensor TSDF.

ROS 2 cung cấp `PointCloud2` trong `sensor_msgs` để biểu diễn point-cloud dạng binary có layout mô tả bởi `PointField`. ([ROS 2 Documentation][5])

Tensor index được map vào current volume theo:

```text
p_neugrasp_volume = (index + 0.5) * voxel_size
```

Không lookup TF trong node. TF tree hiện tại chỉ dùng bởi RViz để đưa
`neugrasp_volume` về fixed frame.

---

## 11.2. Tensor grasp thành Marker

Replay xử lý `qual_vol_raw.npy`, `rot_vol_raw.npy`, `width_vol_raw.npy` bằng
đúng Gaussian/mask/threshold/local-maxima của runtime cũ. Mỗi candidate được
đặt tại local voxel centre, xoay theo quaternion `xyzw`, và có độ mở
`width_vox * voxel_size`; candidate lớn hơn `0.08 m` bị loại.

Node xuất:

```text
visualization_msgs/Marker
type = LINE_LIST
frame_id = neugrasp_volume
```

Mỗi edge `(a,b)` tạo hai phần tử liên tiếp:

```text
points[2k]   = vertex[a]
points[2k+1] = vertex[b]
```

RViz `LINE_LIST` nối các cặp điểm `0–1`, `2–3`, `4–5` theo đúng cấu trúc này. ([ROS Documentation][6])

Node dùng cùng 4 cạnh wireframe gripper của runtime cũ, tạo một
`Marker::LINE_LIST` màu cam. Node không đọc transform hoặc calibration nào
trong run.

Visualization node nên tạo:

```text
/neugrasp/grasp_candidates
    MarkerArray

/neugrasp/selected_grasp
    PoseStamped

/neugrasp/selected_grasp_marker
    Marker
```

Không publish một TF cho mỗi candidate. Chỉ selected grasp hoặc selected pregrasp mới nên có TF debug.

---

## 11.3. Workspace và scan views

```text
Workspace volume:
    Marker::CUBE hoặc LINE_LIST

Occupied voxels nhỏ:
    Marker::CUBE_LIST

Voxel/TSDF nhiều điểm:
    PointCloud2

Scan camera frustum:
    MarkerArray

Scan target poses:
    PoseArray và/hoặc TF
```

Marker batch như `CUBE_LIST` hoặc `LINE_LIST` hiệu quả hơn tạo hàng nghìn marker riêng lẻ. ([ROS Documentation][6])

---

# 12. Topic interface đề xuất

```text
/neugrasp/workspace_marker
    visualization_msgs/Marker

/neugrasp/scan_views
    geometry_msgs/PoseArray

/neugrasp/scan_view_markers
    visualization_msgs/MarkerArray

/neugrasp/planned_camera_poses
    geometry_msgs/PoseArray

/neugrasp/measured_camera_poses
    geometry_msgs/PoseArray

/neugrasp/tsdf_cloud
    sensor_msgs/PointCloud2

/neugrasp/grasp_wireframes
    visualization_msgs/Marker

/neugrasp/replay/status
    std_msgs/String
```

Metadata mỗi view phải giữ:

```text
view_key
source_view_id
capture_index
model_input_index
timestamp
T_base_camera_planned
T_base_camera_measured
intrinsics
background_path
scene_path
```

---

# 13. DDS WLAN và QoS

Không nên gửi `.npy` hoặc `.ply` nguyên file liên tục qua DDS.

Nên chuyển dữ liệu thành message có ý nghĩa:

```text
.npy TSDF      → PointCloud2 hoặc VoxelGrid custom
.npy grasp     → Pose/MarkerArray
.ply points    → PointCloud2
.ply edges     → LINE_LIST
```

Fast DDS tự fragment sample lớn hơn khoảng 64 kB. Mất một fragment có thể làm mất toàn bộ sample ở Best Effort; Reliable có thể retransmit nhưng làm tăng tải và có thể giảm tốc độ. Với dữ liệu cỡ MB như point cloud trên Wi-Fi, tài liệu Fast DDS khuyến nghị giới hạn burst, điều chỉnh socket buffers, dùng flow controller hoặc cân nhắc TCP/Large Data mode. ([Fast DDS Documentation][7])

QoS khởi đầu:

```text
Live preview PointCloud2:
    Best Effort
    Keep Last = 1
    Volatile
    0.2–1 Hz

Final point-cloud snapshot:
    Reliable
    Keep Last = 1
    Transient Local

Scan poses, workspace marker:
    Reliable
    Keep Last = 1
    Transient Local

Grasp candidates:
    Reliable
    Keep Last = 1
    Transient Local
```

Trước khi tuning DDS XML, cần ưu tiên:

1. Downsample point cloud.
2. Chỉ publish khi dữ liệu thay đổi.
3. Giới hạn tần số.
4. Không truyền tensor float32 không cần thiết.
5. Để mesh gripper cố định trên Host, Jetson chỉ gửi pose.

---

# 14. Cấu trúc config tham chiếu

```yaml
frames:
  world: world
  base: base_link
  flange: flange_link
  tool: tool0
  gripper_base: gripper_base_link

  camera_mount: wrist_camera_mount_link
  camera_body: wrist_camera_link
  camera_optical: wrist_camera_optical_frame

  workspace: neugrasp_workspace
  volume: neugrasp_volume

camera_extrinsic:
  parent_frame: gripper_base_link
  child_frame: wrist_camera_optical_frame

  translation_m: [to_be_calibrated]
  quaternion_xyzw: [to_be_calibrated]

workspace:
  parent_frame: base_link
  frame_id: neugrasp_workspace

  translation_m: [to_be_calibrated]
  quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]

  bbox_m:
    min: [-0.15, -0.15, -0.0503]
    max: [ 0.15,  0.15,  0.2497]

  derive_volume_origin_from_bbox_min: true
  volume_resolution: 40

trajectory:
  active_profile: neugrasp_simulation_views_16_19

  spherical_convention:
    reference_frame: neugrasp_workspace
    azimuth_zero_axis: +x
    azimuth_positive_direction: counter_clockwise_about_positive_z
    polar_zero_axis: +z

  profiles:
    paper_phi180:
      type: paper_spiral
      profile_version: 1
      radius_m: [0.45, 0.45, 0.45, 0.45]
      polar_deg: [15.0, 17.5, 20.0, 22.5]
      azimuth_deg: [210.0, 190.0, 170.0, 150.0]
      capture_order: [0, 1, 2, 3]
      model_input_order: [0, 1, 2, 3]
      query_view_key: 1

    neugrasp_simulation_views_16_19:
      type: paper_spiral
      profile_version: 2
      source_view_ids: [16, 17, 18, 19]
      source: simulation_evaluation_render_utils
      source_radius_m: 0.50
      source_azimuth_deg_ccw: [240.0, 260.0, 280.0, 300.0]
      alignment_yaw_deg_ccw: -90.0
      radius_m: [0.40, 0.40, 0.40, 0.40]
      polar_deg: [45.0, 35.0, 25.0, 15.0]
      azimuth_deg: [150.0, 170.0, 190.0, 210.0]
      look_at_m: [0.0, 0.0, 0.0]
      capture_order: [view_16, view_17, view_18, view_19]
      model_input_order: [view_16, view_17, view_18, view_19]
      query_view_key: view_17

    neugrasp_tranning_views_16_19:
      type: paper_spiral
      profile_version: 1
      source: packed_training_render_packed_std_rand
      source_view_ids: [16, 17, 18, 19]
      source_radius_range_m: [0.40, 0.50]
      source_azimuth_deg_ccw: [270.0, 292.5, 292.5, 315.0]
      alignment_yaw_deg_ccw: -112.5
      radius_m: [0.45, 0.45, 0.45, 0.45]
      polar_deg: [28.5, 14.0, 43.0, 28.5]
      azimuth_deg: [157.5, 180.0, 180.0, 202.5]
      look_at_m: [0.0, 0.0, 0.0]
      capture_order: [view_16, view_17, view_18, view_19]
      model_input_order: [view_16, view_17, view_18, view_19]
      query_view_key: view_17

replay:
  run_dir: /path/to/run

  source_priority:
    - npy_json
    - ply

  tsdf:
    threshold_low: -0.85
    threshold_high: 0.0
    max_points: 30000

  grasps:
    use_candidates_json: true
    top_k: 50

  publish_rate_hz: 0.5
```

---

# 15. Những điểm đã chốt và chưa được phép thay đổi tùy ý

## Đã chốt

1. ROS dùng `+X trước, +Y trái, +Z lên`.
2. Azimuth scan dùng chuẩn ROS/REP-103: `φ=90°` ở phía `+Y` (trái robot),
   tăng ngược chiều kim đồng hồ khi nhìn từ `+Z`.
3. Robot nằm ở `φ=180°` khi xét từ tâm workspace.
4. `world → base_link` có thể là identity.
5. Camera nằm sau q6.
6. Camera gắn vật lý vào `gripper_base_link`.
7. `tool0` là TCP điều khiển, không mặc định trùng gripper base.
8. Workspace và scan frames không nằm trong URDF.
9. Không copy toàn bộ URDF cho NeuGrasp.
10. Lâu dài dùng Xacro core + optional wrist-camera macro.
11. Giữ hai profile riêng: paper và sim.
12. Canonical view order phải được lưu.
13. Query view phải theo `view_key`, không theo index cứng.
14. Replay phải hoạt động không cần model.
15. `.npy/.ply` phải được chuyển sang PointCloud2 hoặc Marker.
16. Volume origin phải được suy ra từ bbox min.
17. Không gửi file raw liên tục qua DDS WLAN.

## Cần hiệu chỉnh thực nghiệm

1. `T_base_workspace`.
2. `T_gripper_base_camera_optical`.
3. Camera intrinsic sau resize/undistort.
4. Reachability của bốn scan poses.
5. Pose error planned/measured.
6. Camera exposure và background consistency.
7. Paper trajectory nào phù hợp nhất với workspace thật.
8. QoS và point-cloud density phù hợp WLAN.

---

# 16. Checklist validation trước khi chạy model

```text
[ ] world → base_link đúng hoặc được cố ý bỏ.
[ ] Không có child TF nào có hai parent.
[ ] Camera nằm sau q6.
[ ] Camera optical +Z nhìn về tâm workspace.
[ ] tool0 và gripper_base_link không bị coi là cùng frame.
[ ] T_gripper_base_camera đã được calibrate.
[ ] Background và scene dùng cùng trajectory.
[ ] Intrinsic, ảnh và extrinsic có cùng thứ tự view.
[ ] model_input_order được canonicalize.
[ ] query_view_key resolve đúng index.
[ ] T_workspace_volume được suy ra từ bbox.min.
[ ] Đã loại bỏ mismatch Z 5 cm.
[ ] PointCloud2 nằm đúng frame.
[ ] PLY base-frame không bị transform lần hai.
[ ] Grasp marker khớp selected_grasp.json.
[ ] Replay chạy được khi skip inference.
[ ] Bốn scan pose đã kiểm tra trong RViz.
[ ] Bốn scan pose đã kiểm tra reachability trước khi chạy robot.
[ ] Point-cloud rate không làm nghẽn DDS WLAN.
[ ] URDF baseline trước và sau khi chuyển Xacro sinh cùng kinematic tree.
```

Đây nên được xem là **baseline kiến trúc NeuGrasp ROS 2 v0.1** cho SDK MyArm M750.

[1]: https://arxiv.org/html/2503.03511v1 "NeuGrasp: Generalizable Neural Surface Reconstruction with Background Priors for Material-Agnostic Object Grasp Detection"
[2]: https://www.ros.org/reps/rep-0103.html?utm_source=chatgpt.com "REP 103 -- Standard Units of Measure and Coordinate ..."
[3]: https://github.com/ros/robot_state_publisher "GitHub - ros/robot_state_publisher: Allows you to publish the state of a robot (i.e the position of its base and all joints) via the \"tf\" transform library · GitHub"
[4]: https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html?utm_source=chatgpt.com "Using Xacro to clean up your code"
[5]: https://docs.ros2.org/foxy/api/sensor_msgs/index-msg.html?utm_source=chatgpt.com "sensor_msgs Message / Service / Action Documentation"
[6]: https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html?utm_source=chatgpt.com "Marker: Display types — ROS 2 Documentation"
[7]: https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/large_data/large_data.html "15.5. Large Data Rates - 3.6.2"
