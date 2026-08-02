# myarm_description

URDF, Xacro và mesh của MyArm M750.

`urdf/myarm_m750_poe_v3_2.urdf` là canonical baseline model. Pycore,
Pinocchio, robot driver và trajectory planner tiếp tục dùng đúng plain-URDF
này; không trỏ các thành phần đó trực tiếp vào Xacro.

`urdf/myarm_m750_application.urdf.xacro` là wrapper generic dành cho
application. Nó giữ nguyên toàn bộ 6R kinematic contract của baseline và chỉ
có thể thêm fixed subtree, hiện gồm wrist camera tùy chọn:

```bash
xacro $(ros2 pkg prefix myarm_description)/share/myarm_description/urdf/myarm_m750_application.urdf.xacro use_wrist_camera:=true
```

`urdf/myarm_m750_neugrasp.urdf.xacro` là profile dành riêng cho NeuGrasp. Nó
giữ parent camera là `gripper_base_link`. Nó hỗ trợ `generic` (transform từ
calibration YAML) và các named model profile. Profile hiện có là
`logitech_c925e_wrist_v1`:

```bash
xacro $(ros2 pkg prefix myarm_description)/share/myarm_description/urdf/myarm_m750_neugrasp.urdf.xacro \
  use_wrist_camera:=true \
  wrist_camera_profile:=logitech_c925e_wrist_v1
```

Camera có đúng một owner là `robot_state_publisher` qua URDF/Xacro:

```text
gripper_base_link
└── logitech_c925e_wrist_mount_link
    └── wrist_camera_link
        └── wrist_camera_optical_frame
```

Các transform có nghĩa chính xác:

```text
T_gripper_base_optical =
  T_gripper_base_mount × T_mount_camera_body × T_camera_body_optical
```

- C925e mount nằm cố định và có tên riêng trong
  `xacro/sensors/mounts/logitech_c925e_wrist_mount.xacro`.
- C925e body/optical nằm trong
  `xacro/sensors/profiles/logitech_c925e_wrist_camera.xacro`, gồm cả offset
  đo được `wrist_camera_link -> wrist_camera_optical_frame`.
- `generic` nhận `wrist_camera_mount_*`, `wrist_camera_*` và
  `wrist_camera_optical_*` từ Xacro argument; đây là đường dành cho một model
  camera chưa có profile riêng.

Template [calibration](config/neugrasp_wrist_camera_calibration.template.yaml)
quy định schema version, ID/hash provenance, trạng thái, `camera_profile` và
ba transform. Bản đo C925e được lưu tại
`config/camera_profiles/logitech_c925e_wrist_v1.measurement.yaml` với trạng
thái `CALIBRATED`; nó là nguồn geometry đã được kiểm chứng cho profile wrist
C925e, không phải default dùng trên robot. `calibration_sha256` là hash của
canonical calibration payload, không tính chính field hash đó. Launch live
phải từ chối calibration có
`status != CALIBRATED`, ID/hash chưa điền, profile không hỗ trợ, hoặc
parent/frame không khớp. Profile fake có thể dùng calibration riêng với
`status: FAKE` và tên file rõ ràng.

Không thêm `world`, workspace, volume, scan view, selected grasp hay object
scene vào URDF/Xacro này. Các frame đó thuộc application runtime; scan view là
output động của scan planner, không phải fixed transform trong URDF hay
`/tf_static`.
