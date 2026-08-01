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
giữ cùng tên arguments để tương thích với wrapper generic, nhưng cố định parent
camera là `gripper_base_link`:

```bash
xacro $(ros2 pkg prefix myarm_description)/share/myarm_description/urdf/myarm_m750_neugrasp.urdf.xacro \
  use_wrist_camera:=true \
  wrist_camera_mount_xyz:="x y z" \
  wrist_camera_mount_rpy:="roll pitch yaw" \
  wrist_camera_xyz:="x y z" \
  wrist_camera_rpy:="roll pitch yaw"
```

Camera có đúng một owner là `robot_state_publisher` qua URDF/Xacro:

```text
gripper_base_link
└── wrist_camera_mount_link
    └── wrist_camera_link
        └── wrist_camera_optical_frame
```

Các transform có nghĩa chính xác:

```text
T_gripper_base_optical =
  T_gripper_base_mount × T_mount_camera_body × T_camera_body_optical
```

- `wrist_camera_mount_*`: transform bracket/mount đã calibration.
- `wrist_camera_*`: offset từ mount đến body frame camera.
- `camera_body → optical`: cố định trong macro theo REP-103 (`x` phải, `y`
  xuống, `z` nhìn tới trước).

Template [calibration](config/neugrasp_wrist_camera_calibration.template.yaml)
quy định schema version, ID/hash provenance, trạng thái và hai transform đầu
tiên. `calibration_sha256` là hash của canonical calibration payload, không
tính chính field hash đó. Nó là template, không phải default dùng trên robot.
Launch live phải từ chối calibration có
`status != CALIBRATED`, ID/hash chưa điền, hoặc parent/frame không khớp.
Profile fake có thể dùng một calibration riêng với `status: FAKE` và tên file
rõ ràng.

Không thêm `world`, workspace, volume, scan view, selected grasp hay object
scene vào URDF/Xacro này. Các frame đó thuộc application runtime; scan view là
output động của scan planner, không phải fixed transform trong URDF hay
`/tf_static`.
