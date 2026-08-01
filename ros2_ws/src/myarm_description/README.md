# myarm_description

URDF, Xacro và mesh của MyArm M750.

`urdf/myarm_m750_poe_v3_2.urdf` là canonical baseline model. Pycore,
Pinocchio, robot driver và trajectory planner tiếp tục dùng đúng plain-URDF
này; không trỏ các thành phần đó trực tiếp vào Xacro.

`urdf/myarm_m750_application.urdf.xacro` là wrapper dành cho application. Nó
giữ nguyên toàn bộ 6R kinematic contract của baseline và chỉ có thể thêm fixed
subtree, hiện gồm wrist camera tùy chọn dưới `gripper_base_link`:

```bash
xacro $(ros2 pkg prefix myarm_description)/share/myarm_description/urdf/myarm_m750_application.urdf.xacro use_wrist_camera:=true
```

Không thêm `world`, workspace, scan view hoặc object scene vào URDF/Xacro này.
Các frame đó thuộc application bringup và được publish trên `/tf_static`.
