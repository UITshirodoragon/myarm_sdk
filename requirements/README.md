# Python dependencies

- `runtime.txt`: thư viện cần để chạy Python core.
- `dev.txt`: thêm công cụ test và lint.
- `kinematics.txt`: Pinocchio và pytransform3d cho FK/IK; chỉ cần khi chạy
  `myarm_kinematics`.

Cài bằng `./install.sh`; dùng `./install.sh --kinematics` cho Pinocchio,
hoặc `./install.sh --dev-kinematics` khi cần cả test/lint và FK/IK.

`./install.sh --robot-arm` cài extra `pymycobot` cho
`MyArmM750RobotArm`. Dùng `./install.sh --robot-arm-kinematics` khi Jetson
chạy đồng thời robot driver và FK/IK; thêm `--dev-robot-arm-kinematics` khi
cần test/lint.

Mặc định script dùng venv của dự án tại `../myarm_venv`; có thể thay bằng
`VENV_DIR=/duong/dan/venv ./install.sh --kinematics`. Venv chạy ROS 2 nên được
tạo với `--system-site-packages` để vẫn import được `rclpy` từ ROS Foxy.
