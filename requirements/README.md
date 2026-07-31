# Python dependencies

- `runtime.txt`: thư viện cần để chạy Python core.
- `dev.txt`: thêm công cụ test và lint.
- `kinematics.txt`: Pinocchio và pytransform3d cho FK/IK; chỉ cần khi chạy
  `myarm_kinematics`.

Cài bằng `./install.sh`; dùng `./install.sh --kinematics` cho Pinocchio,
hoặc `./install.sh --dev-kinematics` khi cần cả test/lint và FK/IK.

Mặc định script dùng venv của dự án tại `../myarm_venv`; có thể thay bằng
`VENV_DIR=/duong/dan/venv ./install.sh --kinematics`. Venv chạy ROS 2 nên được
tạo với `--system-site-packages` để vẫn import được `rclpy` từ ROS Foxy.
