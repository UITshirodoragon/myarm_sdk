# Contributing

1. Đọc `AGENTS.md` và tạo thay đổi nhỏ, có mục tiêu rõ ràng.
2. Không commit output ROS 2 (`build/`, `install/`, `log/`) hoặc virtualenv.
3. Mô tả cách kiểm tra thay đổi trong pull request.

Với ROS 2, kiểm tra tối thiểu là build workspace và chạy demo nếu thay đổi có
ảnh hưởng tới URDF, launch hoặc joint state.
