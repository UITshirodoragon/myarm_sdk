# myarm_sdk

SDK mới cho MyArm M750, được phát triển tối giản và độc lập với SDK legacy.


cùng cấp thư mục dự án nên tạo venv:
```bash
python3 -m venv myarm_venv --system-site-packages
```

## Cấu trúc

- `pycore/`: mã nguồn Python core.
- `ros2_ws/`: package và workspace ROS 2.
- `requirements/`: danh sách thư viện Python.
- `docs/`: tài liệu.

## Cài đặt nhanh

```bash
./install.sh
```

Lệnh này tạo `.venv` và cài dependency runtime. Dependency phát triển được
cài bằng `./install.sh --dev`.

Hoặc dùng:

```bash
make install
make install-dev
```

ROS 2 không cài bằng pip; xem [ros2_ws/README.md](ros2_ws/README.md).

## Demo ROS 2

Demo này không kết nối robot thật. Nó phát joint state mô phỏng, dựng TF từ
URDF và mở RViz2:

```bash
cd ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch myarm_joint_state_demo demo.launch.py
```

Thông tin đóng góp nằm trong [CONTRIBUTING.md](CONTRIBUTING.md). Khi đẩy lên
GitHub, chỉ commit source, tài liệu và file cấu hình; `.gitignore` đã loại trừ
output build và cache phổ biến.
