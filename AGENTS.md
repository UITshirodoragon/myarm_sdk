# Coding rules for myarm_sdk

## Mục tiêu

- Giữ SDK nhỏ, dễ đọc và dễ debug.
- Không thêm abstraction nếu chưa có use case rõ ràng.
- ROS 2 là phần tích hợp riêng trong `ros2_ws/`.
- Không phụ thuộc vào thư mục legacy `myarm_m750_sdk`.

## Quy tắc thay đổi

- Ưu tiên một thay đổi nhỏ cho mỗi lần làm việc.
- Chạy kiểm tra tối thiểu trước khi bàn giao.
- Không commit `build/`, `install/`, `log/`, virtualenv hoặc cache.
- Dependency mới phải được ghi vào `requirements/` và giải thích ngắn trong
  changelog hoặc commit message.
- Không đổi tên public API hoặc ROS package nếu chưa cập nhật tài liệu liên
  quan.

## Cấu trúc hiện tại

- `pycore/`: Python SDK core.
- `ros2_ws/`: workspace/package ROS 2.
- `requirements/`: dependency lists.
- `docs/`: tài liệu thiết kế và sử dụng.
