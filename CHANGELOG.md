# Changelog

## Unreleased

- Tái cấu trúc pycore thành `core`, `port_interface`, `plugin_adapter` và
  `service`.
- Thêm service manifest duy nhất `service/config/services.yaml`, service
  kinematics 5 Hz và named poses `zero`/`home`.
- Đổi ROS package FK/IK từ `myarm_kinematics_demo` thành `myarm_kinematics`.

## 0.0.1 - 2026-07-25

- Tạo skeleton tối giản cho `myarm_sdk`.
- Tách khu vực Python core và ROS 2 workspace.
- Thêm bộ file hướng dẫn coding agent và cài đặt dependency.
