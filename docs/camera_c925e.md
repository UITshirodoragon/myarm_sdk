# Camera stack Logitech C925e v1

`cam01` là Logitech C925e gắn wrist. Luồng runtime duy nhất là:

```text
myarm_camera_node -> CameraService -> CameraInterface -> OpenCVCameraAdapter
```

Node không mở V4L2 hoặc publish TF. `robot_state_publisher`, thông qua Xacro,
là owner duy nhất của `wrist_camera_link` và
`wrist_camera_optical_frame`.

## Cấu hình đã chốt

- Raw image: MJPG, `1280x720@30`, `bgr8`, không crop/rotate/resize.
- Thiết bị production: đường dẫn `/dev/v4l/by-id/...-video-index0`; tuyệt đối
  không dùng `/dev/video0` hay `fallback_index`.
- Intrinsic canonical:
  `pycore/src/myarm_sdk/plugin_adapter/camera/config/calibration/logitech_c925e_cam01_1280x720_mjpg.json`.
  Nó lưu SHA-256 provenance của calibration ngày 2026-06-02, RMS 0.357 px.
- Extrinsic canonical:
  `ros2_ws/src/myarm_description/config/camera_profiles/logitech_c925e_wrist_v1.measurement.yaml`.
  Profile đã `CALIBRATED` cho mount riêng; optical offset là `[0.04, 0, 0.03]`,
  RPY `[-pi/2, 0, -pi/2]`.
- `cam02` là slot generic disabled. Nó bị từ chối cho đến khi có device path,
  intrinsic và extrinsic riêng đã calibration.

## Commissioning trước khi bật hardware

1. Cắm camera và xác nhận endpoint bằng `v4l2-ctl --list-devices` và
   `ls -l /dev/v4l/by-id/`. Ghi đúng symlink `*-video-index0` vào
   `plugin_adapter/camera/config/cam01.yaml`; không thay bằng index số.
2. Xác nhận `v4l2-ctl --device <by-id-path> --list-formats-ext` có
   `MJPG 1280x720 30 fps`; kiểm tra các UVC control cần dùng.
   Template [99-myarm-c925e-check.rules.template](udev/99-myarm-c925e-check.rules.template)
   chỉ dùng để kiểm tra mapping persistent khi cần điều tra udev; runtime vẫn
   phải giữ chính symlink `/dev/v4l/by-id/...-video-index0` trong config.
3. Cài SDK vào venv, build ROS workspace và source overlay:

   ```bash
   cd /home/ktmt-agx-xv/Data/khoanhd/MyArmM750_Controller_Lab/myarm_sdk
   ./install.sh --camera
   source /opt/ros/foxy/setup.bash
   cd ros2_ws
   colcon build --symlink-install --packages-up-to myarm_camera myarm_bringup
   source install/setup.bash
   ```

4. Bật stack với `camera_profile:=cam01`. Adapter phải read-back chính xác
   MJPG/1280x720/30, nếu không node offline và không được mở camera khác.

```bash
ros2 launch myarm_bringup myarm_system.launch.py camera_profile:=cam01
```

Topics là `/myarm/cameras/cam01/image_raw`, `camera_info` và `diagnostics`.
`Image` và `CameraInfo` có cùng stamp và frame
`wrist_camera_optical_frame`; `CameraInfo` dùng `plumb_bob`, `R=I`, `P` từ K.

Rút cáp phải chỉ làm `cam01` offline rồi reconnect bằng exponential backoff;
nó không được chuyển sang camera khác và không được ảnh hưởng robot node.

`fake_dual` là profile test của package `myarm_camera`; nó tạo hai adapter
in-memory để kiểm thử launch/QoS và không đại diện cho camera vật lý hay TF
thứ hai.
