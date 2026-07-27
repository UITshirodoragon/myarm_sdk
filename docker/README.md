# Lệnh chạy Docker

```bash
xhost +local:docker

```

```bash
docker run -it  --name foxy_ros_gazebo   --network=host   --ipc=host   --gpus all   --device=/dev/dri   -e DISPLAY=$DISPLAY   -e QT_X11_NO_MITSHM=1   -e XDG_RUNTIME_DIR=/tmp/runtime-root   -e NVIDIA_VISIBLE_DEVICES=all   -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute   -e __NV_PRIME_RENDER_OFFLOAD=1   -e __GLX_VENDOR_LIBRARY_NAME=nvidia   -e ROS_DOMAIN_ID=10   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v /home:/home   foxy_ros_gazebo:20.04

```