FROM osrf/ros:foxy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-rqt \
    ros-foxy-rqt-common-plugins \
    ros-foxy-rqt-graph \
    ros-foxy-rqt-image-view \
    ros-foxy-tf2-tools \
    ros-foxy-image-transport \
    ros-foxy-compressed-image-transport \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-xacro \
    ros-foxy-rosbag2 \
    ros-foxy-rosbag2-storage-default-plugins \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    nano \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc

WORKDIR /root
CMD ["bash"]
