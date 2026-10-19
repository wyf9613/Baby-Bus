# Local, ARM64 ROS test/development environment. No hardware drivers.
ARG ROS_BASE_IMAGE=ros:jazzy-ros-base-noble
FROM ${ROS_BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      python3-colcon-common-extensions python3-pytest python3-yaml python3-numpy \
      python3-rosdep \
      ros-jazzy-ament-cmake-pytest ros-jazzy-tf2-ros-py ros-jazzy-launch-ros \
      ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=218 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    PYTHONDONTWRITEBYTECODE=1
COPY shell.bashrc /opt/babybus/shell.bashrc
CMD ["sleep", "infinity"]
