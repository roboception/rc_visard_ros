ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
LABEL maintainer="felix.ruess@roboception.de"

RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-rc-hand-eye-calibration-client \
    && rm -rf /var/lib/apt/lists/*
