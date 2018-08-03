FROM ros:kinetic-ros-base
MAINTAINER Felix Ruess <felix.ruess@roboception.de>

RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-rc-visard-driver \
    && rm -rf /var/lib/apt/lists/*
