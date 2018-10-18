FROM ros:melodic-ros-base
LABEL maintainer="felix.ruess@roboception.de"

RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-rc-visard-driver \
    && rm -rf /var/lib/apt/lists/*
