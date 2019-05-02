ARG ROS_DISTRO=melodic
FROM ros:${ROS_DISTRO}-ros-base
LABEL maintainer="felix.ruess@roboception.de"

RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-rc-visard \
    && rm -rf /var/lib/apt/lists/*
