FROM ros:noetic-ros-base
LABEL maintainer="felix.ruess@roboception.de"
LABEL org.opencontainers.image.vendor="Roboception GmbH"
LABEL org.opencontainers.image.title="rc_visard_driver"
LABEL org.opencontainers.image.description="The rc_visard_driver provides data from a Roboception rc_visard 3D sensor on several ROS topics."
LABEL org.opencontainers.image.url="https://github.com/roboception/rc_visard_ros"
LABEL org.opencontainers.image.documentation="http://wiki.ros.org/rc_visard_driver"
LABEL org.opencontainers.image.source="https://github.com/roboception/rc_visard_ros"

RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-rc-visard-driver \
    && rm -rf /var/lib/apt/lists/*
