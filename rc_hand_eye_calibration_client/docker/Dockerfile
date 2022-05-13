ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
LABEL maintainer="felix.ruess@roboception.de"
LABEL org.opencontainers.image.vendor="Roboception GmbH"
LABEL org.opencontainers.image.title="rc_hand_eye_calibration_client"
LABEL org.opencontainers.image.description="This node provides ROS service calls and topics to calibrate the rc_visard to a robot (aka hand-eye calibration)."
LABEL org.opencontainers.image.url="https://github.com/roboception/rc_visard_ros"
LABEL org.opencontainers.image.documentation="http://wiki.ros.org/rc_visard_driver"
LABEL org.opencontainers.image.source="https://github.com/roboception/rc_visard_ros"

RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-${ROS_DISTRO}-rc-hand-eye-calibration-client \
    && rm -rf /var/lib/apt/lists/*
