ARG ROS_DISTRO=noetic

FROM ros:${ROS_DISTRO}-ros-base as builder
ARG RC_REPO
# if RC_REPO, add internal roboception apt repo
RUN if [ -n "${RC_REPO}" ]; then \
      echo "Using roboception APT repo ${RC_REPO}"; \
      apt-key adv --keyserver keyserver.ubuntu.com --recv 3EEF55C8 \
      && echo "deb http://apt.roboception.de/stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/roboception-stable.list \
      && echo "deb http://apt.roboception.de/${RC_REPO} $(lsb_release -sc) main" > /etc/apt/sources.list.d/roboception-${RC_REPO}.list; \
    fi
WORKDIR /workspace
COPY . rc_hand_eye_calibration_client
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y file "rcdiscover$|ros-$ROS_DISTRO-rcdiscover$" \
    && rosdep install --from-paths . --ignore-src -r -y
RUN mkdir build && cd build \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/$ROS_DISTRO" -DCMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO" -DCMAKE_BUILD_TYPE=Release ../rc_hand_eye_calibration_client \
    && make -j4 && make package

FROM ros:${ROS_DISTRO}-ros-base
ARG RC_REPO
# if RC_REPO, add internal roboception apt repo
RUN if [ -n "${RC_REPO}" ]; then \
      echo "Using roboception APT repo ${RC_REPO}"; \
      apt-key adv --keyserver keyserver.ubuntu.com --recv 3EEF55C8 \
      && echo "deb http://apt.roboception.de/stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/roboception-stable.list \
      && echo "deb http://apt.roboception.de/${RC_REPO} $(lsb_release -sc) main" > /etc/apt/sources.list.d/roboception-${RC_REPO}.list; \
    fi
COPY --from=builder /workspace/build/*.deb /tmp
RUN DEBIAN_FRONTEND=noninteractive apt update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y "rcdiscover$|ros-$ROS_DISTRO-rcdiscover$" \
    && DEBIAN_FRONTEND=noninteractive apt install -y /tmp/*.deb  \
    && rm -rf /var/lib/apt/lists/*

LABEL org.opencontainers.image.vendor="Roboception GmbH"
LABEL org.opencontainers.image.title="rc_hand_eye_calibration_client"
LABEL org.opencontainers.image.description="This node provides ROS service calls and topics to calibrate the rc_visard to a robot (aka hand-eye calibration)."
LABEL org.opencontainers.image.url="https://github.com/roboception/rc_visard_ros"
LABEL org.opencontainers.image.documentation="http://wiki.ros.org/rc_visard_driver"
LABEL org.opencontainers.image.source="https://github.com/roboception/rc_visard_ros"
