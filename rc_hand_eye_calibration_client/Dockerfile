ARG ROS_DISTRO=melodic
FROM ros:${ROS_DISTRO}-ros-base as builder
WORKDIR /workspace
COPY . rc_hand_eye_calibration_client
RUN DEBIAN_FRONTEND=noninteractive apt-get update && rosdep install --from-paths . --ignore-src -r -y
RUN mkdir build && cd build \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/$ROS_DISTRO" -DCMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO" -DCMAKE_BUILD_TYPE=Release ../rc_hand_eye_calibration_client \
    && make -j4 && make package

FROM ros:${ROS_DISTRO}-ros-base
COPY --from=builder /workspace/build/*.deb /tmp
RUN DEBIAN_FRONTEND=noninteractive apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y /tmp/*.deb  \
    && rm -rf /var/lib/apt/lists/*
