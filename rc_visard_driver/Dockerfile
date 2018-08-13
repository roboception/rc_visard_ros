FROM ros:kinetic-perception as builder
WORKDIR /workspace
COPY . rc_visard_driver
RUN DEBIAN_FRONTEND=noninteractive apt-get update && rosdep install --from-paths . --ignore-src -r -y
RUN mkdir build && cd build \
    && . /opt/ros/kinetic/setup.sh \
    && cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/$ROS_DISTRO" -DCMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO" -DCMAKE_BUILD_TYPE=Release ../rc_visard_driver \
    && make -j4 && make package

FROM ros:kinetic-ros-base
COPY --from=builder /workspace/build/*.deb /tmp
RUN DEBIAN_FRONTEND=noninteractive apt-get update \
    && DEBIAN_FRONTEND=noninteractive dpkg -i /tmp/*.deb; apt-get -f -y install \
    && rm -rf /var/lib/apt/lists/*
