ARG ROS_DISTRO=melodic

FROM ros:${ROS_DISTRO}-perception as builder
ARG RC_REPO
# if RC_REPO, add internal roboception apt repo
RUN if [ -n "${RC_REPO}" ]; then \
      echo "Using roboception APT repo ${RC_REPO}"; \
      apt-key adv --keyserver keyserver.ubuntu.com --recv 3EEF55C8 \
      && echo "deb http://apt.roboception.de/stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/roboception-stable.list \
      && echo "deb http://apt.roboception.de/${RC_REPO} $(lsb_release -sc) main" > /etc/apt/sources.list.d/roboception-${RC_REPO}.list; \
    fi
WORKDIR /workspace
COPY . rc_visard_driver
RUN DEBIAN_FRONTEND=noninteractive apt-get update && rosdep install --from-paths . --ignore-src -r -y
RUN mkdir build && cd build \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/$ROS_DISTRO" -DCMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO" -DCMAKE_BUILD_TYPE=Release ../rc_visard_driver \
    && make -j4 && make package

FROM ros:${ROS_DISTRO}-perception
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
    && DEBIAN_FRONTEND=noninteractive apt install -y /tmp/*.deb \
    && rm -rf /var/lib/apt/lists/*
