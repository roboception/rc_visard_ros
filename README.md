# rc_visard

ROS interface for the [Roboception rc_visard][] 3D sensor.

## rc_visard_driver

Nodelet/node providing a ROS interface to configure the rc_visard and receive images/poses.


### Build/Installation

This rc_visard_driver depends on libprotobuf-dev, protobuf-compiler and curl
(e.g. libcurl4-gnutls-dev, libcurl4-nss-dev or libcurl4-openssl-dev) which need to be installed in the system and can also be installed via rosdep.

    rosdep install --from-paths rc_visard_driver --ignore-src rc_visard_driver -r -y

Some libraries are included as git submodules in this repository
(and themselves include other libs as git submodules).
Hence, before building this package you need to

    git submodule update --init --recursive

Building and installing the package follows the typical ROS catkin workflow.

As an alternative, the cmake build-flow would be something like

    mkdir build && cd build
    cmake -DCATKIN_BUILD_BINARY_PACKAGE="1" -DCMAKE_INSTALL_PREFIX="/opt/ros/$ROS_DISTRO" -DCMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO" -DCMAKE_BUILD_TYPE=Release rc_visard_driver
    make
    make install

### GenICam GenTL Transport Layer

The rc_visard_driver uses the GenICam/GigE Vision interface of the sensor
and requires a transport layer called a GenTL producer (shared library with the suffix `.cti`).
The path to this producer has to be set with the `GENICAM_GENTL64_PATH` environment variable (or `GENICAM_GENTL32_PATH` for 32 bit systems).

For convenience rc_visard_driver comes with producers from Baumer for common architectures (in `rc_visard_driver/ext/rc_genicam_api/baumer/Ubuntu-14.04-*/`).
If the `GENICAM_GENTL64_PATH` environment variable is not set, rc_visard_driver will fall back to searching for the Baumer producer where rc_visard_driver is installed.
This works if the install path is already set correctly during the build step (via standard `CMAKE_INSTALL_PREFIX`).

If the install target is not called (i.e. when only using the catkin devel workspace), the producer .cti can't be found and you will get an error message like

    [ERROR] [1512568083.512790905]: rc_visard_driver: No transport layers found in path GENICAM_GENTL64_PATH

In this case you need either need to actually install it or set the environment variable when running it:
E.g. export:

    GENICAM_GENTL64_PATH=/path/to/rc_visard_ros/rc_visard_driver/ext/rc_genicam_api/baumer/Ubuntu-14.04/x86_64

### Configuration and usage

See the [rc_visard_driver README](rc_visard_driver/README.md) for more details.


[Roboception rc_visard]: http://roboception.com/rc_visard
