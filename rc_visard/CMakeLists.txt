cmake_minimum_required(VERSION 2.8.3)

project(rc_visard)
include(cmake/project_version.cmake)

find_package(catkin REQUIRED)
catkin_metapackage()

######################################
## Define information for packaging ##
######################################
# optionally specify dependencies of the debian package here (comma separated!)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-$ENV{ROS_DISTRO}-rc-hand-eye-calibration-client, ros-$ENV{ROS_DISTRO}-rc-pick-client, ros-$ENV{ROS_DISTRO}-rc-tagdetect-client, ros-$ENV{ROS_DISTRO}-rc-visard-description, ros-$ENV{ROS_DISTRO}-rc-visard-driver")
include(cmake/package_debian.cmake)
