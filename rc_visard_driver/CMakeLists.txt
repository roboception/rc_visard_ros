cmake_minimum_required(VERSION 2.8.3)

project(rc_visard_driver)
include(cmake/project_version.cmake)

# - Configuration for code optimization -

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

# - Standard definitions -

if (CMAKE_MAJOR_VERSION VERSION_LESS "3.1.0")
  add_definitions(-std=c++11)
else ()
  set(CMAKE_CXX_STANDARD 11)
endif ()
add_definitions(-Wall)
add_definitions(-Wno-unknown-pragmas)

# add a target to generate API documentation with Doxygen

add_subdirectory(doc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_srvs
  sensor_msgs
  image_transport
  dynamic_reconfigure
  geometry_msgs
  tf
  nav_msgs
  visualization_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    GetTrajectory.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        nav_msgs
)


## System dependencies are found with CMake's conventions
find_package(RC_GENICAM_API 2.0.0 REQUIRED)
find_package(RC_DYNAMICS_API 0.8 REQUIRED)

#########################################
## Definitions for dynamic reconfigure ##
#########################################

generate_dynamic_reconfigure_options(cfg/rc_visard_driver.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rc_stereocamera
  CATKIN_DEPENDS nodelet roscpp sensor_msgs geometry_msgs std_srvs nav_msgs
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RC_GENICAM_API_INCLUDE_DIRS}
  ${RC_DYNAMICS_API_INCLUDE_DIRS}
)

add_subdirectory(src)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
        rc_visard_driver_nodelet.xml
        README.md
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

######################################
## Define information for packaging ##
######################################
# optionally specify dependencies of the debian package here (comma separated!)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "rc-genicam-api (>= 2.0.0) | ros-$ENV{ROS_DISTRO}-rc-genicam-api (>= 2.0.0), rc-dynamics-api | ros-$ENV{ROS_DISTRO}-rc-dynamics-api, ros-$ENV{ROS_DISTRO}-image-pipeline, ros-$ENV{ROS_DISTRO}-tf")

include(cmake/package_debian.cmake)
