cmake_minimum_required(VERSION 2.8.3)

project(rc_visard_description)
include(cmake/project_version.cmake)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

######################################
## Define information for packaging ##
######################################
include(cmake/package_debian.cmake)
