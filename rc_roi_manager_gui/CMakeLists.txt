cmake_minimum_required(VERSION 2.8.12)

project(rc_roi_manager_gui)
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

find_package(catkin REQUIRED COMPONENTS
        roscpp geometry_msgs shape_msgs visualization_msgs tf
        interactive_markers rc_common_msgs rc_pick_client
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(wxWidgets_USE_STATIC ON)
set(wxWidgets_USE_UNICODE ON)
find_package(wxWidgets REQUIRED COMPONENTS html adv core base)
if(wxWidgets_FOUND)
  include(${wxWidgets_USE_FILE})
endif(wxWidgets_FOUND)

################################################
## Declare ROS messages, services and actions ##
################################################

#file(GLOB_RECURSE MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "msg/*.msg")
#file(GLOB_RECURSE SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "srv/*.srv")

## Generate messages in the 'msg' folder
#add_message_files(FILES ${MSG_FILES})

# Generate services in the 'srv' folder
#add_service_files(FILES ${SRV_FILES})

# Generate added messages and services with any dependencies listed here
#generate_messages(
#        DEPENDENCIES
#
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# Generate dynamic reconfigure parameters in the 'cfg' folder

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
        CATKIN_DEPENDS message_runtime geometry_msgs rc_common_msgs
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
set(rc_roi_manager_gui_src
        src/roi_manager_app.cc
        src/roi_manager_frame.cc
        src/new_roi_frame.cc
        src/interactive_roi_selection.cc
)

add_executable(rc_roi_manager_gui ${rc_roi_manager_gui_src})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(rc_roi_manager_gui rc_pick_client_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_include_directories(rc_roi_manager_gui     PRIVATE       include ${wxWidgets_INCLUDE_DIRS}   )
target_link_libraries(rc_roi_manager_gui ${wxWidgets_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS rc_roi_manager_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES
        launch/interactive_roi_selection.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch  )

install(FILES
        config/config.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config  )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_logging.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

######################################
## Define information for packaging ##
######################################
# optionally specify dependencies of the debian package here (comma separated!)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-$ENV{ROS_DISTRO}-rc-pick-client")
include(cmake/package_debian.cmake)
