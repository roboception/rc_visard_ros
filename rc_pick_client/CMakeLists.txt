cmake_minimum_required(VERSION 2.8.12)

project(rc_pick_client)
include(cmake/project_version.cmake)


# - Configuration for code optimization -

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

# - Standard definitions -

if (CMAKE_VERSION VERSION_LESS "3.1.0")
    add_definitions(-std=c++11)
else ()
    set(CMAKE_CXX_STANDARD 11)
endif ()
add_definitions(-Wall)

add_subdirectory(opt)
# add a target for the submodule CPR doing the REST api calls

find_package(catkin REQUIRED COMPONENTS
        roscpp message_generation std_srvs geometry_msgs dynamic_reconfigure shape_msgs visualization_msgs tf
        rc_common_msgs tf2_geometry_msgs
  )

## System dependencies are found with CMake's conventions
find_package(RCDISCOVER REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
        Box.msg
        CellFillingLevel.msg
        CollisionDetection.msg
        Compartment.msg
        GridSize.msg
        Item.msg
        ItemModel.msg
        LoadCarrier.msg
        LoadCarrierWithFillingLevel.msg
        RangeBox.msg
        RangeRectangle.msg
        RangeValue.msg
        Rectangle.msg
        RegionOfInterest.msg
        SuctionGrasp.msg
)

# Generate services in the 'srv' folder
add_service_files(FILES
        ComputeBoxGrasps.srv
        ComputeGrasps.srv
        DeleteLoadCarriers.srv
        DeleteRegionsOfInterest.srv
        DetectFillingLevel.srv
        DetectItems.srv
        DetectLoadCarriers.srv
        GetLoadCarriers.srv
        GetRegionsOfInterest.srv
        SetLoadCarrier.srv
        SetRegionOfInterest.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        geometry_msgs
        shape_msgs
        rc_common_msgs

)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(cfg/pickModule.cfg)

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
        ${CPR_INCLUDE_DIRS}
        ${JSON_INCLUDE_DIRS}
)


## Declare a cpp executable

set(common_pick_src
        src/rc_pick_client_node.cpp
        src/itempick_client.cpp
        src/boxpick_client.cpp
        src/rest_helper.cpp
        src/visualization.cpp
        src/pick_client.cpp
        )

add_executable(rc_itempick_client_node ${common_pick_src})
target_compile_definitions(rc_itempick_client_node PRIVATE -DPICK_TYPE="rc_itempick")
add_executable(rc_boxpick_client_node ${common_pick_src})
target_compile_definitions(rc_boxpick_client_node PRIVATE -DPICK_TYPE="rc_boxpick")

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(rc_itempick_client_node rc_pick_client_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rc_itempick_client_node rc_pick_client_gencfg)
add_dependencies(rc_boxpick_client_node rc_pick_client_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rc_boxpick_client_node rc_pick_client_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(rc_itempick_client_node
        ${catkin_LIBRARIES}
        ${CPR_LIBRARIES}
        rcdiscover::rcdiscover
        )
target_link_libraries(rc_boxpick_client_node
        ${catkin_LIBRARIES}
        ${CPR_LIBRARIES}
        rcdiscover::rcdiscover
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS rc_itempick_client_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS rc_boxpick_client_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

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
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "cvkit (>= 1.7.0), ros-$ENV{ROS_DISTRO}-rc-imagecommon")
include(cmake/package_debian.cmake)
