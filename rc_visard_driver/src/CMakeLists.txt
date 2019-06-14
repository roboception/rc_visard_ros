
set(SRC
    device_nodelet.cc
    dynamics_handlers.cc
    protobuf2ros_stream.cc
    ThreadedStream.cc
    publishers/camera_info_publisher.cc
    publishers/image_publisher.cc
    publishers/disparity_publisher.cc
    publishers/disparity_color_publisher.cc
    publishers/depth_publisher.cc
    publishers/confidence_publisher.cc
    publishers/error_disparity_publisher.cc
    publishers/error_depth_publisher.cc
    publishers/points2_publisher.cc
    publishers/protobuf2ros_publisher.cc
    publishers/protobuf2ros_conversions.cc
    publishers/genicam2ros_publisher.h)

# build and install the nodelet

link_directories(${RC_GENICAM_API_LIBRARY_DIRS})

add_library(rc_visard_driver_nodelet ${SRC})

target_link_libraries(rc_visard_driver_nodelet
  PUBLIC
    ${catkin_LIBRARIES}
  PRIVATE
    rc_genicam_api::rc_genicam_api
    ${RC_DYNAMICS_API_LIBRARIES})

add_executable(rc_visard_driver device_node.cc)
target_link_libraries(rc_visard_driver ${catkin_LIBRARIES})

install(TARGETS rc_visard_driver rc_visard_driver_nodelet
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_dependencies(rc_visard_driver_nodelet rc_visard_driver_gencfg)
