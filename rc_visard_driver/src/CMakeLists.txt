
set(SRC
  device_nodelet.cc
  camera_info_publisher.cc
  image_publisher.cc
  disparity_publisher.cc
  disparity_color_publisher.cc
  depth_publisher.cc
  confidence_publisher.cc
  error_disparity_publisher.cc
  error_depth_publisher.cc
  points2_publisher.cc
  protobuf2ros_stream.cc
  ThreadedStream.cc)

# build and install the nodelet

add_library(rc_visard_driver_nodelet ${SRC})

target_link_libraries(rc_visard_driver_nodelet rc_genicam_api_static)
target_link_libraries(rc_visard_driver_nodelet rc_dynamics_api_static)
target_link_libraries(rc_visard_driver_nodelet ${catkin_LIBRARIES})

add_executable(rc_visard_driver device_node.cc)
target_link_libraries(rc_visard_driver ${catkin_LIBRARIES})

install(TARGETS rc_visard_driver rc_visard_driver_nodelet
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_dependencies(rc_visard_driver_nodelet rc_visard_driver_gencfg)
