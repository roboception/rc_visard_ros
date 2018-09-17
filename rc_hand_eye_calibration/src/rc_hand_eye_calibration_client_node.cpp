#include "rest_hand_eye_calibration_client.h"
#include <ros/ros.h>



int main(int argc, char **argv)
{
  std::string name = "rc_hand_eye_calibration";
  ros::init(argc, argv, name);

  ros::NodeHandle pnh("~");
  std::string ip_addr = "192.168.0.1"; // TODO: better default behaviour?
  pnh.param("ip", ip_addr);

  // instantiate wrapper and advertise services
  CalibrationWrapper calib_wrapper(name, ip_addr);
  ROS_INFO("Hand eye calibration node started");

  // Use 2 threads, so the service call can wait for the image subscriber
  ros::MultiThreadedSpinner spinner(2); // TODO: is this necessary?
  spinner.spin();
  return 0;
}