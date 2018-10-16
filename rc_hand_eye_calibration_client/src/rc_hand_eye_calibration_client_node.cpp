#include "rest_hand_eye_calibration_client.h"
#include <ros/ros.h>



int main(int argc, char **argv)
{
  std::string name = "rc_hand_eye_calibration";
  ros::init(argc, argv, name);

  ros::NodeHandle pnh("~");
  std::string ip_addr = "";
  pnh.param("ip", ip_addr, ip_addr);

  if (ip_addr.empty()) {
    throw std::invalid_argument("No IP address set! Please set the parameter 'ip'!");
  }

  // instantiate wrapper and advertise services
  CalibrationWrapper calib_wrapper(name, ip_addr, pnh);
  ROS_INFO_STREAM("Hand eye calibration node started for device with ip address: " << ip_addr);

  // start ros node
  ros::spin();
  return 0;
}