#include "rest_hand_eye_calibration_client.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  std::string name = "rc_hand_eye_calibration_client";
  ros::init(argc, argv, name);

  ros::NodeHandle pnh("~");
  std::string host;
  pnh.param("host", host, host);

  if (host.empty()) {
    ROS_FATAL("No host set! Please set the parameter 'host'!");
    return 1;
  }

  std::unique_ptr<CalibrationWrapper> calib_wrapper;

  try
  {
    // instantiate wrapper and advertise services
    calib_wrapper.reset(new CalibrationWrapper(host, pnh));
  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Client could not be created due to an error: %s", ex.what());
    return 1;
  }

  ROS_INFO_STREAM("Hand eye calibration node started for host: " << host);

  // start ros node
  ros::spin();
}
