#include "rest_itempick_client.h"
#include <ros/ros.h>
#include <signal.h>

std::unique_ptr<itempick_client::ItempickWrapper> itempick_wrapper;

void sigintHandler(int sig)
{
  itempick_wrapper.reset();
  ros::shutdown();
}


int main(int argc, char **argv)
{
  std::string name = "rc_itempick";
  ros::init(argc, argv, name);
  signal(SIGINT, sigintHandler);
  ros::NodeHandle pnh("~");
  std::string host;
  pnh.param("host", host, host);
  if (host.empty()) {
    ROS_FATAL("No host set! Please set the parameter 'host'!");
    return 1;
  }

  try
  {
    // instantiate wrapper and advertise services
    itempick_wrapper.reset(new itempick_client::ItempickWrapper(host, pnh));
  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Client could not be created due to an error: %s", ex.what());
    return 1;
  }

  ROS_INFO_STREAM("Itempick node started for host: " << host);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  itempick_wrapper.reset();
}
