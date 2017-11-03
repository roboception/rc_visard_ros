/* 
* Roboception GmbH 
* Munich, Germany 
* www.roboception.com 
* 
* Copyright (c) 2017 Roboception GmbH 
* All rights reserved 
* 
* Author: Christian Emmerich 
*/


#ifndef RC_VISARD_DRIVER_PROTOBUF2ROS_CONVERSIONS_H
#define RC_VISARD_DRIVER_PROTOBUF2ROS_CONVERSIONS_H

#include <roboception/msgs/imu.pb.h>
#include <roboception/msgs/frame.pb.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

namespace rc
{

sensor_msgs::ImuPtr toRosImu(std::shared_ptr<roboception::msgs::Imu> imu);

geometry_msgs::PoseStampedPtr toRosPoseStamped(std::shared_ptr<roboception::msgs::Frame> frame);

geometry_msgs::PoseWithCovarianceStampedPtr toRosPoseWithCovarianceStamped(std::shared_ptr<roboception::msgs::Frame> frame);

tf::Transform toRosTfTransform(std::shared_ptr<roboception::msgs::Frame> frame);

tf::StampedTransform toRosTfStampedTransform(std::shared_ptr<roboception::msgs::Frame> frame);

}

#endif //RC_VISARD_DRIVER_PROTOBUF2ROS_CONVERSIONS_H
