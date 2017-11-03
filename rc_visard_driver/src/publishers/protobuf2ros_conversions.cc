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

#include "protobuf2ros_conversions.h"

#include <boost/make_shared.hpp>

namespace rc {

sensor_msgs::ImuPtr toRosImu(std::shared_ptr<roboception::msgs::Imu> imu)
{
  auto rosImu = boost::make_shared<sensor_msgs::Imu>();
  rosImu->header.frame_id = "imu";
  rosImu->header.stamp.sec = imu->timestamp().sec();
  rosImu->header.stamp.nsec = imu->timestamp().nsec();
  rosImu->orientation_covariance[0] = -1; // we dont support orientation
  rosImu->angular_velocity.x = imu->angular_velocity().x();
  rosImu->angular_velocity.y = imu->angular_velocity().y();
  rosImu->angular_velocity.z = imu->angular_velocity().z();
  rosImu->linear_acceleration.x = imu->linear_acceleration().x();
  rosImu->linear_acceleration.y = imu->linear_acceleration().y();
  rosImu->linear_acceleration.z = imu->linear_acceleration().z();

  return rosImu;
}

geometry_msgs::PoseStampedPtr toRosPoseStamped(std::shared_ptr<roboception::msgs::Frame> frame)
{
  auto protoPoseStamped = frame->pose();
  auto protoPosePose = protoPoseStamped.pose();

  auto rosPose = boost::make_shared<geometry_msgs::PoseStamped>();
  rosPose->header.frame_id = frame->parent();
  rosPose->header.stamp.sec = protoPoseStamped.timestamp().sec();
  rosPose->header.stamp.nsec = protoPoseStamped.timestamp().nsec();
  rosPose->pose.position.x = protoPosePose.position().x();
  rosPose->pose.position.y = protoPosePose.position().y();
  rosPose->pose.position.z = protoPosePose.position().z();
  rosPose->pose.orientation.x = protoPosePose.orientation().x();
  rosPose->pose.orientation.y = protoPosePose.orientation().y();
  rosPose->pose.orientation.z = protoPosePose.orientation().z();
  rosPose->pose.orientation.w = protoPosePose.orientation().w();
  return rosPose;
}

geometry_msgs::PoseWithCovarianceStampedPtr toRosPoseWithCovarianceStamped(std::shared_ptr<roboception::msgs::Frame> frame)
{
  auto protoPoseStamped = frame->pose();
  auto protoPosePose = protoPoseStamped.pose();
  auto protoCov = protoPosePose.covariance();

  auto rosPose = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
  rosPose->header.frame_id = frame->parent();
  rosPose->header.stamp.sec = protoPoseStamped.timestamp().sec();
  rosPose->header.stamp.nsec = protoPoseStamped.timestamp().nsec();
  rosPose->pose.pose.position.x = protoPosePose.position().x();
  rosPose->pose.pose.position.y = protoPosePose.position().y();
  rosPose->pose.pose.position.z = protoPosePose.position().z();
  rosPose->pose.pose.orientation.x = protoPosePose.orientation().x();
  rosPose->pose.pose.orientation.y = protoPosePose.orientation().y();
  rosPose->pose.pose.orientation.z = protoPosePose.orientation().z();
  rosPose->pose.pose.orientation.w = protoPosePose.orientation().w();
  for (int i = 0; i < protoCov.size(); i++)
  {
    rosPose->pose.covariance[i] = protoCov.Get(i);
  }
  return rosPose;
}

tf::Transform toRosTfTransform(std::shared_ptr<roboception::msgs::Frame> frame)
{
  auto pose = frame->pose().pose();
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.position().x(),
                                  pose.position().y(),
                                  pose.position().z()));
  transform.setRotation(
          tf::Quaternion(pose.orientation().x(),
                         pose.orientation().y(),
                         pose.orientation().z(),
                         pose.orientation().w()));
  return transform;
}

tf::StampedTransform toRosTfStampedTransform(std::shared_ptr<roboception::msgs::Frame> frame)
{
  tf::Transform transform = toRosTfTransform(frame);
  ros::Time t(frame->pose().timestamp().sec(),
              frame->pose().timestamp().nsec());

  return tf::StampedTransform(transform, t, frame->parent(), frame->name());
}


}