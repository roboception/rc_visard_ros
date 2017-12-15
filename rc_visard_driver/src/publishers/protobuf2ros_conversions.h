/*
 * Copyright (c) 2017 Roboception GmbH
 * All rights reserved
 *
 * Author: Christian Emmerich
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

ros::Time toRosTime(const roboception::msgs::Time &time);

sensor_msgs::ImuPtr toRosImu(const roboception::msgs::Imu  &imu);

geometry_msgs::PosePtr toRosPose(const roboception::msgs::Pose &pose);

geometry_msgs::PoseStampedPtr toRosPoseStamped(const roboception::msgs::Frame &frame);

geometry_msgs::PoseStampedPtr
toRosPoseStamped(const roboception::msgs::Pose &pose,
                 const roboception::msgs::Time &time,
                 const std::string &frame_id);

geometry_msgs::PoseWithCovarianceStampedPtr toRosPoseWithCovarianceStamped(const roboception::msgs::Frame &frame);

tf::Transform toRosTfTransform(const roboception::msgs::Pose &pose);

tf::StampedTransform toRosTfStampedTransform(const roboception::msgs::Frame &frame);

}

#endif //RC_VISARD_DRIVER_PROTOBUF2ROS_CONVERSIONS_H
