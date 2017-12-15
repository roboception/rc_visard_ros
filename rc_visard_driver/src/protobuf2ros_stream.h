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

#ifndef RC_VISARD_ROS_PROTOBUF2ROS_STREAM_H
#define RC_VISARD_ROS_PROTOBUF2ROS_STREAM_H


#include <ros/ros.h>
#include <rc_dynamics_api/remote_interface.h>

#include "ThreadedStream.h"


namespace rc
{

/**
 * Implementation of a ThreadedStream that receives rc_visard's dynamics
 * protobuf messages and re-publishes them as ros messages
 *
 * The ros topic is the dynamics stream name; the ros message type is determined
 * by the protobuf message type of the dynamics stream.
 */
class Protobuf2RosStream : public ThreadedStream
{
  public:
    Protobuf2RosStream(rc::dynamics::RemoteInterface::Ptr rcdIface,
                       const std::string &stream, ros::NodeHandle &nh,
                       const std::string &frame_id_prefix)
            : ThreadedStream(rcdIface, stream, nh), _tfPrefix(frame_id_prefix)
    {}

  protected:
    virtual bool startReceivingAndPublishingAsRos() override;

    const std::string _tfPrefix;
};


/**
 * Specific implementation for roboception::msgs::Frame messages. It publishes
 * received messages not only as ros StampedPoses but also on tf if desired.
 *
 * Again, the resulting ros topic is the dynamics stream name.
 */
class PoseStream : public Protobuf2RosStream
{
  public:
    PoseStream(rc::dynamics::RemoteInterface::Ptr rcdIface,
               const std::string &stream, ros::NodeHandle &nh,
               const std::string &frame_id_prefix, bool tfEnabled)
            : Protobuf2RosStream(rcdIface, stream, nh, frame_id_prefix),
              _tfEnabled(tfEnabled)
    {}

  protected:
    virtual bool startReceivingAndPublishingAsRos() override;

    bool _tfEnabled;
};

/**
 * Specific implementation for roboception::msgs::Dynamics messages. It publishes
 * the different field of the received message as several messages:
 * * pose -> PoseStamped (topic is the dynamics stream name)
 * * velocities and accelerations -> Marker-Arros (goes to pre-defined rviz-topic)
 *
 */
class DynamicsStream : public Protobuf2RosStream
{
  public:
    DynamicsStream(rc::dynamics::RemoteInterface::Ptr rcdIface,
               const std::string &stream, ros::NodeHandle &nh,
               const std::string &frame_id_prefix)
            : Protobuf2RosStream(rcdIface, stream, nh, frame_id_prefix)
    {}

  protected:
    virtual bool startReceivingAndPublishingAsRos() override;
};

}
#endif //RC_VISARD_ROS_PROTOBUF2ROS_STREAM_H
