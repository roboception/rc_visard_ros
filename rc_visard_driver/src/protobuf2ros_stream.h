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

#ifndef RC_VISARD_DRIVER_DYNAMICS_PUBLISHER_H
#define RC_VISARD_DRIVER_DYNAMICS_PUBLISHER_H


#include <ros/ros.h>

#include <memory>
#include <thread>
#include <atomic>

#include <rc_dynamics_api/remote_interface.h>
#include "ThreadedStream.h"


namespace rc
{


class Protobuf2RosStream : public ThreadedStream
{
  public:
    Protobuf2RosStream(rc::dynamics::RemoteInterface::Ptr rcdIface,
               const std::string &topic, ros::NodeHandle &nh)
            : ThreadedStream(rcdIface, topic, nh)
    {}

  protected:
    static ros::Publisher
    CreateRosPublisherForPbMsgType(const std::string &pbMsgType,
                                   ros::NodeHandle &nh,
                                   const std::string &topic);

    virtual bool startReceivingAndPublishingAsRos() override;

    virtual void
    publishProtobufAsRos(std::shared_ptr<::google::protobuf::Message> pbMsg,
                         ros::Publisher &pub);
};


class PoseStream : public ThreadedStream
{
  public:
    PoseStream(rc::dynamics::RemoteInterface::Ptr rcdIface,
               const std::string &topic, ros::NodeHandle &nh, bool tfEnabled)
            : ThreadedStream(rcdIface, topic, nh), _tfEnabled(tfEnabled)
    {}

    virtual bool startReceivingAndPublishingAsRos() override;

  protected:
    bool _tfEnabled;
};

}
#endif //RC_VISARD_DRIVER_DYNAMICS_PUBLISHER_H
