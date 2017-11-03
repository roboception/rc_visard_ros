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

#include "protobuf2ros_stream.h"

#include "publishers/protobuf2ros_publisher.h"
#include "publishers/protobuf2ros_conversions.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <rc_dynamics_api/unexpected_receive_timeout.h>


using namespace std;

namespace rc
{

namespace rcd = dynamics;


bool Protobuf2RosStream::startReceivingAndPublishingAsRos()
{
  unsigned int timeoutMillis = 500;
  string pbMsgType = _rcdyn->getPbMsgTypeOfStream(_stream);

  Protobuf2RosPublisher rosPublisher(_nh, _stream, pbMsgType, _tfPrefix);

  unsigned int cntNoListener = 0;
  bool failed = false;

  while (!_stop && !failed)
  {

    // start streaming only if someone is listening

    if (!rosPublisher.used())
    {
      // ros getNumSubscribers usually takes a while to register the subscribers
      if (++cntNoListener > 200)
      {
        _requested = false;
      }
      usleep(1000 * 10);
      continue;
    }
    cntNoListener = 0;
    _requested = true;
    _success = false;
    ROS_INFO_STREAM("rc_visard_driver: Enabled rc-dynamics stream: " << _stream);

    // initialize data stream to this host

    rcd::DataReceiver::Ptr receiver;
    try
    {
      receiver = _rcdyn->createReceiverForStream(_stream);
    }
    catch (rcd::UnexpectedReceiveTimeout& e) {
      stringstream msg;
      msg << "Could not initialize rc-dynamics stream: " << _stream << ":" << endl << e.what();
      ROS_WARN_STREAM_THROTTLE(5, msg.str());
      continue;
    }
    catch (exception &e)
    {
      ROS_ERROR_STREAM(std::string("Could not initialize rc-dynamics stream: ")
                       + _stream + ": " + e.what());
      failed = true;
      break;
    }
    receiver->setTimeout(timeoutMillis);
    ROS_INFO_STREAM("rc_visard_driver: rc-dynamics stream ready: " << _stream);

    // main loop for listening, receiving and republishing data to ROS

    shared_ptr<::google::protobuf::Message> pbMsg;
    while (!_stop && rosPublisher.used())
    {

      // try receive msg; blocking call (timeout)
      try
      {
        pbMsg = receiver->receive(pbMsgType);
        _success = true;
      }
      catch (std::exception &e)
      {
        ROS_ERROR_STREAM("Caught exception during receiving "
                                 << _stream << ": " << e.what());
        failed = true;
        break; // stop receiving loop
      }

      // timeout happened; if its only small let's put a warning and increasing recv timeout
      if (!pbMsg)
      {
        ROS_ERROR_STREAM("Did not receive any "
                                 << _stream << " message within the last "
                                 << timeoutMillis << " ms. "
                                 << "Either rc_visard stopped streaming or is turned off, "
                                 << "or you seem to have serious network/connection problems!");
        failed = true;
        break; // stop receiving loop
      }

      ROS_DEBUG_STREAM_THROTTLE(1, "Received protobuf message: "
              << pbMsg->DebugString());

      // convert to ROS message and publish
      rosPublisher.publish(pbMsg);
    }

    ROS_INFO_STREAM("rc_visard_driver: Disabled rc-dynamics stream: " << _stream);
  }

  // return info about stream being stopped externally or failed internally
  return !failed;
}


bool PoseStream::startReceivingAndPublishingAsRos()
{
  unsigned int timeoutMillis = 500;

  ros::Publisher pub = _nh.advertise<geometry_msgs::PoseStamped>(_stream, 1000);
  tf::TransformBroadcaster tf_pub;

  unsigned int cntNoListener = 0;
  bool failed = false;

  while (!_stop && !failed)
  {
    bool someoneListens = _tfEnabled || pub.getNumSubscribers() > 0;

    // start streaming only if someone is listening

    if (!someoneListens)
    {
      // ros getNumSubscribers usually takes a while to register the subscribers
      if (++cntNoListener > 200)
      {
        _requested = false;
      }
      usleep(1000 * 10);
      continue;
    }
    cntNoListener = 0;
    _requested = true;
    _success = false;
    ROS_INFO_STREAM("rc_visard_driver: Enabled rc-dynamics stream: " << _stream);

    // initialize data stream to this host

    rcd::DataReceiver::Ptr receiver;
    try
    {
      receiver = _rcdyn->createReceiverForStream(_stream);
    }
    catch (rcd::UnexpectedReceiveTimeout& e) {
      stringstream msg;
      msg << "Could not initialize rc-dynamics stream: " << _stream << ":" << endl << e.what();
      ROS_WARN_STREAM_THROTTLE(5, msg.str());
      continue;
    }
    catch (exception &e)
    {
      ROS_ERROR_STREAM(std::string("Could not initialize rc-dynamics stream: ")
                       + _stream + ": " + e.what());
      failed = true;
      break;
    }
    receiver->setTimeout(timeoutMillis);
    ROS_INFO_STREAM("rc_visard_driver: rc-dynamics stream ready: " << _stream);

    // main loop for listening, receiving and republishing data to ROS

    shared_ptr <roboception::msgs::Frame> protoFrame;
    while (!_stop && someoneListens)
    {

      // try receive msg; blocking call (timeout)
      try
      {
        protoFrame = receiver->receive<roboception::msgs::Frame>();
        _success = true;
      }
      catch (std::exception &e)
      {
        ROS_ERROR_STREAM("Caught exception during receiving "
                                 << _stream << ": " << e.what());
        failed = true;
        break; // stop receiving loop
      }

      // timeout happened; if its only small let's put a warning and increasing recv timeout
      if (!protoFrame)
      {
        ROS_ERROR_STREAM("Did not receive any "
                                 << _stream << " message within the last "
                                 << timeoutMillis << " ms. "
                                 << "Either rc_visard stopped streaming or is turned off, "
                                 << "or you seem to have serious network/connection problems!");
        failed = true;
        break; // stop receiving loop
      }

      ROS_DEBUG_STREAM_THROTTLE(1, "Received protoFrame: "
              << protoFrame->DebugString());


      // overwrite frame name/ids
      protoFrame->set_parent(_tfPrefix+protoFrame->parent());
      protoFrame->set_name(_tfPrefix+protoFrame->name());

      auto rosPose = toRosPoseStamped(protoFrame);
      pub.publish(rosPose);

      // convert to tf and publish
      if (_tfEnabled)
      {
        tf::StampedTransform transform = toRosTfStampedTransform(protoFrame);
        tf_pub.sendTransform(transform);
      }

      // check if still someone is listening
      someoneListens = _tfEnabled || pub.getNumSubscribers() > 0;
    }

    ROS_INFO_STREAM("rc_visard_driver: Disabled rc-dynamics stream: " << _stream);
  }

  // return info about stream being stopped externally or failed internally
  return !failed;
}



}
