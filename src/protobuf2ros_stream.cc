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

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>

#include <rc_dynamics_api/unexpected_receive_timeout.h>


using namespace std;

namespace rc
{

namespace rcd = dynamics;

ros::Publisher Protobuf2RosStream::CreateRosPublisherForPbMsgType(
        const std::string &pbMsgType,
        ros::NodeHandle &nh,
        const std::string &topic)
{
  if (pbMsgType == "Imu")
  {
    return nh.advertise<sensor_msgs::Imu>(topic, 1000);
  }
  if (pbMsgType == "Frame")
  {
    return nh.advertise<geometry_msgs::PoseStamped>(topic, 1000);
  }

  stringstream msg;
  msg << "Protobuf message type '" << pbMsgType << "' not supported!";
  throw runtime_error(msg.str());
}

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
  // TODO: overwrite frame name/ids?
  return tf::StampedTransform(transform, t, frame->parent(), frame->name());
}


void Protobuf2RosStream::publishProtobufAsRos(
        std::shared_ptr<::google::protobuf::Message> pbMsg,
        ros::Publisher &pub)
{
  string msg_name = pbMsg->GetDescriptor()->name();

  if (msg_name == "Imu")
  {
    // dynamically cast to Imu type, check validity, and transform to ros

    shared_ptr<roboception::msgs::Imu> protoImu =
            dynamic_pointer_cast<roboception::msgs::Imu>(pbMsg);
    if (!protoImu) {
      throw runtime_error("Given protobuf message was not of type Imu!");
    }

    auto rosImu = toRosImu(protoImu);
    pub.publish(rosImu);
  }
  else if (msg_name == "Frame")
  {
    // dynamically cast to Frame type, check validity, and transform to ros

    shared_ptr<roboception::msgs::Frame> protoFrame =
            dynamic_pointer_cast<roboception::msgs::Frame>(pbMsg);
    if (!protoFrame) {
      throw runtime_error("Given protobuf message was not of type Frame!");
    }

    auto rosPose = toRosPoseStamped(protoFrame);
    pub.publish(rosPose);
  }
  else
  {
    stringstream msg;
    msg << "Protobuf message type '" << msg_name << "' not supported!";
    throw runtime_error(msg.str());
  }
}

bool Protobuf2RosStream::startReceivingAndPublishingAsRos()
{
  unsigned int timeoutMillis = 500;
  string pbMsgType = _rcdyn->getPbMsgTypeOfStream(_stream);

  ros::Publisher pub = CreateRosPublisherForPbMsgType(
          pbMsgType, _nh, _stream);

  unsigned int cntNoListener = 0;
  bool failed = false;

  while (!_stop && !failed)
  {
    bool someoneListens = pub.getNumSubscribers() > 0;

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

    shared_ptr<::google::protobuf::Message> pbMsg;
    while (!_stop && someoneListens)
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
      publishProtobufAsRos(pbMsg, pub);

      // check if still someone is listening
      someoneListens = pub.getNumSubscribers() > 0;
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

    tf::TransformBroadcaster tf_pub;
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
