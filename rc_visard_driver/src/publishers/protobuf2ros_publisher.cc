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


#include "protobuf2ros_publisher.h"

#include "protobuf2ros_conversions.h"

using namespace std;

namespace rc
{

Protobuf2RosPublisher::Protobuf2RosPublisher(ros::NodeHandle &nh,
                                             const string &topic,
                                             const string &pbMsgType,
                                             const string &frame_id_prefix)
        : _tfPrefix(frame_id_prefix)
{
  if (pbMsgType == "Imu")
  {
    pub = nh.advertise<sensor_msgs::Imu>(topic, 1000);
  } else if (pbMsgType == "Frame")
  {
    pub = nh.advertise<geometry_msgs::PoseStamped>(topic, 1000);
  } else
  {
    stringstream msg;
    msg << "Protobuf message type '" << pbMsgType << "' not supported!";
    throw runtime_error(msg.str());
  }
}


bool Protobuf2RosPublisher::used()
{
  return pub.getNumSubscribers() > 0;
}


void Protobuf2RosPublisher::publish(shared_ptr<::google::protobuf::Message> pbMsg)
{
  string msg_name = pbMsg->GetDescriptor()->name();

  if (msg_name == "Imu")
  {
    // dynamically cast to Imu type, check validity, and transform to ros

    shared_ptr <roboception::msgs::Imu> protoImu =
            dynamic_pointer_cast<roboception::msgs::Imu>(pbMsg);
    if (!protoImu)
    {
      throw runtime_error("Given protobuf message was not of type Imu!");
    }

    auto rosImu = toRosImu(protoImu);
    rosImu->header.frame_id = _tfPrefix + rosImu->header.frame_id;
    pub.publish(rosImu);
  }
  else if (msg_name == "Frame")
  {
    // dynamically cast to Frame type, check validity, and transform to ros

    shared_ptr <roboception::msgs::Frame> protoFrame =
            dynamic_pointer_cast<roboception::msgs::Frame>(pbMsg);
    if (!protoFrame)
    {
      throw runtime_error("Given protobuf message was not of type Frame!");
    }

    auto rosPose = toRosPoseStamped(protoFrame);
    rosPose->header.frame_id = _tfPrefix + rosPose->header.frame_id;
    pub.publish(rosPose);
  }
  else
  {
    stringstream msg;
    msg << "Protobuf message type '" << msg_name << "' not supported!";
    throw runtime_error(msg.str());
  }
}

}