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


#ifndef RC_VISARD_ROS_PROTOBUF2ROS_PUBLISHER_H
#define RC_VISARD_ROS_PROTOBUF2ROS_PUBLISHER_H


#include <ros/ros.h>
#include <google/protobuf/message.h>


namespace rc
{


/**
 * Generic implementation for publishing protobuf messages to ros
 */
class Protobuf2RosPublisher
{
  public:

    /**
     * Internally creates a corresponding ros publisher
     */
    Protobuf2RosPublisher(ros::NodeHandle &nh, const std::string &topic,
                          const std::string &pbMsgType);

    /**
      Returns true if there are subscribers to the topic.

      @return True if there are subscribers.
    */
    virtual bool used();

    /**
     * Publish the generic protobuf message as a corresponding Ros Message
     */
    virtual void publish(std::shared_ptr<::google::protobuf::Message> pbMsg);

  protected:
    ros::Publisher pub; // needs to be constructed properly by child classes

  private:
    Protobuf2RosPublisher &
    operator=(const Protobuf2RosPublisher &); // forbidden
};

}

#endif //RC_VISARD_ROS_PROTOBUF2ROS_PUBLISHER_H
