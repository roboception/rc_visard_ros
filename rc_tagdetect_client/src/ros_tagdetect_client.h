/*
 * Copyright (c) 2019 Roboception GmbH
 *
 * Author: Monika Florek-Jasinska
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

#ifndef RC_TAGDETECTION_CLIENT_ROS_TAGDETECT_CLIENT_H
#define RC_TAGDETECTION_CLIENT_ROS_TAGDETECT_CLIENT_H

#include <chrono>
#include <thread>
#include <memory>
#include <atomic>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Trigger.h>
#include <rc_tagdetect_client/DetectTags.h>
#include <rc_tagdetect_client/DetectedTags.h>
#include <rc_tagdetect_client/StartContinuousDetection.h>
#include <rc_tagdetect_client/TagDetectConfig.h>
#include <rc_common_msgs/ReturnCode.h>

#include "communication_helper.h"
#include "visualization.h"
#include "exceptions.h"

namespace rc_tagdetect_client
{

class RosTagdetectClient
{
  public:
    RosTagdetectClient(const std::string &host, const ros::NodeHandle &nh,
                           const std::string &detection_type);

    ~RosTagdetectClient();

    typedef std::tuple<rc_tagdetect_client::DetectedTags, rc_common_msgs::ReturnCode> Result;

  private:

    bool detectService(rc_tagdetect_client::DetectTagsRequest &request,
                       rc_tagdetect_client::DetectTagsResponse &response);

    bool startContinousDetection(rc_tagdetect_client::StartContinuousDetectionRequest &request,
                                 rc_tagdetect_client::StartContinuousDetectionResponse &response);

    bool stopContinousDetection(std_srvs::TriggerRequest &request,
                                std_srvs::TriggerResponse &response);

    void startTagDetect();

    void stopTagDetect();

    void advertiseServicesAndTopics();

    Result detect(const std::vector<rc_tagdetect_client::Tag> &tags);

    /*
     * Reads tagdetect parameters from sensor and ros parameter (if a value for a parameter is defined in ROS parameter
     * server this value is used), and start dynamic reconfigure server.
     */
    void initConfiguration();

    void dynamicReconfigureCallback(rc_tagdetect_client::TagDetectConfig &config, uint32_t);

    ros::NodeHandle nh_;
    ros::ServiceServer srv_detect_;
    ros::ServiceServer srv_start_continuous_detection_;
    ros::ServiceServer srv_stop_continuous_detection_;

    ros::Publisher detections_pub;

    std::thread continuous_mode_thread_;
    std::atomic_bool stop_continuous_mode_thread_;

    std::unique_ptr<dynamic_reconfigure::Server<rc_tagdetect_client::TagDetectConfig>> server_;
    std::unique_ptr<rc_tagdetect_client::CommunicationHelper> rc_visard_communication_;
    std::unique_ptr<rc_tagdetect_client::Visualization> visualizer_;

    std::tuple<size_t, size_t, size_t> image_version_;
};

}

#endif //RC_TAGDETECTION_CLIENT_ROS_TAGDETECT_CLIENT_H
