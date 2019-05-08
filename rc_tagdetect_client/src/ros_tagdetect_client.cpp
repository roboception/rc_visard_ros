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

#include "ros_tagdetect_client.h"
#include "exceptions.h"

using std::string;

namespace rc_tagdetect_client
{

RosTagdetectClient::RosTagdetectClient(const std::string &host, const ros::NodeHandle &nh,
                                       const std::string &detection_type)
        : nh_(nh)

{
  if (detection_type == "rc_april_tag_detect" || detection_type == "rc_qr_code_detect")
  {
    rc_visard_communication_.reset(
            new CommunicationHelper(host, detection_type, 10000));
  }
  else
  {
    throw std::runtime_error("Acceptable detection types are \"rc_april_tag_detect\" and \"rc_qr_code_detect\"");
  }
  image_version_ = rc_visard_communication_->getImageVersion();
  initConfiguration();
  advertiseServicesAndTopics();
  startTagDetect();
}

RosTagdetectClient::~RosTagdetectClient()
{
  try
  {
    if (continuous_mode_thread_.joinable())
    {
      stop_continuous_mode_thread_ = true;
      continuous_mode_thread_.join();
    }

    stopTagDetect();
  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Exception during destruction of RosTagdetectClient: %s", ex.what());
  }
  catch (...)
  {
    ROS_FATAL("Exception during destruction of RosTagdetectClient");
  }
}


RosTagdetectClient::Result RosTagdetectClient::detect(
        const std::vector<rc_tagdetect_client::Tag> &tags)
{
  json request;
  auto &args = request["args"];
  auto &req_tags = args["tags"];
  req_tags = json::array();
  for (const auto &t : tags)
  {
    if (image_version_ <= std::make_tuple(1ul, 2ul, 1ul))
    {
      req_tags.push_back({{"id", t.id}});
      if (t.size > 0)
      {
        throw NotAvailableInThisVersionException("size");
      }
    }
    else
    {
      req_tags.push_back({{"id",   t.id},
                          {"size", t.size}});
    }
  }

  const auto j = rc_visard_communication_->servicePutRequest("detect", request);

  try
  {
    Result result;
    rc_common_msgs::ReturnCode &status = std::get<1>(result);
    status.message = j.at("return_code").at("message").get<std::string>();
    status.value = j.at("return_code").at("value").get<int>();

    rc_tagdetect_client::DetectedTags &result_tags = std::get<0>(result);
    result_tags.timestamp.sec = j.at("timestamp").at("sec").get<int>();
    result_tags.timestamp.nsec = j.at("timestamp").at("nsec").get<int>();

    for (const auto &t : j.at("tags"))
    {
      result_tags.tags.emplace_back();
      auto &result_tag = result_tags.tags.back();

      result_tag.instance_id = t.at("instance_id").get<std::string>();

      result_tag.tag.id = t.at("id").get<std::string>();
      result_tag.tag.size = t.at("size").get<double>();

      result_tag.header.frame_id = t.at("pose_frame").get<std::string>();
      result_tag.pose.header.frame_id = result_tag.header.frame_id;

      const auto &pose_ts = t.at("timestamp");
      result_tag.header.stamp.sec = pose_ts.at("sec").get<int>();
      result_tag.header.stamp.nsec = pose_ts.at("nsec").get<int>();
      result_tag.pose.header.stamp.sec = result_tag.header.stamp.sec;
      result_tag.pose.header.stamp.nsec = result_tag.header.stamp.nsec;

      const auto &pose = t.at("pose");
      const auto &position = pose.at("position");
      result_tag.pose.pose.position.x = position.at("x").get<double>();
      result_tag.pose.pose.position.y = position.at("y").get<double>();
      result_tag.pose.pose.position.z = position.at("z").get<double>();

      const auto &orientation = pose.at("orientation");
      result_tag.pose.pose.orientation.x = orientation.at("x").get<double>();
      result_tag.pose.pose.orientation.y = orientation.at("y").get<double>();
      result_tag.pose.pose.orientation.z = orientation.at("z").get<double>();
      result_tag.pose.pose.orientation.w = orientation.at("w").get<double>();
    }

    return result;
  }
  catch (const std::exception &ex)
  {
    throw MiscException(std::string("Could not parse response: ") + ex.what());
  }
}

void RosTagdetectClient::startTagDetect()
{
  rc_visard_communication_->servicePutRequest("start");
}

void RosTagdetectClient::stopTagDetect()
{
  rc_visard_communication_->servicePutRequest("stop");
}


bool RosTagdetectClient::detectService(DetectTagsRequest &req, DetectTagsResponse &res)
{
  if (continuous_mode_thread_.joinable())
  {
    ROS_ERROR("Cannot execute detect when continuous mode is enabled.");
    return false;
  }

  try
  {
    auto result = detect(req.tags);
    res.tags = std::move(std::get<0>(result).tags);
    res.timestamp = std::move(std::get<0>(result).timestamp);
    res.return_code = std::move(std::get<1>(result));

    if (visualizer_) visualizer_->publishTags(res.tags);

    return true;
  }
  catch (const NotAvailableInThisVersionException &ex)
  {
    ROS_ERROR("This rc_visard firmware does not support \"%s\"", ex.what());
    return false;
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool RosTagdetectClient::stopContinousDetection(std_srvs::TriggerRequest &request,
                                                   std_srvs::TriggerResponse &response)
{
  if (continuous_mode_thread_.joinable())
  {
    stop_continuous_mode_thread_ = true;
    continuous_mode_thread_.join();
  }
  response.success = true;
  return true;
}

bool RosTagdetectClient::startContinousDetection(StartContinuousDetectionRequest &request,
                                                 StartContinuousDetectionResponse &response)
{
  {
    std_srvs::TriggerRequest request;
    std_srvs::TriggerResponse response;
    stopContinousDetection(request, response);
  }

  if (!continuous_mode_thread_.joinable())
  {
    stop_continuous_mode_thread_ = false;
    continuous_mode_thread_ = std::thread(
            [request, this]()
            {
              ROS_INFO("Entering continuous mode...");

              const double rate = 1.0;

              while (!stop_continuous_mode_thread_)
              {
                const auto start_time = std::chrono::steady_clock::now();

                rc_tagdetect_client::DetectedTags res;

                try
                {
                  auto result = detect(request.tags);

                  res.tags = std::move(std::get<0>(result).tags);
                  res.timestamp = std::move(std::get<0>(result).timestamp);
                  res.return_code = std::move(std::get<1>(result));

                  if (res.return_code.value < 0)
                  {
                    ROS_ERROR("TagDetect responded with error [%d]: %s",
                              res.return_code.value,
                              res.return_code.message.c_str());
                  }
                  else if (res.return_code.value > 0)
                  {
                    ROS_WARN("TagDetect responded with message [%d]: %s",
                             res.return_code.value,
                             res.return_code.message.c_str());
                  }
                }
                catch (const NotAvailableInThisVersionException &ex)
                {
                  ROS_ERROR("This rc_visard firmware does not support \"%s\"", ex.what());
                  break;
                }
                catch (const std::exception &ex)
                {
                  ROS_ERROR("%s", ex.what());
                  break;
                }

                detections_pub.publish(res);

                if (visualizer_) visualizer_->publishTags(res.tags);

                while (!stop_continuous_mode_thread_ &&
                       std::chrono::duration_cast<std::chrono::duration<double>>(
                               std::chrono::steady_clock::now() -
                               start_time).count() < rate)
                {
                  std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
              }

              ROS_INFO("Stopping continuous mode...");
            });
  }

  return true;
}

void RosTagdetectClient::advertiseServicesAndTopics()
{
  detections_pub =
          nh_.advertise<rc_tagdetect_client::DetectedTags>("detected_tags", 100);

  srv_detect_ = nh_.advertiseService("detect", &RosTagdetectClient::detectService, this);
  srv_start_continuous_detection_ = nh_.advertiseService("start_continuous_detection",
                                                         &RosTagdetectClient::startContinousDetection, this);
  srv_stop_continuous_detection_ = nh_.advertiseService("stop_continuous_detection",
                                                        &RosTagdetectClient::stopContinousDetection, this);

}

void RosTagdetectClient::initConfiguration()
{
  rc_tagdetect_client::TagDetectConfig cfg;

  // first get the current values from sensor
  auto json_resp = rc_visard_communication_->getParameters();

  for (auto &param : json_resp)
  {
    string name = param["name"];
    if (param["name"] == "quality")
    {
      cfg.quality = param["value"];
    }
    else if (param["name"] == "detect_inverted_tags")
    {
      //TODO is it working with all versions?
      cfg.detect_inverted_tags = param["value"];
    }
    else if (param["name"] == "forget_after_n_detections")
    {
      cfg.forget_after_n_detections = param["value"];
    }
    else if (param["name"] == "max_corner_distance")
    {
      cfg.max_corner_distance = param["value"];
    }
    else if (param["name"] == "use_cached_images")
    {
      cfg.use_cached_images = param["value"];
    }

  }

  // second, try to get ROS parameters:
  // if parameter is not set in parameter server, we default to current sensor configuration
  nh_.param("quality", cfg.quality, cfg.quality);
  nh_.param("detect_inverted_tags", cfg.detect_inverted_tags, cfg.detect_inverted_tags);
  nh_.param("forget_after_n_detections", cfg.forget_after_n_detections, cfg.forget_after_n_detections);
  nh_.param("max_corner_distance", cfg.max_corner_distance, cfg.max_corner_distance);
  nh_.param("use_cached_images", cfg.use_cached_images, cfg.use_cached_images);

  // instantiate dynamic reconfigure server that will initially read those values
  using RCFSRV = dynamic_reconfigure::Server<rc_tagdetect_client::TagDetectConfig>;
  server_ = std::unique_ptr<RCFSRV>(new dynamic_reconfigure::Server<rc_tagdetect_client::TagDetectConfig>(nh_));
  server_->setCallback(boost::bind(&RosTagdetectClient::dynamicReconfigureCallback, this, _1, _2));
}

void RosTagdetectClient::dynamicReconfigureCallback(rc_tagdetect_client::TagDetectConfig
                                                       &config,
                                                       uint32_t)
{
  // fill json request from dynamic reconfigure request
  json js_params, js_param;
  js_param["name"] = "quality";
  js_param["value"] = config.quality;
  js_params.push_back(js_param);
  if (image_version_ >= std::make_tuple(1ul, 2ul, 1ul))
  {
    js_param["name"] = "detect_inverted_tags";
    js_param["value"] = config.detect_inverted_tags;
    js_params.push_back(js_param);
  }
  else
  {
    ROS_WARN("\"detect_inverted_tags\" only available in newer software versions.");
    config.detect_inverted_tags = false;
  }
  js_param["name"] = "forget_after_n_detections";
  js_param["value"] = config.forget_after_n_detections;
  js_params.push_back(js_param);
  js_param["name"] = "max_corner_distance";
  js_param["value"] = config.max_corner_distance;
  js_params.push_back(js_param);
  js_param["name"] = "use_cached_images";
  js_param["value"] = config.use_cached_images;
  js_params.push_back(js_param);

  if (config.publish_visualization)
  {
    if (!visualizer_)
    {
      visualizer_.reset(new Visualization(nh_));
    }
  }
  else
  {
    visualizer_.reset();
  }
  rc_visard_communication_->setParameters(js_params);
}

}
