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
#include "json_conversions.h"

using std::string;

namespace rc_tagdetect_client
{
RosTagdetectClient::RosTagdetectClient(const std::string& host, const ros::NodeHandle& nh,
                                       const std::string& detection_type)
  : nh_(nh)

{
  if (detection_type == "rc_april_tag_detect" || detection_type == "rc_qr_code_detect")
  {
    rest_helper_.reset(new rc_rest_api::RestHelper(host, detection_type, 10000));
  }
  else
  {
    throw std::runtime_error("Acceptable detection types are \"rc_april_tag_detect\" and \"rc_qr_code_detect\"");
  }
  image_version_ = rest_helper_->getImageVersion();
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
  catch (const std::exception& ex)
  {
    ROS_FATAL("Exception during destruction of RosTagdetectClient: %s", ex.what());
  }
  catch (...)
  {
    ROS_FATAL("Exception during destruction of RosTagdetectClient");
  }
}

void RosTagdetectClient::startTagDetect()
{
  rest_helper_->servicePutRequest("start");
}

void RosTagdetectClient::stopTagDetect()
{
  rest_helper_->servicePutRequest("stop");
}

template <typename Request, typename Response>
bool RosTagdetectClient::callService(const std::string& name, const Request& req, Response& res)
{
  try
  {
    json j_req = req;
    const auto j_res = rest_helper_->servicePutRequest(name, j_req);
    res = j_res;
    return true;
  }
  catch (const rc_rest_api::NotAvailableInThisVersionException& ex)
  {
    ROS_ERROR("This rc_visard firmware does not support \"%s\"", ex.what());
    res.return_code.value = -8;  // NOT_APPLICABLE
    res.return_code.message = "Not available in this firmware version";
    return false;
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    res.return_code.value = -2;  // INTERNAL_ERROR
    res.return_code.message = ex.what();
    return false;
  }
}

bool RosTagdetectClient::detect(DetectTagsRequest& req, DetectTagsResponse& res)
{
  bool success = callService("detect", req, res);
  if (success && res.return_code.value >= 0 && visualizer_)
  {
    visualizer_->publishTags(res.tags);
  }
  detections_pub_.publish(res);
  return success;
}

bool RosTagdetectClient::detectService(DetectTagsRequest& req, DetectTagsResponse& res)
{
  if (continuous_mode_thread_.joinable())
  {
    ROS_ERROR("Cannot execute detect when continuous mode is enabled.");
    res.return_code.value = -8;  // NOT_APPLICABLE
    res.return_code.message = "Cannot execute detect when continuous mode is enabled.";
    return true;
  }

  detect(req, res);
  return true;
}

bool RosTagdetectClient::stopContinousDetection(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
  if (continuous_mode_thread_.joinable())
  {
    stop_continuous_mode_thread_ = true;
    continuous_mode_thread_.join();
  }
  response.success = true;
  return true;
}

bool RosTagdetectClient::startContinousDetection(StartContinuousDetectionRequest& request,
                                                 StartContinuousDetectionResponse& response)
{
  {
    std_srvs::TriggerRequest request;
    std_srvs::TriggerResponse response;
    stopContinousDetection(request, response);
  }

  if (!continuous_mode_thread_.joinable())
  {
    stop_continuous_mode_thread_ = false;
    continuous_mode_thread_ = std::thread([request, this]() {
      ROS_INFO("Entering continuous mode...");

      const double rate = 1.0;

      while (!stop_continuous_mode_thread_)
      {
        const auto start_time = std::chrono::steady_clock::now();

        DetectTagsRequest req;
        req.tags = request.tags;
        req.pose_frame = request.pose_frame;
        req.robot_pose = request.robot_pose;
        DetectTagsResponse res;
        if (!detect(req, res))
        {
          break;
        }

        while (!stop_continuous_mode_thread_ &&
               std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time)
                       .count() < rate)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

      ROS_INFO("Stopping continuous mode...");
    });
  }

  response.success = true;
  return true;
}

void RosTagdetectClient::advertiseServicesAndTopics()
{
  detections_pub_ = nh_.advertise<rc_tagdetect_client::DetectedTags>("detected_tags", 100);

  srv_detect_ = nh_.advertiseService("detect", &RosTagdetectClient::detectService, this);
  srv_start_continuous_detection_ =
      nh_.advertiseService("start_continuous_detection", &RosTagdetectClient::startContinousDetection, this);
  srv_stop_continuous_detection_ =
      nh_.advertiseService("stop_continuous_detection", &RosTagdetectClient::stopContinousDetection, this);
}

void paramsToCfg(const json& params, TagDetectConfig& cfg)
{
  for (const auto& param : params)
  {
    const auto& name = param["name"];
    if (name == "quality")
    {
      cfg.quality = param["value"];
    }
    else if (name == "detect_inverted_tags")
    {
      // TODO is it working with all versions?
      cfg.detect_inverted_tags = param["value"];
    }
    else if (name == "forget_after_n_detections")
    {
      cfg.forget_after_n_detections = param["value"];
    }
    else if (name == "max_corner_distance")
    {
      cfg.max_corner_distance = param["value"];
    }
    else if (name == "use_cached_images")
    {
      cfg.use_cached_images = param["value"];
    }
  }
}

void RosTagdetectClient::initConfiguration()
{
  rc_tagdetect_client::TagDetectConfig cfg;

  // first get the current values from sensor
  const auto j_params = rest_helper_->getParameters();

  paramsToCfg(j_params, cfg);

  // second, try to get ROS parameters:
  // if parameter is not set in parameter server, we default to current sensor configuration
  nh_.param("quality", cfg.quality, cfg.quality);
  nh_.param("detect_inverted_tags", cfg.detect_inverted_tags, cfg.detect_inverted_tags);
  nh_.param("forget_after_n_detections", cfg.forget_after_n_detections, cfg.forget_after_n_detections);
  nh_.param("max_corner_distance", cfg.max_corner_distance, cfg.max_corner_distance);
  nh_.param("use_cached_images", cfg.use_cached_images, cfg.use_cached_images);

  // set parameters on parameter server so that dynamic reconfigure picks them up
  nh_.setParam("quality", cfg.quality);
  nh_.setParam("detect_inverted_tags", cfg.detect_inverted_tags);
  nh_.setParam("forget_after_n_detections", cfg.forget_after_n_detections);
  nh_.setParam("max_corner_distance", cfg.max_corner_distance);
  nh_.setParam("use_cached_images", cfg.use_cached_images);

  // instantiate dynamic reconfigure server that will initially read those values
  using ReconfServer = dynamic_reconfigure::Server<rc_tagdetect_client::TagDetectConfig>;
  server_ = std::unique_ptr<ReconfServer>(new ReconfServer(nh_));
  server_->setCallback(boost::bind(&RosTagdetectClient::dynamicReconfigureCallback, this, _1, _2));
}

void RosTagdetectClient::dynamicReconfigureCallback(rc_tagdetect_client::TagDetectConfig& config, uint32_t)
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

  json j_params_new = rest_helper_->setParameters(js_params);
  // set config with new params so they are updated if needed
  paramsToCfg(j_params_new, config);
}

}  // namespace rc_tagdetect_client
