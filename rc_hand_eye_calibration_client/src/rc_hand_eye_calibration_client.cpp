/*
 * Copyright (c) 2018-2020 Roboception GmbH
 *
 * Author: Felix Endres
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

#include "rc_hand_eye_calibration_client.h"

#include "json_conversions.h"

#include <ifaddrs.h>
#include <arpa/inet.h>
#include <unistd.h>

using json = nlohmann::json;

namespace rc_hand_eye_calibration_client
{
HandEyeCalibClient::HandEyeCalibClient(std::string host, ros::NodeHandle nh)
  : nh_(nh), rest_helper_(host, "rc_hand_eye_calibration", 2000)
{
  initConfiguration();
  advertiseServices();
  initTimers();
  image_version_ = rest_helper_.getImageVersion();
}

template <typename Request, typename Response>
bool HandEyeCalibClient::callService(const std::string& name, const Request& req, Response& res)
{
  try
  {
    json j_req = req;
    const auto j_res = rest_helper_.servicePutRequest(name, j_req);
    res = j_res;
    return true;
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

void HandEyeCalibClient::initTimers()
{
  if (calib_publish_period_ > 0.0 &&  // only need the timer at all if there's a valid intervall
      // every calibration request broadcasts: no need for extra publishing if request is done more often
      (calib_request_period_ <= 0.0 || calib_publish_period_ < calib_request_period_))
  {
    ROS_INFO("Broadcasting calibration every %.3f seconds on /tf", calib_publish_period_);
    // the two booleans at the end: *one-shot* is kept default (false), *autostart* is set to false,
    // because we start publishing only when the first calibration result was received
    calib_publish_timer_ = nh_.createSteadyTimer(ros::WallDuration(calib_publish_period_),
                                                 &HandEyeCalibClient::sendCachedCalibration, this, false, false);
  }

  if (calib_request_period_ >= 0.0)  // negative is documented to turn all auto-requesting off
  {
    if (calib_request_period_ == 0.0)  // special meaning as documented in README.md
    {
      ROS_INFO("Requesting (and broadcasting) calibration from rc_visard once");
    }
    else
    {
      ROS_INFO("Requesting (and broadcasting) calibration every %.3f seconds from rc_visard", calib_request_period_);
      calib_request_timer_ = nh_.createSteadyTimer(ros::WallDuration(calib_request_period_),
                                                   &HandEyeCalibClient::requestCalibration, this);
    }
    // request once immediately at startup
    requestCalibration({});
  }
}

bool HandEyeCalibClient::saveSrv(TriggerRequest& req, TriggerResponse& res)
{
  callService("save_calibration", req, res);
  return true;
}

bool HandEyeCalibClient::resetSrv(TriggerRequest& req, TriggerResponse& res)
{
  callService("reset_calibration", req, res);
  return true;
}

bool HandEyeCalibClient::removeSrv(TriggerRequest& req, TriggerResponse& res)
{
  callService("remove_calibration", req, res);

  if (res.success)
  {
    calib_publish_timer_.stop();  // does nothing if already stopped
    ROS_INFO("Calibration has been removed, stopped /tf broadcasting.");
  }
  else
  {
    ROS_WARN("Failed to remove calibration: %s", res.message.c_str());
  }

  return true;
}

bool HandEyeCalibClient::setSlotSrv(SetCalibrationPoseRequest& req, SetCalibrationPoseResponse& res)
{
  callService("set_pose", req, res);
  return true;
}

void HandEyeCalibClient::calibResponseHandler(CalibrationResponse& response)
{
  if (response.success)
  {
    ROS_INFO("Calibration request successful. Broadcasting new calibration.");
    updateCalibrationCache(response);
    sendCachedCalibration();
    if (calib_publish_timer_.isValid())  // don't start it if it is invalid
    {
      calib_publish_timer_.start();  // does nothing if already started.
    }
  }
  else
  {
    ROS_WARN_STREAM("Could not get calibration: " << response.message);
  }
}

bool HandEyeCalibClient::calibSrv(CalibrationRequest& req, CalibrationResponse& res)
{
  callService("calibrate", req, res);
  calibResponseHandler(res);
  return true;
}

bool HandEyeCalibClient::getCalibResultSrv(CalibrationRequest& req, CalibrationResponse& res)
{
  callService("get_calibration", req, res);
  calibResponseHandler(res);
  return true;
}

bool  HandEyeCalibClient::setCalibSrv(SetCalibrationRequest& req, SetCalibrationResponse& res)
{
  if (image_version_ >= std::make_tuple(20ul, 4ul, 0ul))
  {
    callService("set_calibration", req, res);
  }
  else
  {
    res.message = "set_calibration is not supported in this firmware version";
    res.success = false;
    ROS_WARN_STREAM(res.message);
  }
  return true;
}

void HandEyeCalibClient::requestCalibration(const ros::SteadyTimerEvent&)
{
  CalibrationRequest req;
  CalibrationResponse res;
  getCalibResultSrv(req, res);
}

void HandEyeCalibClient::updateCalibrationCache(const CalibrationResponse& response)
{
  // Select frame_id based on the type of calibration hand-eye (on-robot-cam) or base-eye (external cam)
  current_calibration_.header.frame_id = response.robot_mounted ? endeff_frame_id_ : base_frame_id_;
  current_calibration_.child_frame_id = camera_frame_id_;
  current_calibration_.transform.translation.x = response.pose.position.x;
  current_calibration_.transform.translation.y = response.pose.position.y;
  current_calibration_.transform.translation.z = response.pose.position.z;
  current_calibration_.transform.rotation = response.pose.orientation;
}

void HandEyeCalibClient::sendCachedCalibration(const ros::SteadyTimerEvent&)
{
  if (calib_publish_period_ <= 0.0)  // if there's no period use static tf
  {
    // Timestamp doesn't (or at least shouldn't) matter for static transforms
    // Time::now makes it easy to see when it was updated though
    current_calibration_.header.stamp = ros::Time::now();
    static_tf2_broadcaster_.sendTransform(current_calibration_);
  }
  else  // periodic sending
  {
    // Pre-date, so the transformation can be directly used by clients until the next one is sent.
    // I.e., don't cause lag when looking up transformations because one has to wait for
    // the current calibration.
    current_calibration_.header.stamp = ros::Time::now() + ros::Duration(calib_publish_period_);
    dynamic_tf2_broadcaster_.sendTransform(current_calibration_);
  }
}

void HandEyeCalibClient::advertiseServices()
{
  using CW = HandEyeCalibClient;
  // Save pose and image/grid pair for later calibration
  srv_set_slot_ = nh_.advertiseService("set_pose", &CW::setSlotSrv, this);
  // Save calibration to disk
  srv_save_ = nh_.advertiseService("save_calibration", &CW::saveSrv, this);
  // Compute and return calibration
  srv_calibrate_ = nh_.advertiseService("calibrate", &CW::calibSrv, this);
  // Get result (but don't compute)
  srv_get_result_ = nh_.advertiseService("get_calibration", &CW::getCalibResultSrv, this);
  // Delete all slots
  srv_reset_ = nh_.advertiseService("reset_calibration", &CW::resetSrv, this);
  // remove calibration
  srv_remove_ = nh_.advertiseService("remove_calibration", &CW::removeSrv, this);
  srv_set_calibration_ = nh_.advertiseService("set_calibration", &CW::setCalibSrv, this);
}

void paramsToCfg(const json& params, hand_eye_calibrationConfig& cfg)
{
  for (const auto& param : params)
  {
    const auto& name = param["name"];
    if (name == "grid_width")
      cfg.grid_width = param["value"];
    else if (name == "grid_height")
      cfg.grid_height = param["value"];
    else if (name == "robot_mounted")
      cfg.robot_mounted = (bool)param["value"];
    else if (name == "tcp_rotation_axis")
      cfg.tcp_rotation_axis = (int)param["value"];
    else if (name == "tcp_offset")
      cfg.tcp_offset = param["value"];
  }
}

void HandEyeCalibClient::initConfiguration()
{
  hand_eye_calibrationConfig cfg;

  // first get the current values from sensor
  const auto j_params = rest_helper_.getParameters();

  paramsToCfg(j_params, cfg);

  // second, try to get ROS parameters:
  // if parameter is not set in parameter server, we default to current sensor configuration
  nh_.param("grid_width", cfg.grid_width, cfg.grid_width);
  nh_.param("grid_height", cfg.grid_height, cfg.grid_height);
  nh_.param("robot_mounted", cfg.robot_mounted, cfg.robot_mounted);
  nh_.param("tcp_rotation_axis", cfg.tcp_rotation_axis, cfg.tcp_rotation_axis);
  nh_.param("tcp_offset", cfg.tcp_offset, cfg.tcp_offset);

  // see if those parameters are available otherwise use default (set in class header)
  nh_.param("rc_visard_frame_id", camera_frame_id_, camera_frame_id_);
  nh_.param("end_effector_frame_id", endeff_frame_id_, endeff_frame_id_);
  nh_.param("base_frame_id", base_frame_id_, base_frame_id_);
  nh_.param("calibration_publication_period", calib_publish_period_, calib_publish_period_);
  nh_.param("calibration_request_period", calib_request_period_, calib_request_period_);

  // set parameters on parameter server so that dynamic reconfigure picks them up
  nh_.setParam("grid_width", cfg.grid_width);
  nh_.setParam("grid_height", cfg.grid_height);
  nh_.setParam("robot_mounted", cfg.robot_mounted);
  nh_.setParam("tcp_rotation_axis", cfg.tcp_rotation_axis);
  nh_.setParam("tcp_offset", cfg.tcp_offset);

  // instantiate dynamic reconfigure server that will initially read those values
  using RCFSRV = dynamic_reconfigure::Server<hand_eye_calibrationConfig>;
  server_ = std::unique_ptr<RCFSRV>(new RCFSRV(nh_));
  server_->setCallback(boost::bind(&HandEyeCalibClient::dynamicReconfigureCb, this, _1, _2));
}

void HandEyeCalibClient::dynamicReconfigureCb(hand_eye_calibrationConfig& config, uint32_t)
{
  ROS_DEBUG("Reconfigure Request: (%f x %f) %s, 4DOF: %d, offset: %f", config.grid_width, config.grid_height,
            config.robot_mounted ? "True" : "False", config.tcp_rotation_axis, config.tcp_offset);

  // fill json request from dynamic reconfigure request
  json j_params, j_param;
  j_param["name"] = "grid_width";
  j_param["value"] = config.grid_width;
  j_params.push_back(j_param);
  j_param["name"] = "grid_height";
  j_param["value"] = config.grid_height;
  j_params.push_back(j_param);
  j_param["name"] = "robot_mounted";
  j_param["value"] = config.robot_mounted;
  j_params.push_back(j_param);
  j_param["name"] = "tcp_rotation_axis";
  j_param["value"] = config.tcp_rotation_axis;
  j_params.push_back(j_param);
  j_param["name"] = "tcp_offset";
  j_param["value"] = config.tcp_offset;
  j_params.push_back(j_param);

  json j_params_new = rest_helper_.setParameters(j_params);
  // set config with new params so they are updated if needed
  paramsToCfg(j_params_new, config);
}

}  // namespace rc_hand_eye_calibration_client
