/*
 * Copyright (c) 2020 Roboception GmbH
 *
 * Author: Elena Gambaro
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

#include "rc_silhouettematch_client.h"
#include "json_conversions.h"
#include "communication_helper.h"
#include "visualizer.h"

namespace rc_silhouettematch_client
{
SilhouetteMatchClient::SilhouetteMatchClient(const std::string& host,
                                             ros::NodeHandle& nh)
  : nh_(nh)
  , rc_visard_comm_if_(
        new CommunicationHelper(host, "rc_silhouettematch", 10000))
  , dyn_reconf_(new dynamic_reconfigure::Server<SilhouetteMatchConfig>(nh))
{
  srvs_.emplace_back(nh.advertiseService(
      "detect_object", &SilhouetteMatchClient::detectObject, this));
  srvs_.emplace_back(
      nh.advertiseService("calibrate_base_plane",
                          &SilhouetteMatchClient::calibrateBasePlane, this));
  srvs_.emplace_back(
      nh.advertiseService("get_base_plane_calibration",
                          &SilhouetteMatchClient::getBasePlaneCalib, this));
  srvs_.emplace_back(
      nh.advertiseService("delete_base_plane_calibration",
                          &SilhouetteMatchClient::deleteBasePlaneCalib, this));
  srvs_.emplace_back(nh.advertiseService("set_region_of_interest_2d",
                                         &SilhouetteMatchClient::setROI, this));
  srvs_.emplace_back(nh.advertiseService(
      "get_regions_of_interest_2d", &SilhouetteMatchClient::getROIs, this));
  srvs_.emplace_back(nh.advertiseService("delete_regions_of_interest_2d",
                                         &SilhouetteMatchClient::deleteROIs,
                                         this));

  initParameters();
}

SilhouetteMatchClient::~SilhouetteMatchClient() = default;

nlohmann::json requestToJson(const DetectObject::Request& req)
{
  nlohmann::json json_req;
  const auto& obj = req.object_to_detect;
  auto& args = json_req["args"];
  args["object_to_detect"] = { { "object_id", obj.object_id },
                               { "region_of_interest_2d_id",
                                 obj.region_of_interest_2d_id } };
  args["offset"] = req.offset;
  args["pose_frame"] = req.pose_frame;
  args["robot_pose"] = req.robot_pose;
  return json_req;
}

void responseFromJson(const nlohmann::json& json_res,
                      DetectObject::Response& res)
{
  res.timestamp = json_res["timestamp"];
  res.return_code = json_res["return_code"];
  res.object_id = json_res["object_id"];
  for (const auto& instance : json_res["instances"])
  {
    res.instances.emplace_back(instance);
  }
}

nlohmann::json requestToJson(const CalibrateBasePlane::Request& req)
{
  nlohmann::json json_req;
  auto& args = json_req["args"];
  args["pose_frame"] = req.pose_frame;
  args["region_of_interest_2d_id"] = req.region_of_interest_2d_id;
  args["plane_estimation_method"] = req.plane_estimation_method;
  if (req.plane_estimation_method == "STEREO")
  {
    args["stereo"]= { {"plane_preference", req.stereo.plane_preference} };
  }
  args["offset"] = req.offset;
  args["plane"] = req.plane;
  args["robot_pose"] = req.robot_pose;
  return json_req;
}

void responseFromJson(const nlohmann::json& json_res,
                      CalibrateBasePlane::Response& res)
{
  res.timestamp = json_res["timestamp"];
  res.return_code = json_res["return_code"];
  res.plane = json_res["plane"];
}

nlohmann::json requestToJson(const GetBasePlaneCalibration::Request& req)
{
  nlohmann::json json_req;
  auto& args = json_req["args"];
  args["pose_frame"] = req.pose_frame;
  args["robot_pose"] = req.robot_pose;
  return json_req;
}

void responseFromJson(const nlohmann::json& json_res,
                      GetBasePlaneCalibration::Response& res)
{
  res.return_code = json_res["return_code"];
  res.plane = json_res["plane"];
}

nlohmann::json requestToJson(const DeleteBasePlaneCalibration::Request& req)
{
  return {};
}

void responseFromJson(const nlohmann::json& json_res,
                      DeleteBasePlaneCalibration::Response& res)
{
  res.return_code = json_res["return_code"];
}

nlohmann::json requestToJson(const SetRegionOfInterest::Request& req)
{
  nlohmann::json json_req;
  auto& args = json_req["args"];
  args["region_of_interest_2d"] = req.region_of_interest_2d;
  return json_req;
}

void responseFromJson(const nlohmann::json& json_res,
                      SetRegionOfInterest::Response& res)
{
  res.return_code = json_res["return_code"];
}

nlohmann::json requestToJson(const GetRegionsOfInterest::Request& req)
{
  nlohmann::json json_req;
  auto& args = json_req["args"];
  args["region_of_interest_2d_ids"] = req.region_of_interest_2d_ids;
  return json_req;
}

void responseFromJson(const nlohmann::json& json_res,
                      GetRegionsOfInterest::Response& res)
{
  res.return_code = json_res["return_code"];
  for (const auto& roi : json_res["regions_of_interest"])
  {
    res.regions_of_interest.emplace_back(roi);
  }
}

nlohmann::json requestToJson(const DeleteRegionsOfInterest::Request& req)
{
  nlohmann::json json_req;
  auto& args = json_req["args"];
  args["region_of_interest_2d_ids"] = req.region_of_interest_2d_ids;
  return json_req;
}

void responseFromJson(const nlohmann::json& json_res,
                      DeleteRegionsOfInterest::Response& res)
{
  res.return_code = json_res["return_code"];
}

template <typename Request, typename Response>
bool SilhouetteMatchClient::callService(const std::string& name,
                                        const Request& req, Response& res)
{
  try
  {
    auto j_req = requestToJson(req);
    const auto j_res = rc_visard_comm_if_->servicePutRequest(name, j_req);
    responseFromJson(j_res, res);
    return true;
  }
  catch (const NotAvailableInThisVersionException& ex)
  {
    // XXX TODO: this should return true and set return_code to error
    ROS_ERROR("This rc_visard firmware does not support \"%s\"", ex.what());
    return false;
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SilhouetteMatchClient::detectObject(DetectObject::Request& req,
                                         DetectObject::Response& res)
{
  bool success = callService("detect_object", req, res);
  if (success && res.return_code.value >= 0 && visualizer_)
  {
    visualizer_->visInstances(res.instances);
  }
  return success;
}

bool SilhouetteMatchClient::calibrateBasePlane(
    CalibrateBasePlane::Request& req, CalibrateBasePlane::Response& res)
{
  bool success = callService("calibrate_base_plane", req, res);
  if (success && res.return_code.value >= 0 && visualizer_)
  {
    visualizer_->visBasePlane(res.plane);
  }
  return success;
}

bool SilhouetteMatchClient::getBasePlaneCalib(
    GetBasePlaneCalibration::Request& req,
    GetBasePlaneCalibration::Response& res)
{
  return callService("get_base_plane_calibration", req, res);
}

bool SilhouetteMatchClient::deleteBasePlaneCalib(
    DeleteBasePlaneCalibration::Request& req,
    DeleteBasePlaneCalibration::Response& res)
{
  return callService("delete_base_plane_calibration", req, res);
}

bool SilhouetteMatchClient::setROI(SetRegionOfInterest::Request& req,
                                   SetRegionOfInterest::Response& res)
{
  return callService("set_region_of_interest_2d", req, res);
}

bool SilhouetteMatchClient::getROIs(GetRegionsOfInterest::Request& req,
                                    GetRegionsOfInterest::Response& res)
{
  return callService("get_regions_of_interest_2d", req, res);
}

bool SilhouetteMatchClient::deleteROIs(DeleteRegionsOfInterest::Request& req,
                                       DeleteRegionsOfInterest::Response& res)
{
  return callService("delete_regions_of_interest_2d", req, res);
}

void SilhouetteMatchClient::initParameters()
{
  // first get the current values from sensor
  const auto j_params = rc_visard_comm_if_->getParameters();
  SilhouetteMatchConfig cfg;

  for (const auto& param : j_params)
  {
    const auto& name = param["name"];
    if (name == "max_number_of_detected_objects")
      cfg.max_number_of_detected_objects = param["value"];
    else if (name == "edge_sensitivity")
      cfg.edge_sensitivity = param["value"];
    else if (name == "match_max_distance")
      cfg.match_max_distance = param["value"];
    else if (name == "match_percentile")
      cfg.match_percentile = param["value"];
    else if (name == "quality")
      cfg.quality = param["value"];
  }

  // second, try to get ROS parameters:
  // if param is not set in parameter server, default to current sensor value
  nh_.param("max_number_of_detected_objects",
            cfg.max_number_of_detected_objects,
            cfg.max_number_of_detected_objects);
  nh_.param("edge_sensitivity", cfg.edge_sensitivity, cfg.edge_sensitivity);
  nh_.param("match_max_distance", cfg.match_max_distance,
            cfg.match_max_distance);
  nh_.param("match_percentile", cfg.match_percentile, cfg.match_percentile);
  nh_.param("quality", cfg.quality, cfg.quality);

  // set callback for dynamic reconfigure that will initially read those values
  dyn_reconf_->setCallback(
      boost::bind(&SilhouetteMatchClient::updateParameters, this, _1, _2));
}

void SilhouetteMatchClient::updateParameters(SilhouetteMatchConfig& config,
                                             uint32_t)
{
  nlohmann::json j_params;
  j_params.emplace_back(
      nlohmann::json{ { "name", "max_number_of_detected_objects" },
                      { "value", config.max_number_of_detected_objects } });
  j_params.emplace_back(nlohmann::json{ { "name", "edge_sensitivity" },
                                        { "value", config.edge_sensitivity } });
  j_params.emplace_back(
      nlohmann::json{ { "name", "match_max_distance" },
                      { "value", config.match_max_distance } });
  j_params.emplace_back(nlohmann::json{ { "name", "match_percentile" },
                                        { "value", config.match_percentile } });
  j_params.emplace_back(
      nlohmann::json{ { "name", "quality" }, { "value", config.quality } });

  if (config.publish_vis)
  {
    if (!visualizer_)
      visualizer_.reset(new Visualizer(nh_));
  }
  else
  {
    visualizer_.reset();
  }

  // XXX TODO set parameters back in config?
  rc_visard_comm_if_->setParameters(j_params);
}

}  // namespace rc_silhouettematch_client
