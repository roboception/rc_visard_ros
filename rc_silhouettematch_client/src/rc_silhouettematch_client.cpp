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
#include "visualizer.h"

using json = nlohmann::json;

namespace rc_silhouettematch_client
{
SilhouetteMatchClient::SilhouetteMatchClient(const std::string& host,
                                             ros::NodeHandle& nh)
  : nh_(nh)
  , rest_helper_(
        new rc_rest_api::RestHelper(host, "rc_silhouettematch", 10000))
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


template <typename Request, typename Response>
bool SilhouetteMatchClient::callService(const std::string& name,
                                        const Request& req, Response& res)
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
    res.return_code.value = -8; // NOT_APPLICABLE
    res.return_code.message = "Not available in this firmware version";
    return false;
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    res.return_code.value = -2; // INTERNAL_ERROR
    res.return_code.message = ex.what();
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
  return true;
}

bool SilhouetteMatchClient::calibrateBasePlane(
    CalibrateBasePlane::Request& req, CalibrateBasePlane::Response& res)
{
  bool success = callService("calibrate_base_plane", req, res);
  if (success && res.return_code.value >= 0 && visualizer_)
  {
    visualizer_->visBasePlane(res.plane, res.timestamp);
  }
  return true;
}

bool SilhouetteMatchClient::getBasePlaneCalib(
    GetBasePlaneCalibration::Request& req,
    GetBasePlaneCalibration::Response& res)
{
  callService("get_base_plane_calibration", req, res);
  return true;
}

bool SilhouetteMatchClient::deleteBasePlaneCalib(
    DeleteBasePlaneCalibration::Request& req,
    DeleteBasePlaneCalibration::Response& res)
{
  callService("delete_base_plane_calibration", req, res);
  return true;
}

bool SilhouetteMatchClient::setROI(SetRegionOfInterest::Request& req,
                                   SetRegionOfInterest::Response& res)
{
  callService("set_region_of_interest_2d", req, res);
  return true;
}

bool SilhouetteMatchClient::getROIs(GetRegionsOfInterest::Request& req,
                                    GetRegionsOfInterest::Response& res)
{
  callService("get_regions_of_interest_2d", req, res);
  return true;
}

bool SilhouetteMatchClient::deleteROIs(DeleteRegionsOfInterest::Request& req,
                                       DeleteRegionsOfInterest::Response& res)
{
  callService("delete_regions_of_interest_2d", req, res);
  return true;
}

void paramsToCfg(const json& params, SilhouetteMatchConfig& cfg)
{
  for (const auto& param : params)
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
}

void SilhouetteMatchClient::initParameters()
{
  // first get the current values from sensor
  const auto j_params = rest_helper_->getParameters();
  SilhouetteMatchConfig cfg;

  paramsToCfg(j_params, cfg);

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
  json j_params;
  j_params.emplace_back(
      json{ { "name", "max_number_of_detected_objects" },
                      { "value", config.max_number_of_detected_objects } });
  j_params.emplace_back(json{ { "name", "edge_sensitivity" },
                                        { "value", config.edge_sensitivity } });
  j_params.emplace_back(
      json{ { "name", "match_max_distance" },
                      { "value", config.match_max_distance } });
  j_params.emplace_back(json{ { "name", "match_percentile" },
                                        { "value", config.match_percentile } });
  j_params.emplace_back(
      json{ { "name", "quality" }, { "value", config.quality } });

  if (config.publish_vis)
  {
    if (!visualizer_)
      visualizer_.reset(new Visualizer(nh_));
  }
  else
  {
    visualizer_.reset();
  }

  json j_params_new = rest_helper_->setParameters(j_params);
  // set config with new params so they are updated if needed
  paramsToCfg(j_params_new, config);
}

}  // namespace rc_silhouettematch_client
