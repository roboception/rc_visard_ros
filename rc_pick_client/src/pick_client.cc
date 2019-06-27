/*
 * Copyright (c) 2019 Roboception GmbH
 *
 * Author: Monika Florek-Jasinska, Carlos Xavier Garcia Briones
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

#include "pick_client.h"

namespace ros_pick_client
{

PickClient::PickClient(const string &host, const string &node_name, const ros::NodeHandle &nh)
        : nh_(nh), rc_visard_communication_(host, node_name, 10000), visualizer_(nh)
{
  initConfiguration();
  advertiseServices();
  startPick();
}

PickClient::~PickClient()
{
  try
  {
    stopPick();
  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Exception during destruction of PickClient: %s", ex.what());
  }
  catch (...)
  {
    ROS_FATAL("Exception during destruction of PickClient");
  }
}

void PickClient::startPick()
{
  rc_visard_communication_.servicePutRequest("start");
}

void PickClient::stopPick()
{
  rc_visard_communication_.servicePutRequest("stop");
}

bool PickClient::detectLoadCarriersSrv(rc_pick_client::DetectLoadCarriersRequest &request,
                                       rc_pick_client::DetectLoadCarriersResponse &response)
{
  //parsing arguments
  json js_args;
  js_args["args"]["pose_frame"] = request.pose_frame;
  if (request.pose_frame == "external")
  {
    utils::rosPoseToJson(request.robot_pose, js_args["args"]["robot_pose"]);
  }
  js_args["args"]["region_of_interest_id"] = request.region_of_interest_id;
  for (auto lc_id : request.load_carrier_ids)
  {
    js_args["args"]["load_carrier_ids"].push_back(lc_id);
  }

  //communicating with rc_visard
  auto json_resp = rc_visard_communication_.servicePutRequest("detect_load_carriers", js_args);
  utils::parseReturnCode(response.return_code, json_resp["return_code"]);
  utils::jsonLoadCarriersToRos(response.load_carriers, json_resp["load_carriers"], json_resp["timestamp"]);
  utils::jsonTimestampToRos(response.timestamp, json_resp["timestamp"]);
  visualizer_.visualizeLoadCarriers(response.load_carriers);

  return true;
}

bool PickClient::deleteLoadCarriersSrv(rc_pick_client::DeleteLoadCarriersRequest &request,
                                       rc_pick_client::DeleteLoadCarriersResponse &response)
{
  //parsing arguments
  json json_args;
  for (auto single_lc : request.load_carrier_ids)
  {
    json_args["args"]["load_carrier_ids"].push_back(single_lc);
  }

  //communicating with rc_visard
  auto json_resp = rc_visard_communication_.servicePutRequest("delete_load_carriers", json_args);
  utils::parseReturnCode(response.return_code, json_resp["return_code"]);

  return true;
}

bool PickClient::deleteROISrv(rc_pick_client::DeleteRegionsOfInterestRequest &request,
                              rc_pick_client::DeleteRegionsOfInterestResponse &response)
{
  //parsing arguments
  json json_args;
  for (auto single_roi : request.region_of_interest_ids)
  {
    json_args["args"]["region_of_interest_ids"].push_back(single_roi);
  }

  //communicating with rc_visard
  auto json_resp = rc_visard_communication_.servicePutRequest("delete_regions_of_interest", json_args);
  utils::parseReturnCode(response.return_code, json_resp["return_code"]);

  return true;
}

bool PickClient::getROIs(rc_pick_client::GetRegionsOfInterestRequest &request,
                         rc_pick_client::GetRegionsOfInterestResponse &response)
{
  //parsing arguments
  json json_args;
  for (auto single_roi : request.region_of_interest_ids)
  {
    if (single_roi != "") json_args["args"]["region_of_interest_ids"].push_back(single_roi);
  }

  //communicating with rc_visard
  json json_resp = rc_visard_communication_.servicePutRequest("get_regions_of_interest", json_args);
  utils::jsonROIsToRos(response.regions_of_interest, json_resp["regions_of_interest"]);
  utils::parseReturnCode(response.return_code, json_resp["return_code"]);
  return true;
}


bool PickClient::getLoadCarriers(rc_pick_client::GetLoadCarriersRequest &request,
                                 rc_pick_client::GetLoadCarriersResponse &response)
{
  //parsing arguments
  json json_args;
  for (auto single_lc : request.load_carrier_ids)
  {
    if (single_lc != "") json_args["args"]["load_carrier_ids"].push_back(single_lc);
  }

  //communicating with rc_visard
  json json_resp = rc_visard_communication_.servicePutRequest("get_load_carriers", json_args);
  utils::jsonLoadCarriersToRos(response.load_carriers, json_resp["load_carriers"]);
  utils::parseReturnCode(response.return_code, json_resp["return_code"]);
  return true;
}

bool PickClient::setLoadCarrier(rc_pick_client::SetLoadCarrierRequest &request,
                                rc_pick_client::SetLoadCarrierResponse &response)
{
  //parsing arguments
  json json_args;
  json_args["id"] = request.load_carrier.id;
  json_args["rim_thickness"]["x"] = request.load_carrier.rim_thickness.x;
  json_args["rim_thickness"]["y"] = request.load_carrier.rim_thickness.y;
  utils::rosBoxToJson(request.load_carrier.inner_dimensions, json_args["inner_dimensions"]);
  utils::rosBoxToJson(request.load_carrier.outer_dimensions, json_args["outer_dimensions"]);
  json json_args_lc;
  json_args_lc["args"]["load_carrier"] = json_args;

  //communicating with rc_visard
  json json_resp = rc_visard_communication_.servicePutRequest("set_load_carrier", json_args_lc);
  utils::parseReturnCode(response.return_code, json_resp["return_code"]);
  return true;
}

bool PickClient::setROIs(rc_pick_client::SetRegionOfInterestRequest &request,
                         rc_pick_client::SetRegionOfInterestResponse &response)
{
  //parsing arguments
  json json_args;
  json_args["id"] = request.region_of_interest.id;
  if (request.region_of_interest.pose.header.frame_id == "camera")
  {
    json_args["pose_frame"] = "camera";
  }
  else if (request.region_of_interest.pose.header.frame_id == "external")
  {
    json_args["pose_frame"] = "external";
  }
  else
  {
    throw runtime_error("frame_id of a region of interest has to be of type \"camera\" or \"external\"");
  }
  utils::rosSolidPrimitiveToJson(request.region_of_interest.primitive, json_args);
  utils::rosPoseToJson(request.region_of_interest.pose.pose, json_args["pose"]);
  json json_args_roi;
  json_args_roi["args"]["region_of_interest"] = json_args;
  utils::rosPoseToJson(request.robot_pose, json_args_roi["args"]["robot_pose"]);

  //communicating with rc_visard
  json json_resp = rc_visard_communication_.servicePutRequest("set_region_of_interest", json_args_roi);
  utils::parseReturnCode(response.return_code, json_resp["return_code"]);
  return true;
}

void PickClient::advertiseServices()
{
  srv_detect_lc_ = nh_.advertiseService("detect_load_carriers", &PickClient::detectLoadCarriersSrv, this);
  srv_delete_lcs_ = nh_.advertiseService("delete_load_carriers", &PickClient::deleteLoadCarriersSrv, this);
  srv_get_lcs_ = nh_.advertiseService("get_load_carriers", &PickClient::getLoadCarriers, this);
  srv_set_lc_ = nh_.advertiseService("set_load_carrier", &PickClient::setLoadCarrier, this);
  srv_delete_rois_ = nh_.advertiseService("delete_regions_of_interest", &PickClient::deleteROISrv, this);
  srv_get_rois_ = nh_.advertiseService("get_regions_of_interest", &PickClient::getROIs, this);
  srv_set_roi_ = nh_.advertiseService("set_region_of_interest", &PickClient::setROIs, this);
}

void PickClient::initConfiguration()
{
  rc_pick_client::pickModuleConfig cfg;

  // first get the current values from sensor
  auto json_resp = rc_visard_communication_.getParameters();

  for (auto &param : json_resp)
  {
    string name = param["name"];
    if (param["name"] == "cluster_max_curvature")
    {
      cfg.cluster_max_curvature = param["value"];
    }
    else if (param["name"] == "cluster_max_dimension")
    {
      cfg.cluster_max_dimension = param["value"];
    }
    else if (param["name"] == "clustering_discontinuity_factor")
    {
      cfg.clustering_discontinuity_factor = param["value"];
    }
    else if (param["name"] == "clustering_max_surface_rmse")
    {
      cfg.clustering_max_surface_rmse = param["value"];
    }
    else if (param["name"] == "clustering_patch_size")
    {
      cfg.clustering_patch_size = param["value"];
    }
    else if (param["name"] == "load_carrier_crop_distance")
    {
      cfg.load_carrier_crop_distance = param["value"];
    }
    else if (param["name"] == "load_carrier_model_tolerance")
    {
      cfg.load_carrier_model_tolerance = param["value"];
    }
  }

  // second, try to get ROS parameters:
  // if parameter is not set in parameter server, we default to current sensor configuration
  nh_.param("cluster_max_curvature", cfg.cluster_max_curvature, cfg.cluster_max_curvature);
  nh_.param("cluster_max_dimension", cfg.cluster_max_dimension, cfg.cluster_max_dimension);
  nh_.param("clustering_discontinuity_factor", cfg.clustering_discontinuity_factor,
            cfg.clustering_discontinuity_factor);
  nh_.param("clustering_max_surface_rmse", cfg.clustering_max_surface_rmse, cfg.clustering_max_surface_rmse);
  nh_.param("clustering_patch_size", cfg.clustering_patch_size, cfg.clustering_patch_size);
  nh_.param("load_carrier_crop_distance", cfg.load_carrier_crop_distance, cfg.load_carrier_crop_distance);
  nh_.param("load_carrier_model_tolerance", cfg.load_carrier_model_tolerance, cfg.load_carrier_model_tolerance);

  // set parameters on parameter server so that dynamic reconfigure picks them up
  nh_.setParam("cluster_max_curvature", cfg.cluster_max_curvature);
  nh_.setParam("cluster_max_dimension", cfg.cluster_max_dimension);
  nh_.setParam("clustering_discontinuity_factor", cfg.clustering_discontinuity_factor);
  nh_.setParam("clustering_max_surface_rmse", cfg.clustering_max_surface_rmse);
  nh_.setParam("clustering_patch_size", cfg.clustering_patch_size);
  nh_.setParam("load_carrier_crop_distance", cfg.load_carrier_crop_distance);
  nh_.setParam("load_carrier_model_tolerance", cfg.load_carrier_model_tolerance);

  // instantiate dynamic reconfigure server that will initially read those values
  using RCFSRV = dynamic_reconfigure::Server<rc_pick_client::pickModuleConfig>;
  server_ = unique_ptr<RCFSRV>(new dynamic_reconfigure::Server<rc_pick_client::pickModuleConfig>(nh_));
}

json PickClient::createSharedParameters(rc_pick_client::pickModuleConfig &config)
{
// fill json request from dynamic reconfigure request
  json js_params, js_param;
  js_param["name"] = "cluster_max_curvature";
  js_param["value"] = config.cluster_max_curvature;
  js_params.push_back(js_param);
  js_param["name"] = "clustering_discontinuity_factor";
  js_param["value"] = config.clustering_discontinuity_factor;
  js_param["name"] = "clustering_max_surface_rmse";
  js_param["value"] = config.clustering_max_surface_rmse;
  js_params.push_back(js_param);
  js_param["name"] = "load_carrier_crop_distance";
  js_param["value"] = config.load_carrier_crop_distance;
  js_params.push_back(js_param);
  js_param["name"] = "load_carrier_model_tolerance";
  js_param["value"] = config.load_carrier_model_tolerance;
  js_params.push_back(js_param);
  return js_params;
}

}

