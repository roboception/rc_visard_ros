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

#include "rest_itempick_client.h"
#include "json/json.hpp"

#include <rc_common_msgs/ReturnCode.h>

using namespace std;
using json = nlohmann::json;

namespace //anonymous
{

void parseReturnCode(rc_common_msgs::ReturnCode &return_code, json &json_resp)
{
  return_code.value = json_resp["value"];
  return_code.message = json_resp["message"];
}

void rosPoseToJson(const geometry_msgs::Pose &pose, json &js_pose)
{
  json js_ori, js_pos;
  js_pos["x"] = pose.position.x;
  js_pos["y"] = pose.position.y;
  js_pos["z"] = pose.position.z;
  js_ori["x"] = pose.orientation.x;
  js_ori["y"] = pose.orientation.y;
  js_ori["z"] = pose.orientation.z;
  js_ori["w"] = pose.orientation.w;
  js_pose["position"] = js_pos;
  js_pose["orientation"] = js_ori;
}

geometry_msgs::Pose jsonPoseToRos(const json &js_pose)
{
  json js_pos = js_pose["position"];
  json js_ori = js_pose["orientation"];
  geometry_msgs::Pose pose;
  pose.position.x = js_pos["x"];
  pose.position.y = js_pos["y"];
  pose.position.z = js_pos["z"];
  pose.orientation.x = js_ori["x"];
  pose.orientation.y = js_ori["y"];
  pose.orientation.z = js_ori["z"];
  pose.orientation.w = js_ori["w"];
  return pose;
}

void jsonTimestampToRos(ros::Time &timestamp, const json &json_time)
{
  timestamp.sec = json_time["sec"];
  timestamp.nsec = json_time["nsec"];
}

void jsonBoxToRos(rc_pick_client::Box &box, const json &json_box)
{
  box.x = json_box["x"];
  box.y = json_box["y"];
  box.z = json_box["z"];
}

void jsonBoxToSolidPrimitive(shape_msgs::SolidPrimitive &box, const json &json_box)
{
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions[shape_msgs::SolidPrimitive::BOX_X] = json_box["x"];
  box.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = json_box["y"];
  box.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = json_box["z"];
}

void rosSolidPrimitiveToJson(const shape_msgs::SolidPrimitive &box, json &json_box)
{
  if (box.type == shape_msgs::SolidPrimitive::BOX)
  {
    json_box["type"] = "BOX";
    json_box["box"]["x"] = box.dimensions[shape_msgs::SolidPrimitive::BOX_X];
    json_box["box"]["y"] = box.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    json_box["box"]["z"] = box.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
  }
  else if (box.type == shape_msgs::SolidPrimitive::SPHERE)
  {
    json_box["type"] = "SPHERE";
    json_box["sphere"]["radius"] = box.dimensions[shape_msgs::SolidPrimitive::BOX_X];
  }
  else
  {
    throw runtime_error("Type of solid primitive has to be of type \"box\" or \"sphere\"");
  }
}

void rosBoxToJson(const rc_pick_client::Box &box, json &json_box)
{
  json_box["x"] = box.x;
  json_box["y"] = box.y;
  json_box["z"] = box.z;
}

void jsonRectangleToRos(rc_pick_client::Rectangle &box, const json &json_rectangle)
{
  box.x = json_rectangle["x"];
  box.y = json_rectangle["y"];
}

void jsonGraspToRos(std::vector<rc_pick_client::SuctionGrasp> &ros_grasp, const json &json_grasps)
{
  for (auto single_grasp : json_grasps)
  {
    rc_pick_client::SuctionGrasp new_grasp;
    jsonTimestampToRos(new_grasp.pose.header.stamp, single_grasp["timestamp"]);
    new_grasp.pose.header.frame_id = single_grasp["pose_frame"];
    new_grasp.pose.pose = jsonPoseToRos(single_grasp["pose"]);
    new_grasp.uuid = single_grasp["uuid"];
    new_grasp.max_suction_surface_length = single_grasp["max_suction_surface_length"];
    new_grasp.max_suction_surface_width = single_grasp["max_suction_surface_width"];
    new_grasp.uuid = single_grasp["uuid"];
    new_grasp.quality = single_grasp["quality"];
    ros_grasp.push_back(new_grasp);
  }
}

void jsonLoadCarriersToRos(std::vector<rc_pick_client::LoadCarrier> &ros_lcs, const json &json_lcs,
                           const json &timestamp)
{
  for (auto single_load_carrier : json_lcs)
  {
    rc_pick_client::LoadCarrier new_lc;
    jsonTimestampToRos(new_lc.pose.header.stamp, timestamp);
    new_lc.pose.pose = jsonPoseToRos(single_load_carrier["pose"]);
    new_lc.pose.header.frame_id = single_load_carrier["pose_frame"];
    new_lc.id = single_load_carrier["id"];
    jsonBoxToRos(new_lc.inner_dimensions, single_load_carrier["inner_dimensions"]);
    jsonBoxToRos(new_lc.outer_dimensions, single_load_carrier["outer_dimensions"]);
    jsonRectangleToRos(new_lc.rim_thickness, single_load_carrier["rim_thickness"]);
    ros_lcs.push_back(new_lc);
  }
}

void jsonLoadCarriersToRos(std::vector<rc_pick_client::LoadCarrier> &ros_lcs, const json &json_lcs)
{
  for (auto single_load_carrier : json_lcs)
  {
    rc_pick_client::LoadCarrier new_lc;
    new_lc.pose.pose = jsonPoseToRos(single_load_carrier["pose"]);
    new_lc.pose.header.frame_id = single_load_carrier["pose_frame"];
    new_lc.id = single_load_carrier["id"];
    jsonBoxToRos(new_lc.inner_dimensions, single_load_carrier["inner_dimensions"]);
    jsonBoxToRos(new_lc.outer_dimensions, single_load_carrier["outer_dimensions"]);
    jsonRectangleToRos(new_lc.rim_thickness, single_load_carrier["rim_thickness"]);
    ros_lcs.push_back(new_lc);
  }
}


void jsonROIsToRos(std::vector<rc_pick_client::RegionOfInterest> &ros_lcs, const json &json_rois)
{
  for (auto single_roi : json_rois)
  {
    rc_pick_client::RegionOfInterest new_roi;
    new_roi.pose.pose = jsonPoseToRos(single_roi["pose"]);
    new_roi.pose.header.frame_id = single_roi["frame"];
    if (single_roi["type"] == "SPHERE")
    {
      new_roi.primitive.type = shape_msgs::SolidPrimitive::SPHERE;
      new_roi.primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = single_roi["sphere"]["radius"];
    }
    else if (single_roi["type"] == "BOX")
    {
      new_roi.primitive.type = shape_msgs::SolidPrimitive::BOX;
      jsonBoxToSolidPrimitive(new_roi.primitive, single_roi["box"]);
    }
    new_roi.id = single_roi["id"];
    ros_lcs.push_back(new_roi);
  }
}

void rosItemModelsToJson(const std::vector<rc_pick_client::ItemModel> &ros_item_models, json &json_item_models)
{
  for (auto single_model : ros_item_models)
  {
    json new_model;
    if (single_model.type == "UNKNOWN")
    {
      new_model["type"] = "UNKNOWN";
      rosBoxToJson(single_model.unknown.max_dimensions, new_model["unknown"]["max_dimensions"]);
      rosBoxToJson(single_model.unknown.min_dimensions, new_model["unknown"]["min_dimensions"]);
    }
    else
    {
      throw runtime_error("Type of the model has to be of type \"UNKNOWN\"");
    }
    json_item_models.push_back(new_model);
  }
}

void rosCompartmentToJson(const rc_pick_client::Compartment &ros_compartment, json &json_compartment)
{
  rosBoxToJson(ros_compartment.box, json_compartment["box"]);
  rosPoseToJson(ros_compartment.pose, json_compartment["pose"]);
}

}//anonymous ns


namespace itempick_client
{

ItempickWrapper::ItempickWrapper(const string &host, const ros::NodeHandle &nh)
        : nh_(nh), rc_visard_communication_(host, "rc_itempick", 10000), visualizer_(nh)
{
  initConfiguration();
  advertiseServices();
  startItempick();
}

ItempickWrapper::~ItempickWrapper()
{
  try
  {
    stopItempick();
  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Exception during destruction of ItempickWrapper: %s", ex.what());
  }
  catch (...)
  {
    ROS_FATAL("Exception during destruction of ItempickWrapper");
  }
}

void ItempickWrapper::startItempick()
{
  rc_visard_communication_.servicePutRequest("start");
}

void ItempickWrapper::stopItempick()
{
  rc_visard_communication_.servicePutRequest("stop");
}

bool ItempickWrapper::detectLoadCarriersSrv(rc_pick_client::DetectLoadCarriersRequest &request,
                                            rc_pick_client::DetectLoadCarriersResponse &response)
{
  //parsing arguments
  json js_args;
  js_args["args"]["pose_frame"] = request.pose_frame;
  if (request.pose_frame == "external")
  {
    rosPoseToJson(request.robot_pose, js_args["args"]["robot_pose"]);
  }
  js_args["args"]["region_of_interest_id"] = request.region_of_interest_id;
  for (auto lc_id : request.load_carrier_ids)
  {
    js_args["args"]["load_carrier_ids"].push_back(lc_id);
  }

  //communicating with rc_visard
  auto json_resp = rc_visard_communication_.servicePutRequest("detect_load_carriers", js_args);
  parseReturnCode(response.return_code, json_resp["return_code"]);
  jsonLoadCarriersToRos(response.load_carriers, json_resp["load_carriers"], json_resp["timestamp"]);
  jsonTimestampToRos(response.timestamp, json_resp["timestamp"]);
  visualizer_.visualizeLoadCarriers(response.load_carriers);

  return true;
}

bool ItempickWrapper::computeGraspsSrv(rc_pick_client::ComputeGraspsRequest &request,
                                       rc_pick_client::ComputeGraspsResponse &response)
{
  //parsing arguments
  json js_args;
  js_args["args"]["suction_surface_length"] = request.suction_surface_length;
  js_args["args"]["suction_surface_width"] = request.suction_surface_width;
  js_args["args"]["pose_frame"] = request.pose_frame;
  if (request.pose_frame == "external")
  {
    rosPoseToJson(request.robot_pose, js_args["args"]["robot_pose"]);
  }
  js_args["args"]["region_of_interest_id"] = request.region_of_interest_id;
  if (request.load_carrier_id != "")
  {
    js_args["args"]["load_carrier_id"] = request.load_carrier_id;
    rosCompartmentToJson(request.load_carrier_compartment, js_args["args"]["load_carrier_compartment"]);
  }
  if (!request.item_models.empty())rosItemModelsToJson(request.item_models, js_args["args"]["item_models"]);

  //communicating with rc_visard
  auto json_resp = rc_visard_communication_.servicePutRequest("compute_grasps", js_args);
  parseReturnCode(response.return_code, json_resp["return_code"]);
  jsonGraspToRos(response.grasps, json_resp["grasps"]);
  jsonLoadCarriersToRos(response.load_carriers, json_resp["load_carriers"], json_resp["timestamp"]);
  jsonTimestampToRos(response.timestamp, json_resp["timestamp"]);
  visualizer_.visualizeGrasps(response.grasps);
  visualizer_.visualizeLoadCarriers(response.load_carriers);

  return true;
}

bool ItempickWrapper::deleteLoadCarriersSrv(rc_pick_client::DeleteLoadCarriersRequest &request,
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
  parseReturnCode(response.return_code, json_resp["return_code"]);

  return true;
}

bool ItempickWrapper::deleteROISrv(rc_pick_client::DeleteRegionsOfInterestRequest &request,
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
  parseReturnCode(response.return_code, json_resp["return_code"]);

  return true;
}

bool ItempickWrapper::getROIs(rc_pick_client::GetRegionsOfInterestRequest &request,
                              rc_pick_client::GetRegionsOfInterestResponse &response)
{
  //parsing arguments
  json json_args;
  for (auto single_roi : request.region_of_interest_ids)
  {
    json_args["args"]["region_of_interest_ids"].push_back(single_roi);
  }

  //communicating with rc_visard
  json json_resp = rc_visard_communication_.servicePutRequest("get_regions_of_interest", json_args);
  jsonROIsToRos(response.regions_of_interest, json_resp["regions_of_interest"]);
  parseReturnCode(response.return_code, json_resp["return_code"]);
  return true;
}


bool ItempickWrapper::getLoadCarriers(rc_pick_client::GetLoadCarriersRequest &request,
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
  jsonLoadCarriersToRos(response.load_carriers, json_resp["load_carriers"]);
  parseReturnCode(response.return_code, json_resp["return_code"]);
  return true;
}

bool ItempickWrapper::setLoadCarrier(rc_pick_client::SetLoadCarrierRequest &request,
                                     rc_pick_client::SetLoadCarrierResponse &response)
{
  //parsing arguments
  json json_args;
  json_args["id"] = request.load_carrier.id;
  json_args["rim_thickness"]["x"] = request.load_carrier.rim_thickness.x;
  json_args["rim_thickness"]["y"] = request.load_carrier.rim_thickness.y;
  rosBoxToJson(request.load_carrier.inner_dimensions, json_args["inner_dimensions"]);
  rosBoxToJson(request.load_carrier.outer_dimensions, json_args["outer_dimensions"]);
  json json_args_lc;
  json_args_lc["args"]["load_carrier"] = json_args;

  //communicating with rc_visard
  json json_resp = rc_visard_communication_.servicePutRequest("set_load_carrier", json_args_lc);
  parseReturnCode(response.return_code, json_resp["return_code"]);
  return true;
}

bool ItempickWrapper::setROIs(rc_pick_client::SetRegionOfInterestRequest &request,
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
  rosSolidPrimitiveToJson(request.region_of_interest.primitive, json_args);
  rosPoseToJson(request.region_of_interest.pose.pose, json_args["pose"]);
  json json_args_roi;
  json_args_roi["args"]["region_of_interest"] = json_args;
  rosPoseToJson(request.robot_pose, json_args_roi["args"]["robot_pose"]);

  //communicating with rc_visard
  json json_resp = rc_visard_communication_.servicePutRequest("set_region_of_interest", json_args_roi);
  parseReturnCode(response.return_code, json_resp["return_code"]);
  return true;
}

void ItempickWrapper::advertiseServices()
{
  srv_compute_grasps_ = nh_.advertiseService("compute_grasps", &ItempickWrapper::computeGraspsSrv, this);
  srv_detect_lc_ = nh_.advertiseService("detect_load_carriers", &ItempickWrapper::detectLoadCarriersSrv, this);
  srv_delete_lcs_ = nh_.advertiseService("delete_load_carriers", &ItempickWrapper::deleteLoadCarriersSrv, this);
  srv_get_lcs_ = nh_.advertiseService("get_load_carriers", &ItempickWrapper::getLoadCarriers, this);
  srv_set_lc_ = nh_.advertiseService("set_load_carrier", &ItempickWrapper::setLoadCarrier, this);
  srv_delete_rois_ = nh_.advertiseService("delete_regions_of_interest", &ItempickWrapper::deleteROISrv, this);
  srv_get_rois_ = nh_.advertiseService("get_regions_of_interest", &ItempickWrapper::getROIs, this);
  srv_set_roi_ = nh_.advertiseService("set_region_of_interest", &ItempickWrapper::setROIs, this);
}

void ItempickWrapper::initConfiguration()
{
  rc_pick_client::itempickConfig cfg;

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
  using RCFSRV = dynamic_reconfigure::Server<rc_pick_client::itempickConfig>;
  server_ = unique_ptr<RCFSRV>(new dynamic_reconfigure::Server<rc_pick_client::itempickConfig>(nh_));
  server_->setCallback(boost::bind(&ItempickWrapper::dynamicReconfigureCallback, this, _1, _2));
}

void ItempickWrapper::dynamicReconfigureCallback(rc_pick_client::itempickConfig
                                                 &config,
                                                 uint32_t)
{
  // fill json request from dynamic reconfigure request
  json js_params, js_param;
  js_param["name"] = "cluster_max_curvature";
  js_param["value"] = config.cluster_max_curvature;
  js_params.push_back(js_param);
  js_param["name"] = "cluster_max_dimension";
  js_param["value"] = config.cluster_max_dimension;
  js_params.push_back(js_param);
  js_param["name"] = "clustering_discontinuity_factor";
  js_param["value"] = config.clustering_discontinuity_factor;
  js_param["name"] = "clustering_max_surface_rmse";
  js_param["value"] = config.clustering_max_surface_rmse;
  js_params.push_back(js_param);
  js_param["name"] = "clustering_patch_size";
  js_param["value"] = config.clustering_patch_size;
  js_params.push_back(js_param);
  js_param["name"] = "load_carrier_crop_distance";
  js_param["value"] = config.load_carrier_crop_distance;
  js_params.push_back(js_param);
  js_param["name"] = "load_carrier_model_tolerance";
  js_param["value"] = config.load_carrier_model_tolerance;
  js_params.push_back(js_param);

  rc_visard_communication_.setParameters(js_params);
}

}
