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
#include "utils.h"

using namespace std;
using json = nlohmann::json;

namespace utils
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

void rosRectangleToJson(const rc_pick_client::Rectangle &rectangle, json &json_rectangle)
{
  json_rectangle["x"] = rectangle.x;
  json_rectangle["y"] = rectangle.y;
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


void rosBoxModelsToJson(const std::vector<rc_pick_client::BoxModel> &ros_box_models, json &json_item_models)
{
  for (auto single_model : ros_box_models)
  {
    json new_model;
    new_model["type"] = single_model.type;
    rosRectangleToJson(single_model.rectangle.max_dimensions, new_model["rectangle"]["max_dimensions"]);
    rosRectangleToJson(single_model.rectangle.min_dimensions, new_model["rectangle"]["min_dimensions"]);
    json_item_models.push_back(new_model);
  }
}

void jsonBoxItemToRos(std::vector<rc_pick_client::BoxItem> &ros_box_items, const json &json_box_items)
{
  for (auto single_box_item : json_box_items)
  {
    rc_pick_client::BoxItem new_box_item;
    new_box_item.uuid = single_box_item["uuid"];
    if (!single_box_item["grasp_uuids"].empty())
    {
      for (auto &grasp_uuid : single_box_item["grasp_uuids"])
      {
        new_box_item.grasp_uuids.push_back(grasp_uuid);
      }
    }
    new_box_item.pose = jsonPoseToRos(single_box_item["pose"]);
    new_box_item.pose_frame = single_box_item["pose_frame"];
    new_box_item.type = single_box_item["type"];
    jsonRectangleToRos(new_box_item.rectangle, single_box_item["rectangle"]);
    jsonTimestampToRos(new_box_item.timestamp, single_box_item["timestamp"]);
    ros_box_items.push_back(new_box_item);
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
    new_roi.id = single_roi["id"];
    new_roi.pose.pose = jsonPoseToRos(single_roi["pose"]);
    new_roi.pose.header.frame_id = single_roi["pose_frame"];
    new_roi.primitive.dimensions.resize(3);
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
}
