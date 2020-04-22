/*
 * Copyright (c) 2020 Roboception GmbH
 *
 * Author: Felix Ruess
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

#ifndef RC_PICK_CLIENT_JSON_CONVERSIONS_H
#define RC_PICK_CLIENT_JSON_CONVERSIONS_H

#include "json_conversions_common.h"

#include <rc_pick_client/Box.h>
#include <rc_pick_client/Item.h>
#include <rc_pick_client/ItemModel.h>
#include <rc_pick_client/LoadCarrier.h>
#include <rc_pick_client/SuctionGrasp.h>
#include <rc_pick_client/RegionOfInterest.h>
#include <rc_pick_client/Compartment.h>

#include <rc_pick_client/DetectLoadCarriers.h>
#include <rc_pick_client/DeleteLoadCarriers.h>
#include <rc_pick_client/DeleteRegionsOfInterest.h>
#include <rc_pick_client/GetLoadCarriers.h>
#include <rc_pick_client/GetRegionsOfInterest.h>
#include <rc_pick_client/SetLoadCarrier.h>
#include <rc_pick_client/SetRegionOfInterest.h>
#include <rc_pick_client/ComputeGrasps.h>
#include <rc_pick_client/ComputeBoxGrasps.h>
#include <rc_pick_client/DetectItems.h>

namespace rc_pick_client
{
inline void to_json(nlohmann::json& j, const Box& r)
{
  j["x"] = r.x;
  j["y"] = r.y;
  j["z"] = r.z;
}

inline void from_json(const nlohmann::json& j, Box& r)
{
  j.at("x").get_to(r.x);
  j.at("y").get_to(r.y);
  j.at("z").get_to(r.z);
}

inline void to_json(nlohmann::json& j, const RangeBox& r)
{
  j["min_dimensions"] = r.min_dimensions;
  j["max_dimensions"] = r.max_dimensions;
}

inline void from_json(const nlohmann::json& j, RangeBox& r)
{
  j.at("min_dimensions").get_to(r.min_dimensions);
  j.at("max_dimensions").get_to(r.max_dimensions);
}

inline void to_json(nlohmann::json& j, const Rectangle& r)
{
  j["x"] = r.x;
  j["y"] = r.y;
}

inline void from_json(const nlohmann::json& j, Rectangle& r)
{
  j.at("x").get_to(r.x);
  j.at("y").get_to(r.y);
}

inline void to_json(nlohmann::json& j, const RangeRectangle& r)
{
  j["min_dimensions"] = r.min_dimensions;
  j["max_dimensions"] = r.max_dimensions;
}

inline void from_json(const nlohmann::json& j, RangeRectangle& r)
{
  j.at("min_dimensions").get_to(r.min_dimensions);
  j.at("max_dimensions").get_to(r.max_dimensions);
}

inline void from_json(const nlohmann::json& j, SuctionGrasp& r)
{
  j.at("uuid").get_to(r.uuid);
  j.at("item_uuid").get_to(r.item_uuid);
  j.at("pose").get_to(r.pose.pose);
  j.at("timestamp").get_to(r.pose.header.stamp);
  j.at("pose_frame").get_to(r.pose.header.frame_id);
  j.at("quality").get_to(r.quality);
  j.at("max_suction_surface_length").get_to(r.max_suction_surface_length);
  j.at("max_suction_surface_width").get_to(r.max_suction_surface_width);
}

inline void to_json(nlohmann::json& j, const ItemModel& r)
{
  j["type"] = r.type;
  if (r.type == r.UNKNOWN)
  {
    j["unknown"] = r.unknown;
  }
  else if (r.type == r.RECTANGLE)
  {
    j["rectangle"] = r.rectangle;
  }
}

inline void from_json(const nlohmann::json& j, Item& r)
{
  j.at("uuid").get_to(r.uuid);
  j.at("grasp_uuids").get_to(r.grasp_uuids);
  j.at("type").get_to(r.type);
  j.at("rectangle").get_to(r.rectangle);
  j.at("pose").get_to(r.pose.pose);
  j.at("timestamp").get_to(r.pose.header.stamp);
  j.at("pose_frame").get_to(r.pose.header.frame_id);;
}

inline void to_json(nlohmann::json& j, const LoadCarrier& r)
{
  j["id"] = r.id;
  j["outer_dimensions"] = r.outer_dimensions;
  j["inner_dimensions"] = r.inner_dimensions;
  j["rim_thickness"] = r.rim_thickness;
  j["pose"] = r.pose.pose;
  j["pose_frame"] = r.pose.header.frame_id;
  j["timestamp"] = r.pose.header.stamp;
}

inline void from_json(const nlohmann::json& j, LoadCarrier& r)
{
  j.at("id").get_to(r.id);
  j.at("outer_dimensions").get_to(r.outer_dimensions);
  j.at("inner_dimensions").get_to(r.inner_dimensions);
  j.at("rim_thickness").get_to(r.rim_thickness);
  j.at("pose").get_to(r.pose.pose);
  j.at("pose_frame").get_to(r.pose.header.frame_id);
  j.at("timestamp").get_to(r.pose.header.stamp);
}

inline void to_json(nlohmann::json& j, const Compartment& r)
{
  j["pose"] = r.pose;
  j["box"] = r.box;
}

inline void to_json(nlohmann::json& j, const RegionOfInterest& r)
{
  j["id"] = r.id;
  if (r.primitive.type == shape_msgs::SolidPrimitive::BOX)
  {
    j["type"] = "BOX";
    j["box"]["x"] = r.primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X];
    j["box"]["y"] = r.primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    j["box"]["z"] = r.primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z];
  }
  else if (r.primitive.type == shape_msgs::SolidPrimitive::SPHERE)
  {
    j["type"] = "SPHERE";
    j["sphere"]["radius"] = r.primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X];
  }
  else
  {
    throw std::runtime_error("Type of solid primitive has to be of type \"box\" or \"sphere\"");
  }
  j["pose"] = r.pose.pose;
  j["pose_frame"] = r.pose.header.frame_id;
}

inline void from_json(const nlohmann::json& j, RegionOfInterest& r)
{
  j.at("id").get_to(r.id);
  if (j.at("type") == "BOX")
  {
    r.primitive.type = shape_msgs::SolidPrimitive::BOX;
    r.primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = j.at("x");
    r.primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = j.at("y");
    r.primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = j.at("z");
  }
  else if (j.at("type") == "SPHERE")
  {
    r.primitive.type = shape_msgs::SolidPrimitive::SPHERE;
    r.primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = j.at("sphere").at("radius");
  }
  else
  {
    throw std::runtime_error("Type has to be \"BOX\" or \"SPHERE\"");
  }
  j.at("pose").get_to(r.pose.pose);
  j.at("pose_frame").get_to(r.pose.header.frame_id);
}

inline void to_json(nlohmann::json& j, const CollisionDetection& r)
{
  j["gripper_id"] = r.gripper_id;
  if (r.pre_grasp_offset.x != 0 || r.pre_grasp_offset.y != 0 || r.pre_grasp_offset.z != 0)
  {
    j["pre_grasp_offset"] = r.pre_grasp_offset;
  }
}

// Services

inline void to_json(nlohmann::json& j, const SetLoadCarrierRequest& r)
{
  j["load_carrier"] = r.load_carrier;
}

inline void from_json(const nlohmann::json& j, SetLoadCarrierResponse& r)
{
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const GetLoadCarriersRequest& r)
{
  j["load_carrier_ids"] = r.load_carrier_ids;
}

inline void from_json(const nlohmann::json& j, GetLoadCarriersResponse& r)
{
  j.at("load_carriers").get_to(r.load_carriers);
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const DeleteLoadCarriersRequest& r)
{
  j["load_carrier_ids"] = r.load_carrier_ids;
}

inline void from_json(const nlohmann::json& j, DeleteLoadCarriersResponse& r)
{
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const DetectLoadCarriersRequest& r)
{
  j["pose_frame"] = r.pose_frame;
  j["region_of_interest_id"] = r.region_of_interest_id;
  j["load_carrier_ids"] = r.load_carrier_ids;
  if (r.pose_frame == "external")
  {
    j["robot_pose"] = r.robot_pose;
  }
}

inline void from_json(const nlohmann::json& j, DetectLoadCarriersResponse& r)
{
  j.at("timestamp").get_to(r.timestamp);
  j.at("load_carriers").get_to(r.load_carriers);
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const SetRegionOfInterestRequest& r)
{
  j["region_of_interest"] = r.region_of_interest;
  j["pose_frame"] = r.region_of_interest.pose.header.frame_id;
  if (r.region_of_interest.pose.header.frame_id == "external")
  {
    j["robot_pose"] = r.robot_pose;
  }
}

inline void from_json(const nlohmann::json& j, SetRegionOfInterestResponse& r)
{
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const GetRegionsOfInterestRequest& r)
{
  j["region_of_interest_ids"] = r.region_of_interest_ids;
}

inline void from_json(const nlohmann::json& j, GetRegionsOfInterestResponse& r)
{
  j.at("regions_of_interest").get_to(r.regions_of_interest);
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const DeleteRegionsOfInterestRequest& r)
{
  j["load_carrier_ids"] = r.region_of_interest_ids;
}

inline void from_json(const nlohmann::json& j, DeleteRegionsOfInterestResponse& r)
{
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const ComputeGraspsRequest& r)
{
  j["pose_frame"] = r.pose_frame;
  if (r.pose_frame == "external")
  {
    j["robot_pose"] = r.robot_pose;
  }
  if (!r.region_of_interest_id.empty())
  {
    j["region_of_interest_id"] = r.region_of_interest_id;
  }
  if (!r.load_carrier_id.empty())
  {
    j["load_carrier_id"] = r.load_carrier_id;
    j["load_carrier_compartment"] = r.load_carrier_compartment;
  }
  j["item_models"] = r.item_models;
  j["suction_surface_length"] = r.suction_surface_length;
  j["suction_surface_width"] = r.suction_surface_width;
  if (!r.collision_detection.gripper_id.empty())
  {
    j["collision_detection"] = r.collision_detection;
  }
}

inline void from_json(const nlohmann::json& j, ComputeGraspsResponse& r)
{
  j.at("timestamp").get_to(r.timestamp);
  j.at("load_carriers").get_to(r.load_carriers);
  j.at("grasps").get_to(r.grasps);
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const ComputeBoxGraspsRequest& r)
{
  j["pose_frame"] = r.pose_frame;
  if (r.pose_frame == "external")
  {
    j["robot_pose"] = r.robot_pose;
  }
  if (!r.region_of_interest_id.empty())
  {
    j["region_of_interest_id"] = r.region_of_interest_id;
  }
  if (!r.load_carrier_id.empty())
  {
    j["load_carrier_id"] = r.load_carrier_id;
    j["load_carrier_compartment"] = r.load_carrier_compartment;
  }
  if (!r.item_models.empty())
  {
    j["item_models"] = r.item_models;
  }
  j["suction_surface_length"] = r.suction_surface_length;
  j["suction_surface_width"] = r.suction_surface_width;
  if (!r.collision_detection.gripper_id.empty())
  {
    j["collision_detection"] = r.collision_detection;
  }
}

inline void from_json(const nlohmann::json& j, ComputeBoxGraspsResponse& r)
{
  j.at("timestamp").get_to(r.timestamp);
  j.at("load_carriers").get_to(r.load_carriers);
  j.at("items").get_to(r.items);
  j.at("grasps").get_to(r.grasps);
  j.at("return_code").get_to(r.return_code);
}

inline void to_json(nlohmann::json& j, const DetectItemsRequest& r)
{
  j["item_models"] = r.item_models;
  j["pose_frame"] = r.pose_frame;
  if (r.pose_frame == "external")
  {
    j["robot_pose"] = r.robot_pose;
  }
  if (!r.region_of_interest_id.empty())
  {
    j["region_of_interest_id"] = r.region_of_interest_id;
  }
  if (!r.load_carrier_id.empty())
  {
    j["load_carrier_id"] = r.load_carrier_id;
    j["load_carrier_compartment"] = r.load_carrier_compartment;
  }
}

inline void from_json(const nlohmann::json& j, DetectItemsResponse& r)
{
  j.at("timestamp").get_to(r.timestamp);
  j.at("load_carriers").get_to(r.load_carriers);
  j.at("items").get_to(r.items);
  j.at("return_code").get_to(r.return_code);
}

}  // namespace rc_pick_client

#endif  // RC_PICK_CLIENT_JSON_CONVERSIONS_H
