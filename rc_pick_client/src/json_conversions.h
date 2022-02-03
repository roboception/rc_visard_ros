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

#include <rc_common_msgs/ReturnCode.h>

#include <rc_pick_client/Box.h>
#include <rc_pick_client/Item.h>
#include <rc_pick_client/ItemModel.h>
#include <rc_pick_client/LoadCarrier.h>
#include <rc_pick_client/SuctionGrasp.h>
#include <rc_pick_client/Compartment.h>

#include <rc_pick_client/ComputeGrasps.h>
#include <rc_pick_client/ComputeBoxGrasps.h>
#include <rc_pick_client/DetectItems.h>

namespace rc_common_msgs
{
inline void from_json(const nlohmann::json& j, ReturnCode& r)
{
  j.at("value").get_to(r.value);
  j.at("message").get_to(r.message);
}

}  // namespace rc_common_msgs

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
  // timestamp and overfilled flag is not used when setting load carrier
}

inline void from_json(const nlohmann::json& j, LoadCarrier& r)
{
  j.at("id").get_to(r.id);
  j.at("outer_dimensions").get_to(r.outer_dimensions);
  j.at("inner_dimensions").get_to(r.inner_dimensions);
  j.at("rim_thickness").get_to(r.rim_thickness);
  j.at("pose").get_to(r.pose.pose);
  j.at("pose_frame").get_to(r.pose.header.frame_id);
  // timestamp is set from enclosing msg
  if (j.count("overfilled"))
    j.at("overfilled").get_to(r.overfilled);
}

inline void to_json(nlohmann::json& j, const Compartment& r)
{
  j["pose"] = r.pose;
  j["box"] = r.box;
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
