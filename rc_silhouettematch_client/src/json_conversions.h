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

#ifndef RC_SILHOUETTEMATCH_CLIENT_JSON_CONVERSIONS_H
#define RC_SILHOUETTEMATCH_CLIENT_JSON_CONVERSIONS_H

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <rc_common_msgs/ReturnCode.h>
#include <rc_silhouettematch_client/Instance.h>
#include <rc_silhouettematch_client/Plane.h>
#include <rc_silhouettematch_client/EstimatedPlane.h>
#include <rc_silhouettematch_client/RegionOfInterest.h>

#include <json/json.hpp>

namespace geometry_msgs
{
inline void to_json(nlohmann::json& j, const geometry_msgs::Pose& p)
{
  nlohmann::json j_rot, j_pos;
  j_pos["x"] = p.position.x;
  j_pos["y"] = p.position.y;
  j_pos["z"] = p.position.z;
  j_rot["x"] = p.orientation.x;
  j_rot["y"] = p.orientation.y;
  j_rot["z"] = p.orientation.z;
  j_rot["w"] = p.orientation.w;
  j["position"] = j_pos;
  j["orientation"] = j_rot;
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Pose& p)
{
  const auto& j_pos = j["position"];
  const auto& j_rot = j["orientation"];
  p.position.x = j_pos["x"];
  p.position.y = j_pos["y"];
  p.position.z = j_pos["z"];
  p.orientation.x = j_rot["x"];
  p.orientation.y = j_rot["y"];
  p.orientation.z = j_rot["z"];
  p.orientation.w = j_rot["w"];
}

}  // namespace geometry_msgs

namespace rc_common_msgs
{
inline void from_json(const nlohmann::json& j, ReturnCode& p)
{
  p.value = j.at("value");
  p.message = j.at("message");
}

}  // namespace rc_common_msgs

namespace ros
{
inline void from_json(const nlohmann::json& j, ros::Time& p)
{
  p.sec = j.at("sec");
  p.nsec = j.at("nsec");
}

}  // namespace ros

namespace rc_silhouettematch_client
{
inline void from_json(const nlohmann::json& j, Instance& p)
{
  p.timestamp = j.at("timestamp");
  p.id = j.at("id");
  p.object_id = j.at("object_id");
  p.pose_frame = j.at("pose_frame");
  p.pose = j.at("pose");
}

inline void to_json(nlohmann::json& j, const Plane& p)
{
  j["distance"] = p.distance;
  auto& j_normal = j["normal"];
  j_normal["x"] = p.normal.x;
  j_normal["y"] = p.normal.y;
  j_normal["z"] = p.normal.z;
}

inline void from_json(const nlohmann::json& j, EstimatedPlane& p)
{
  p.pose_frame = j.at("pose_frame");
  p.distance = j.at("distance");
  const auto& j_normal = j.at("normal");
  p.normal.x = j_normal.at("x");
  p.normal.y = j_normal.at("y");
  p.normal.z = j_normal.at("z");
}

inline void to_json(nlohmann::json& j, const RegionOfInterest& p)
{
  j["id"] = p.id;
  j["offset_x"] = p.offset_x;
  j["offset_y"] = p.offset_y;
  j["width"] = p.width;
  j["height"] = p.height;
}

inline void from_json(const nlohmann::json& j, RegionOfInterest& p)
{
  p.id = j.at("id");
  p.offset_x = j.at("offset_x");
  p.offset_y = j.at("offset_y");
  p.width = j.at("width");
  p.height = j.at("height");
}

}  // namespace rc_silhouettematch_client

#endif  // RC_SILHOUETTEMATCH_CLIENT_JSON_CONVERSIONS_H
