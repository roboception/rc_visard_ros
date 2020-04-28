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

#ifndef RC_JSON_CONVERSIONS_COMMON_H
#define RC_JSON_CONVERSIONS_COMMON_H

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#include <json/json.hpp>

namespace geometry_msgs
{
inline void to_json(nlohmann::json& j, const geometry_msgs::Point& r)
{
  j = nlohmann::json{ { "x", r.x }, { "y", r.y }, { "z", r.z } };
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Point& r)
{
  j.at("x").get_to(r.x);
  j.at("y").get_to(r.y);
  j.at("z").get_to(r.z);
}

inline void to_json(nlohmann::json& j, const geometry_msgs::Vector3& r)
{
  j = nlohmann::json{ { "x", r.x }, { "y", r.y }, { "z", r.z } };
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Vector3& r)
{
  j.at("x").get_to(r.x);
  j.at("y").get_to(r.y);
  j.at("z").get_to(r.z);
}

inline void to_json(nlohmann::json& j, const geometry_msgs::Quaternion& r)
{
  j = nlohmann::json{ { "x", r.x }, { "y", r.y }, { "z", r.z }, { "w", r.w } };
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Quaternion& r)
{
  j.at("x").get_to(r.x);
  j.at("y").get_to(r.y);
  j.at("z").get_to(r.z);
  j.at("w").get_to(r.w);
}

inline void to_json(nlohmann::json& j, const geometry_msgs::Pose& r)
{
  j["position"] = r.position;
  j["orientation"] = r.orientation;
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Pose& r)
{
  j.at("position").get_to(r.position);
  j.at("orientation").get_to(r.orientation);
}

}  // namespace geometry_msgs

namespace ros
{
inline void to_json(nlohmann::json& j, const ros::Time& r)
{
  j["sec"] = r.sec;
  j["nsec"] = r.nsec;
}

inline void from_json(const nlohmann::json& j, ros::Time& r)
{
  j.at("sec").get_to(r.sec);
  j.at("nsec").get_to(r.nsec);
}

}  // namespace ros

#endif  // RC_JSON_CONVERSIONS_COMMON_H
