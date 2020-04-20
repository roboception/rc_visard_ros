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

#include <rc_common_msgs/ReturnCode.h>

#include <json/json.hpp>

namespace geometry_msgs
{

inline void to_json(nlohmann::json& j, const geometry_msgs::Point& p)
{
  j = nlohmann::json{ {"x", p.x}, {"y", p.y}, {"z", p.z} };
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Point& p)
{
  j.at("x").get_to(p.x);
  j.at("y").get_to(p.y);
  j.at("z").get_to(p.z);
}

inline void to_json(nlohmann::json& j, const geometry_msgs::Vector3& p)
{
  j = nlohmann::json{ {"x", p.x}, {"y", p.y}, {"z", p.z} };
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Vector3& p)
{
  j.at("x").get_to(p.x);
  j.at("y").get_to(p.y);
  j.at("z").get_to(p.z);
}

inline void to_json(nlohmann::json& j, const geometry_msgs::Quaternion& p)
{
  j = nlohmann::json{ {"x", p.x}, {"y", p.y}, {"z", p.z}, {"w", p.w} };
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Quaternion& p)
{
  j.at("x").get_to(p.x);
  j.at("y").get_to(p.y);
  j.at("z").get_to(p.z);
  j.at("w").get_to(p.w);
}

inline void to_json(nlohmann::json& j, const geometry_msgs::Pose& p)
{
  j["position"] = p.position;
  j["orientation"] = p.orientation;
}

inline void from_json(const nlohmann::json& j, geometry_msgs::Pose& p)
{
  j.at("position").get_to(p.position);
  j.at("orientation").get_to(p.orientation);
}

}  // namespace geometry_msgs

namespace rc_common_msgs
{
inline void from_json(const nlohmann::json& j, ReturnCode& p)
{
  j.at("value").get_to(p.value);
  j.at("message").get_to(p.message);
}

}  // namespace rc_common_msgs

namespace ros
{
inline void from_json(const nlohmann::json& j, ros::Time& p)
{
  j.at("sec").get_to(p.sec);
  j.at("nsec").get_to(p.nsec);
}

}  // namespace ros

#endif  // RC_JSON_CONVERSIONS_COMMON_H
