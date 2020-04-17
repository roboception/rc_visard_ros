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

#include "json_conversions_common.h"

#include <rc_silhouettematch_client/Instance.h>
#include <rc_silhouettematch_client/Plane.h>
#include <rc_silhouettematch_client/EstimatedPlane.h>
#include <rc_silhouettematch_client/RegionOfInterest.h>
#include <rc_silhouettematch_client/ObjectToDetect.h>

#include <rc_silhouettematch_client/CalibrateBasePlane.h>
#include <rc_silhouettematch_client/GetBasePlaneCalibration.h>
#include <rc_silhouettematch_client/DeleteBasePlaneCalibration.h>
#include <rc_silhouettematch_client/SetRegionOfInterest.h>
#include <rc_silhouettematch_client/GetRegionsOfInterest.h>
#include <rc_silhouettematch_client/DeleteRegionsOfInterest.h>
#include <rc_silhouettematch_client/DetectObject.h>


#include <json/json.hpp>

namespace rc_silhouettematch_client
{
inline void from_json(const nlohmann::json& j, Instance& p)
{
  j.at("timestamp").get_to(p.timestamp);
  j.at("id").get_to(p.id);
  j.at("object_id").get_to(p.object_id);
  j.at("pose_frame").get_to(p.pose_frame);
  j.at("pose").get_to(p.pose);
}

inline void to_json(nlohmann::json& j, const Plane& p)
{
  j["distance"] = p.distance;
  j["normal"] = p.normal;
}

inline void from_json(const nlohmann::json& j, EstimatedPlane& p)
{
  j.at("pose_frame").get_to(p.pose_frame);
  j.at("distance").get_to(p.distance);
  j.at("normal").get_to(p.normal);
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
  j.at("id").get_to(p.id);
  j.at("offset_x").get_to(p.offset_x);
  j.at("offset_y").get_to(p.offset_y);
  j.at("width").get_to(p.width);
  j.at("height").get_to(p.height);
}

inline void to_json(nlohmann::json& j, const ObjectToDetect& p)
{
  j["object_id"] = p.object_id;
  j["region_of_interest_2d_id"] = p.region_of_interest_2d_id;
}

inline void from_json(const nlohmann::json& j, ObjectToDetect& p)
{
  j.at("object_id").get_to(p.object_id);
  j.at("region_of_interest_2d_id").get_to(p.region_of_interest_2d_id);
}

inline void to_json(nlohmann::json& j, const DetectObject::Request& p)
{
  j["object_to_detect"] = p.object_to_detect;
  j["offset"] = p.offset;
  j["pose_frame"] = p.pose_frame;
  j["robot_pose"] = p.robot_pose;
}

inline void from_json(const nlohmann::json& j, DetectObject::Response& p)
{
  j.at("timestamp").get_to(p.timestamp);
  j.at("return_code").get_to(p.return_code);
  j.at("object_id").get_to(p.object_id);
  j.at("instances").get_to(p.instances);
}

inline void to_json(nlohmann::json& j, const CalibrateBasePlane::Request& p)
{
  j["pose_frame"] = p.pose_frame;
  j["robot_pose"] = p.robot_pose;
  j["plane_estimation_method"] = p.plane_estimation_method;
  j["region_of_interest_2d_id"] = p.region_of_interest_2d_id;
  j["offset"] = p.offset;
  if (p.plane_estimation_method == "STEREO")
  {
    j["stereo"] = { {"plane_preference", p.stereo.plane_preference} };
  }
  else
  {
    j["plane"] = p.plane;
  }
}

inline void from_json(const nlohmann::json& j, CalibrateBasePlane::Response& p)
{
  j.at("timestamp").get_to(p.timestamp);
  j.at("return_code").get_to(p.return_code);
  j.at("plane").get_to(p.plane);
}

inline void to_json(nlohmann::json& j, const GetBasePlaneCalibration::Request& p)
{
  j["pose_frame"] = p.pose_frame;
  j["robot_pose"] = p.robot_pose;
}

inline void from_json(const nlohmann::json& j, GetBasePlaneCalibration::Response& p)
{
  j.at("return_code").get_to(p.return_code);
  j.at("plane").get_to(p.plane);
}

inline void to_json(nlohmann::json& j, const DeleteBasePlaneCalibration::Request& p)
{
  j = {};
}

inline void from_json(const nlohmann::json& j, DeleteBasePlaneCalibration::Response& p)
{
  j.at("return_code").get_to(p.return_code);
}

inline void to_json(nlohmann::json& j, const SetRegionOfInterest::Request& p)
{
  j["region_of_interest_2d"] = p.region_of_interest_2d;
}

inline void from_json(const nlohmann::json& j, SetRegionOfInterest::Response& p)
{
  j.at("return_code").get_to(p.return_code);
}

inline void to_json(nlohmann::json& j, const GetRegionsOfInterest::Request& p)
{
  j["region_of_interest_2d_ids"] = p.region_of_interest_2d_ids;
}

inline void from_json(const nlohmann::json& j, GetRegionsOfInterest::Response& p)
{
  j.at("regions_of_interest").get_to(p.regions_of_interest);
  j.at("return_code").get_to(p.return_code);
}

inline void to_json(nlohmann::json& j, const DeleteRegionsOfInterest::Request& p)
{
  j["region_of_interest_2d_ids"] = p.region_of_interest_2d_ids;
}

inline void from_json(const nlohmann::json& j, DeleteRegionsOfInterest::Response& p)
{
  j.at("return_code").get_to(p.return_code);
}

}  // namespace rc_silhouettematch_client

#endif  // RC_SILHOUETTEMATCH_CLIENT_JSON_CONVERSIONS_H
