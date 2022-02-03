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

#include <rc_common_msgs/ReturnCode.h>

#include <rc_silhouettematch_client/Instance.h>
#include <rc_silhouettematch_client/Plane.h>
#include <rc_silhouettematch_client/EstimatedPlane.h>
#include <rc_silhouettematch_client/ObjectToDetect.h>

#include <rc_silhouettematch_client/CalibrateBasePlane.h>
#include <rc_silhouettematch_client/GetBasePlaneCalibration.h>
#include <rc_silhouettematch_client/DeleteBasePlaneCalibration.h>
#include <rc_silhouettematch_client/DetectObject.h>

namespace rc_common_msgs
{
inline void from_json(const nlohmann::json& j, ReturnCode& r)
{
  j.at("value").get_to(r.value);
  j.at("message").get_to(r.message);
}

}  // namespace rc_common_msgs

namespace rc_silhouettematch_client
{
inline void from_json(const nlohmann::json& j, Instance& r)
{
  j.at("timestamp").get_to(r.timestamp);
  j.at("id").get_to(r.id);
  j.at("object_id").get_to(r.object_id);
  j.at("pose_frame").get_to(r.pose_frame);
  j.at("pose").get_to(r.pose);
}

inline void to_json(nlohmann::json& j, const Plane& r)
{
  j["distance"] = r.distance;
  j["normal"] = r.normal;
}

inline void from_json(const nlohmann::json& j, EstimatedPlane& r)
{
  j.at("pose_frame").get_to(r.pose_frame);
  j.at("distance").get_to(r.distance);
  j.at("normal").get_to(r.normal);
}

inline void to_json(nlohmann::json& j, const ObjectToDetect& r)
{
  j["object_id"] = r.object_id;
  j["region_of_interest_2d_id"] = r.region_of_interest_2d_id;
}

inline void from_json(const nlohmann::json& j, ObjectToDetect& r)
{
  j.at("object_id").get_to(r.object_id);
  j.at("region_of_interest_2d_id").get_to(r.region_of_interest_2d_id);
}

inline void to_json(nlohmann::json& j, const DetectObject::Request& r)
{
  j["object_to_detect"] = r.object_to_detect;
  j["offset"] = r.offset;
  j["pose_frame"] = r.pose_frame;
  j["robot_pose"] = r.robot_pose;
}

inline void from_json(const nlohmann::json& j, DetectObject::Response& r)
{
  j.at("timestamp").get_to(r.timestamp);
  j.at("return_code").get_to(r.return_code);
  j.at("object_id").get_to(r.object_id);
  j.at("instances").get_to(r.instances);
}

inline void to_json(nlohmann::json& j, const CalibrateBasePlane::Request& r)
{
  j["pose_frame"] = r.pose_frame;
  j["robot_pose"] = r.robot_pose;
  j["plane_estimation_method"] = r.plane_estimation_method;
  j["region_of_interest_2d_id"] = r.region_of_interest_2d_id;
  j["offset"] = r.offset;
  if (r.plane_estimation_method == "STEREO")
  {
    j["stereo"] = { { "plane_preference", r.stereo.plane_preference } };
  }
  else
  {
    j["plane"] = r.plane;
  }
}

inline void from_json(const nlohmann::json& j, CalibrateBasePlane::Response& r)
{
  j.at("timestamp").get_to(r.timestamp);
  j.at("return_code").get_to(r.return_code);
  j.at("plane").get_to(r.plane);
}

inline void to_json(nlohmann::json& j, const GetBasePlaneCalibration::Request& r)
{
  j["pose_frame"] = r.pose_frame;
  j["robot_pose"] = r.robot_pose;
}

inline void from_json(const nlohmann::json& j, GetBasePlaneCalibration::Response& r)
{
  j.at("return_code").get_to(r.return_code);
  j.at("plane").get_to(r.plane);
}

inline void to_json(nlohmann::json& j, const DeleteBasePlaneCalibration::Request& r)
{
  j = {};
}

inline void from_json(const nlohmann::json& j, DeleteBasePlaneCalibration::Response& r)
{
  j.at("return_code").get_to(r.return_code);
}

}  // namespace rc_silhouettematch_client

#endif  // RC_SILHOUETTEMATCH_CLIENT_JSON_CONVERSIONS_H
