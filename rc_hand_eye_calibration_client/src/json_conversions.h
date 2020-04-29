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

#ifndef RC_HAND_EYE_CALIB_JSON_CONVERSIONS_H
#define RC_HAND_EYE_CALIB_JSON_CONVERSIONS_H

#include "json_conversions_common.h"

#include <rc_hand_eye_calibration_client/SetCalibrationPose.h>
#include <rc_hand_eye_calibration_client/SetCalibration.h>
#include <rc_hand_eye_calibration_client/Calibration.h>
#include <rc_hand_eye_calibration_client/Trigger.h>

namespace rc_hand_eye_calibration_client
{
inline void to_json(nlohmann::json& j, const TriggerRequest& r)
{
  j = {};
}

inline void from_json(const nlohmann::json& j, TriggerResponse& r)
{
  j.at("success").get_to(r.success);
  j.at("status").get_to(r.status);
  j.at("message").get_to(r.message);
}

inline void to_json(nlohmann::json& j, const SetCalibrationPoseRequest& r)
{
  j["slot"] = r.slot;
  j["pose"] = r.pose;
}

inline void from_json(const nlohmann::json& j, SetCalibrationPoseResponse& r)
{
  j.at("success").get_to(r.success);
  j.at("status").get_to(r.status);
  j.at("message").get_to(r.message);
}

inline void to_json(nlohmann::json& j, const CalibrationRequest& r)
{
  j = {};
}

inline void from_json(const nlohmann::json& j, CalibrationResponse& r)
{
  j.at("success").get_to(r.success);
  j.at("status").get_to(r.status);
  j.at("message").get_to(r.message);
  j.at("pose").get_to(r.pose);
  j.at("error").get_to(r.error);
  j.at("robot_mounted").get_to(r.robot_mounted);
}

inline void to_json(nlohmann::json& j, const SetCalibrationRequest& r)
{
  j["pose"] = r.pose;
  j["robot_mounted"] = r.robot_mounted;
}

inline void from_json(const nlohmann::json& j, SetCalibrationResponse& r)
{
  j.at("success").get_to(r.success);
  j.at("status").get_to(r.status);
  j.at("message").get_to(r.message);
}

}  // namespace rc_hand_eye_calibration_client

#endif  // RC_HAND_EYE_CALIB_JSON_CONVERSIONS_H
