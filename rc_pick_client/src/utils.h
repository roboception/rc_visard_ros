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

#ifndef RC_PICK_CLIENT_UTILS_H
#define RC_PICK_CLIENT_UTILS_H

#include "json/json.hpp"
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <rc_pick_client/Box.h>
#include <rc_pick_client/Item.h>
#include <rc_pick_client/BoxItem.h>
#include <rc_pick_client/BoxModel.h>
#include <rc_pick_client/ItemModel.h>
#include <rc_pick_client/LoadCarrier.h>
#include <rc_pick_client/SuctionGrasp.h>
#include <rc_pick_client/RegionOfInterest.h>
#include <rc_pick_client/Compartment.h>

#include <rc_common_msgs/ReturnCode.h>

using namespace std;
using json = nlohmann::json;

namespace utils
{
void parseReturnCode(rc_common_msgs::ReturnCode &return_code, json &json_resp);

void rosPoseToJson(const geometry_msgs::Pose &pose, json &js_pose);

geometry_msgs::Pose jsonPoseToRos(const json &js_pose);

void jsonTimestampToRos(ros::Time &timestamp, const json &json_time);

void jsonBoxToRos(rc_pick_client::Box &box, const json &json_box);

void jsonBoxToSolidPrimitive(shape_msgs::SolidPrimitive &box, const json &json_box);

void rosSolidPrimitiveToJson(const shape_msgs::SolidPrimitive &box, json &json_box);

void rosBoxToJson(const rc_pick_client::Box &box, json &json_box);

void jsonRectangleToRos(rc_pick_client::Rectangle &box, const json &json_rectangle);

void jsonGraspToRos(std::vector<rc_pick_client::SuctionGrasp> &ros_grasp, const json &json_grasps);

void jsonLoadCarriersToRos(std::vector<rc_pick_client::LoadCarrier> &ros_lcs, const json &json_lcs,
                           const json &timestamp);

void jsonLoadCarriersToRos(std::vector<rc_pick_client::LoadCarrier> &ros_lcs, const json &json_lcs);

void rosBoxModelsToJson(const std::vector<rc_pick_client::BoxModel> &ros_box_models, json &json_item_models);

void jsonBoxItemToRos(std::vector<rc_pick_client::BoxItem> &ros_box_items, const json &json_box_items);

void rosRectangleToJson(const rc_pick_client::Rectangle &rectangle, json &json_rectangle);

void jsonROIsToRos(std::vector<rc_pick_client::RegionOfInterest> &ros_lcs, const json &json_rois);

void rosItemModelsToJson(const std::vector<rc_pick_client::ItemModel> &ros_item_models, json &json_item_models);

void rosCompartmentToJson(const rc_pick_client::Compartment &ros_compartment, json &json_compartment);
}

#endif //RC_PICK_CLIENT_UTILS_H
