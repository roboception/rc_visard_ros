/*
 * Copyright (c) 2019 Roboception GmbH
 *
 * Author: Carlos Xavier Garcia Briones, Monika Florek-Jasinska
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

#include "itempick_client.h"

namespace ros_pick_client
{

    ItempickClient::ItempickClient(const std::string &host, const ros::NodeHandle &nh): PickClient(host, "rc_itempick", nh)
    {
      srv_compute_grasps_ = nh_.advertiseService("compute_grasps", &ItempickClient::computeGraspsSrv, this);
      server_->setCallback(boost::bind(&ItempickClient::dynamicReconfigureCallback, this, _1, _2));

    }

    bool ItempickClient::computeGraspsSrv(rc_pick_client::ComputeGraspsRequest &request,
                                          rc_pick_client::ComputeGraspsResponse &response)
    {
      //parsing arguments
      json js_args;
      js_args["args"]["suction_surface_length"] = request.suction_surface_length;
      js_args["args"]["suction_surface_width"] = request.suction_surface_width;
      js_args["args"]["pose_frame"] = request.pose_frame;
      if (request.pose_frame == "external")
      {
        utils::rosPoseToJson(request.robot_pose, js_args["args"]["robot_pose"]);
      }
      js_args["args"]["region_of_interest_id"] = request.region_of_interest_id;
      if (request.load_carrier_id != "")
      {
        js_args["args"]["load_carrier_id"] = request.load_carrier_id;
        utils::rosCompartmentToJson(request.load_carrier_compartment, js_args["args"]["load_carrier_compartment"]);
      }
      if (!request.item_models.empty()) utils::rosItemModelsToJson(request.item_models, js_args["args"]["item_models"]);

      //communicating with rc_visard
      auto json_resp = rc_visard_communication_.servicePutRequest("compute_grasps", js_args);
      utils::parseReturnCode(response.return_code, json_resp["return_code"]);
      utils::jsonGraspToRos(response.grasps, json_resp["grasps"]);
      utils::jsonLoadCarriersToRos(response.load_carriers, json_resp["load_carriers"], json_resp["timestamp"]);
      utils::jsonTimestampToRos(response.timestamp, json_resp["timestamp"]);
      visualizer_.visualizeGrasps(response.grasps);
      visualizer_.visualizeLoadCarriers(response.load_carriers);

      return true;
    }

    void ItempickClient::dynamicReconfigureCallback(rc_pick_client::pickModuleConfig &config, uint32_t)
    {
      json js_params = createSharedParameters(config);
      json js_param;
      js_param["name"] = "cluster_max_dimension";
      js_param["value"] = config.cluster_max_dimension;
      js_params.push_back(js_param);
      js_param["name"] = "clustering_patch_size";
      js_param["value"] = config.clustering_patch_size;
      js_params.push_back(js_param);

      rc_visard_communication_.setParameters(js_params);
    }

}
