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

#ifndef RC_PICK_CLIENT_PICK_CLIENT_H
#define RC_PICK_CLIENT_PICK_CLIENT_H

#include <std_srvs/Trigger.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <memory>
#include <rc_pick_client/pickModuleConfig.h>
#include "json/json.hpp"
#include "json_conversions.h"

#include "rest_helper.h"
#include "visualization.h"

using json = nlohmann::json;

namespace ros_pick_client
{
class PickClient
{
public:
  PickClient(const std::string& host, const std::string& node_name, const ros::NodeHandle& nh);

  virtual ~PickClient();

protected:
  ros::NodeHandle nh_;
  std::unique_ptr<dynamic_reconfigure::Server<rc_pick_client::pickModuleConfig>> server_;

  rc_rest_api::RestHelper rest_helper_;

  pick_visualization::Visualization visualizer_;

  json createSharedParameters(rc_pick_client::pickModuleConfig& config);
  void paramsToCfg(const json& params, rc_pick_client::pickModuleConfig& cfg);

  template <typename Request, typename Response>
  bool callService(const std::string& name, const Request& req, Response& res)
  {
    try
    {
      json j_req = req;
      const auto j_res = rest_helper_.servicePutRequest(name, j_req);
      res = j_res;
      return true;
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
      res.return_code.value = -2;  // INTERNAL_ERROR
      res.return_code.message = ex.what();
      return false;
    }
  }

  void startPick();

  void stopPick();

  void initConfiguration();

  virtual void dynamicReconfigureCallback(rc_pick_client::pickModuleConfig& config, uint32_t) = 0;
};
}  // namespace ros_pick_client

#endif  // RC_PICK_CLIENT_PICK_CLIENT_H
