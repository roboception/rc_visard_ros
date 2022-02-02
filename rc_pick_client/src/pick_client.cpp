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

#include "pick_client.h"

using std::string;

namespace ros_pick_client
{
PickClient::PickClient(const string& host, const string& node_name, const ros::NodeHandle& nh)
  : nh_(nh), rest_helper_(host, node_name, 10000), visualizer_(nh)
{
  initConfiguration();
  startPick();
}

PickClient::~PickClient()
{
  try
  {
    stopPick();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("Exception during destruction of PickClient: %s", ex.what());
  }
  catch (...)
  {
    ROS_FATAL("Exception during destruction of PickClient");
  }
}

void PickClient::startPick()
{
  rest_helper_.servicePutRequest("start");
}

void PickClient::stopPick()
{
  rest_helper_.servicePutRequest("stop");
}

void PickClient::paramsToCfg(const json& params, rc_pick_client::pickModuleConfig& cfg)
{
  for (const auto& param : params)
  {
    const auto& name = param["name"];
    if (name == "max_grasps")
    {
      cfg.max_grasps = param["value"];
    }
    if (name == "cluster_max_curvature")
    {
      cfg.cluster_max_curvature = param["value"];
    }
    else if (name == "cluster_max_dimension")
    {
      cfg.cluster_max_dimension = param["value"];
    }
    else if (name == "clustering_discontinuity_factor")
    {
      cfg.clustering_discontinuity_factor = param["value"];
    }
    else if (name == "clustering_max_surface_rmse")
    {
      cfg.clustering_max_surface_rmse = param["value"];
    }
    else if (name == "clustering_patch_size")
    {
      cfg.clustering_patch_size = param["value"];
    }
  }
}

void PickClient::initConfiguration()
{
  rc_pick_client::pickModuleConfig cfg;

  // first get the current values from sensor
  const auto j_params = rest_helper_.getParameters();

  paramsToCfg(j_params, cfg);

  // second, try to get ROS parameters:
  // if parameter is not set in parameter server, we default to current sensor configuration
  nh_.param("cluster_max_curvature", cfg.cluster_max_curvature, cfg.cluster_max_curvature);
  nh_.param("cluster_max_dimension", cfg.cluster_max_dimension, cfg.cluster_max_dimension);
  nh_.param("clustering_discontinuity_factor", cfg.clustering_discontinuity_factor,
            cfg.clustering_discontinuity_factor);
  nh_.param("clustering_max_surface_rmse", cfg.clustering_max_surface_rmse, cfg.clustering_max_surface_rmse);
  nh_.param("clustering_patch_size", cfg.clustering_patch_size, cfg.clustering_patch_size);
  nh_.param("max_grasps", cfg.max_grasps, cfg.max_grasps);

  // set parameters on parameter server so that dynamic reconfigure picks them up
  nh_.setParam("cluster_max_curvature", cfg.cluster_max_curvature);
  nh_.setParam("cluster_max_dimension", cfg.cluster_max_dimension);
  nh_.setParam("clustering_discontinuity_factor", cfg.clustering_discontinuity_factor);
  nh_.setParam("clustering_max_surface_rmse", cfg.clustering_max_surface_rmse);
  nh_.setParam("clustering_patch_size", cfg.clustering_patch_size);
  nh_.setParam("max_grasps", cfg.max_grasps);

  // instantiate dynamic reconfigure server that will initially read those values
  using ReconfServer = dynamic_reconfigure::Server<rc_pick_client::pickModuleConfig>;
  server_ = std::unique_ptr<ReconfServer>(new ReconfServer(nh_));
}

json PickClient::createSharedParameters(rc_pick_client::pickModuleConfig& config)
{
  // fill json request from dynamic reconfigure request
  json js_params, js_param;
  js_param["name"] = "cluster_max_curvature";
  js_param["value"] = config.cluster_max_curvature;
  js_params.push_back(js_param);
  js_param["name"] = "clustering_discontinuity_factor";
  js_param["value"] = config.clustering_discontinuity_factor;
  js_params.push_back(js_param);
  js_param["name"] = "clustering_max_surface_rmse";
  js_param["value"] = config.clustering_max_surface_rmse;
  js_params.push_back(js_param);
  js_param["name"] = "max_grasps";
  js_param["value"] = config.max_grasps;
  js_params.push_back(js_param);
  return js_params;
}

}  // namespace ros_pick_client
