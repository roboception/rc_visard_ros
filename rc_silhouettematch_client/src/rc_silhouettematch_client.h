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

#ifndef RC_SILHOUETTEMATCH_CLIENT_H
#define RC_SILHOUETTEMATCH_CLIENT_H

#include <ros/ros.h>

#include <rc_silhouettematch_client/DetectObject.h>
#include <rc_silhouettematch_client/CalibrateBasePlane.h>
#include <rc_silhouettematch_client/DeleteBasePlaneCalibration.h>
#include <rc_silhouettematch_client/GetBasePlaneCalibration.h>
#include <rc_silhouettematch_client/SetRegionOfInterest.h>
#include <rc_silhouettematch_client/GetRegionsOfInterest.h>
#include <rc_silhouettematch_client/DeleteRegionsOfInterest.h>

#include <dynamic_reconfigure/server.h>
#include <rc_silhouettematch_client/SilhouetteMatchConfig.h>

#include "rest_helper.h"

namespace rc_silhouettematch_client
{
class Visualizer;

class SilhouetteMatchClient
{
public:
  SilhouetteMatchClient(const std::string& host, ros::NodeHandle& nh);

  ~SilhouetteMatchClient();

protected:
  bool detectObject(DetectObject::Request& req, DetectObject::Response& res);

  bool calibrateBasePlane(CalibrateBasePlane::Request& req,
                          CalibrateBasePlane::Response& res);

  bool getBasePlaneCalib(GetBasePlaneCalibration::Request& req,
                         GetBasePlaneCalibration::Response& res);

  bool deleteBasePlaneCalib(DeleteBasePlaneCalibration::Request& req,
                            DeleteBasePlaneCalibration::Response& res);

  bool setROI(SetRegionOfInterest::Request& req,
              SetRegionOfInterest::Response& res);

  bool getROIs(GetRegionsOfInterest::Request& req,
               GetRegionsOfInterest::Response& res);

  bool deleteROIs(DeleteRegionsOfInterest::Request& req,
                  DeleteRegionsOfInterest::Response& res);

  void initParameters();

  void updateParameters(SilhouetteMatchConfig& config, uint32_t);

  template <typename Request, typename Response>
  bool callService(const std::string& name, const Request& req, Response& res);

private:
  ros::NodeHandle nh_;
  std::vector<ros::ServiceServer> srvs_;

  std::unique_ptr<rc_rest_api::RestHelper> rest_helper_;
  std::unique_ptr<dynamic_reconfigure::Server<SilhouetteMatchConfig>>
      dyn_reconf_;
  std::unique_ptr<Visualizer> visualizer_;
};

}  // namespace rc_silhouettematch_client

#endif  // RC_SILHOUETTEMATCH_CLIENT_H
