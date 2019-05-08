/*
 * Copyright (c) 2019 Roboception GmbH
 *
 * Author: Monika Florek-Jasinska
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

#ifndef REST_itempick_CLIENT_H
#define REST_itempick_CLIENT_H


#include <std_srvs/Trigger.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <memory>
#include <rc_pick_client/ComputeGrasps.h>
#include <rc_pick_client/DetectLoadCarriers.h>
#include <rc_pick_client/DeleteLoadCarriers.h>
#include <rc_pick_client/DeleteRegionsOfInterest.h>
#include <rc_pick_client/GetLoadCarriers.h>
#include <rc_pick_client/GetRegionsOfInterest.h>
#include <rc_pick_client/SetLoadCarrier.h>
#include <rc_pick_client/SetRegionOfInterest.h>
#include <rc_pick_client/itempickConfig.h>
#include "communication_helper.h"
#include "visualization.h"

namespace itempick_client
{
class ItempickWrapper
{
  public:
    ItempickWrapper(const std::string &host, const ros::NodeHandle &nh);
    ~ItempickWrapper();

  private:

    bool computeGraspsSrv(rc_pick_client::ComputeGraspsRequest &request,
                          rc_pick_client::ComputeGraspsResponse &response);

    bool detectLoadCarriersSrv(rc_pick_client::DetectLoadCarriersRequest &request,
                          rc_pick_client::DetectLoadCarriersResponse &response);

    bool deleteLoadCarriersSrv(rc_pick_client::DeleteLoadCarriersRequest &request,
                               rc_pick_client::DeleteLoadCarriersResponse &response);

    bool getLoadCarriers(rc_pick_client::GetLoadCarriersRequest &request,
                      rc_pick_client::GetLoadCarriersResponse &response);

    bool setLoadCarrier(rc_pick_client::SetLoadCarrierRequest &request,
                        rc_pick_client::SetLoadCarrierResponse &response);

    bool deleteROISrv(rc_pick_client::DeleteRegionsOfInterestRequest &request,
                      rc_pick_client::DeleteRegionsOfInterestResponse &response);

    bool getROIs(rc_pick_client::GetRegionsOfInterestRequest &request,
                         rc_pick_client::GetRegionsOfInterestResponse &response);

    bool setROIs(rc_pick_client::SetRegionOfInterestRequest &request,
                 rc_pick_client::SetRegionOfInterestResponse &response);


    void startItempick();

    void stopItempick();

    void advertiseServices();

    /*
     * Reads itempick parameters from sensor and ros parameter (if a value for a parameter is defined in ROS parameter
     * server this value is used), and start dynamic reconfigure server.
     */
    void initConfiguration();

    void dynamicReconfigureCallback(rc_pick_client::itempickConfig &config, uint32_t);

    ros::NodeHandle nh_;
    ros::ServiceServer srv_compute_grasps_;
    ros::ServiceServer srv_detect_lc_;
    ros::ServiceServer srv_set_lc_;
    ros::ServiceServer srv_get_lcs_;
    ros::ServiceServer srv_delete_lcs_;
    ros::ServiceServer srv_set_roi_;
    ros::ServiceServer srv_get_rois_;
    ros::ServiceServer srv_delete_rois_;

    std::unique_ptr<dynamic_reconfigure::Server<rc_pick_client::itempickConfig> > server_;

    rc_itempick_cpr::CommunicationHelper rc_visard_communication_;
    pick_visualization::Visualization visualizer_;

};
}
#endif
