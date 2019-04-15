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
