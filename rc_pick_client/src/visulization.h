/* 
* Roboception GmbH 
* Munich, Germany 
* www.roboception.com 
* 
* Copyright (c) 2019 Roboception GmbH 
* All rights reserved 
* 
* Author: Monika Florek-Jasinska
*/
#ifndef RC_ITEMPICK_CLIENT_PUBLISH_ITEMPICK_VISULIZATION_H
#define RC_ITEMPICK_CLIENT_PUBLISH_ITEMPICK_VISULIZATION_H

#include <ros/ros.h>
#include <rc_pick_client/SuctionGrasp.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <rc_pick_client/LoadCarrier.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pick_visualization
{

class Visualization
{
  public:
    Visualization(const ros::NodeHandle &nh);

    ~Visualization();

    /*
     * Remove previously published markers of this type.
     * Publish grasps as tf frames (name: node_namespace/grasp_#) and marker spheres on topic grasp in node_namespace
     * of size (max_suction_surface_length, max_suction_surface_width, 0)
     */
    void visualizeGrasps(const std::vector<rc_pick_client::SuctionGrasp> &ros_grasps);

    /*
     * Remove previously published markers of this type.
     * Publish load carrier position as tf frame (name: lc_#) and load carrier model as 5 cube markers on topic
     * on topic lc in node_namespace
     */
    void visualizeLoadCarriers(const std::vector<rc_pick_client::LoadCarrier> &ros_lcs);

    /*
     * Remove previously published load carrier markers.
     */
    void deleteLoadCarrierMarkers();

    /*
     * Remove previously published grasp markers.
     */
    void deleteGraspMarkers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher grasp_marker_pub;
    ros::Publisher lc_marker_pub;
    tf::TransformBroadcaster br;
    visualization_msgs::MarkerArray markers_lcs;
    visualization_msgs::MarkerArray markers_grasps;

    /*
     * Construct model of the load carrier that consist of 5 cube meshes representing 5 walls of the load carrier and
     * append it to marker_array
     */
    void constructLoadCarrier(visualization_msgs::MarkerArray &marker_array, const rc_pick_client::LoadCarrier &lc,
                              const int &lc_no);

    /*
     * Convert rc_pick_client::SuctionGrasp into markers and publish it
     */
    void publishGraspMarkers(const std::vector<rc_pick_client::SuctionGrasp> &ros_grasps);

    void publishLCMarkers(const std::vector<rc_pick_client::LoadCarrier> &ros_lcs);

    void publishGraspTf(const std::vector<rc_pick_client::SuctionGrasp> &ros_grasps);

    void publishLCTf(const std::vector<rc_pick_client::LoadCarrier> &ros_lcs);


};
}

#endif //RC_ITEMPICK_CLIENT_PUBLISH_ITEMPICK_VISULIZATION_H
