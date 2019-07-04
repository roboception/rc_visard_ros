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

#ifndef RC_PICK_CLIENT_PUBLISH_PICK_VISULIZATION_H
#define RC_PICK_CLIENT_PUBLISH_PICK_VISULIZATION_H

#include <ros/ros.h>
#include <rc_pick_client/SuctionGrasp.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <rc_pick_client/LoadCarrier.h>
#include <rc_pick_client/BoxItem.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pick_visualization
{

class Visualization
{
  public:
    explicit Visualization(const ros::NodeHandle &nh);

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
     * Remove previously published markers of this type.
     * Publish box item position as tf frame (name: boxitem_#) and cube markers on topic boxitem in node_namespace
     */
    void visualizeDetectedBoxes(const std::vector<rc_pick_client::BoxItem> &ros_boxitems);

    /*
     * Remove previously published box item markers.
     */
    void deleteBoxItemMarkers();

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

    ros::Publisher grasp_marker_pub_;
    ros::Publisher lc_marker_pub_;
    ros::Publisher box_marker_pub_;
    tf::TransformBroadcaster br_;
    visualization_msgs::MarkerArray markers_lcs_;
    visualization_msgs::MarkerArray markers_grasps_;
    visualization_msgs::MarkerArray markers_boxes_;
    /*
     * Construct model of the load carrier that consist of 5 cube meshes representing 5 walls of the load carrier and
     * append it to marker_array
     */
    static void constructLoadCarrier(visualization_msgs::MarkerArray &marker_array,
                                     const rc_pick_client::LoadCarrier &lc,
                                     const int &lc_no);
    /*
     * Set markers characteristics
     */
    void setMarker(visualization_msgs::Marker &marker, const geometry_msgs::Pose &item_pose, const rc_pick_client::Rectangle &rectangle, std::string frame_id, int marker_id);

    /*
     * Publish given pose as tf frame
     */
    void publishTf(const geometry_msgs::Pose &ros_pose, std::string frame_id, std::string id);


};
}

#endif //RC_PICK_CLIENT_PUBLISH_PICK_VISULIZATION_H
