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

#include "visualization.h"

namespace pick_visualization
{
Visualization::Visualization(const ros::NodeHandle& nh) : nh_(nh)
{
  grasp_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("grasp", 1);
  lc_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lc", 1);
  box_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("boxitem", 1);
}

Visualization::~Visualization()
{
  try
  {
    deleteGraspMarkers();
    deleteLoadCarrierMarkers();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL("Exception during destruction of Visualization: %s", ex.what());
  }
  catch (...)
  {
    ROS_FATAL("Exception during destruction of Visualization");
  }
}

void Visualization::constructLoadCarrier(visualization_msgs::MarkerArray& marker_array,
                                         const rc_pick_client::LoadCarrier& lc, const int& lc_no)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = lc.pose.header.frame_id;
  marker.type = marker.CUBE;
  marker.color.r = 0.0f;
  marker.color.g = 0.20f;
  marker.color.b = 0.80f;
  marker.color.a = 0.3;
  tf2::Transform bin_transform(tf2::Quaternion(lc.pose.pose.orientation.x, lc.pose.pose.orientation.y,
                                               lc.pose.pose.orientation.z, lc.pose.pose.orientation.w),
                               tf2::Vector3(lc.pose.pose.position.x, lc.pose.pose.position.y, lc.pose.pose.position.z));

  double rim_thickness_x = (lc.outer_dimensions.x - lc.inner_dimensions.x) / 2;
  double dist_from_center_x = (lc.outer_dimensions.x - rim_thickness_x) / 2;
  double rim_thickness_y = (lc.outer_dimensions.y - lc.inner_dimensions.y) / 2;
  double dist_from_center_y = (lc.outer_dimensions.y - rim_thickness_y) / 2;
  double bottom_thickness = lc.outer_dimensions.z - lc.inner_dimensions.z;
  tf2::Vector3 pose(0, 0, -(lc.outer_dimensions.z - bottom_thickness) / 2);

  // calculate bottom
  pose = bin_transform * pose;
  marker.id = lc_no;
  marker.pose = lc.pose.pose;
  marker.pose.position.x = pose.getX();
  marker.pose.position.y = pose.getY();
  marker.pose.position.z = pose.getZ();
  marker.scale.x = lc.outer_dimensions.x;
  marker.scale.y = lc.outer_dimensions.y;
  marker.scale.z = bottom_thickness;
  marker_array.markers.push_back(marker);

  // calculate longer sides
  pose = tf2::Vector3(dist_from_center_x, 0, 0);
  pose = bin_transform * pose;

  marker.id = lc_no + 1;
  marker.pose = lc.pose.pose;
  marker.pose.position.x = pose.getX();
  marker.pose.position.y = pose.getY();
  marker.pose.position.z = pose.getZ();
  marker.scale.x = rim_thickness_x;
  marker.scale.y = lc.outer_dimensions.y;
  marker.scale.z = lc.outer_dimensions.z;
  marker_array.markers.push_back(marker);

  marker.id = lc_no + 2;
  pose = tf2::Vector3(-dist_from_center_x, 0, 0);
  pose = bin_transform * pose;
  marker.pose.position.x = pose.getX();
  marker.pose.position.y = pose.getY();
  marker.pose.position.z = pose.getZ();
  marker_array.markers.push_back(marker);

  // calculate shorter side
  marker.id = lc_no + 3;
  pose = tf2::Vector3(0, dist_from_center_y, 0);
  pose = bin_transform * pose;
  marker.pose = lc.pose.pose;
  marker.pose.position.x = pose.getX();
  marker.pose.position.y = pose.getY();
  marker.pose.position.z = pose.getZ();
  marker.scale.x = lc.outer_dimensions.x;
  marker.scale.y = rim_thickness_y;
  marker.scale.z = lc.outer_dimensions.z;
  marker_array.markers.push_back(marker);

  marker.id = lc_no + 4;
  pose = tf2::Vector3(0, -dist_from_center_y, 0);
  pose = bin_transform * pose;
  marker.pose = lc.pose.pose;
  marker.pose.position.x = pose.getX();
  marker.pose.position.y = pose.getY();
  marker.pose.position.z = pose.getZ();
  marker.scale.x = lc.outer_dimensions.x;
  marker_array.markers.push_back(marker);
}

void Visualization::deleteLoadCarrierMarkers()
{
  for (auto& i : markers_lcs_.markers)
  {
    i.action = visualization_msgs::Marker::DELETE;
  }
  lc_marker_pub_.publish(markers_lcs_);
  markers_lcs_.markers.clear();
}

void Visualization::deleteGraspMarkers()
{
  for (auto& i : markers_grasps_.markers)
  {
    i.action = visualization_msgs::Marker::DELETE;
  }
  grasp_marker_pub_.publish(markers_grasps_);
  markers_grasps_.markers.clear();
}

void Visualization::deleteBoxItemMarkers()
{
  for (auto& i : markers_boxes_.markers)
  {
    i.action = visualization_msgs::Marker::DELETE;
  }
  box_marker_pub_.publish(markers_boxes_);
  markers_boxes_.markers.clear();
}

void Visualization::visualizeLoadCarriers(const std::vector<rc_pick_client::LoadCarrier>& ros_lcs)
{
  deleteLoadCarrierMarkers();
  if (!ros_lcs.empty())
  {
    int counter = 0;
    for (auto& single_lc : ros_lcs)
    {
      constructLoadCarrier(markers_lcs_, single_lc, counter);
      publishTf(single_lc.pose.pose, single_lc.pose.header.frame_id, "lc_" + std::to_string(counter));
      counter++;
    }
    lc_marker_pub_.publish(markers_lcs_);
  }
}

void Visualization::visualizeLoadCarriers(const std::vector<rc_pick_client::LoadCarrierWithFillingLevel>& ros_lcs)
{
  deleteLoadCarrierMarkers();
  if (!ros_lcs.empty())
  {
    int counter = 0;
    for (auto& lc_with_level : ros_lcs)
    {
      rc_pick_client::LoadCarrier lc;
      lc.id = lc_with_level.id;
      lc.outer_dimensions = lc_with_level.outer_dimensions;
      lc.inner_dimensions = lc_with_level.inner_dimensions;
      lc.rim_thickness = lc_with_level.rim_thickness;
      lc.pose = lc_with_level.pose;
      lc.overfilled = lc_with_level.overfilled;
      constructLoadCarrier(markers_lcs_, lc, counter);
      publishTf(lc.pose.pose, lc.pose.header.frame_id, "lc_" + std::to_string(counter));
      // TODO: visualize filling level somehow?
      counter++;
    }
    lc_marker_pub_.publish(markers_lcs_);
  }
}

void Visualization::visualizeGrasps(const std::vector<rc_pick_client::SuctionGrasp>& ros_grasps)
{
  deleteGraspMarkers();
  if (!ros_grasps.empty())
  {
    int counter = 0;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = marker.SPHERE;
    rc_pick_client::Rectangle rect;
    for (auto& single_grasp : ros_grasps)
    {
      rect.x = single_grasp.max_suction_surface_length;
      rect.y = single_grasp.max_suction_surface_width;
      setMarker(marker, single_grasp.pose.pose, rect, single_grasp.pose.header.frame_id, counter);
      publishTf(single_grasp.pose.pose, single_grasp.pose.header.frame_id, "grasp_" + std::to_string(counter));
      markers_grasps_.markers.push_back(marker);
      counter++;
    }
    grasp_marker_pub_.publish(markers_grasps_);
  }
}

void Visualization::visualizeDetectedBoxes(const std::vector<rc_pick_client::Item>& ros_boxitems)
{
  deleteBoxItemMarkers();
  if (!ros_boxitems.empty())
  {
    int counter = 0;
    visualization_msgs::Marker marker;
    marker.type = marker.CUBE;
    for (auto& item : ros_boxitems)
    {
      setMarker(marker, item.pose.pose, item.rectangle, item.pose.header.frame_id, counter);
      publishTf(item.pose.pose, item.pose.header.frame_id, "boxitem_" + std::to_string(counter));
      markers_boxes_.markers.push_back(marker);
      counter++;
    }
    box_marker_pub_.publish(markers_boxes_);
  }
}

void Visualization::setMarker(visualization_msgs::Marker& marker, const geometry_msgs::Pose& item_pose,
                              const rc_pick_client::Rectangle& rectangle, std::string frame_id, int marker_id)
{
  marker.scale.z = 0.001;
  marker.color.r = 0.800f;
  marker.color.g = 0.20f;
  marker.color.b = 0.0f;
  marker.color.a = 0.80;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = marker_id;
  marker.header.frame_id = frame_id;
  marker.pose = item_pose;
  marker.scale.x = rectangle.x;
  marker.scale.y = rectangle.y;
}

void Visualization::publishTf(const geometry_msgs::Pose& ros_pose, std::string frame_id, std::string id)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(ros_pose.position.x, ros_pose.position.y, ros_pose.position.z));
  transform.setRotation(
      tf::Quaternion(ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w));
  br_.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), frame_id, std::string(nh_.getNamespace() + "/" + id)));
}

}  // namespace pick_visualization
