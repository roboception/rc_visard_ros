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
#include "visulization.h"

namespace pick_visualization
{

Visualization::Visualization(const ros::NodeHandle &nh) : nh_(nh)
{
  grasp_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("grasp", 1);
  lc_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("lc", 1);

};

Visualization::~Visualization()
{
  deleteGraspMarkers();
  deleteLoadCarrierMarkers();
}

void Visualization::constructLoadCarrier(visualization_msgs::MarkerArray &marker_array,
                                              const rc_pick_client::LoadCarrier &lc, const int &lc_no)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = lc.pose.header.frame_id;
  marker.type = marker.CUBE;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.3;
  tf2::Transform bin_transform(tf2::Quaternion(lc.pose.pose.orientation.x, lc.pose.pose.orientation.y,
                                               lc.pose.pose.orientation.z,
                                               lc.pose.pose.orientation.w),
                               tf2::Vector3(lc.pose.pose.position.x, lc.pose.pose.position.y,
                                            lc.pose.pose.position.z));

  double rim_thickness_x = (lc.outer_dimensions.x - lc.inner_dimensions.x) / 2;
  double dist_from_center_x = (lc.outer_dimensions.x - rim_thickness_x) / 2;
  double rim_thickness_y = (lc.outer_dimensions.y - lc.inner_dimensions.y) / 2;
  double dist_from_center_y = (lc.outer_dimensions.y - rim_thickness_y) / 2;
  double bottom_thickness = lc.outer_dimensions.z - lc.inner_dimensions.z;
  tf2::Vector3 pose(0, 0, -(lc.outer_dimensions.z - bottom_thickness) / 2);

  //calculate bottom
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

  //calculate longer sides
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

  //calculate shorter side
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
  for (auto &i : markers_lcs.markers)
  {
    i.action = visualization_msgs::Marker::DELETE;
  }
  lc_marker_pub.publish(markers_lcs);
  markers_lcs.markers.clear();

}

void Visualization::deleteGraspMarkers()
{
  for (auto &i : markers_grasps.markers)
  {
    i.action = visualization_msgs::Marker::DELETE;
  }
  grasp_marker_pub.publish(markers_grasps);
  markers_grasps.markers.clear();
}

void Visualization::publishLCTf(const std::vector<rc_pick_client::LoadCarrier> &ros_lcs)
{
  tf::Transform transform;
  for (unsigned int i = 0; i < ros_lcs.size(); i++)
  {
    transform.setOrigin(tf::Vector3(ros_lcs[i].pose.pose.position.x, ros_lcs[i].pose.pose.position.y,
                                    ros_lcs[i].pose.pose.position.z));
    transform.setRotation(tf::Quaternion(ros_lcs[i].pose.pose.orientation.x, ros_lcs[i].pose.pose.orientation.y,
                                         ros_lcs[i].pose.pose.orientation.z, ros_lcs[i].pose.pose.orientation.w));
    br.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), ros_lcs[i].pose.header.frame_id,
                                 std::string(nh_.getNamespace() + "/lc_" + std::to_string(i))));
  }
}

void Visualization::publishLCMarkers(const std::vector<rc_pick_client::LoadCarrier> &ros_lcs)
{
  for (unsigned int i = 0; i < ros_lcs.size(); i++) constructLoadCarrier(markers_lcs, ros_lcs[i], i);

  lc_marker_pub.publish(markers_lcs);
}


void Visualization::visualizeLoadCarriers(const std::vector<rc_pick_client::LoadCarrier> &ros_lcs)
{
  deleteLoadCarrierMarkers();
  if (!ros_lcs.empty())
  {
    publishLCMarkers(ros_lcs);
    publishLCTf(ros_lcs);
  }
}

void Visualization::publishGraspTf(const std::vector<rc_pick_client::SuctionGrasp> &ros_grasps)
{
  tf::Transform transform;
  for (unsigned int i = 0; i < ros_grasps.size(); i++)
  {
    transform.setOrigin(tf::Vector3(ros_grasps[i].pose.pose.position.x, ros_grasps[i].pose.pose.position.y,
                                    ros_grasps[i].pose.pose.position.z));

    transform.setRotation(tf::Quaternion(ros_grasps[i].pose.pose.orientation.x, ros_grasps[i].pose.pose.orientation.y,
                                         ros_grasps[i].pose.pose.orientation.z, ros_grasps[i].pose.pose.orientation.w));
    br.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), ros_grasps[i].pose.header.frame_id,
                                 std::string(nh_.getNamespace() + "/grasp_" + std::to_string(i))));
  }
}

void Visualization::publishGraspMarkers(const std::vector<rc_pick_client::SuctionGrasp> &ros_grasps)
{
  visualization_msgs::Marker marker;
  marker.type = marker.SPHERE;
  marker.scale.z = 0.001;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.action = visualization_msgs::Marker::ADD;

  for (unsigned int i = 0; i < ros_grasps.size(); i++)
  {
    marker.id = i;
    marker.header.frame_id = ros_grasps[i].pose.header.frame_id;
    marker.pose = ros_grasps[i].pose.pose;
    marker.scale.y = ros_grasps[i].max_suction_surface_width;
    marker.scale.x = ros_grasps[i].max_suction_surface_length;
    markers_grasps.markers.push_back(marker);
  }
  grasp_marker_pub.publish(markers_grasps);
}


void Visualization::visualizeGrasps(const std::vector<rc_pick_client::SuctionGrasp> &ros_grasps)
{
  deleteGraspMarkers();
  if (!ros_grasps.empty())
  {
    publishGraspMarkers(ros_grasps);
    publishGraspTf(ros_grasps);

  }
}

}
