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

#include "visualizer.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rc_silhouettematch_client
{
static std::string make_tf_prefix(const ros::NodeHandle& nh)
{
  std::string ns = ros::this_node::getNamespace();
  if (ns.empty())
  {
    return {};
  }
  if (ns[0] == '/')
  {
    ns = ns.substr(1);
  }
  if (ns.empty())
  {
    return {};
  }
  else
  {
    if (ns[0] == '/')
    {
      ns = ns.substr(1);
    }
    std::replace(ns.begin(), ns.end(), '/', '_');
    return ns + '_';
  }
}

Visualizer::Visualizer(ros::NodeHandle& nh) : tf_prefix_(make_tf_prefix(nh))
{
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("silhouettematch_markers", 10);
}

Visualizer::~Visualizer()
{
  try
  {
    deleteMarkers();
  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Exception during destruction of Visualization: %s", ex.what());
  }
  catch (...)
  {
    ROS_FATAL("Exception during destruction of Visualization");
  }
}

void Visualizer::deleteMarkers()
{
  for (auto &i : markers_.markers)
  {
    i.action = visualization_msgs::Marker::DELETE;
  }
  marker_pub_.publish(markers_);
  markers_.markers.clear();
}

void Visualizer::visBasePlane(const EstimatedPlane& plane, const ros::Time timestamp)
{
  deleteMarkers();
  // visualize base plane as flat disk
  visualization_msgs::Marker marker;
  marker.header.stamp = timestamp;
  marker.header.frame_id = tf_prefix_ + plane.pose_frame;
  marker.ns = tf_prefix_ + "base_plane";
  marker.id = 0;
  marker.lifetime = ros::Duration();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 0.001;
  marker.pose.position.x = - plane.normal.x * plane.distance;
  marker.pose.position.y = - plane.normal.y * plane.distance;
  marker.pose.position.z = - plane.normal.z * plane.distance;
  // quaternion yielding double the desired rotation (if normal is normalized):
  // q.w = dot(zaxis, normal), q.xyz = cross(zaxis, normal)
  // add quaternion with zero rotation (xyz=0, w=1) to get half the rotation from above
  // and normalize again
  tf2::Quaternion q(-plane.normal.y, plane.normal.x, 0, plane.normal.z + 1);
  q.normalize();
  tf2::convert(q, marker.pose.orientation);
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  markers_.markers.push_back(marker);
  marker_pub_.publish(markers_);
}

inline geometry_msgs::Transform toRosTf(const geometry_msgs::Pose& pose)
{
  geometry_msgs::Transform msg;
  msg.translation.x = pose.position.x;
  msg.translation.y = pose.position.y;
  msg.translation.z = pose.position.z;
  msg.rotation.x = pose.orientation.x;
  msg.rotation.y = pose.orientation.y;
  msg.rotation.z = pose.orientation.z;
  msg.rotation.w = pose.orientation.w;
  return msg;
}

void Visualizer::visInstances(const std::vector<Instance>& instances)
{
  for (const auto& instance : instances)
  {
    geometry_msgs::TransformStamped tf;
    tf.transform = toRosTf(instance.pose);
    tf.header.frame_id = tf_prefix_ + instance.pose_frame;
    tf.child_frame_id = tf_prefix_ + instance.object_id + "_" + instance.id;
    tf.header.stamp = instance.timestamp;
    tfb_.sendTransform(tf);
  }
}

}  // namespace rc_silhouettematch_client
