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

namespace rc_tagdetect_client
{

static std::string make_tf_prefix(const ros::NodeHandle &nh)
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

Visualization::Visualization(const ros::NodeHandle &nh) :
        nh_(nh),
        tf_prefix_(make_tf_prefix(nh_))
{
  tag_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tag_markers", 10);
}

Visualization::~Visualization()
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

void Visualization::deleteMarkers()
{
  for (auto &i : markers_.markers)
  {
    i.action = visualization_msgs::Marker::DELETE;
  }
  tag_marker_pub_.publish(markers_);
  markers_.markers.clear();
}

void Visualization::publishTags(
        const std::vector<rc_tagdetect_client::DetectedTag> &tags)
{
  deleteMarkers();
  for (const auto &t : tags)
  {
    transform_broadcaster_.sendTransform(createTf(t));
    markers_.markers.push_back(createMarker(t));
  }
  tag_marker_pub_.publish(markers_);
}

visualization_msgs::Marker
Visualization::createMarker(const rc_tagdetect_client::DetectedTag &tag) const
{
  visualization_msgs::Marker marker;
  marker.header.stamp = tag.header.stamp;
  marker.header.frame_id = tf_prefix_ + tag.tag.id + '_' + tag.instance_id;
  marker.id = static_cast<int32_t>(std::stol(tag.instance_id) %
                                   std::numeric_limits<int32_t>::max());
  marker.lifetime = ros::Duration();
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = tag.tag.size;
  marker.scale.y = tag.tag.size;
  marker.scale.z = 0.001;
  marker.pose.orientation.w = 1;
  marker.pose.position.x = tag.tag.size / 2;
  marker.pose.position.y = tag.tag.size / 2;
  marker.pose.position.z = 0.001 / 2;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  return marker;
}

geometry_msgs::TransformStamped
Visualization::createTf(const rc_tagdetect_client::DetectedTag &tag) const
{
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = tf_prefix_ + tag.header.frame_id;
  tf.child_frame_id = tf_prefix_ + tag.tag.id + '_' + tag.instance_id;
  tf.header.stamp = tag.header.stamp;
  tf.transform.translation.x = tag.pose.pose.position.x;
  tf.transform.translation.y = tag.pose.pose.position.y;
  tf.transform.translation.z = tag.pose.pose.position.z;
  tf.transform.rotation.x = tag.pose.pose.orientation.x;
  tf.transform.rotation.y = tag.pose.pose.orientation.y;
  tf.transform.rotation.z = tag.pose.pose.orientation.z;
  tf.transform.rotation.w = tag.pose.pose.orientation.w;
  return tf;
}


}
