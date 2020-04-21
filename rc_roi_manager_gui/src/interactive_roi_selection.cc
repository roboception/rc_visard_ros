/*
 * Copyright (c) 2019 Roboception GmbH
 *
 * Author: Carlos Xavier Garcia Briones
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
#include "interactive_roi_selection.h"

#include <rc_pick_client/SetRegionOfInterest.h>
#include <rc_pick_client/GetRegionsOfInterest.h>

namespace rc_roi_manager_gui
{
InteractiveRoiSelection::InteractiveRoiSelection()
{
  nh_ = std::make_shared<ros::NodeHandle>();
  server_.reset(new interactive_markers::InteractiveMarkerServer("rc_roi_manager_gui", "", true));
  ros::Duration(0.1).sleep();
}

InteractiveRoiSelection::~InteractiveRoiSelection()
{
  ROS_INFO("Disconnecting the interactive region of interest server..");
  server_.reset();
}

void InteractiveRoiSelection::computeVectorRotation(const tf::Vector3& v, const tf::Quaternion& q, tf::Vector3& rot_v)
{
  tf::Vector3 u(q.x(), q.y(), q.z());
  double w = q.w();
  rot_v = 2.0f * u.dot(v) * u + (w * w - u.dot(u)) * v + 2.0f * w * u.cross(v);
}

void InteractiveRoiSelection::processRoiPoseFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_DEBUG_STREAM(feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                                         << feedback->pose.position.y << ", " << feedback->pose.position.z);

  visualization_msgs::InteractiveMarker corner_int_marker;
  server_->get("corner_0", corner_int_marker);

  geometry_msgs::Pose corner_pose;

  if (corner_int_marker.pose.orientation.w == 0)
    corner_int_marker.pose.orientation.w = 1;

  // Find out if a rotation has taken place
  double orientation_change = (feedback->pose.orientation.x + feedback->pose.orientation.y +
                               feedback->pose.orientation.z + feedback->pose.orientation.w) -
                              (corner_int_marker.pose.orientation.x + corner_int_marker.pose.orientation.y +
                               corner_int_marker.pose.orientation.z + corner_int_marker.pose.orientation.w);
  if (orientation_change < 0)
    orientation_change *= -1;
  bool center_has_rotated = (orientation_change > 0.001);

  corner_pose.orientation = feedback->pose.orientation;
  // Calculate new corner position due to rotation
  if (center_has_rotated)
  {
    tf::Quaternion q_center, q_corner;
    tf::quaternionMsgToTF(corner_int_marker.pose.orientation, q_corner);
    tf::quaternionMsgToTF(feedback->pose.orientation, q_center);
    // Compute rotation quaternion
    tf::Quaternion rot_quaternion = q_center * q_corner.inverse();

    // Move corner back to its relative position
    tf::Vector3 transf_position;
    tf::Vector3 translation_corner_to_center(corner_int_marker.pose.position.x - feedback->pose.position.x,
                                             corner_int_marker.pose.position.y - feedback->pose.position.y,
                                             corner_int_marker.pose.position.z - feedback->pose.position.z);
    computeVectorRotation(translation_corner_to_center, rot_quaternion, transf_position);
    corner_pose.position.x = transf_position.x() + feedback->pose.position.x;
    corner_pose.position.y = transf_position.y() + feedback->pose.position.y;
    corner_pose.position.z = transf_position.z() + feedback->pose.position.z;

    ROS_DEBUG_STREAM("center quaternion: " << q_center.x() << "," << q_center.y() << "," << q_center.z() << ","
                                           << q_center.w());
    ROS_DEBUG_STREAM("corner quaternion: " << q_corner.x() << "," << q_corner.y() << "," << q_corner.z() << ","
                                           << q_corner.w());
    ROS_DEBUG_STREAM("rotation quaternion: " << rot_quaternion.x() << "," << rot_quaternion.y() << ","
                                             << rot_quaternion.z() << "," << rot_quaternion.w());
  }
  else
  {
    // Corner position has to be updated
    corner_pose.position.x = corner_int_marker.pose.position.x + feedback->pose.position.x - center_position_.x();
    corner_pose.position.y = corner_int_marker.pose.position.y + feedback->pose.position.y - center_position_.y();
    corner_pose.position.z = corner_int_marker.pose.position.z + feedback->pose.position.z - center_position_.z();
  }
  server_->setPose("corner_0", corner_pose);
  server_->applyChanges();

  // Saving the present center position to compute the future displacement of the center
  tf::quaternionMsgToTF(corner_pose.orientation, center_orientation_);
  tf::pointMsgToTF(feedback->pose.position, center_position_);

  ROS_DEBUG_STREAM("corner_0"
                   << " is now at " << corner_pose.position.x << ", " << corner_pose.position.y << ", "
                   << corner_pose.position.z);
}

void InteractiveRoiSelection::processSphereSizeFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  tf::Vector3 position;
  tf::pointMsgToTF(feedback->pose.position, position);
  float radius = (position - center_position_).length();
  dimensions_ = tf::Vector3(radius, radius, radius);
  updateCenterMarker();
}

void InteractiveRoiSelection::updateCenterMarker()
{
  server_->erase("center");
  makeInteractiveMarker("center", "Pose", center_position_, true, dimensions_, center_orientation_);
  server_->applyChanges();
}

void InteractiveRoiSelection::processRoiSizeFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_DEBUG_STREAM(feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                                         << feedback->pose.position.y << ", " << feedback->pose.position.z);
  tf::Vector3 present_dimensions(feedback->pose.position.x - center_position_.x(),
                                 feedback->pose.position.y - center_position_.y(),
                                 feedback->pose.position.z - center_position_.z());

  // Compute rotated dimensions of the box
  computeVectorRotation(present_dimensions, center_orientation_.inverse(), dimensions_);
  dimensions_ = dimensions_.absolute();

  updateCenterMarker();
}

// Generate marker box
visualization_msgs::Marker InteractiveRoiSelection::makeMarker(tf::Vector3 box_dimensions, bool is_center)
{
  visualization_msgs::Marker marker;

  marker.type = interactive_roi_.primitive.type;

  if (is_center)
  {
    float center_scale = 2;
    marker.scale.x = center_scale * box_dimensions.x();
    marker.scale.y = center_scale * box_dimensions.y();
    marker.scale.z = center_scale * box_dimensions.z();
    marker.color.r = 150 / 255.0;
    marker.color.g = 104 / 2;
    marker.color.b = 251 / 255.0;
    marker.color.a = 0.3;
  }
  else
  {
    marker.scale.x = box_dimensions.x();
    marker.scale.y = box_dimensions.y();
    marker.scale.z = box_dimensions.z();
    marker.color.r = 255 / 255.0;
    marker.color.g = 204 / 2;
    marker.color.b = 0 / 255.0;
    marker.color.a = 1.0;
  }

  return marker;
}

void InteractiveRoiSelection::makeSphereControls(visualization_msgs::InteractiveMarker& interactive_marker,
                                                 bool is_center)
{
  // Add controls for axis rotation and positioning
  if (is_center)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);

    control.name = "move_y";
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);

    control.name = "move_z";
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  }
  else
  {
    // Rotate corner marker orientation 45 degrees on the y-Axis.
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    control.orientation.w = 0.923879468105164;
    control.orientation.x = 0;
    control.orientation.y = 0.382683587855188;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  }
}

// Generate interactive markers
void InteractiveRoiSelection::makeBoxControls(visualization_msgs::InteractiveMarker& interactive_marker, bool is_center)
{
  // Add controls for axis rotation and positioning

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);
  if (is_center)
  {
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  control.name = "move_y";
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);
  if (is_center)
  {
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  control.name = "move_z";
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(control);

  if (is_center)
  {
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker.controls.push_back(control);
  }
}

// Generate interactive markers
void InteractiveRoiSelection::makeInteractiveMarker(std::string int_marker_name, std::string int_marker_description,
                                                    const tf::Vector3& position, bool is_center,
                                                    tf::Vector3 box_dimensions, tf::Quaternion box_orientation)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = interactive_roi_.pose.header.frame_id;
  tf::pointTFToMsg(position, int_marker.pose.position);
  tf::quaternionTFToMsg(box_orientation, int_marker.pose.orientation);
  int_marker.scale = 1.7 * box_dimensions[box_dimensions.maxAxis()];

  int_marker.name = int_marker_name;
  int_marker.description = int_marker_description;

  // insert a marker
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = is_center;
  control.markers.push_back(makeMarker(box_dimensions, is_center));
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  int_marker.controls.push_back(control);
  int_marker.controls.back();

  if (interactive_roi_.primitive.type == shape_msgs::SolidPrimitive::Type::BOX)
  {
    makeBoxControls(int_marker, is_center);
    server_->insert(int_marker);
    if (is_center)
    {
      server_->setCallback(int_marker.name, boost::bind(&InteractiveRoiSelection::processRoiPoseFeedback, this, _1));
    }
    else
    {
      server_->setCallback(int_marker.name, boost::bind(&InteractiveRoiSelection::processRoiSizeFeedback, this, _1));
    }
  }
  else if (interactive_roi_.primitive.type == shape_msgs::SolidPrimitive::Type::SPHERE)
  {
    makeSphereControls(int_marker, is_center);
    server_->insert(int_marker);
    if (is_center)
    {
      server_->setCallback(int_marker.name, boost::bind(&InteractiveRoiSelection::processRoiPoseFeedback, this, _1));
    }
    else
    {
      server_->setCallback(int_marker.name, boost::bind(&InteractiveRoiSelection::processSphereSizeFeedback, this, _1));
    }
  }
  else
  {
    ROS_FATAL("The provided shape is currently not supported.");
  }
}

bool InteractiveRoiSelection::setInteractiveRoi(const rc_pick_client::RegionOfInterest& roi)
{
  interactive_roi_ = roi;
  tf::quaternionMsgToTF(interactive_roi_.pose.pose.orientation, center_orientation_);
  tf::pointMsgToTF(interactive_roi_.pose.pose.position, center_position_);
  tf::Vector3 corner_position_rot;
  if (interactive_roi_.primitive.type == shape_msgs::SolidPrimitive::Type::BOX)
  {
    dimensions_ =
        tf::Vector3(interactive_roi_.primitive.dimensions[0] / 2, interactive_roi_.primitive.dimensions[1] / 2,
                    interactive_roi_.primitive.dimensions[2] / 2);

    // Rotate corner marker position with the roi orientation.
    computeVectorRotation(dimensions_, center_orientation_, corner_position_rot);
  }
  else if (interactive_roi_.primitive.type == shape_msgs::SolidPrimitive::Type::SPHERE)
  {
    dimensions_ = tf::Vector3(interactive_roi_.primitive.dimensions[0], interactive_roi_.primitive.dimensions[0],
                              interactive_roi_.primitive.dimensions[0]);

    // Rotate corner marker position 45 degrees on the y-Axis.
    corner_position_rot = tf::Vector3(dimensions_.x(), 0, 0);
    computeVectorRotation(corner_position_rot, tf::Quaternion(0, 0.382683587855188, 0, 0.923879468105164),
                          corner_position_rot);
  }
  else
  {
    ROS_ERROR_STREAM("An unsupported primitive was used.");
    return false;
  }

  makeInteractiveMarker("center", "Pose", center_position_, true, dimensions_, center_orientation_);
  makeInteractiveMarker("corner_0", "Size", center_position_ + corner_position_rot, false,
                        tf::Vector3(0.02, 0.02, 0.02), center_orientation_);

  server_->applyChanges();
  return true;
}

bool InteractiveRoiSelection::getInteractiveRoi(rc_pick_client::RegionOfInterest& roi)
{
  if (interactive_roi_.primitive.type == shape_msgs::SolidPrimitive::Type::BOX)
  {
    dimensions_ = dimensions_ * 2;
  }
  tf::pointTFToMsg(center_position_, interactive_roi_.pose.pose.position);
  tf::quaternionTFToMsg(center_orientation_, interactive_roi_.pose.pose.orientation);

  interactive_roi_.primitive.dimensions[0] = dimensions_.x();
  interactive_roi_.primitive.dimensions[1] = dimensions_.y();
  interactive_roi_.primitive.dimensions[2] = dimensions_.z();
  roi = interactive_roi_;
  server_->clear();
  server_->applyChanges();
  return true;
}

}  // namespace rc_roi_manager_gui
