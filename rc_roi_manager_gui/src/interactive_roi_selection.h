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

#ifndef INTERACTIVE_ROI_SELECTION_H
#define INTERACTIVE_ROI_SELECTION_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <interactive_markers/interactive_marker_server.h>
#include <rc_pick_client/RegionOfInterest.h>

namespace rc_roi_manager_gui
{
class InteractiveRoiSelection
{
public:
  /**
   * @brief Constructor
   */
  InteractiveRoiSelection();

  virtual ~InteractiveRoiSelection();

  /**
   * @brief Sets a new region of interest in the server
   * @param roi Region of interest
   * @return
   */
  bool setInteractiveRoi(const rc_pick_client::RegionOfInterest& roi);

  /**
   * @brief Provides the region of interest from the server
   * @param roi Region of interest
   * @return
   */
  bool getInteractiveRoi(rc_pick_client::RegionOfInterest& roi);

private:
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  tf::Vector3 center_position_;
  tf::Quaternion center_orientation_;
  tf::Vector3 dimensions_;
  rc_pick_client::RegionOfInterest interactive_roi_;
  std::shared_ptr<ros::NodeHandle> nh_;

  /**
   * @brief Compute the rotation of a vector by a quaternion
   * @param v Vector to be rotated
   * @param q Quaternion that provides the wanted rotation
   * @param rot_v Rotated vector
   */
  void computeVectorRotation(const tf::Vector3& v, const tf::Quaternion& q, tf::Vector3& rot_v);

  /**
   * @brief Process changes on the position and rotation of the region of interest
   * @param feedback center marker with updates done in rviz
   */
  void processRoiPoseFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Process changes on the region of interest's size
   * @param feedback corner marker with updates done in rviz
   */
  void processRoiSizeFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Process changes on the region of interest's size
   * @param feedback corner marker with updates done in rviz
   */
  void processSphereSizeFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * @brief Update the center interactive marker in rviz
   */
  void updateCenterMarker();

  /**
   * @brief Generate a marker
   * @param box_dimensions Dimensions of the marker
   * @param is_center Set if the marker to create is the center of the region of interest
   * @return Generated marker
   */
  visualization_msgs::Marker makeMarker(tf::Vector3 box_dimensions, bool is_center);

  /**
   * @brief Generate controls of a box interactive marker
   * @param interactive_marker Interactive marker
   * @param is_center Set if the marker to create is the center of the region of interest
   */
  void makeBoxControls(visualization_msgs::InteractiveMarker& interactive_marker, bool is_center);

  /**
   * @brief Generate controls of a sphere interactive marker
   * @param interactive_marker Interactive marker
   * @param is_center Set if the marker to create is the center of the region of interest
   */
  void makeSphereControls(visualization_msgs::InteractiveMarker& interactive_marker, bool is_center);

  /**
   * @brief Generate an interactive marker
   * @param int_marker_name Interactive marker's name
   * @param int_marker_description Interactive marker's description
   * @param position Position of the interactive marker
   * @param is_center Set if the marker to create is the center of the region of interest
   * @param box_dimensions Dimensions of the interactive marker
   * @param box_orientation Orientation of the interactive marker
   */
  void makeInteractiveMarker(std::string int_marker_name, std::string int_marker_description,
                             const tf::Vector3& position, bool is_center, tf::Vector3 box_dimensions,
                             tf::Quaternion box_orientation);
};
}  // namespace rc_roi_manager_gui
#endif  // INTERACTIVE_ROI_SELECTION_H
