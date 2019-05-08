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

#ifndef RC_ITEMPICK_CLIENT_PUBLISH_ITEMPICK_VISULIZATION_H
#define RC_ITEMPICK_CLIENT_PUBLISH_ITEMPICK_VISULIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rc_tagdetect_client/DetectedTags.h>

namespace rc_tagdetect_client
{

class Visualization
{
  public:
    explicit Visualization(const ros::NodeHandle &nh);

    ~Visualization();

    /*
     * Remove previously published tag markers.
     */
    void deleteMarkers();

    /*
     * Publish tags as tf and marker
     */
    void publishTags(const std::vector<rc_tagdetect_client::DetectedTag> &tags);

  private:
    geometry_msgs::TransformStamped createTf(const rc_tagdetect_client::DetectedTag &tag) const;
    visualization_msgs::Marker createMarker(const rc_tagdetect_client::DetectedTag &tag) const;

    ros::NodeHandle nh_;
    std::string tf_prefix_;
    ros::Publisher tag_marker_pub_;
    visualization_msgs::MarkerArray markers_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;
};
}

#endif //RC_ITEMPICK_CLIENT_PUBLISH_ITEMPICK_VISULIZATION_H
