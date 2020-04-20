/*
 * Copyright (c) 2018-2020 Roboception GmbH
 *
 * Author: Felix Endres
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

#ifndef RC_HAND_EYE_CALIBRATION_CLIENT_H
#define RC_HAND_EYE_CALIBRATION_CLIENT_H

#include <rc_hand_eye_calibration_client/SetCalibrationPose.h>
#include <rc_hand_eye_calibration_client/Calibration.h>
#include <rc_hand_eye_calibration_client/hand_eye_calibrationConfig.h>
#include <rc_hand_eye_calibration_client/Trigger.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <memory>

#include "rest_helper.h"

namespace rc_hand_eye_calibration_client
{

class HandEyeCalibClient
{
  public:

    HandEyeCalibClient(std::string host, ros::NodeHandle nh);

  private:

    template <typename Request, typename Response>
    bool callService(const std::string& name, const Request& req, Response& res);

    /**Perform the calibration and return its result.
     * Does not save persistently. Call saveSrv for that.
     */
    bool calibSrv(rc_hand_eye_calibration_client::CalibrationRequest &req,
                  rc_hand_eye_calibration_client::CalibrationResponse &res);

    ///Service call to get the result. Also returned directly via calibrate
    bool getCalibResultSrv(rc_hand_eye_calibration_client::CalibrationRequest &req,
                           rc_hand_eye_calibration_client::CalibrationResponse &res);

    ///Wraps the above getCalibResultSrv. Called by calib_request_timer_ for the side effect of
    ///updating and broadcasting the cached calibration via tf.
    void requestCalibration(const ros::SteadyTimerEvent&);

    ///store hand eye calibration on sensor
    bool saveSrv(rc_hand_eye_calibration_client::TriggerRequest &req,
                 rc_hand_eye_calibration_client::TriggerResponse &res);

    ///Delete all stored data, e.g., to start over.
    bool resetSrv(rc_hand_eye_calibration_client::TriggerRequest &req,
                  rc_hand_eye_calibration_client::TriggerResponse &res);

    ///Remove calibration so sensor reports as uncalibrated
    bool removeSrv(rc_hand_eye_calibration_client::TriggerRequest &req,
                  rc_hand_eye_calibration_client::TriggerResponse &res);

    ///Save given pose and the pose of the grid indexed by the given request.slot.
    ///If slot exists, it is overwritten.
    bool setSlotSrv(rc_hand_eye_calibration_client::SetCalibrationPoseRequest &req,
                    rc_hand_eye_calibration_client::SetCalibrationPoseResponse &res);

    ///Advertises the services in the namespace of nh_
    void advertiseServices();

    void initConfiguration();

    ///Initialize timers to periodically publish and/or request the calibration
    ///Depends on parameters: calibration_publication_period and calibration_request_period
    void initTimers();

    void dynamicReconfigureCb(rc_hand_eye_calibration_client::hand_eye_calibrationConfig &config, uint32_t);

    void sendCachedCalibration(const ros::SteadyTimerEvent& = ros::SteadyTimerEvent());

    ///update and broadcast the internally cached calibration result.
    void calibResponseHandler(CalibrationResponse &res);

    ///Copy resp into current_calibration_
    void updateCalibrationCache(const rc_hand_eye_calibration_client::CalibrationResponse& res);

    //ROS Stuff
    ros::NodeHandle nh_;
    ros::ServiceServer srv_set_slot_;
    ros::ServiceServer srv_save_;
    ros::ServiceServer srv_calibrate_;
    ros::ServiceServer srv_reset_;
    ros::ServiceServer srv_remove_;
    ros::ServiceServer srv_publish_transform_;
    ros::ServiceServer srv_get_result_;
    ///To be used for on-change sending only
    tf2_ros::StaticTransformBroadcaster static_tf2_broadcaster_;
    ///To be used for periodic sending
    tf2_ros::TransformBroadcaster dynamic_tf2_broadcaster_;
    geometry_msgs::TransformStamped current_calibration_;//initialized all-zero
    ///Default Frame Ids to be used for the tf broadcast
    std::string camera_frame_id_ = "camera";
    ///Which one is used depends on the value of "robot_mounted" in the calibration response
    std::string endeff_frame_id_ = "end_effector";
    std::string base_frame_id_ = "base_link";
    ///Whether to send the transformation periodically instead of as a static transformation
    ///Zero or negative value: static tf (via latched topic, sends updates on new calibration response)
    double calib_publish_period_ = 0.0;//seconds
    ros::SteadyTimer calib_publish_timer_;
    ///If non-negative: Periodically update the calibration by requesting it.
    ///If zero: Request only once
    double calib_request_period_ = 0.0;//seconds
    ros::SteadyTimer calib_request_timer_;
    std::unique_ptr<dynamic_reconfigure::Server<rc_hand_eye_calibration_client::hand_eye_calibrationConfig> > server_;

    // REST stuff
    rc_rest_api::RestHelper rest_helper_;

};

}  // namespace rc_hand_eye_calibration_client

#endif //RC_HAND_EYE_CALIBRATION_CLIENT_H
