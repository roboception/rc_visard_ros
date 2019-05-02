#ifndef REST_HAND_EYE_CALIBRATION_CLIENT_H
#define REST_HAND_EYE_CALIBRATION_CLIENT_H

#include <rc_hand_eye_calibration_client/SetCalibrationPose.h>
#include <rc_hand_eye_calibration_client/Calibration.h>
#include <rc_hand_eye_calibration_client/hand_eye_calibrationConfig.h>

#include <std_srvs/Trigger.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <memory>


/**
 * TODO
 */
class CalibrationWrapper
{
  public:

    CalibrationWrapper(std::string host, ros::NodeHandle nh);

  private:

    /**Perform the calibration and return its result.
     * Does not save persistently. Call saveSrv for that.
     */
    bool calibSrv(rc_hand_eye_calibration_client::CalibrationRequest &request,
                  rc_hand_eye_calibration_client::CalibrationResponse &response);

    ///Service call to get the result. Also returned directly via calibrate
    bool getCalibResultSrv(rc_hand_eye_calibration_client::CalibrationRequest &request,
                           rc_hand_eye_calibration_client::CalibrationResponse &response);

    ///Wraps the above getCalibResultSrv. Called by calib_request_timer_ for the side effect of
    ///updating and broadcasting the cached calibration via tf.
    void requestCalibration(const ros::SteadyTimerEvent&);

    ///store hand eye calibration on sensor
    bool saveSrv(std_srvs::TriggerRequest &request,
                 std_srvs::TriggerResponse &response);

    ///Delete all stored data, e.g., to start over.
    bool resetSrv(std_srvs::TriggerRequest &request,
                  std_srvs::TriggerResponse &response);

    ///Remove calibration so sensor reports as uncalibrated
    bool removeSrv(std_srvs::TriggerRequest &request,
                  std_srvs::TriggerResponse &response);

    ///Save given pose and the pose of the grid indexed by the given request.slot.
    ///If slot exists, it is overwritten.
    bool setSlotSrv(rc_hand_eye_calibration_client::SetCalibrationPoseRequest &request,
                    rc_hand_eye_calibration_client::SetCalibrationPoseResponse &response);

    ///Advertises the services in the namespace of nh_
    void advertiseServices();

    void initConfiguration();

    ///Initialize timers to periodically publish and/or request the calibration
    ///Depends on parameters: calibration_publication_period and calibration_request_period
    void initTimers();

    void dynamicReconfigureCb(rc_hand_eye_calibration_client::hand_eye_calibrationConfig &config, uint32_t);

    void sendCachedCalibration(const ros::SteadyTimerEvent& = ros::SteadyTimerEvent());

    ///calibSrv and getCalibResultSrv have almost all functionality in common:
    ///Make the RestAPI call \p service_name, parse the json, copy it into \p response,
    ///and update and broadcast the internally cached calibration result. Then return true.
    bool calibResultCommon(const char* service_name,
                           rc_hand_eye_calibration_client::CalibrationResponse &response);

    ///Copy resp into current_calibration_
    void updateCalibrationCache(const rc_hand_eye_calibration_client::CalibrationResponse& resp);

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
    std::string host_, servicesUrl_, paramsUrl_;
    int timeoutCurl_; // ms

};
#endif
