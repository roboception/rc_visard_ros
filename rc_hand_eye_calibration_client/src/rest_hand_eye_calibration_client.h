#ifndef REST_HAND_EYE_CALIBRATION_CLIENT_H
#define REST_HAND_EYE_CALIBRATION_CLIENT_H

#include <rc_hand_eye_calibration_client/SetCalibrationPose.h>
#include <rc_hand_eye_calibration_client/Calibration.h>
#include <rc_hand_eye_calibration_client/hand_eye_calibrationConfig.h>

#include <std_srvs/Trigger.h>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

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

    void dynamicReconfigureCb(rc_hand_eye_calibration_client::hand_eye_calibrationConfig &config, uint32_t);

    //ROS Stuff
    ros::NodeHandle nh_; 
    ros::ServiceServer srv_set_slot_;
    ros::ServiceServer srv_save_;
    ros::ServiceServer srv_calibrate_;
    ros::ServiceServer srv_reset_;
    ros::ServiceServer srv_remove_;
    ros::ServiceServer srv_publish_transform_;
    ros::ServiceServer srv_get_result_;
    std::unique_ptr<dynamic_reconfigure::Server<rc_hand_eye_calibration_client::hand_eye_calibrationConfig> > server_;

    // REST stuff
    std::string host_, servicesUrl_, paramsUrl_;
    int timeoutCurl_; // ms

};
#endif
