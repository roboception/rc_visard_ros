#include "rest_hand_eye_calibration_client.h"


namespace //anonymous
{
  
}//anonymous ns


CalibrationWrapper::CalibrationWrapper(std::string name, std::string ip_addr)
  : nh_ {name}
{
  advertiseServices();
}

///Perform the calibration. Does not save
bool CalibrationWrapper::calibSrv(rc_hand_eye_calibration_client::CalibrationRequest &,
                                  rc_hand_eye_calibration_client::CalibrationResponse &response)
{
  return true;
}

///store hand eye calibration to standard file /opt/rc/etc/camera_calib.txt
bool CalibrationWrapper::saveSrv(std_srvs::TriggerRequest &,
                                 std_srvs::TriggerResponse &response)
{

  return true;
}


///Delete all stored data
bool CalibrationWrapper::resetSrv(std_srvs::TriggerRequest &,
                                  std_srvs::TriggerResponse &response)
{
  return true;
}


///remove calibration
bool CalibrationWrapper::removeSrv(std_srvs::TriggerRequest &,
                                   std_srvs::TriggerResponse &response)
{
  return true;
}


bool CalibrationWrapper::setSlotSrv(rc_hand_eye_calibration_client::SetCalibrationPoseRequest &request,
                                    rc_hand_eye_calibration_client::SetCalibrationPoseResponse &response)
{
  
  return true;
}

bool CalibrationWrapper::getCalibResultSrv(rc_hand_eye_calibration_client::CalibrationRequest &,
                                           rc_hand_eye_calibration_client::CalibrationResponse &response)
{

  return true;
}



void CalibrationWrapper::advertiseServices()
{
  using CW = CalibrationWrapper;
  //Save pose and image/grid pair for later calibration
  srv_set_slot_ = nh_.advertiseService("set_pose", &CW::setSlotSrv, this);
  //Save calibration to disk
  srv_save_ = nh_.advertiseService("save_calibration", &CW::saveSrv, this);
  //Compute and return calibration
  srv_calibrate_ = nh_.advertiseService("calibrate", &CW::calibSrv, this);
  //Get result (but don't compute)
  srv_get_result_ = nh_.advertiseService("get_calibration", &CW::getCalibResultSrv, this);
  //Delete all slots
  srv_reset_ = nh_.advertiseService("reset_calibration", &CW::resetSrv, this);
  // remove calibration
  srv_remove_ = nh_.advertiseService("remove_calibration", &CW::removeSrv, this);
}
