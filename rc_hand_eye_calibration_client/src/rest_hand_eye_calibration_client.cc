#include "rest_hand_eye_calibration_client.h"

#include <ifaddrs.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "json.hpp"
#include <cpr/cpr.h>

using namespace std;
using json = nlohmann::json;


namespace //anonymous
{

string toString(cpr::Response resp)
{
  stringstream s;
  s << "status code: " << resp.status_code << endl
    << "url: " << resp.url << endl
    << "text: " << resp.text << endl
    << "error: " << resp.error.message;
  return s.str();
}

void handleCPRResponse(cpr::Response r)
{
  if (r.status_code != 200)
  {
    throw runtime_error(toString(r));
  }
}

}//anonymous ns


CalibrationWrapper::CalibrationWrapper(std::string host, ros::NodeHandle nh)
  : nh_(nh), host_(host),
    servicesUrl_("http://" + host + "/api/v1/nodes/rc_hand_eye_calibration/services/"),
    paramsUrl_("http://" + host + "/api/v1/nodes/rc_hand_eye_calibration/parameters")
{
  timeoutCurl_ = 2000;
  initConfiguration();
  advertiseServices();
  initTimers();
}

void CalibrationWrapper::initTimers()
{
  if (calib_publish_period_ > 0.0 && //only need the timer at all if there's a valid intervall
  //every calibration request broadcasts: no need for extra publishing if request is done more often
     (calib_request_period_ <= 0.0 || calib_publish_period_ < calib_request_period_))
  {
    ROS_INFO("Broadcasting calibration every %.3f seconds on /tf", calib_publish_period_);
    //the two booleans at the end: *one-shot* is kept default (false), *autostart* is set to false,
    //because we start publishing only when the first calibration result was received
    calib_publish_timer_ = nh_.createSteadyTimer(ros::WallDuration(calib_publish_period_),
                                                 &CalibrationWrapper::sendCachedCalibration, this,
                                                 false, false);
  }

  if (calib_request_period_ >= 0.0)//negative is documented to turn all auto-requesting off
  {
    if (calib_request_period_ == 0.0) //special meaning as documented in README.md
    {
      ROS_INFO("Requesting (and broadcasting) calibration from rc_visard once");
    }
    else
    {
      ROS_INFO("Requesting (and broadcasting) calibration every %.3f seconds from rc_visard",
               calib_request_period_);
      calib_request_timer_ = nh_.createSteadyTimer(ros::WallDuration(calib_request_period_),
                                                   &CalibrationWrapper::requestCalibration, this);
    }
    // request once immediately at startup
    requestCalibration({});
  }
}

bool CalibrationWrapper::saveSrv(std_srvs::TriggerRequest &,
                                 std_srvs::TriggerResponse &response)
{
  cpr::Url url = cpr::Url{ servicesUrl_ + "save_calibration"};
  auto rest_resp = cpr::Put(url, cpr::Timeout{ timeoutCurl_ });
  handleCPRResponse(rest_resp);

  auto json_resp = json::parse(rest_resp.text)["response"];
  response.success = (bool) json_resp["success"];
  response.message = json_resp["message"];
  return true;
}


bool CalibrationWrapper::resetSrv(std_srvs::TriggerRequest &,
                                  std_srvs::TriggerResponse &response)
{
  cpr::Url url = cpr::Url{ servicesUrl_ + "reset_calibration"};
  auto rest_resp = cpr::Put(url, cpr::Timeout{ timeoutCurl_ });
  handleCPRResponse(rest_resp);

  auto json_resp = json::parse(rest_resp.text)["response"];
  response.success = (bool) json_resp["success"];
  response.message = json_resp["message"];
  return true;
}


bool CalibrationWrapper::removeSrv(std_srvs::TriggerRequest &,
                                   std_srvs::TriggerResponse &response)
{
  cpr::Url url = cpr::Url{ servicesUrl_ + "remove_calibration"};
  auto rest_resp = cpr::Put(url, cpr::Timeout{ timeoutCurl_ });
  handleCPRResponse(rest_resp);

  auto json_resp = json::parse(rest_resp.text)["response"];
  response.success = (bool) json_resp["success"];
  response.message = json_resp["message"];

  if (response.success)
  {
    calib_publish_timer_.stop();//does nothing if already stopped
    ROS_INFO("Calibration has been removed, stopped /tf broadcasting.");
  }
  else { ROS_WARN("Failed to remove calibration: %s", response.message.c_str()); }

  return true;
}


bool CalibrationWrapper::setSlotSrv(rc_hand_eye_calibration_client::SetCalibrationPoseRequest &request,
                                    rc_hand_eye_calibration_client::SetCalibrationPoseResponse &response)
{
  // convert ros pose to json obj
  json js_pos, js_ori, js_pose;
  js_pos["x"] = request.pose.position.x;
  js_pos["y"] = request.pose.position.y;
  js_pos["z"] = request.pose.position.z;
  js_ori["x"] = request.pose.orientation.x;
  js_ori["y"] = request.pose.orientation.y;
  js_ori["z"] = request.pose.orientation.z;
  js_ori["w"] = request.pose.orientation.w;
  js_pose["position"] = js_pos;
  js_pose["orientation"] = js_ori;

  // fill service args
  json js_args;
  js_args["args"]["pose"] = js_pose;
  js_args["args"]["slot"] = request.slot;

  // do service request
  cpr::Url url = cpr::Url{ servicesUrl_ + "set_pose" };
  auto rest_resp = cpr::Put(url, cpr::Timeout{ timeoutCurl_ }, cpr::Body{ js_args.dump() },
                      cpr::Header{ { "Content-Type", "application/json" } });
  handleCPRResponse(rest_resp);

  // parse json response into ros message
  auto json_resp = json::parse(rest_resp.text)["response"];
  response.success = (bool) json_resp["success"];
  response.status = json_resp["status"];
  response.message = json_resp["message"];
  return true;
}

bool CalibrationWrapper::calibResultCommon(const char* service_name,
                                           rc_hand_eye_calibration_client::CalibrationResponse &response)
{
  // do service request
  cpr::Url url = cpr::Url{ servicesUrl_ + service_name};
  auto rest_resp = cpr::Put(url, cpr::Timeout{ timeoutCurl_ });
  handleCPRResponse(rest_resp);

  // parse json response into ros message
  auto json_resp = json::parse(rest_resp.text)["response"];
  response.success = (bool) json_resp["success"];
  response.status = json_resp["status"];
  response.message = json_resp["message"];
  response.error = json_resp["error"];
  response.robot_mounted = (bool) json_resp["robot_mounted"];

  json js_pose = json_resp["pose"];
  response.pose.position.x = js_pose["position"]["x"];
  response.pose.position.y = js_pose["position"]["y"];
  response.pose.position.z = js_pose["position"]["z"];
  response.pose.orientation.x = js_pose["orientation"]["x"];
  response.pose.orientation.y = js_pose["orientation"]["y"];
  response.pose.orientation.z = js_pose["orientation"]["z"];
  response.pose.orientation.w = js_pose["orientation"]["w"];

  if (response.success)
  {
    ROS_INFO("Calibration request successful. Broadcasting new calibration.");
    updateCalibrationCache(response);
    sendCachedCalibration();
    if(calib_publish_timer_.isValid())//don't start it if it is invalid
    {
      calib_publish_timer_.start();//does nothing if already started.
    }
  }
  else { ROS_WARN_STREAM("Could not get calibration: " << response.message); }
  return true;
}

bool CalibrationWrapper::calibSrv(rc_hand_eye_calibration_client::CalibrationRequest &,
                                  rc_hand_eye_calibration_client::CalibrationResponse &response)
{
  return calibResultCommon("calibrate", response);
}


bool CalibrationWrapper::getCalibResultSrv(rc_hand_eye_calibration_client::CalibrationRequest &,
                                           rc_hand_eye_calibration_client::CalibrationResponse &response)
{
  return calibResultCommon("get_calibration" , response);
}

void CalibrationWrapper::requestCalibration(const ros::SteadyTimerEvent&)
{
  rc_hand_eye_calibration_client::CalibrationRequest request;
  rc_hand_eye_calibration_client::CalibrationResponse response;
  getCalibResultSrv(request, response);
}


void CalibrationWrapper::updateCalibrationCache(const rc_hand_eye_calibration_client::CalibrationResponse& response)
{
  current_calibration_.header.frame_id = camera_frame_id_;
  //Select child frame based on the type of calibration hand-eye (on-robot-cam) or base-eye (external cam)
  current_calibration_.child_frame_id = response.robot_mounted ? endeff_frame_id_ : base_frame_id_;
  current_calibration_.transform.translation.x = response.pose.position.x;
  current_calibration_.transform.translation.y = response.pose.position.y;
  current_calibration_.transform.translation.z = response.pose.position.z;
  current_calibration_.transform.rotation.x = response.pose.orientation.x;
  current_calibration_.transform.rotation.y = response.pose.orientation.y;
  current_calibration_.transform.rotation.z = response.pose.orientation.z;
  current_calibration_.transform.rotation.w = response.pose.orientation.w;
}

void CalibrationWrapper::sendCachedCalibration(const ros::SteadyTimerEvent&)
{
  if (calib_publish_period_ <= 0.0)//if there's no period use static tf
  {
    //Timestamp doesn't (or at least shouldn't) matter for static transforms
    //Time::now makes it easy to see when it was updated though
    current_calibration_.header.stamp = ros::Time::now();
    static_tf2_broadcaster_.sendTransform(current_calibration_);
  }
  else //periodic sending
  {
    //Pre-date, so the transformation can be directly used by clients until the next one is sent.
    //I.e., don't cause lag when looking up transformations because one has to wait for
    //the current calibration.
    current_calibration_.header.stamp = ros::Time::now() + ros::Duration(calib_publish_period_);
    dynamic_tf2_broadcaster_.sendTransform(current_calibration_);
  }
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

void CalibrationWrapper::initConfiguration()
{
  rc_hand_eye_calibration_client::hand_eye_calibrationConfig cfg;

  // first get the current values from sensor
  auto rest_resp = cpr::Get(paramsUrl_, cpr::Timeout{ timeoutCurl_ });
  handleCPRResponse(rest_resp);
  auto json_resp = json::parse(rest_resp.text);
  for (auto& param : json_resp) {
    string name = param["name"];
    if (param["name"] == "grid_width")
    {
      cfg.grid_width = param["value"];
    } else if (param["name"] == "grid_height")
    {
      cfg.grid_height = param["value"];
    } else if (param["name"] == "robot_mounted")
    {
      cfg.robot_mounted = (bool) param["value"];
    }
  }

  // second, try to get ROS parameters:
  // if parameter is not set in parameter server, we default to current sensor configuration
  nh_.param("grid_width", cfg.grid_width, cfg.grid_width);
  nh_.param("grid_height", cfg.grid_height, cfg.grid_height);
  nh_.param("robot_mounted", cfg.robot_mounted, cfg.robot_mounted);

  // see if those parameters are available otherwise use default (set in class header)
  nh_.param("rc_visard_frame_id", camera_frame_id_, camera_frame_id_);
  nh_.param("end_effector_frame_id", endeff_frame_id_, endeff_frame_id_);
  nh_.param("base_frame_id", base_frame_id_, base_frame_id_);
  nh_.param("calibration_publication_period", calib_publish_period_, calib_publish_period_);
  nh_.param("calibration_request_period", calib_request_period_, calib_request_period_);

  // set parameters on parameter server so that dynamic reconfigure picks them up
  nh_.setParam("grid_width", cfg.grid_width);
  nh_.setParam("grid_height", cfg.grid_height);
  nh_.setParam("robot_mounted", cfg.robot_mounted);

  // instantiate dynamic reconfigure server that will initially read those values
  using RCFSRV = dynamic_reconfigure::Server<rc_hand_eye_calibration_client::hand_eye_calibrationConfig>;
  server_ = unique_ptr<RCFSRV>(new dynamic_reconfigure::Server<rc_hand_eye_calibration_client::hand_eye_calibrationConfig>(nh_));
  server_->setCallback(boost::bind(&CalibrationWrapper::dynamicReconfigureCb, this, _1, _2));
}


void CalibrationWrapper::dynamicReconfigureCb(rc_hand_eye_calibration_client::hand_eye_calibrationConfig
    &config,
    uint32_t)
{
  ROS_DEBUG("Reconfigure Request: (%f x %f) %s",
           config.grid_width, config.grid_height,
           config.robot_mounted ? "True" : "False");

  // fill json request from dynamic reconfigure request
  json js_params, js_param;
  js_param["name"] ="grid_width";
  js_param["value"] = config.grid_width;
  js_params.push_back(js_param);
  js_param["name"] ="grid_height";
  js_param["value"] = config.grid_height;
  js_params.push_back(js_param);
  js_param["name"] ="robot_mounted";
  js_param["value"] = config.robot_mounted;
  js_params.push_back(js_param);

  // do service request
  auto rest_resp = cpr::Put(paramsUrl_, cpr::Timeout{ timeoutCurl_ },
                      cpr::Body{ js_params.dump() },
                      cpr::Header{ { "Content-Type", "application/json" } });

  handleCPRResponse(rest_resp);
}
