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

string toString(list<string> list)
{
  stringstream s;
  s << "[";
  for (auto it = list.begin(); it != list.end();)
  {
    s << *it;
    if (++it != list.end())
    {
      s << ", ";
    }
  }
  s << "]";
  return s.str();
}

void handleCPRResponse(cpr::Response r)
{
  if (r.status_code != 200)
  {
    throw runtime_error(toString(r));
  }
}

bool isValidIPAddress(const std::string& ip)
{
  // use inet_pton to check if given string is a valid IP address
  static struct sockaddr_in sa;
  return TEMP_FAILURE_RETRY(inet_pton(AF_INET, ip.c_str(), &(sa.sin_addr))) == 1;
}
  
}//anonymous ns


CalibrationWrapper::CalibrationWrapper(std::string name, std::string ip_addr)
  : nh_ {name}, ip_addr_(ip_addr), baseUrl_("http://" + ip_addr + "/api/v1/nodes/rc_hand_eye_calibration/services/")
{

  // check if given string is a valid IP address
  if (!isValidIPAddress(ip_addr))
  {
    throw invalid_argument("Given IP address is not a valid address: " + ip_addr);
  }

  timeoutCurl_ = 2000;
  advertiseServices();
}


bool CalibrationWrapper::saveSrv(std_srvs::TriggerRequest &,
                                 std_srvs::TriggerResponse &response)
{
  cpr::Url url = cpr::Url{ baseUrl_ + "save_calibration"};
  auto rest_resp = cpr::Put(url, cpr::Timeout{ timeoutCurl_ });
  handleCPRResponse(rest_resp);

  auto json_resp = json::parse(rest_resp.text)["response"];
  response.success = (bool) json_resp["success"];
  response.message = json_resp["message"];
  return true;

  return true;
}


bool CalibrationWrapper::resetSrv(std_srvs::TriggerRequest &,
                                  std_srvs::TriggerResponse &response)
{
  cpr::Url url = cpr::Url{ baseUrl_ + "reset_calibration"};
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
  cpr::Url url = cpr::Url{ baseUrl_ + "remove_calibration"};
  auto rest_resp = cpr::Put(url, cpr::Timeout{ timeoutCurl_ });
  handleCPRResponse(rest_resp);

  auto json_resp = json::parse(rest_resp.text)["response"];
  response.success = (bool) json_resp["success"];
  response.message = json_resp["message"];
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
  cpr::Url url = cpr::Url{ baseUrl_ + "set_pose" };
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


bool CalibrationWrapper::calibSrv(rc_hand_eye_calibration_client::CalibrationRequest &,
                                  rc_hand_eye_calibration_client::CalibrationResponse &response)
{
  // do service request
  cpr::Url url = cpr::Url{ baseUrl_ + "calibrate" };
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

  return true;
}


bool CalibrationWrapper::getCalibResultSrv(rc_hand_eye_calibration_client::CalibrationRequest &,
                                           rc_hand_eye_calibration_client::CalibrationResponse &response)
{
  // do service request
  cpr::Url url = cpr::Url{ baseUrl_ + "get_calibration" };
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
