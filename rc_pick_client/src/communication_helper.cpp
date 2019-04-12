/* 
* Roboception GmbH 
* Munich, Germany 
* www.roboception.com 
* 
* Copyright (c) 2019 Roboception GmbH 
* All rights reserved 
* 
* Author: Monika Florek-Jasinska
*/
#include "communication_helper.h"

namespace rc_itempick_cpr
{
using namespace std;
string toString(cpr::Response resp)
{
  stringstream s;
  s << "status code: " << resp.status_code << endl
    << "url: " << resp.url << endl
    << "text: " << resp.text << endl
    << "error: " << resp.error.message;
  return s.str();
}

void handleCPRResponse(const cpr::Response &r)
{
  if (r.status_code != 200)
  {
    throw std::runtime_error(toString(r));
  }
}

CommunicationHelper::CommunicationHelper(const std::string &host, const std::string &node_name, int timeout)
        : host_(host),
          servicesUrl_("http://" + host + "/api/v1/nodes/"+node_name+"/services/"),
          paramsUrl_("http://" + host + "/api/v1/nodes/"+node_name+"/parameters")
{
  timeoutCurl_ = timeout;
}

json CommunicationHelper::servicePutRequest(std::string service_name)
{

  cpr::Url url = cpr::Url{servicesUrl_ + service_name};
  auto rest_resp = cpr::Put(url, cpr::Timeout{timeoutCurl_});
  handleCPRResponse(rest_resp);
  return json::parse(rest_resp.text)["response"];
}

json CommunicationHelper::servicePutRequest(std::string service_name, json js_args)
{
  cpr::Url url = cpr::Url{servicesUrl_ + service_name};
  auto rest_resp = cpr::Put(url, cpr::Timeout{timeoutCurl_}, cpr::Body{js_args.dump()},
                            cpr::Header{{"Content-Type", "application/json"}});
  handleCPRResponse(rest_resp);
  return json::parse(rest_resp.text)["response"];
}

json CommunicationHelper::getParameters(){
  auto rest_resp = cpr::Get(paramsUrl_, cpr::Timeout{timeoutCurl_});
  handleCPRResponse(rest_resp);
  return json::parse(rest_resp.text);

}

void CommunicationHelper::setParameters(const json& js_params){
  auto rest_resp = cpr::Put(paramsUrl_, cpr::Timeout{timeoutCurl_},
                            cpr::Body{js_params.dump()},
                            cpr::Header{{"Content-Type", "application/json"}});

  handleCPRResponse(rest_resp);
}

}