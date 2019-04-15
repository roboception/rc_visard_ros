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

#include <cpr/cpr.h>

#include <exception>
#include <iostream>

namespace rc_itempick_cpr
{
using namespace std;

static string toString(cpr::Response resp)
{
  stringstream s;
  s << "status code: " << resp.status_code << endl
    << "url: " << resp.url << endl
    << "text: " << resp.text << endl
    << "error: " << resp.error.message;
  return s.str();
}

static void handleCPRResponse(const cpr::Response &r)
{
  if (r.status_code != 200)
  {
    throw std::runtime_error(toString(r));
  }
}

CommunicationHelper::CommunicationHelper(const string &host,
                                         const string &node_name,
                                         int timeout)
        : host_(host),
          services_url_("http://" + host + "/api/v1/nodes/"+ node_name + "/services/"),
          params_url_("http://" + host + "/api/v1/nodes/"+ node_name + "/parameters"),
          timeout_curl_(timeout)
{ }

json CommunicationHelper::servicePutRequest(const std::string &service_name)
{

  cpr::Url url = cpr::Url{services_url_ + service_name};
  auto rest_resp = cpr::Put(url, cpr::Timeout{timeout_curl_});
  handleCPRResponse(rest_resp);
  return json::parse(rest_resp.text)["response"];
}

json CommunicationHelper::servicePutRequest(const std::string &service_name, const json &js_args)
{
  cpr::Url url = cpr::Url{services_url_ + service_name};
  auto rest_resp = cpr::Put(url, cpr::Timeout{timeout_curl_}, cpr::Body{js_args.dump()},
                            cpr::Header{{"Content-Type", "application/json"}});
  handleCPRResponse(rest_resp);
  return json::parse(rest_resp.text)["response"];
}

json CommunicationHelper::getParameters(){
  auto rest_resp = cpr::Get(params_url_, cpr::Timeout{timeout_curl_});
  handleCPRResponse(rest_resp);
  return json::parse(rest_resp.text);
}

void CommunicationHelper::setParameters(const json& js_params){
  auto rest_resp = cpr::Put(params_url_, cpr::Timeout{timeout_curl_},
                            cpr::Body{js_params.dump()},
                            cpr::Header{{"Content-Type", "application/json"}});

  handleCPRResponse(rest_resp);
}

}