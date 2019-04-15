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
#ifndef RC_ITEMPICK_CLIENT_CPR_HELPER_H
#define RC_ITEMPICK_CLIENT_CPR_HELPER_H

#include <json/json.hpp>

#include <string>

using json = nlohmann::json;

namespace rc_itempick_cpr
{
class CommunicationHelper
{
  public:
    CommunicationHelper(const std::string &host, const std::string &node_name,
                        int timeout);

    json servicePutRequest(const std::string &service_name);

    json servicePutRequest(const std::string &service_name, const json &js_args);

    json getParameters();

    void setParameters(const json& js_params);

  private:
    // REST stuff
    const std::string host_, services_url_, params_url_;
    const int timeout_curl_; // ms
};

}
#endif //RC_ITEMPICK_CLIENT_CPR_HELPER_H
