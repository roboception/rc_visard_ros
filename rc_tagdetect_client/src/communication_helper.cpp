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

#include "communication_helper.h"
#include "exceptions.h"
#include <cpr/cpr.h>

#include <exception>
#include <iostream>

namespace rc_tagdetect_client
{

using std::string;
using std::stringstream;
using std::endl;

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
    throw RequestException(toString(r));
  }
}

CommunicationHelper::CommunicationHelper(const string &host,
                                         const string &node_name,
                                         int timeout)
        : host_(host),
          services_url_("http://" + host + "/api/v1/nodes/" + node_name + "/services/"),
          params_url_("http://" + host + "/api/v1/nodes/" + node_name + "/parameters"),
          version_url_("http://" + host + "/api/v1/system"),
          timeout_curl_(timeout)
{

  int num_tries = 5;
  bool comm_established = false;

  while (!comm_established)
  {
    const auto response = cpr::Get(version_url_, cpr::Timeout{timeout_curl_});
    if (response.status_code == 200)
    {
      comm_established = true;
    }
    else
    {
      num_tries--;
      if (num_tries == 0)
      {
        throw std::runtime_error("Could not connect to rc_visard");
      }
    }
  }
}

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

json CommunicationHelper::getParameters()
{
  auto rest_resp = cpr::Get(params_url_, cpr::Timeout{timeout_curl_});
  handleCPRResponse(rest_resp);
  return json::parse(rest_resp.text);
}

void CommunicationHelper::setParameters(const json &js_params)
{
  auto rest_resp = cpr::Put(params_url_, cpr::Timeout{timeout_curl_},
                            cpr::Body{js_params.dump()},
                            cpr::Header{{"Content-Type", "application/json"}});

  handleCPRResponse(rest_resp);
}

std::tuple<size_t, size_t, size_t> CommunicationHelper::getImageVersion()
{
  const auto response = cpr::Get(version_url_, cpr::Timeout{timeout_curl_});
  handleCPRResponse(response);
  try
  {
    const auto j = json::parse(response.text);
    std::string image_version = j.at("firmware").
            at("active_image").
            at("image_version").
            get<std::string>();
    const std::string prefix = "rc_visard_v";
    if (image_version.find(prefix) == 0)
    {
      image_version = image_version.substr(prefix.size());
    }
    const auto minus_pos = image_version.find('-');
    if (minus_pos != std::string::npos)
    {
      image_version = image_version.substr(0, minus_pos);
    }
    std::istringstream iss(image_version);
    std::string v;
    std::vector<size_t> version_components;
    while (std::getline(iss, v, '.'))
    {
      version_components.push_back(std::stoul(v));
    }
    if (version_components.size() == 3)
    {
      auto image_version_tuple = std::make_tuple(version_components[0],
                                                 version_components[1],
                                                 version_components[2]);
      return image_version_tuple;
    }
    else
    {
      throw std::runtime_error("Could not parse image version");
    }
  }
  catch (const std::exception &ex)
  {
    throw MiscException(std::string("Could not parse response: ") + ex.what());
  }
}


}
