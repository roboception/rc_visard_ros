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

#include "ros_tagdetect_client.h"
#include <ros/ros.h>
#include <signal.h>
#include <rcdiscover/discover.h>
#include <rcdiscover/utils.h>

std::unique_ptr<rc_tagdetect_client::RosTagdetectClient> tagdetect_wrapper;

void sigintHandler(int)
{
  tagdetect_wrapper.reset();
  ros::shutdown();
}

std::string getHost(const std::string &device_name, const std::string &interface)
{
  // broadcast discover request
  rcdiscover::Discover discover;
  discover.broadcastRequest();

  std::vector<rcdiscover::DeviceInfo> infos;

  // get responses
  while (discover.getResponse(infos, 100)) { }

  std::vector<std::vector<std::string>> devices;
  std::vector<std::string> filtered_info = {"", ""};
  for (const auto& info : infos)
  {
    if (!interface.empty() && interface != info.getIfaceName())
    {
      continue;
    }

    // if used defined name is not set, fall back to model name
    std::string user_defined_name = info.getUserName();
    if (user_defined_name.empty())
    {
      user_defined_name = info.getModelName();
    }

    // if no device is given, return any rc_visard
    if (device_name.empty())
    {
      if (info.getModelName().find("rc_visard") != std::string::npos)
      {
        filtered_info[0] = user_defined_name;
        filtered_info[1] = ip2string(info.getIP());
        devices.push_back(filtered_info);
      }
    }
    else if ((device_name == info.getSerialNumber()) || (device_name == user_defined_name))
    {
      filtered_info[0] = user_defined_name;
      filtered_info[1] = ip2string(info.getIP());
      devices.push_back(filtered_info);
    }
  }
  std::sort(devices.begin(), devices.end());
  auto unique_devices_it = std::unique(devices.begin(), devices.end());
  devices.erase(unique_devices_it, devices.end());

  if (devices.empty())
  {
    ROS_FATAL_STREAM("No device found with the name '" << device_name << "'");
    return "";
  }
  else if (devices.size() > 1)
  {
    ROS_FATAL_STREAM("Found " << devices.size() << " devices with the name '" << device_name
                     << "'. Please specify a unique device name.");
    return "";
  }

  ROS_INFO_STREAM("Using device '" << device_name << "' with name '" << devices[0][0] << "' and IP address " << devices[0][1]);
  return devices[0][1];
}

int main(int argc, char **argv)
{
  std::string name = DETECTION_TAG_TYPE;

  ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
  signal(SIGINT, sigintHandler);

  ros::NodeHandle pnh("~");


  std::string host;
  std::string device;
  pnh.getParam("host", host);
  pnh.getParam("device", device);
  if (!host.empty() && !device.empty())
  {
    ROS_WARN("Both parameters: 'device' and 'host' are set. Using 'device' to start the client.");
  }

  if (!device.empty() || host.empty())
  {
    std::size_t delim_pos = 0;
    delim_pos = device.find(':');
    std::string interface;
    if (delim_pos != std::string::npos)
    {
      interface = device.substr(0, delim_pos);
      device = device.substr(delim_pos + 1);
    }

    host = getHost(device, interface);
  }

  if (host.empty())
  {
    return 1;
  }

  try
  {
    // instantiate wrapper and advertise services
    tagdetect_wrapper.reset(new rc_tagdetect_client::RosTagdetectClient(host, pnh, name));
  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Client could not be created due to an error: %s", ex.what());
    return 1;
  }

  ROS_INFO_STREAM("TagDetect node started for host: " << host);

  ros::spin();
}
