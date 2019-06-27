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

#include "boxpick_client.h"
#include "itempick_client.h"

#include <ros/ros.h>
#include <signal.h>

std::unique_ptr<ros_pick_client::PickClient> pick_client;

void sigintHandler(int)
{
  pick_client.reset();
  ros::shutdown();
}

int main(int argc, char **argv)
{
  std::string pick_type = PICK_TYPE;
  ros::init(argc, argv, pick_type, ros::init_options::NoSigintHandler);

  if (pick_type != "rc_boxpick" && pick_type !="rc_itempick")
  {
    ROS_FATAL("node type not defined");
  }

  signal(SIGINT, sigintHandler);

  ros::NodeHandle pnh("~");
  std::string host;
  pnh.getParam("host", host);
  if (host.empty())
  {
    ROS_FATAL("No host set! Please set the parameter 'host'!");
    return 1;
  }

  try
  {
    // instantiate wrapper and advertise services
    if (pick_type == "rc_boxpick")
    {
      pick_client.reset(new ros_pick_client::BoxpickClient(host, pnh));
    }
    else
    {
      pick_client.reset(new ros_pick_client::ItempickClient(host, pnh));
    }
    ROS_INFO_STREAM(pick_type << " node started for host: " << host);

  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Client could not be created due to an error: %s", ex.what());
    return 1;
  }

  ros::Rate loop_rate(10);

  ros::spin();
}
