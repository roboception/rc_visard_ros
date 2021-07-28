/*
 * Copyright (c) 2019 Roboception GmbH
 * All rights reserved
 *
 * Author: Christian Emmerich
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

#include "camera_param_publisher.h"

#include <rc_genicam_api/pixel_formats.h>

namespace rc
{
CameraParamPublisher::CameraParamPublisher(ros::NodeHandle& nh,
                    const std::string& frame_id_prefix, bool left)
  : frame_id(frame_id_prefix + "camera")
{
  // advertise topic

  if (left)
  {
    pub = nh.advertise<rc_common_msgs::CameraParam>("left/camera_param", 1);
  }
  else
  {
    pub = nh.advertise<rc_common_msgs::CameraParam>("right/camera_param", 1);
  }
}

bool CameraParamPublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void CameraParamPublisher::publish(const rcg::Buffer* buffer, const rc_common_msgs::CameraParam& p,
  uint64_t pixelformat)
{
  if (pub.getNumSubscribers() > 0 && (pixelformat == Mono8 || pixelformat == YCbCr411_8 || pixelformat == RGB8))
  {
    const uint64_t freq = 1000000000ul;
    uint64_t time = buffer->getTimestampNS();

    // params.header.seq++;
    auto params = p;
    params.header.frame_id = frame_id;
    params.header.stamp.sec = time / freq;
    params.header.stamp.nsec = time - freq * params.header.stamp.sec;

    pub.publish(params);
  }
}

}
