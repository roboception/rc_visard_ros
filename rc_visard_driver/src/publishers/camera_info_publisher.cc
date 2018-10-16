/*
 * Copyright (c) 2017 Roboception GmbH
 * All rights reserved
 *
 * Author: Heiko Hirschmueller
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

#include "camera_info_publisher.h"

#include <rc_genicam_api/pixel_formats.h>

namespace rc
{
CameraInfoPublisher::CameraInfoPublisher(ros::NodeHandle& nh, const std::string& frame_id_prefix, double _f, double t,
                                         bool left)
  : GenICam2RosPublisher(frame_id_prefix)
{
  f = _f;

  // prepare camera info message

  info.header.frame_id = frame_id;

  info.width = 0;
  info.height = 0;
  info.distortion_model = "plumb_bob";  // we have to choose a model
  info.D.resize(5);                     // all 0, since images are rectified

  info.K[0] = 1;
  info.K[1] = 0;
  info.K[2] = 0;
  info.K[3] = 0;
  info.K[4] = 1;
  info.K[5] = 0;
  info.K[6] = 0;
  info.K[7] = 0;
  info.K[8] = 1;

  info.R[0] = 1;
  info.R[1] = 0;
  info.R[2] = 0;
  info.R[3] = 0;
  info.R[4] = 1;
  info.R[5] = 0;
  info.R[6] = 0;
  info.R[7] = 0;
  info.R[8] = 1;

  info.P[0] = 1;
  info.P[1] = 0;
  info.P[2] = 0;
  info.P[3] = 0;
  info.P[4] = 0;
  info.P[5] = 1;
  info.P[6] = 0;
  info.P[7] = 0;
  info.P[8] = 0;
  info.P[9] = 0;
  info.P[10] = 1;
  info.P[11] = 0;

  info.binning_x = 1;
  info.binning_y = 1;

  // advertise topic

  if (left)
  {
    pub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
  }
  else
  {
    pub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
    info.P[3] = t;
  }
}

bool CameraInfoPublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void CameraInfoPublisher::publish(const rcg::Buffer* buffer, uint32_t part, uint64_t pixelformat)
{
  if (pub.getNumSubscribers() > 0 && (pixelformat == Mono8 || pixelformat == YCbCr411_8))
  {
    const uint64_t freq = 1000000000ul;
    uint64_t time = buffer->getTimestampNS();

    info.header.seq++;
    info.header.stamp.sec = time / freq;
    info.header.stamp.nsec = time - freq * info.header.stamp.sec;

    info.width = static_cast<uint32_t>(buffer->getWidth(part));
    info.height = static_cast<uint32_t>(buffer->getHeight(part));

    if (info.height > info.width)
    {
      info.height >>= 1;  // left and right images are stacked together
    }

    info.K[0] = info.K[4] = f * info.width;
    info.P[0] = info.P[5] = f * info.width;

    info.P[2] = info.K[2] = info.width / 2.0;
    info.P[6] = info.K[5] = info.height / 2.0;

    pub.publish(info);
  }
}
}
