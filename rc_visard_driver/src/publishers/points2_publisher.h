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

#ifndef RC_POINTS2PUBLISHER_H
#define RC_POINTS2PUBLISHER_H

#include "genicam2ros_publisher.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <rc_genicam_api/imagelist.h>

namespace rc
{
class Points2Publisher : public GenICam2RosPublisher
{
public:
  /**
    Initialization of publisher for depth errors.

    @param nh     Node handle.
    @param f      Focal length, normalized to image width of 1.
    @param t      Basline in m.
    @param scale  Factor for raw disparities.
    @param frame_id Parent frame id of points.
  */

  Points2Publisher(ros::NodeHandle& nh, const std::string& frame_id_prefix, double f, double t, double scale);

  void setOut1Alternate(bool alternate);

  bool used() override;

  void publish(const rcg::Buffer* buffer, uint32_t part, uint64_t pixelformat) override;
  void publish(const rcg::Buffer* buffer, uint32_t part, uint64_t pixelformat, bool out1);

private:
  Points2Publisher(const Points2Publisher&);             // forbidden
  Points2Publisher& operator=(const Points2Publisher&);  // forbidden

  rcg::ImageList left_list;
  rcg::ImageList disp_list;

  uint32_t seq;
  float f;
  float t;
  float scale;

  uint64_t tolerance_ns;

  ros::Publisher pub;
};
}

#endif
