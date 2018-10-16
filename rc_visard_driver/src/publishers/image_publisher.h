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

#ifndef RC_IMAGEPUBLISHER_H
#define RC_IMAGEPUBLISHER_H

#include "genicam2ros_publisher.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace rc
{
class ImagePublisher : public GenICam2RosPublisher
{
public:
  /**
    Initialization of publisher.

    @param it              Image transport handle.
    @param frame_id_prefix Prefix for frame ids in published ros messages.
    @param left            True for left and false for right camera.
    @param color           True for sending color instead of monochrome images.
    @param out1_filter     True for also providing ...out1_low and ...out1_high
                           topics.
  */

  ImagePublisher(image_transport::ImageTransport& it, const std::string& frame_id_prefix, bool left, bool color,
                 bool out1_filter);

  bool used() override;

  void setOut1Alternate(bool alternate)
  {
    out1_alternate = alternate;
  }

  void publish(const rcg::Buffer* buffer, uint32_t part, uint64_t pixelformat) override;
  void publish(const rcg::Buffer* buffer, uint32_t part, uint64_t pixelformat, bool out1);

private:
  ImagePublisher(const ImagePublisher&);             // forbidden
  ImagePublisher& operator=(const ImagePublisher&);  // forbidden

  bool left;
  bool color;
  uint32_t seq;
  bool out1_alternate;

  image_transport::Publisher pub;
  image_transport::Publisher pub_out1_low;
  image_transport::Publisher pub_out1_high;
};
}

#endif
