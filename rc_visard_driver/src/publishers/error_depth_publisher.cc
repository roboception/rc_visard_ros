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

#include "error_depth_publisher.h"

#include <rc_genicam_api/pixel_formats.h>

#include <sensor_msgs/image_encodings.h>

namespace rc
{
ErrorDepthPublisher::ErrorDepthPublisher(ros::NodeHandle& nh, const std::string& frame_id_prefix, double _f, double _t,
                                         double _scale)
  : GenICam2RosPublisher(frame_id_prefix)
{
  f = _f;
  t = _t;
  scale = _scale;

  pub = nh.advertise<sensor_msgs::Image>("error_depth", 1);
}

bool ErrorDepthPublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void ErrorDepthPublisher::publish(const rcg::Buffer* buffer, uint64_t pixelformat)
{
  if (pub.getNumSubscribers() > 0)
  {
    // buffer disparity and error images

    if (pixelformat == Coord3D_C16)
    {
      disp_list.add(buffer);
    }
    else if (pixelformat == Error8)
    {
      err_list.add(buffer);
    }

    // get disparity and error image pair with current time stamp

    uint64_t timestamp = buffer->getTimestampNS();

    std::shared_ptr<const rcg::Image> disp = disp_list.find(timestamp);
    std::shared_ptr<const rcg::Image> err = err_list.find(timestamp);

    if (disp && err)
    {
      if (disp->getWidth() == err->getWidth() && disp->getHeight() == err->getHeight())
      {
        // create image and initialize header

        sensor_msgs::ImagePtr im = boost::make_shared<sensor_msgs::Image>();

        const uint64_t freq = 1000000000ul;

        im->header.seq = seq++;
        im->header.stamp.sec = timestamp / freq;
        im->header.stamp.nsec = timestamp - freq * im->header.stamp.sec;
        im->header.frame_id = frame_id;

        // set image size

        im->width = static_cast<uint32_t>(disp->getWidth());
        im->height = static_cast<uint32_t>(disp->getHeight());

        // get pointer to image data in buffer

        size_t dpx = disp->getXPadding();
        const uint8_t* dps = disp->getPixels();

        size_t epx = err->getXPadding();
        const uint8_t* eps = err->getPixels();

        // convert image data

        im->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        im->is_bigendian = rcg::isHostBigEndian();
        im->step = im->width * sizeof(float);

        im->data.resize(im->step * im->height);
        float* pt = reinterpret_cast<float*>(&im->data[0]);

        float s = scale * f * im->width * t;

        bool bigendian = disp->isBigEndian();

        for (uint32_t k = 0; k < im->height; k++)
        {
          for (uint32_t i = 0; i < im->width; i++)
          {
            float d;

            if (bigendian)
            {
              d = scale * ((dps[0] << 8) | dps[1]);
            }
            else
            {
              d = scale * ((dps[1] << 8) | dps[0]);
            }

            dps += 2;

            if (d > 0)
            {
              *pt++ = *eps * s / (d * d);
            }
            else
            {
              *pt++ = std::numeric_limits<float>::infinity();
            }

            eps++;
          }

          dps += dpx;
          eps += epx;
        }

        // publish message

        pub.publish(im);
      }
      else
      {
        ROS_ERROR_STREAM("Size of disparity and error images differ: " << disp->getWidth() << "x" << disp->getHeight()
                                                                       << " != " << err->getWidth() << "x"
                                                                       << err->getHeight());
      }

      // remove all old images, including the current ones

      disp_list.removeOld(timestamp);
      err_list.removeOld(timestamp);
    }
  }
}
}
