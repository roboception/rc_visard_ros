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

#include "image_publisher.h"

#include <rc_genicam_api/image.h>
#include <rc_genicam_api/pixel_formats.h>

#include <sensor_msgs/image_encodings.h>

namespace rc
{

ImagePublisher::ImagePublisher(image_transport::ImageTransport &it, std::string frame_id_prefix,
                               bool _left, bool _color)
        : GenICam2RosPublisher(frame_id_prefix)
{
  left=_left;
  color=_color;
  seq=0;

  if (left)
  {
    if (color)
    {
      pub=it.advertise("left/image_rect_color", 1);
    }
    else
    {
      pub=it.advertise("left/image_rect", 1);
    }
  }
  else
  {
    if (color)
    {
      pub=it.advertise("right/image_rect_color", 1);
    }
    else
    {
      pub=it.advertise("right/image_rect", 1);
    }
  }
}

bool ImagePublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

namespace
{

/**
  Clamp the given value to the range of 0 to 255 and cast to byte.
*/

uint8_t clamp8(int v)
{
  if (v < 0) { v=0; }

  if (v > 255) { v=255; }

  return static_cast<uint8_t>(v);
}

}

void ImagePublisher::publish(const rcg::Buffer *buffer, uint64_t pixelformat)
{
  if (pub.getNumSubscribers() > 0 && (pixelformat == Mono8 || pixelformat == YCbCr411_8))
  {
    // create image and initialize header

    sensor_msgs::ImagePtr im=boost::make_shared<sensor_msgs::Image>();

    const uint64_t freq=1000000000ul;
    uint64_t time=buffer->getTimestampNS();

    im->header.seq=seq++;
    im->header.stamp.sec=time/freq;
    im->header.stamp.nsec=time-freq*im->header.stamp.sec;
    im->header.frame_id=frame_id;

    // set image size

    im->width=static_cast<uint32_t>(buffer->getWidth());
    im->height=static_cast<uint32_t>(buffer->getHeight());
    im->is_bigendian=false;

    bool stacked=false;

    if (im->height > im->width)
    {
      stacked=true;
      im->height>>=1;
    }

    // get pointer to image data in buffer

    const uint8_t *ps=static_cast<const uint8_t *>(buffer->getBase())+buffer->getImageOffset();
    size_t pstep=im->width+buffer->getXPadding();

    if (pixelformat == YCbCr411_8)
    {
      pstep=(im->width>>2)*6+buffer->getXPadding();
    }

    if (!left)
    {
      if (stacked)
      {
        ps+=pstep*im->height;
      }
      else
      {
        return; // buffer does not contain a right image
      }
    }

    // convert image data

    if (color) // convert to color
    {
      im->encoding=sensor_msgs::image_encodings::RGB8;
      im->step=3*im->width*sizeof(uint8_t);

      im->data.resize(im->step*im->height);
      uint8_t *pt=reinterpret_cast<uint8_t *>(&im->data[0]);

      if (pixelformat == Mono8) // convert from monochrome
      {
        return; // do not convert from monochrome, skip instead
      }
      else if (pixelformat == YCbCr411_8) // convert from YUV 411
      {
        for (uint32_t k=0; k<im->height; k++)
        {
          for (uint32_t i=0; i<im->width; i+=4)
          {
            rcg::convYCbCr411toQuadRGB(pt, ps, i);
            pt+=12;
          }

          ps+=pstep;
        }
      }
    }
    else // convert to monochrome
    {
      im->encoding=sensor_msgs::image_encodings::MONO8;
      im->step=im->width*sizeof(uint8_t);

      im->data.resize(im->step*im->height);
      uint8_t *pt=reinterpret_cast<uint8_t *>(&im->data[0]);

      if (pixelformat == Mono8) // copy monochrome image
      {
        for (uint32_t k=0; k<im->height; k++)
        {
          for (uint32_t i=0; i<im->width; i++)
          {
            *pt++=ps[i];
          }

          ps+=pstep;
        }
      }
      else if (pixelformat == YCbCr411_8) // copy monochrome part of YUV 411 image
      {
        for (uint32_t k=0; k<im->height; k++)
        {
          int j=0;

          for (uint32_t i=0; i<im->width; i+=4)
          {
            *pt++=ps[j];
            *pt++=ps[j+1];
            *pt++=ps[j+3];
            *pt++=ps[j+4];
            j+=6;
          }

          ps+=pstep;
        }
      }
    }

    // publish message

    pub.publish(im);
  }
}

}
