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

#include "disparity_publisher.h"

#include <rc_genicam_api/pixel_formats.h>

#include <sensor_msgs/image_encodings.h>

namespace rc
{

DisparityPublisher::DisparityPublisher(ros::NodeHandle &nh,
                                       std::string frame_id_prefix, double _f,
                                       double _t, double _scale)
        : GenICam2RosPublisher(frame_id_prefix)
{
  seq=0;
  f=_f;
  t=_t;
  scale=_scale;
  disprange=0;

  pub=nh.advertise<stereo_msgs::DisparityImage>("disparity", 1);
}

void DisparityPublisher::setDisprange(int _disprange)
{
  disprange=_disprange;
}

bool DisparityPublisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void DisparityPublisher::publish(const rcg::Buffer *buffer, uint64_t pixelformat)
{
  if (pub.getNumSubscribers() > 0 && pixelformat == Coord3D_C16)
  {
    // allocate new image message and set meta information

    stereo_msgs::DisparityImagePtr p=boost::make_shared<stereo_msgs::DisparityImage>();

    const uint64_t freq=1000000000ul;
    uint64_t time=buffer->getTimestampNS();

    p->header.seq=seq++;
    p->header.stamp.sec=time/freq;
    p->header.stamp.nsec=time-freq*p->header.stamp.sec;
    p->header.frame_id=frame_id;

    // prepare size and format of outgoing image

    p->image.header=p->header;
    p->image.width=static_cast<uint32_t>(buffer->getWidth());
    p->image.height=static_cast<uint32_t>(buffer->getHeight());
    p->image.encoding=sensor_msgs::image_encodings::TYPE_32FC1;
    p->image.is_bigendian=rcg::isHostBigEndian();
    p->image.step=p->image.width*sizeof(float);

    size_t px=buffer->getXPadding();
    const uint8_t *ps=static_cast<const uint8_t *>(buffer->getBase())+buffer->getImageOffset();

    // convert image information

    p->image.data.resize(p->image.step*p->image.height);

    float *pt=reinterpret_cast<float *>(&p->image.data[0]);
    float dmax=0;

    bool bigendian=buffer->isBigEndian();

    for (uint32_t k=0; k<p->image.height; k++)
    {
      if (bigendian)
      {
        for (uint32_t i=0; i<p->image.width; i++)
        {
          uint16_t d=(ps[0]<<8)|ps[1];

          *pt=-1.0f;

          if (d != 0)
          {
            *pt=scale*d;
            dmax=std::max(dmax, *pt);
          }

          ps+=2;
          pt++;
        }
      }
      else
      {
        for (uint32_t i=0; i<p->image.width; i++)
        {
          uint16_t d=(ps[1]<<8)|ps[0];

          *pt=-1.0f;

          if (d != 0)
          {
            *pt=scale*d;
            dmax=std::max(dmax, *pt);
          }

          ps+=2;
          pt++;
        }
      }

      ps+=px;
    }

    p->f=f*p->image.width;
    p->T=t;
    p->valid_window.x_offset=0;
    p->valid_window.y_offset=0;
    p->valid_window.width=p->image.width;
    p->valid_window.height=p->image.height;
    p->min_disparity=0;
    p->max_disparity=std::max(dmax, static_cast<float>(disprange));
    p->delta_d=scale;

    // publish message

    pub.publish(p);
  }
}

}
