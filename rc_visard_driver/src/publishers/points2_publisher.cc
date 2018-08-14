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

#include "points2_publisher.h"

#include <rc_genicam_api/pixel_formats.h>

#include <sensor_msgs/PointCloud2.h>

namespace rc
{
Points2Publisher::Points2Publisher(ros::NodeHandle& nh, const std::string& frame_id_prefix, double _f, double _t,
                                   double _scale)
  : GenICam2RosPublisher(frame_id_prefix), left_list(50)
{
  f = _f;
  t = _t;
  scale = _scale;

  tolerance_ns = 0;

  pub = nh.advertise<sensor_msgs::PointCloud2>("points2", 1);
}

void Points2Publisher::setOut1Alternate(bool alternate)
{
  if (alternate)
  {
    tolerance_ns = static_cast<uint64_t>(0.050 * 1000000000ull);
  }
  else
  {
    tolerance_ns = 0;
  }
}

bool Points2Publisher::used()
{
  return pub.getNumSubscribers() > 0;
}

void Points2Publisher::publish(const rcg::Buffer* buffer, uint64_t pixelformat)
{
  publish(buffer, pixelformat, false);
}

void Points2Publisher::publish(const rcg::Buffer* buffer, uint64_t pixelformat, bool out1)
{
  if (pub.getNumSubscribers() > 0)
  {
    // buffer left and disparity images

    if (pixelformat == Mono8 || pixelformat == YCbCr411_8)
    {
      // in alternate exposure mode, skip images for texture with out1 == true,
      // i.e. with projected pattern

      if (tolerance_ns > 0 && out1)
      {
        return;
      }

      left_list.add(buffer);
    }
    else if (pixelformat == Coord3D_C16)
    {
      disp_list.add(buffer);
    }

    // get corresponding left and disparity image

    uint64_t timestamp = buffer->getTimestampNS();

    std::shared_ptr<const rcg::Image> left = left_list.find(timestamp, tolerance_ns);
    std::shared_ptr<const rcg::Image> disp = disp_list.find(timestamp, tolerance_ns);

    // print warning with reason if no left image can be found for disparity image

    if (pixelformat == Coord3D_C16 && !left)
    {
      if (timestamp < left_list.getOldestTime())
      {
        ROS_DEBUG_STREAM("Cannot find left image for disparity image. Internal queue size to small.");
      }
      else
      {
        ROS_DEBUG_STREAM("Cannot find left image for disparity image. Left image possibly dropped.");
      }
    }

    if (left && disp)
    {
      // determine integer factor between size of left and disparity image

      uint32_t lw = left->getWidth();
      uint32_t lh = left->getHeight();

      if (lh > lw)  // there may be a stacked right image
      {
        lh >>= 1;
      }

      int ds = (lw + disp->getWidth() - 1) / disp->getWidth();

      if ((lw + ds - 1) / ds == disp->getWidth() && (lh + ds - 1) / ds == disp->getHeight())
      {
        // allocate new image message and set meta information

        sensor_msgs::PointCloud2Ptr p = boost::make_shared<sensor_msgs::PointCloud2>();

        const uint64_t freq = 1000000000ul;

        p->header.seq = seq++;
        p->header.stamp.sec = timestamp / freq;
        p->header.stamp.nsec = timestamp - freq * p->header.stamp.sec;
        p->header.frame_id = frame_id;

        // set meta data of point cloud

        p->width = lw / ds;   // consider only full pixels if downscaled
        p->height = lh / ds;  // consider only full pixels if downscaled

        p->is_bigendian = rcg::isHostBigEndian();
        p->is_dense = false;

        p->fields.resize(4);
        p->fields[0].name = "x";
        p->fields[0].offset = 0;
        p->fields[0].count = 1;
        p->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        p->fields[1].name = "y";
        p->fields[1].offset = 4;
        p->fields[1].count = 1;
        p->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        p->fields[2].name = "z";
        p->fields[2].offset = 8;
        p->fields[2].count = 1;
        p->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        p->fields[3].name = "rgb";
        p->fields[3].offset = 12;
        p->fields[3].count = 1;
        p->fields[3].datatype = sensor_msgs::PointField::FLOAT32;

        p->point_step = 16;
        p->row_step = p->point_step * p->width;

        // allocate memory

        p->data.resize(p->row_step * p->height);
        float* pd = reinterpret_cast<float*>(&p->data[0]);

        // pointer to disparity data

        const uint8_t* dps = disp->getPixels();
        size_t dstep = disp->getWidth() * sizeof(uint16_t) + disp->getXPadding();

        // convert disparity to point cloud using left image for texture

        float ff = f * disp->getWidth();

        bool bigendian = disp->isBigEndian();

        for (uint32_t k = 0; k < p->height; k++)
        {
          for (uint32_t i = 0; i < p->width; i++)
          {
            // get disparity

            uint32_t j = i << 1;

            float d;

            if (bigendian)
            {
              d = scale * ((dps[j] << 8) | dps[j + 1]);
            }
            else
            {
              d = scale * ((dps[j + 1] << 8) | dps[j]);
            }

            // if disparity is valid and color can be obtained

            if (d > 0)
            {
              // reconstruct 3D point

              pd[0] = (i + 0.5 - disp->getWidth() / 2.0) * t / d;
              pd[1] = (k + 0.5 - disp->getHeight() / 2.0) * t / d;
              pd[2] = ff * t / d;

              // store color of point

              uint8_t rgb[3];
              rcg::getColor(rgb, left, ds, i, k);

              uint8_t* bgra = reinterpret_cast<uint8_t*>(pd + 3);

              bgra[0] = rgb[2];
              bgra[1] = rgb[1];
              bgra[2] = rgb[0];
              bgra[3] = 0;
            }
            else
            {
              for (int i = 0; i < 4; i++)
              {
                pd[i] = std::numeric_limits<float>::quiet_NaN();
              }
            }

            pd += 4;
          }

          dps += dstep;
        }

        // publish message

        pub.publish(p);
      }
      else
      {
        ROS_ERROR_STREAM("Size of left and disparity image must differ only by an integer factor: "
                         << left->getWidth() << "x" << left->getHeight() << " != " << disp->getWidth() << "x"
                         << disp->getHeight());
      }

      // remove all old images, including the current ones

      left_list.removeOld(timestamp);
      disp_list.removeOld(timestamp);
    }
  }
}
}
