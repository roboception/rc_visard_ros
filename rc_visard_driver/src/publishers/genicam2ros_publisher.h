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

#ifndef RC_VISARD_ROS_GENICAM2ROS_PUBLISHER_H
#define RC_VISARD_ROS_GENICAM2ROS_PUBLISHER_H

#include <string>

#include <rc_genicam_api/buffer.h>

namespace rc
{


/**
 * Interface for all publishers relating to images, point clouds or
 * other stereo-camera data
 */
class GenICam2RosPublisher
{
  public:

    /**
     * @param frame_id_prefix prefix for frame ids in published ros messages
     */
    GenICam2RosPublisher(std::string frame_id_prefix) : frame_id(
            frame_id_prefix + "camera")
    {}

    virtual ~GenICam2RosPublisher()
    {}

    /**
      Offers a buffer for publication. It depends on the the kind of buffer
      data and the implementation and configuration of the sub-class if the
      data is published.

      @param buffer      Buffer with data to be published.
      @param pixelformat The pixelformat as given by buffer.getPixelFormat().
    */

    virtual void publish(const rcg::Buffer *buffer, uint64_t pixelformat)=0;

    /**
      Returns true if there are subscribers to the topic.

      @return True if there are subscribers.
    */
    virtual bool used()=0;

  protected:

    std::string frame_id;

  private:

    GenICam2RosPublisher &operator=(const GenICam2RosPublisher &); // forbidden
};



}

#endif
