/*
 * Copyright (c) 2017 Roboception GmbH
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

#include "ThreadedStream.h"

using namespace std;

namespace rc
{
namespace rcd = dynamics;

ThreadedStream::ThreadedStream(rc::dynamics::RemoteInterface::Ptr rcdIface, const std::string& stream,
                               ros::NodeHandle& nh)
  : _stop(false), _requested(false), _success(false), _rcdyn(rcdIface), _stream(stream), _nh(nh)
{
}

void ThreadedStream::start()
{
  _stop = false;
  _requested = true;
  _success = false;
  _thread = std::thread(&ThreadedStream::work, this);
}

void ThreadedStream::stop()
{
  _stop = true;
}

void ThreadedStream::join()
{
  if (_thread.joinable())
    _thread.join();
}

void ThreadedStream::work()
{
  if (!this->startReceivingAndPublishingAsRos())
  {
    _success = false;
    if (_manager)
      _manager->_any_failed = true;
    ROS_ERROR_STREAM("rc_visard_driver: rc-dynamics streaming failed: " << _stream);
  }
}

ThreadedStream::Manager::Ptr ThreadedStream::Manager::create()
{
  return Ptr(new Manager());
}

ThreadedStream::Manager::Manager() : _any_failed(false)
{
}

void ThreadedStream::Manager::add(ThreadedStream::Ptr stream)
{
  stream->_manager = shared_from_this();
  _streams.push_back(stream);
}

const std::list<ThreadedStream::Ptr>& ThreadedStream::Manager::get()
{
  return _streams;
}

void ThreadedStream::Manager::start_all()
{
  for (auto& s : _streams)
    s->start();
}

void ThreadedStream::Manager::stop_all()
{
  for (auto& s : _streams)
    s->stop();
}

void ThreadedStream::Manager::join_all()
{
  for (auto& s : _streams)
    s->join();
}

bool ThreadedStream::Manager::all_succeeded() const
{
  if (_streams.empty())
    return true;

  for (const auto& s : _streams)
    if (s->requested() && !s->succeeded())
      return false;

  return true;
}
}
