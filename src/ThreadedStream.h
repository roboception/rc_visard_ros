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

#ifndef RC_VISARD_DRIVER_THREADEDSTREAMER_H
#define RC_VISARD_DRIVER_THREADEDSTREAMER_H

#include <memory>
#include <thread>
#include <atomic>

#include <rc_dynamics_api/remote_interface.h>
#include <ros/ros.h>


namespace rc
{

/**
 * Convenience classes to implement and manage different types of
 * data streams in separate threads.
 *
 * Intended use:
 *  1 - create a manager
 *  2 - create different types of threaded streams
 *  3 - add them to the manager
 *  4 - start the threads
 *  5 - let the threaded streams to their work and supervise their state
 *  6 - stop the threads and join them
 *  7 - destroy manager (Re-use after joining some threads is not intended!!!)
 */
class ThreadedStream
{
  public:
    typedef std::shared_ptr<ThreadedStream> Ptr;

    class Manager: public std::enable_shared_from_this<Manager>
    {
      public:
        typedef std::shared_ptr<Manager> Ptr;

        static Ptr create();

        void add(ThreadedStream::Ptr stream);
        const std::list<ThreadedStream::Ptr>& get();

        void start_all();
        void stop_all();
        void join_all();

        bool all_succeeded() const;
        inline const std::atomic_bool& any_failed() const { return _any_failed; }

      protected:
        Manager();

        std::atomic_bool _any_failed;
        std::list<ThreadedStream::Ptr> _streams;

        friend ThreadedStream;
    };

    void start();
    void stop();
    void join();

    inline const std::string &name() const { return _stream; }
    inline const std::atomic_bool &requested() const { return _requested; }
    inline const std::atomic_bool &succeeded() const { return _success; }

  protected:
    ThreadedStream(rc::dynamics::RemoteInterface::Ptr rcdIface,
                   const std::string &stream, ros::NodeHandle &nh);

    /**
     * @return true, if stopped without fails
     */
    virtual bool startReceivingAndPublishingAsRos() = 0;

    virtual void work();

    std::atomic_bool _stop, _requested, _success;

    std::thread _thread;
    Manager::Ptr _manager;

    rc::dynamics::RemoteInterface::Ptr _rcdyn;
    std::string _stream;
    ros::NodeHandle _nh;
};

}


#endif //RC_VISARD_DRIVER_THREADEDSTREAMER_H
