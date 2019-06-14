/*
 * Copyright (c) 2019 Roboception GmbH
 * All rights reserved
 *
 * Author: Heiko Hirschmueller, Christian Emmerich
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

#include "device_nodelet.h"

namespace rc
{
namespace rcd = dynamics;

// Anonymous namespace for local linkage
namespace
{
/// Commands taken by handleDynamicsStateChangeRequest()
enum class DynamicsCmd
{
  START = 0,
  START_SLAM,
  STOP,
  STOP_SLAM,
  RESTART,
  RESTART_SLAM,
  RESET_SLAM
};

///@return whether the service call has been accepted
void handleDynamicsStateChangeRequest(rcd::RemoteInterface::Ptr dynIF, DynamicsCmd state,
                                      std_srvs::Trigger::Response& resp)
{
  resp.success = true;
  resp.message = "";

  std::string new_state;

  if (dynIF)
  {
    try
    {
      switch (state)
      {
        case DynamicsCmd::STOP:
          new_state = dynIF->stop();
          break;
        case DynamicsCmd::STOP_SLAM:
          new_state = dynIF->stopSlam();
          break;
        case DynamicsCmd::START:
          new_state = dynIF->start();
          break;
        case DynamicsCmd::START_SLAM:
          new_state = dynIF->startSlam();
          break;
        case DynamicsCmd::RESTART_SLAM:
          new_state = dynIF->restartSlam();
          break;
        case DynamicsCmd::RESTART:
          new_state = dynIF->restart();
          break;
        case DynamicsCmd::RESET_SLAM:
          new_state = dynIF->resetSlam();
          break;
        default:
          throw std::runtime_error("handleDynamicsStateChangeRequest: unrecognized state change request");
      }
      if (new_state == rcd::RemoteInterface::State::FATAL)
      {
        resp.success = false;
        resp.message = "rc_dynamics module is in " + new_state + " state. Check the log files.";
      }
    }
    catch (std::exception& e)
    {
      resp.success = false;
      resp.message = std::string("Failed to change state of rcdynamics module: ") + e.what();
    }
  }
  else
  {
    resp.success = false;
    resp.message = "rcdynamics remote interface not yet initialized!";
  }

  if (!resp.success)
    ROS_ERROR_STREAM(resp.message);
}
}

bool DeviceNodelet::dynamicsStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  handleDynamicsStateChangeRequest(dynamicsInterface, DynamicsCmd::START, resp);
  return true;
}

bool DeviceNodelet::dynamicsStartSlam(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  handleDynamicsStateChangeRequest(dynamicsInterface, DynamicsCmd::START_SLAM, resp);
  return true;
}

bool DeviceNodelet::dynamicsRestart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  handleDynamicsStateChangeRequest(dynamicsInterface, DynamicsCmd::RESTART, resp);
  return true;
}

bool DeviceNodelet::dynamicsRestartSlam(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  handleDynamicsStateChangeRequest(dynamicsInterface, DynamicsCmd::RESTART_SLAM, resp);
  return true;
}

bool DeviceNodelet::dynamicsStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  handleDynamicsStateChangeRequest(dynamicsInterface, DynamicsCmd::STOP, resp);
  return true;
}

bool DeviceNodelet::dynamicsStopSlam(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  handleDynamicsStateChangeRequest(dynamicsInterface, DynamicsCmd::STOP_SLAM, resp);
  return true;
}

bool DeviceNodelet::dynamicsResetSlam(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  handleDynamicsStateChangeRequest(dynamicsInterface, DynamicsCmd::RESET_SLAM, resp);
  return true;
}

bool DeviceNodelet::getSlamTrajectory(rc_visard_driver::GetTrajectory::Request& req,
                                      rc_visard_driver::GetTrajectory::Response& resp)
{
  TrajectoryTime start(req.start_time.sec, req.start_time.nsec, req.start_time_relative);
  TrajectoryTime end(req.end_time.sec, req.end_time.nsec, req.end_time_relative);

  auto pbTraj = dynamicsInterface->getSlamTrajectory(start, end);
  resp.trajectory.header.frame_id = pbTraj.parent();
  resp.trajectory.header.stamp.sec = pbTraj.timestamp().sec();
  resp.trajectory.header.stamp.nsec = pbTraj.timestamp().nsec();

  for (auto pbPose : pbTraj.poses())
  {
    geometry_msgs::PoseStamped rosPose;
    rosPose.header.frame_id = pbTraj.parent();
    rosPose.header.stamp.sec = pbPose.timestamp().sec();
    rosPose.header.stamp.nsec = pbPose.timestamp().nsec();
    rosPose.pose.position.x = pbPose.pose().position().x();
    rosPose.pose.position.y = pbPose.pose().position().y();
    rosPose.pose.position.z = pbPose.pose().position().z();
    rosPose.pose.orientation.x = pbPose.pose().orientation().x();
    rosPose.pose.orientation.y = pbPose.pose().orientation().y();
    rosPose.pose.orientation.z = pbPose.pose().orientation().z();
    rosPose.pose.orientation.w = pbPose.pose().orientation().w();
    resp.trajectory.poses.push_back(rosPose);
  }

  // additionally publish extracted trajectory on topic
  if (autopublishTrajectory)
  {
    trajPublisher.publish(resp.trajectory);
  }

  return true;
}

bool DeviceNodelet::saveSlamMap(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.success = false;

  if (dynamicsInterface)
  {
    try
    {
      rcd::RemoteInterface::ReturnCode rc = dynamicsInterface->saveSlamMap();
      resp.success = (rc.value >= 0);
      resp.message = rc.message;
    }
    catch (std::exception& e)
    {
      resp.message = std::string("Failed to save SLAM map: ") + e.what();
    }
  }
  else
  {
    resp.message = "rcdynamics remote interface not yet initialized!";
  }

  if (!resp.success)
    ROS_ERROR_STREAM(resp.message);

  return true;
}

bool DeviceNodelet::loadSlamMap(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.success = false;

  if (dynamicsInterface)
  {
    try
    {
      rcd::RemoteInterface::ReturnCode rc = dynamicsInterface->loadSlamMap();
      resp.success = (rc.value >= 0);
      resp.message = rc.message;
    }
    catch (std::exception& e)
    {
      resp.message = std::string("Failed to load SLAM map: ") + e.what();
    }
  }
  else
  {
    resp.message = "rcdynamics remote interface not yet initialized!";
  }

  if (!resp.success)
    ROS_ERROR_STREAM(resp.message);

  return true;
}

bool DeviceNodelet::removeSlamMap(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.success = false;

  if (dynamicsInterface)
  {
    try
    {
      rcd::RemoteInterface::ReturnCode rc = dynamicsInterface->removeSlamMap();
      resp.success = (rc.value >= 0);
      resp.message = rc.message;
    }
    catch (std::exception& e)
    {
      resp.message = std::string("Failed to remove SLAM map: ") + e.what();
    }
  }
  else
  {
    resp.message = "rcdynamics remote interface not yet initialized!";
  }

  if (!resp.success)
    ROS_ERROR_STREAM(resp.message);

  return true;
}

} // namespace rc
