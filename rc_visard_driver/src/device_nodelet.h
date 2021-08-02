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

#ifndef RC_VISARD_DRIVERNODELET_H
#define RC_VISARD_DRIVERNODELET_H

#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <rc_visard_driver/rc_visard_driverConfig.h>
#include <rc_common_msgs/Trigger.h>

#include <GenApi/GenApi.h>
#include <rc_genicam_api/device.h>
#include <rc_dynamics_api/remote_interface.h>

#include <thread>
#include <mutex>
#include <atomic>

#include "protobuf2ros_stream.h"
#include "ThreadedStream.h"

#include <rc_visard_driver/GetTrajectory.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <tf2_ros/static_transform_broadcaster.h>

namespace rc
{
class DeviceNodelet : public nodelet::Nodelet
{
public:
  DeviceNodelet();
  virtual ~DeviceNodelet();

  virtual void onInit();

  /// Trigger stereo matching in mode 'SingleFrame'
  ///@return always true, check resp.return_code.value == 0
  bool depthAcquisitionTrigger(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Start Stereo INS
  ///@return always true, check resp.return_code.value == 0 for whether the dynamics service has been called
  bool dynamicsStart(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Start Stereo INS+SLAM
  ///@return always true, check resp.return_code.value == 0 for whether the dynamics service has been called
  bool dynamicsStartSlam(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Restart Stereo INS
  ///@return always true, check resp.return_code.value == 0 for whether the dynamics service has been called
  bool dynamicsRestart(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Restart Stereo INS+SLAM
  ///@return always true, check resp.return_code.value == 0 for whether the dynamics service has been called
  bool dynamicsRestartSlam(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Stop Stereo INS(+SLAM if running)
  ///@return always true, check resp.return_code.value == 0 for whether the dynamics service has been called
  bool dynamicsStop(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Stop SLAM (keep Stereo INS running)
  ///@return always true, check resp.return_code.value == 0 for whether the dynamics service has been called
  bool dynamicsStopSlam(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Reset SLAM (keep Stereo INS running)
  ///@return always true, check resp.return_code.value == 0 for whether the dynamics service has been called
  bool dynamicsResetSlam(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Get the Slam trajectory
  ///@return always true
  bool getSlamTrajectory(rc_visard_driver::GetTrajectory::Request& req,
                         rc_visard_driver::GetTrajectory::Response& resp);
  /// Save the onboard SLAM map
  ///@return always true, check resp.return_code.value == 0 for wheter map could be saved
  bool saveSlamMap(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Load the onboard SLAM map
  ///@return always true, check resp.return_code.value == 0 for wheter map could be loaded
  bool loadSlamMap(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);
  /// Remove the onboard SLAM map
  ///@return always true, check resp.return_code.value == 0 for wheter map could be removed
  bool removeSlamMap(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);


private:
  static ThreadedStream::Ptr CreateDynamicsStreamOfType(rc::dynamics::RemoteInterface::Ptr rcdIface,
                                                        const std::string& stream, ros::NodeHandle& nh,
                                                        const std::string& frame_id_prefix, bool tfEnabled,
                                                        bool staticImu2CamTf);

  void initConfiguration(const std::shared_ptr<GenApi::CNodeMapRef>& nodemap,
                         rc_visard_driver::rc_visard_driverConfig& cfg, rcg::Device::ACCESS access);

  void reconfigure(rc_visard_driver::rc_visard_driverConfig& config, uint32_t level);

  void grab(std::string device, rcg::Device::ACCESS access);

  void keepAliveAndRecoverFromFails();

  void produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void produce_device_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);


  dynamic_reconfigure::Server<rc_visard_driver::rc_visard_driverConfig>* reconfig;

  std::string color_format;

  bool dev_supports_gain;
  bool dev_supports_color;
  bool dev_supports_wb;
  bool dev_supports_depth_acquisition_trigger;
  bool dev_supports_chunk_data;
  bool dev_supports_double_shot;

  bool perform_depth_acquisition_trigger;

  std::shared_ptr<rcg::Device> rcgdev;
  std::shared_ptr<GenApi::CNodeMapRef> rcgnodemap;

  boost::recursive_mutex mtx;
  bool stereo_plus_avail;
  bool iocontrol_avail;
  rc_visard_driver::rc_visard_driverConfig config;
  std::atomic_uint_least32_t level;

  std::thread imageThread;
  std::atomic_bool stopImageThread, imageRequested, imageSuccess;

  std::thread recoverThread;
  std::atomic_bool stopRecoverThread;
  bool recoveryRequested;
  int cntConsecutiveRecoveryFails;
  int maxNumRecoveryTrials;
  bool atLeastOnceSuccessfullyStarted;

  ThreadedStream::Manager::Ptr dynamicsStreams;

  /// wrapper for REST-API calls relating to rc_visard's dynamics interface
  ros::ServiceServer depthAcquisitionTriggerService;
  rc::dynamics::RemoteInterface::Ptr dynamicsInterface;
  ros::ServiceServer dynamicsStartService;
  ros::ServiceServer dynamicsStartSlamService;
  ros::ServiceServer dynamicsRestartService;
  ros::ServiceServer dynamicsRestartSlamService;
  ros::ServiceServer dynamicsStopService;
  ros::ServiceServer dynamicsStopSlamService;
  ros::ServiceServer dynamicsResetSlamService;
  ros::ServiceServer getSlamTrajectoryService;
  ros::ServiceServer slamSaveMapService;
  ros::ServiceServer slamLoadMapService;
  ros::ServiceServer slamRemoveMapService;
  ros::Publisher trajPublisher;
  tf2_ros::StaticTransformBroadcaster tfStaticBroadcaster;
  bool autostartDynamics, autostopDynamics, autostartSlam, autopublishTrajectory;

  /// all frame names must be prefixed when using more than one rc_visard
  std::string tfPrefix;

  /// should poses published also via tf?
  bool tfEnabled;

  /// diagnostics publishing
  diagnostic_updater::Updater updater;
  std::string dev_serialno, dev_macaddr, dev_ipaddr, dev_version, gev_userid, gev_packet_size;
  unsigned int totalCompleteBuffers, totalIncompleteBuffers, totalImageReceiveTimeouts, totalConnectionLosses;
};
}

#endif
