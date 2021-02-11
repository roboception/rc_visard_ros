/*
 * Copyright (c) 2017 Roboception GmbH
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
#include "publishers/camera_info_publisher.h"
#include "publishers/camera_param_publisher.h"
#include "publishers/image_publisher.h"
#include "publishers/disparity_publisher.h"
#include "publishers/disparity_color_publisher.h"
#include "publishers/depth_publisher.h"
#include "publishers/confidence_publisher.h"
#include "publishers/error_disparity_publisher.h"
#include "publishers/error_depth_publisher.h"
#include "publishers/points2_publisher.h"

#include "publishers/protobuf2ros_conversions.h"

#include <rc_genicam_api/device.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/buffer.h>
#include <rc_genicam_api/config.h>
#include <rc_genicam_api/pixel_formats.h>

#include <rc_dynamics_api/trajectory_time.h>

#include <pluginlib/class_list_macros.h>
#include <exception>

#include <sstream>
#include <stdexcept>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <rc_common_msgs/ReturnCodeConstants.h>
#include <rc_common_msgs/CameraParam.h>

namespace {

  /// Returns true if vendor name and model name indicate that genicam device is an rc_visard
  bool isRcVisardDevice(const std::string& vendor, const std::string& model)
  {
    bool isKuka3DPerception = vendor.find("KUKA") != std::string::npos && model.find("3d_perception") != std::string::npos;
    bool isRcVisard = model.find("rc_visard") != std::string::npos;
    return isKuka3DPerception || isRcVisard;
  }

  /// Iterates through all interfaces and looks for rc_visard devices. Returns all found device ids.
  std::vector<std::string> discoverRcVisardDevices()
  {
    std::vector<std::string> device_ids;
    for (auto system : rcg::System::getSystems())
    {
      system->open();
      for (auto interf : system->getInterfaces())
      {
        interf->open();
        for (auto dev : interf->getDevices())
        {
          if (isRcVisardDevice(dev->getVendor(), dev->getModel()))
          {
            device_ids.push_back(dev->getID());
          }
        }
        interf->close();
      }
      system->close();
    }
    return device_ids;
  }
}

namespace rc
{
namespace rcd = dynamics;
using rc_common_msgs::ReturnCodeConstants;

ThreadedStream::Ptr DeviceNodelet::CreateDynamicsStreamOfType(rcd::RemoteInterface::Ptr rcdIface,
                                                              const std::string& stream, ros::NodeHandle& nh,
                                                              const std::string& frame_id_prefix, bool tfEnabled,
                                                              bool staticImu2CamTf)
{
  if (stream == "pose")
  {
    return ThreadedStream::Ptr(new PoseAndTFStream(rcdIface, stream, nh, frame_id_prefix, tfEnabled));
  }
  if (stream == "pose_ins" || stream == "pose_rt" || stream == "pose_rt_ins" || stream == "imu")
  {
    return ThreadedStream::Ptr(new Protobuf2RosStream(rcdIface, stream, nh, frame_id_prefix));
  }
  if (stream == "dynamics" || stream == "dynamics_ins")
  {
    return ThreadedStream::Ptr(new DynamicsStream(rcdIface, stream, nh, frame_id_prefix, !staticImu2CamTf));
  }

  throw std::runtime_error(std::string("Not yet implemented! Stream type: ") + stream);
}

DeviceNodelet::DeviceNodelet()
{
  reconfig = 0;
  dev_supports_gain = false;
  dev_supports_color = false;
  dev_supports_chunk_data = false;
  dev_supports_wb = false;
  dev_supports_depth_acquisition_trigger = false;
  perform_depth_acquisition_trigger = false;
  iocontrol_avail = false;
  dev_supports_double_shot = false;
  level = 0;

  stopImageThread = imageRequested = imageSuccess = false;

  dynamicsStreams = ThreadedStream::Manager::create();

  stopRecoverThread = false;
  recoveryRequested = true;
  cntConsecutiveRecoveryFails = -1;  // first time not giving any warnings
  atLeastOnceSuccessfullyStarted = false;
  totalCompleteBuffers = 0;
  totalIncompleteBuffers = 0;
  totalConnectionLosses = 0;
  totalImageReceiveTimeouts = 0;
}

DeviceNodelet::~DeviceNodelet()
{
  std::cout << "rc_visard_driver: Shutting down" << std::endl;

  // signal running threads and wait until they finish

  stopImageThread = true;
  dynamicsStreams->stop_all();
  stopRecoverThread = true;

  if (imageThread.joinable())
    imageThread.join();
  dynamicsStreams->join_all();
  if (recoverThread.joinable())
    recoverThread.join();

  delete reconfig;

  rcg::System::clearSystems();
}

void DeviceNodelet::onInit()
{
  // run initialization and recover routine in separate thread
  recoverThread = std::thread(&DeviceNodelet::keepAliveAndRecoverFromFails, this);

  // add callbacks for diagnostics publishing
  updater.add("Connection", this, &DeviceNodelet::produce_connection_diagnostics);
  updater.add("Device", this, &DeviceNodelet::produce_device_diagnostics);
}

void DeviceNodelet::keepAliveAndRecoverFromFails()
{
  // get parameter configuration

  ros::NodeHandle pnh(getPrivateNodeHandle());

  // device defaults to empty string which serves as a wildcard to connect to any
  // rc_visard as long as only one sensor is available in the network
  std::string device = "";
  std::string access = "control";
  maxNumRecoveryTrials = 5;

  tfEnabled = false;
  autostartDynamics = autostopDynamics = autostartSlam = autopublishTrajectory = false;
  std::string ns = tf::strip_leading_slash(ros::this_node::getNamespace());
  tfPrefix = (ns != "") ? ns + "_" : "";

  pnh.param("device", device, device);
  pnh.param("gev_access", access, access);
  pnh.param("max_reconnects", maxNumRecoveryTrials, maxNumRecoveryTrials);
  pnh.param("enable_tf", tfEnabled, tfEnabled);
  pnh.param("autostart_dynamics", autostartDynamics, autostartDynamics);
  pnh.param("autostart_dynamics_with_slam", autostartSlam, autostartSlam);
  pnh.param("autostop_dynamics", autostopDynamics, autostopDynamics);
  pnh.param("autopublish_trajectory", autopublishTrajectory, autopublishTrajectory);

  rcg::Device::ACCESS access_id;
  if (access == "exclusive")
  {
    access_id = rcg::Device::EXCLUSIVE;
  }
  else if (access == "control")
  {
    access_id = rcg::Device::CONTROL;
  }
  else if (access == "off")
  {
    access_id = rcg::Device::READONLY;
  }
  else
  {
    ROS_FATAL_STREAM("rc_visard_driver: Access must be 'control', 'exclusive' or 'off': " << access);
    return;
  }

  // setup service for depth acquisition trigger

  depthAcquisitionTriggerService = pnh.advertiseService("depth_acquisition_trigger", &DeviceNodelet::depthAcquisitionTrigger, this);

  // setup services for starting and stopping rcdynamics module

  dynamicsStartService = pnh.advertiseService("dynamics_start", &DeviceNodelet::dynamicsStart, this);
  dynamicsStartSlamService = pnh.advertiseService("dynamics_start_slam", &DeviceNodelet::dynamicsStartSlam, this);
  dynamicsRestartService = pnh.advertiseService("dynamics_restart", &DeviceNodelet::dynamicsRestart, this);
  dynamicsRestartSlamService = pnh.advertiseService("dynamics_restart_slam", &DeviceNodelet::dynamicsRestartSlam, this);
  dynamicsStopService = pnh.advertiseService("dynamics_stop", &DeviceNodelet::dynamicsStop, this);
  dynamicsStopSlamService = pnh.advertiseService("dynamics_stop_slam", &DeviceNodelet::dynamicsStopSlam, this);

  dynamicsResetSlamService = pnh.advertiseService("slam_reset", &DeviceNodelet::dynamicsResetSlam, this);
  getSlamTrajectoryService = pnh.advertiseService("slam_get_trajectory", &DeviceNodelet::getSlamTrajectory, this);
  slamSaveMapService = pnh.advertiseService("slam_save_map", &DeviceNodelet::saveSlamMap, this);
  slamLoadMapService = pnh.advertiseService("slam_load_map", &DeviceNodelet::loadSlamMap, this);
  slamRemoveMapService = pnh.advertiseService("slam_remove_map", &DeviceNodelet::removeSlamMap, this);

  if (autopublishTrajectory)
  {
    trajPublisher = getNodeHandle().advertise<nav_msgs::Path>("trajectory", 10);
  }

  // run start-keep-alive-and-recover loop

  while (!stopRecoverThread && (
          (maxNumRecoveryTrials < 0) ||
          (cntConsecutiveRecoveryFails <= maxNumRecoveryTrials)
        ))
  {
    // check if everything is running smoothly

    if (!recoveryRequested)
    {
      bool allSucceeded = (!imageRequested || imageSuccess);
      if ((cntConsecutiveRecoveryFails > 0) && allSucceeded)
      {
        cntConsecutiveRecoveryFails = 0;
        ROS_INFO("rc_visard_driver: Device successfully recovered from previous fail(s)!");
      }

      updater.update(); // regularly update the status for publishing diagnostics  (rate limited by parameter '~diagnostic_period')
      usleep(1000 * 100);
      continue;
    }

    // it's not running smoothly, we need recovery

    cntConsecutiveRecoveryFails++;
    if (cntConsecutiveRecoveryFails==1) {
      totalConnectionLosses++;
    }
    updater.force_update(); // immediately update the diagnostics status

    // stop image and dynamics threads

    stopImageThread = true;
    dynamicsStreams->stop_all();

    if (imageThread.joinable())
      imageThread.join();
    dynamicsStreams->join_all();

    // try discover, open and bring up device

    bool successfullyOpened = false;
    while (!stopRecoverThread && !successfullyOpened && (
          (maxNumRecoveryTrials < 0) ||
          (cntConsecutiveRecoveryFails <= maxNumRecoveryTrials)
        ))
    {
      // if we are recovering, put warning and wait before retrying

      if (cntConsecutiveRecoveryFails > 0)
      {
        ROS_ERROR_STREAM("rc_visard_driver: Failed or lost connection. Trying to recover"
                         " rc_visard_driver from failed state ("
                         << cntConsecutiveRecoveryFails
                         << ((maxNumRecoveryTrials>=0) ? std::string("/") + std::to_string(maxNumRecoveryTrials) : "" )
                         << ")...");
        usleep(1000 * 500);
      }

      try
      {
        if (rcgdev)
        {
          rcgdev->close();
        }

        if (device.size() == 0)
        {
          ROS_INFO("No device ID given in the private parameter 'device'! Trying to auto-detect a single rc_visard device in the network...");
          auto device_ids = discoverRcVisardDevices();
          if (device_ids.size() != 1)
          {
            updater.force_update();
            throw std::runtime_error("Auto-connection with rc_visard device failed because none or multiple devices were found!");
          }
          ROS_INFO_STREAM("Found rc_visard device '" << device_ids[0] << "'");
          rcgdev = rcg::getDevice(device_ids[0].c_str());
        } else {
          rcgdev = rcg::getDevice(device.c_str());
        }

        if (!rcgdev)
        {
          updater.force_update();
          throw std::invalid_argument("Unknown or non-unique device '" + device + "'");
        }

        ROS_INFO_STREAM("rc_visard_driver: Opening connection to '" << rcgdev->getID() << "'");
        rcgdev->open(access_id);
        rcgnodemap = rcgdev->getRemoteNodeMap();

        // in newer rc_visard image versions, we can check via genicam if rc_visard is ready

        try {
          bool isReady = rcg::getBoolean(rcgnodemap, "RcSystemReady", true, true);
          if (!isReady)
          {
            ROS_INFO_STREAM("rc_visard_driver: rc_visard device not yet ready (GEV). Trying again...");
            continue;  // to next trial!
          }
        } catch (std::invalid_argument& e)
        {
          // genicam feature is not implemented in this version of rc_visard's firmware,
          // so we have to assume rc_visard is ready and continue
        }

        // extract some diagnostics data from device
        dev_serialno = rcg::getString(rcgnodemap, "DeviceID", true);
        dev_macaddr = rcg::getString(rcgnodemap, "GevMACAddress", true);
        dev_ipaddr = rcg::getString(rcgnodemap, "GevCurrentIPAddress", true);
        dev_version = rcg::getString(rcgnodemap, "DeviceVersion", true);
        gev_userid = rcg::getString(rcgnodemap, "DeviceUserID", true);
        gev_packet_size = rcg::getString(rcgnodemap, "GevSCPSPacketSize", true);

        updater.setHardwareID(dev_serialno);
        updater.force_update();

        // instantiating dynamics interface, check if ready, and autostart dynamics on sensor if desired

        dynamicsInterface = rcd::RemoteInterface::create(dev_ipaddr);
        try {
          if (!dynamicsInterface->checkSystemReady())
          {
            ROS_INFO_STREAM("rc_visard_driver: rc_visard device not yet ready (REST). Trying again...");
            continue;  // to next trial!
          }
        } catch (std::exception& e)
        {
          ROS_ERROR_STREAM("rc_visard_driver: checking system ready failed: " << e.what());
        }

        if ((autostartDynamics || autostartSlam) && !atLeastOnceSuccessfullyStarted)
        {
          ROS_INFO_STREAM("rc_visard_driver: Auto-start requested (" <<
                          "autostart_dynamics=" << autostartDynamics << ", "
                          "autostart_dynamics_with_slam=" << autostartSlam << ")");

          rc_common_msgs::Trigger::Request dummyreq;
          rc_common_msgs::Trigger::Response dummyresp;

          bool autostart_failed = false;
          autostart_failed |= (autostartSlam     && !this->dynamicsStartSlam(dummyreq, dummyresp));
          autostart_failed |= (!autostartSlam && autostartDynamics && !this->dynamicsStart(dummyreq, dummyresp));
          autostart_failed |= dummyresp.return_code.value < ReturnCodeConstants::SUCCESS;

          if (autostart_failed)
          {
            ROS_WARN_STREAM("rc_visard_driver: Could not auto-start dynamics module! " << dummyresp.return_code.message);
            cntConsecutiveRecoveryFails++;
            continue;  // to next trial!
          }
        }

        // get and publish static transform between IMU and camera (if feature available)

        bool staticImu2CamTf = false;
        try {
          auto pbFrame = dynamicsInterface->getCam2ImuTransform();

          // first prefix all frame ids
          pbFrame.set_name(tfPrefix + pbFrame.name());
          pbFrame.set_parent(tfPrefix + pbFrame.parent());

          // invert cam2imu and convert to tf transform
          auto imu2cam = tf::StampedTransform(
                            toRosTfTransform(pbFrame.pose().pose()).inverse(),
                            toRosTime(pbFrame.pose().timestamp()),
                            pbFrame.name(), pbFrame.parent());
          // send it
          geometry_msgs::TransformStamped msg;
          tf::transformStampedTFToMsg(imu2cam, msg);
          tfStaticBroadcaster.sendTransform(msg);
          staticImu2CamTf = true;

        } catch (rcd::RemoteInterface::NotAvailable& e)
        {
          ROS_WARN_STREAM("rc_visard_driver: Could not get and publish static transformation between camera and IMU "
                          << "because feature is not avilable in that rc_visard firmware version. Please update the firmware image "
                          << "or subscribe to the dynamics topic to have that transformation available in tf.");
        }

        // add streaming thread for each available stream on rc_visard device

        auto availStreams = dynamicsInterface->getAvailableStreams();
        dynamicsStreams = ThreadedStream::Manager::create();
        for (const auto& streamName : availStreams)
        {
          try
          {
            auto newStream =
                CreateDynamicsStreamOfType(dynamicsInterface, streamName, getNodeHandle(), tfPrefix, tfEnabled, staticImu2CamTf);
            dynamicsStreams->add(newStream);
          }
          catch (const std::exception& e)
          {
            ROS_WARN_STREAM("rc_visard_driver: Unable to create dynamics stream of type " << streamName << ": "
                                                                                          << e.what());
          }
        }

        successfullyOpened = true;
        atLeastOnceSuccessfullyStarted = true;
      }
      catch (std::exception& ex)
      {
        cntConsecutiveRecoveryFails++;
        ROS_ERROR_STREAM("rc_visard_driver: " << ex.what());
      }
    }
    if (stopRecoverThread)
      break;

    if ( (maxNumRecoveryTrials >= 0) && (cntConsecutiveRecoveryFails > maxNumRecoveryTrials) )
    {
      ROS_FATAL_STREAM("rc_visard_driver: could not recover from failed state!");
      break;
    }

    // start grabbing threads

    recoveryRequested = false;
    if (access_id != rcg::Device::READONLY)
    {
      imageThread = std::thread(&DeviceNodelet::grab, this, device, access_id);
    }
    dynamicsStreams->start_all();

  }

  if (autostopDynamics)
  {
    rc_common_msgs::Trigger::Request dummyreq;
    rc_common_msgs::Trigger::Response dummyresp;
    ROS_INFO("rc_visard_driver: Autostop dynamics ...");
    if (!this->dynamicsStop(dummyreq, dummyresp))
    {  // autostop failed!
      ROS_WARN("rc_visard_driver: Could not auto-stop dynamics module!");
    }
  }
  std::cout << "rc_visard_driver: stopped." << std::endl;
}

void DeviceNodelet::initConfiguration(const std::shared_ptr<GenApi::CNodeMapRef>& nodemap,
                                      rc_visard_driver::rc_visard_driverConfig& cfg, rcg::Device::ACCESS access)
{
  ros::NodeHandle pnh(getPrivateNodeHandle());

  // get current camera configuration

  cfg.camera_fps = rcg::getFloat(nodemap, "AcquisitionFrameRate", 0, 0, true);

  std::string v = rcg::getEnum(nodemap, "ExposureAuto", true);
  cfg.camera_exp_auto = (v != "Off");

  if (cfg.camera_exp_auto)
  {
    if (v == "Continuous")
    {
      cfg.camera_exp_auto_mode = "Normal";
    }
    else
    {
      cfg.camera_exp_auto_mode = v;
    }
  }

  cfg.camera_exp_value = rcg::getFloat(nodemap, "ExposureTime", 0, 0, true) / 1000000;
  cfg.camera_exp_max = rcg::getFloat(nodemap, "ExposureTimeAutoMax", 0, 0, true) / 1000000;

  // get optional exposure region
  cfg.camera_exp_width = rcg::getInteger(nodemap, "ExposureRegionWidth", 0, 0, false);
  cfg.camera_exp_height = rcg::getInteger(nodemap, "ExposureRegionHeight", 0, 0, false);
  cfg.camera_exp_offset_x = rcg::getInteger(nodemap, "ExposureRegionOffsetX", 0, 0, false);
  cfg.camera_exp_offset_y = rcg::getInteger(nodemap, "ExposureRegionOffsetY", 0, 0, false);

  // get optional gain value

  v = rcg::getEnum(nodemap, "GainSelector", false);
  if (v.size() > 0)
  {
    dev_supports_gain = true;

    if (v != "All")
    {
      dev_supports_gain = rcg::setEnum(nodemap, "GainSelector", "All", true);
    }

    if (dev_supports_gain)
    {
      cfg.camera_gain_value = rcg::getFloat(nodemap, "Gain", 0, 0, true);
    }
  }
  else
  {
    ROS_WARN("rc_visard_driver: Device does not support setting gain. gain_value is without function.");

    dev_supports_gain = false;
    cfg.camera_gain_value = 0;
  }

  // check if camera is color camera

  std::vector<std::string> formats;
  rcg::setEnum(rcgnodemap, "ComponentSelector", "Intensity", true);
  rcg::getEnum(rcgnodemap, "PixelFormat", formats, true);
  for (auto&& format : formats)
  {
    if (format == "YCbCr411_8")
    {
      dev_supports_color = true;
      break;
    }
  }

  // get optional white balancing values (only for color camera)

  v = rcg::getEnum(nodemap, "BalanceWhiteAuto", false);
  if (dev_supports_color && v.size() > 0)
  {
    dev_supports_wb = true;
    cfg.camera_wb_auto = (v != "Off");
    rcg::setEnum(nodemap, "BalanceRatioSelector", "Red", true);
    cfg.camera_wb_ratio_red = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, true);
    rcg::setEnum(nodemap, "BalanceRatioSelector", "Blue", true);
    cfg.camera_wb_ratio_blue = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, true);
  }
  else
  {
    ROS_WARN("rc_visard_driver: Not a color camera. wb_auto, wb_ratio_red and wb_ratio_blue are without function.");

    dev_supports_wb = false;
    cfg.camera_wb_auto = true;
    cfg.camera_wb_ratio_red = 1;
    cfg.camera_wb_ratio_blue = 1;
  }

  // get current depth image configuration

  v = rcg::getEnum(nodemap, "DepthAcquisitionMode", false);
  if (v.size() > 0)
  {
    dev_supports_depth_acquisition_trigger = true;
    cfg.depth_acquisition_mode = v;
  }
  else
  {
    ROS_WARN("rc_visard_driver: Device does not support triggering depth images. depth_acquisition_mode is without function.");

    dev_supports_depth_acquisition_trigger = false;
    cfg.depth_acquisition_mode = "Continuous";
  }

  v = rcg::getEnum(nodemap, "DepthQuality", true);
  cfg.depth_quality = v;

  cfg.depth_static_scene = rcg::getBoolean(nodemap, "DepthStaticScene", false);
  cfg.depth_seg = rcg::getInteger(nodemap, "DepthSeg", 0, 0, true);
  cfg.depth_fill = rcg::getInteger(nodemap, "DepthFill", 0, 0, true);
  cfg.depth_minconf = rcg::getFloat(nodemap, "DepthMinConf", 0, 0, true);
  cfg.depth_mindepth = rcg::getFloat(nodemap, "DepthMinDepth", 0, 0, true);
  cfg.depth_maxdepth = rcg::getFloat(nodemap, "DepthMaxDepth", 0, 0, true);
  cfg.depth_maxdeptherr = rcg::getFloat(nodemap, "DepthMaxDepthErr", 0, 0, true);

  cfg.ptp_enabled = rcg::getBoolean(nodemap, "GevIEEE1588", false);

  // fix for rc_visard < 1.5

  if (cfg.depth_quality[0] == 'S')
  {
    cfg.depth_quality = "High";
    cfg.depth_static_scene = true;
  }

  // check for stereo_plus license

  try
  {
    cfg.depth_smooth = rcg::getBoolean(nodemap, "DepthSmooth", true);
    stereo_plus_avail = nodemap->_GetNode("DepthSmooth")->GetAccessMode() == GenApi::RW;

    if (!stereo_plus_avail)
    {
      ROS_INFO("rc_visard_driver: License for stereo_plus not available, disabling depth_quality=Full and depth_smooth.");
    }
  }
  catch (const std::exception&)
  {
    ROS_WARN("rc_visard_driver: rc_visard has an older firmware, disabling depth_quality=Full and depth_smooth.");
    stereo_plus_avail = false;
  }

  try
  {
    cfg.depth_double_shot = rcg::getBoolean(nodemap, "DepthDoubleShot", true);
    dev_supports_double_shot = true;
  }
  catch (const std::exception&)
  {
    dev_supports_double_shot = false;
    ROS_WARN("rc_visard_driver: rc_visard has an older firmware, depth_double_shot is not available.");
  }

  // check if io-control is available and get values

  try
  {
    // disable filtering images on the sensor

    rcg::setEnum(nodemap, "AcquisitionAlternateFilter", "Off", false);

    // get current values

    rcg::setEnum(nodemap, "LineSelector", "Out1", true);
    cfg.out1_mode = rcg::getString(nodemap, "LineSource", true);

    rcg::setEnum(nodemap, "LineSelector", "Out2", true);
    cfg.out2_mode = rcg::getString(nodemap, "LineSource", true);

    // check if license for IO control functions is available

    iocontrol_avail = nodemap->_GetNode("LineSource")->GetAccessMode() == GenApi::RW;

    if (!iocontrol_avail)
    {
      ROS_INFO("rc_visard_driver: License for iocontrol module not available, disabling out1_mode and out2_mode.");
    }
  }
  catch (const std::exception&)
  {
    ROS_WARN("rc_visard_driver: rc_visard has an older firmware, IO control functions are not available.");

    cfg.out1_mode = "ExposureActive";
    cfg.out2_mode = "Low";

    iocontrol_avail = false;
  }

  // enabling chunk data for getting live camera/image parameters
  // (e.g. line status, current gain and exposure, etc.)

  dev_supports_chunk_data = rcg::setBoolean(nodemap, "ChunkModeActive", true, false);
  if (!dev_supports_chunk_data)
  {
    ROS_WARN("rc_visard_driver: rc_visard has an older firmware that does not support chunk data.");

  }

  // try to get ROS parameters: if parameter is not set in parameter server, default to current sensor configuration

  pnh.param("camera_fps", cfg.camera_fps, cfg.camera_fps);
  pnh.param("camera_exp_auto", cfg.camera_exp_auto, cfg.camera_exp_auto);
  pnh.param("camera_exp_auto_mode", cfg.camera_exp_auto_mode, cfg.camera_exp_auto_mode);
  pnh.param("camera_exp_value", cfg.camera_exp_value, cfg.camera_exp_value);
  pnh.param("camera_gain_value", cfg.camera_gain_value, cfg.camera_gain_value);
  pnh.param("camera_exp_max", cfg.camera_exp_max, cfg.camera_exp_max);
  pnh.param("camera_exp_width", cfg.camera_exp_width, cfg.camera_exp_width);
  pnh.param("camera_exp_height", cfg.camera_exp_height, cfg.camera_exp_height);
  pnh.param("camera_exp_offset_x", cfg.camera_exp_offset_x, cfg.camera_exp_offset_x);
  pnh.param("camera_exp_offset_y", cfg.camera_exp_offset_y, cfg.camera_exp_offset_y);
  pnh.param("camera_wb_auto", cfg.camera_wb_auto, cfg.camera_wb_auto);
  pnh.param("camera_wb_ratio_red", cfg.camera_wb_ratio_red, cfg.camera_wb_ratio_red);
  pnh.param("camera_wb_ratio_blue", cfg.camera_wb_ratio_blue, cfg.camera_wb_ratio_blue);
  pnh.param("depth_acquisition_mode", cfg.depth_acquisition_mode, cfg.depth_acquisition_mode);
  pnh.param("depth_quality", cfg.depth_quality, cfg.depth_quality);
  pnh.param("depth_static_scene", cfg.depth_static_scene, cfg.depth_static_scene);
  pnh.param("depth_double_shot", cfg.depth_double_shot, cfg.depth_double_shot);
  pnh.param("depth_seg", cfg.depth_seg, cfg.depth_seg);
  pnh.param("depth_smooth", cfg.depth_smooth, cfg.depth_smooth);
  pnh.param("depth_fill", cfg.depth_fill, cfg.depth_fill);
  pnh.param("depth_minconf", cfg.depth_minconf, cfg.depth_minconf);
  pnh.param("depth_mindepth", cfg.depth_mindepth, cfg.depth_mindepth);
  pnh.param("depth_maxdepth", cfg.depth_maxdepth, cfg.depth_maxdepth);
  pnh.param("depth_maxdeptherr", cfg.depth_maxdeptherr, cfg.depth_maxdeptherr);
  pnh.param("ptp_enabled", cfg.ptp_enabled, cfg.ptp_enabled);
  pnh.param("out1_mode", cfg.out1_mode, cfg.out1_mode);
  pnh.param("out2_mode", cfg.out2_mode, cfg.out2_mode);

  // set parameters on parameter server so that dynamic reconfigure picks them up

  pnh.setParam("camera_fps", cfg.camera_fps);
  pnh.setParam("camera_exp_auto", cfg.camera_exp_auto);
  pnh.setParam("camera_exp_auto_mode", cfg.camera_exp_auto_mode);
  pnh.setParam("camera_exp_value", cfg.camera_exp_value);
  pnh.setParam("camera_gain_value", cfg.camera_gain_value);
  pnh.setParam("camera_exp_max", cfg.camera_exp_max);
  pnh.setParam("camera_exp_width", cfg.camera_exp_width);
  pnh.setParam("camera_exp_height", cfg.camera_exp_height);
  pnh.setParam("camera_exp_offset_x", cfg.camera_exp_offset_x);
  pnh.setParam("camera_exp_offset_y", cfg.camera_exp_offset_y);
  pnh.setParam("camera_wb_auto", cfg.camera_wb_auto);
  pnh.setParam("camera_wb_ratio_red", cfg.camera_wb_ratio_red);
  pnh.setParam("camera_wb_ratio_blue", cfg.camera_wb_ratio_blue);
  pnh.setParam("depth_acquisition_mode", cfg.depth_acquisition_mode);
  pnh.setParam("depth_quality", cfg.depth_quality);
  pnh.setParam("depth_static_scene", cfg.depth_static_scene);
  pnh.setParam("depth_double_shot", cfg.depth_double_shot);
  pnh.setParam("depth_seg", cfg.depth_seg);
  pnh.setParam("depth_smooth", cfg.depth_smooth);
  pnh.setParam("depth_fill", cfg.depth_fill);
  pnh.setParam("depth_minconf", cfg.depth_minconf);
  pnh.setParam("depth_mindepth", cfg.depth_mindepth);
  pnh.setParam("depth_maxdepth", cfg.depth_maxdepth);
  pnh.setParam("depth_maxdeptherr", cfg.depth_maxdeptherr);
  pnh.setParam("ptp_enabled", cfg.ptp_enabled);
  pnh.setParam("out1_mode", cfg.out1_mode);
  pnh.setParam("out2_mode", cfg.out2_mode);

  if (reconfig == 0)
  {
    // TODO: we need to dismangle initialization of dynreconfserver from not-READONLY-access-condition
    reconfig = new dynamic_reconfigure::Server<rc_visard_driver::rc_visard_driverConfig>(mtx, pnh);
  }
  // always set callback to (re)load configuration even after recovery
  dynamic_reconfigure::Server<rc_visard_driver::rc_visard_driverConfig>::CallbackType cb;
  cb = boost::bind(&DeviceNodelet::reconfigure, this, _1, _2);
  reconfig->setCallback(cb);
}

void DeviceNodelet::reconfigure(rc_visard_driver::rc_visard_driverConfig& c, uint32_t l)
{
  boost::recursive_mutex::scoped_lock lock(mtx);

  // check and correct parameters

  if (!dev_supports_gain)
  {
    c.camera_gain_value = 0;
    l &= ~8192;
  }

  if (!dev_supports_double_shot)
  {
    c.depth_double_shot = false;
    l &= ~32;
  }

  c.camera_gain_value = round(c.camera_gain_value / 6) * 6;

  if (!dev_supports_wb)
  {
    c.camera_wb_auto = true;
    c.camera_wb_ratio_red = 1;
    c.camera_wb_ratio_blue = 1;
    l &= ~(16384 | 32768 | 65536);
  }

  if (!dev_supports_depth_acquisition_trigger)
  {
    c.depth_acquisition_mode = "Continuous";
    l &= ~1048576;
  }

  if (c.depth_quality[0] == 'L')
  {
    c.depth_quality = "Low";
  }
  else if (c.depth_quality[0] == 'M')
  {
    c.depth_quality = "Medium";
  }
  else if (c.depth_quality[0] == 'F' && stereo_plus_avail)
  {
    c.depth_quality = "Full";
  }
  else
  {
    c.depth_quality = "High";
  }

  if (!stereo_plus_avail)
  {
    c.depth_smooth=false;
    l &= ~4194304;
  }

  if (iocontrol_avail)
  {
    if (c.out1_mode != "Low" && c.out1_mode != "High" && c.out1_mode != "ExposureActive" &&
        c.out1_mode != "ExposureAlternateActive")
    {
      c.out1_mode = "Low";
    }

    if (c.out2_mode != "Low" && c.out2_mode != "High" && c.out2_mode != "ExposureActive" &&
        c.out2_mode != "ExposureAlternateActive")
    {
      c.out2_mode = "Low";
    }
  }
  else
  {
    c.out1_mode = "Low";
    c.out2_mode = "Low";
  }

  // if out1_mode or out2_mode are changed, immediately set new value here
  // If we do this in the grabbing thread we have a race condition and another parameter update
  if (l & 262144)
  {
    l &= ~262144;

    if (iocontrol_avail && rcgnodemap)
    {
      rcg::setEnum(rcgnodemap, "LineSelector", "Out1", false);
      rcg::setEnum(rcgnodemap, "LineSource", c.out1_mode.c_str(), false);
      ROS_DEBUG_STREAM("Set LineSource Out1 to " << c.out1_mode);
    }
  }

  if (l & 524288)
  {
    l &= ~524288;

    if (iocontrol_avail && rcgnodemap)
    {
      rcg::setEnum(rcgnodemap, "LineSelector", "Out2", false);
      rcg::setEnum(rcgnodemap, "LineSource", c.out2_mode.c_str(), false);
      ROS_DEBUG_STREAM("Set LineSource Out2 to " << c.out2_mode);
    }
  }

  // If camera_exp_auto (ExposureAuto) is changed, immediately set new value here
  // so that ExposureTime and Gain can be read and published again.
  // If we do this in the grabbing thread we have a race condition and another parameter update
  // in the meantime could be lost because we need to overwrite the config with the updated time and gain values.
  if (l & 2)
  {
    l &= ~2;

    if (rcgnodemap)
    {
      if (c.camera_exp_auto)
      {
        std::string mode = c.camera_exp_auto_mode;
        if (mode == "Normal") mode = "Continuous";

        if (!rcg::setEnum(rcgnodemap, "ExposureAuto", mode.c_str(), false))
        {
          c.camera_exp_auto_mode = "Normal";
          rcg::setEnum(rcgnodemap, "ExposureAuto", "Continuous", true);
        }
      }
      else
      {
        rcg::setEnum(rcgnodemap, "ExposureAuto", "Off", true);
      }

      // if exposure mode changed to manual, read current exposure value and gain again
      if (!c.camera_exp_auto)
      {
        c.camera_exp_value = rcg::getFloat(rcgnodemap, "ExposureTime", 0, 0, true, true) / 1000000;
        if (dev_supports_gain)
        {
          c.camera_gain_value = rcg::getFloat(rcgnodemap, "Gain", 0, 0, true, true);
        }
      }
    }
  }

  // copy config for using it in the grabbing thread

  config = c;
  level |= l;
}

namespace
{
/*
  Set changed values of the given configuration.
*/

void setConfiguration(const std::shared_ptr<GenApi::CNodeMapRef>& nodemap,
                      const rc_visard_driver::rc_visard_driverConfig& cfg, uint32_t lvl, bool iocontrol_avail)
{
  uint32_t prev_lvl = 0;

  while (lvl != 0 && prev_lvl != lvl)
  {
    prev_lvl = lvl;  // used to avoid endless loops

    try
    {
      // NOTE: The flags used in lvl are defined in cfg/rc_visard_driver.cfg

      // set changed values via genicam

      if (lvl & 1)
      {
        lvl &= ~1;
        rcg::setFloat(nodemap, "AcquisitionFrameRate", cfg.camera_fps, true);
        ROS_DEBUG_STREAM("Set AcquisitionFrameRate to " << cfg.camera_fps);
      }

      // lvl 2 (camera_exp_auto) is immediately set in dynamic reconfigure callback

      if (lvl & 4)
      {
        lvl &= ~4;
        rcg::setFloat(nodemap, "ExposureTime", 1000000 * cfg.camera_exp_value, true);
        ROS_DEBUG_STREAM("Set ExposureTime to " << cfg.camera_exp_value);
      }

      if (lvl & 8192)
      {
        lvl &= ~8192;
        rcg::setFloat(nodemap, "Gain", cfg.camera_gain_value, true);
        ROS_DEBUG_STREAM("Set Gain to " << cfg.camera_gain_value);
      }

      if (lvl & 8)
      {
        lvl &= ~8;
        rcg::setFloat(nodemap, "ExposureTimeAutoMax", 1000000 * cfg.camera_exp_max, true);
        ROS_DEBUG_STREAM("Set ExposureTimeAutoMax to " << cfg.camera_exp_max);
      }

      if (lvl & 8388608)
      {
        lvl &= ~8388608;
        rcg::setInteger(nodemap, "ExposureRegionWidth", cfg.camera_exp_width, false);
        ROS_DEBUG_STREAM("Set ExposureRegionWidth to " << cfg.camera_exp_width);
      }

      if (lvl & 16777216)
      {
        lvl &= ~16777216;
        rcg::setInteger(nodemap, "ExposureRegionHeight", cfg.camera_exp_height, false);
        ROS_DEBUG_STREAM("Set ExposureRegionHeight to " << cfg.camera_exp_height);
      }

      if (lvl & 33554432)
      {
        lvl &= ~33554432;
        rcg::setInteger(nodemap, "ExposureRegionOffsetX", cfg.camera_exp_offset_x, false);
        ROS_DEBUG_STREAM("Set ExposureRegionOffsetX to " << cfg.camera_exp_offset_x);
      }

      if (lvl & 67108864)
      {
        lvl &= ~67108864;
        rcg::setInteger(nodemap, "ExposureRegionOffsetY", cfg.camera_exp_offset_y, false);
        ROS_DEBUG_STREAM("Set ExposureRegionOffsetY to " << cfg.camera_exp_offset_y);
      }

      if (lvl & 16384)
      {
        lvl &= ~16384;

        if (cfg.camera_wb_auto)
        {
          rcg::setEnum(nodemap, "BalanceWhiteAuto", "Continuous", false);
          ROS_DEBUG_STREAM("Set BalanceWhiteAuto to " << "Continuous");
        }
        else
        {
          rcg::setEnum(nodemap, "BalanceWhiteAuto", "Off", false);
          ROS_DEBUG_STREAM("Set BalanceWhiteAuto to " << "Off");
        }
      }

      if (lvl & 32768)
      {
        lvl &= ~32768;

        rcg::setEnum(nodemap, "BalanceRatioSelector", "Red", false);
        rcg::setFloat(nodemap, "BalanceRatio", cfg.camera_wb_ratio_red, false);
        ROS_DEBUG_STREAM("Set BalanceRatio Red to " << cfg.camera_wb_ratio_red);
      }

      if (lvl & 65536)
      {
        lvl &= ~65536;

        rcg::setEnum(nodemap, "BalanceRatioSelector", "Blue", false);
        rcg::setFloat(nodemap, "BalanceRatio", cfg.camera_wb_ratio_blue, false);
        ROS_DEBUG_STREAM("Set BalanceRatio Blue to " << cfg.camera_wb_ratio_blue);
      }

      if (lvl & 1048576)
      {
        lvl &= ~1048576;

        std::vector<std::string> list;
        rcg::getEnum(nodemap, "DepthAcquisitionMode", list, true);

        std::string val;
        for (size_t i = 0; i < list.size(); i++)
        {
          if (list[i].compare(cfg.depth_acquisition_mode) == 0)
          {
            val = list[i];
          }
        }

        if (val.size() > 0)
        {
          rcg::setEnum(nodemap, "DepthAcquisitionMode", val.c_str(), true);
          ROS_DEBUG_STREAM("Set DepthAcquisitionMode to " << val);
        }
        else
        {
          ROS_WARN_STREAM("DepthAcquisitionMode " << cfg.depth_acquisition_mode << " not supported by rc_visard");
        }

      }

      if (lvl & 16)
      {
        lvl &= ~16;

        std::vector<std::string> list;
        rcg::getEnum(nodemap, "DepthQuality", list, true);

        std::string val;

        if (cfg.depth_quality == "High" && cfg.depth_static_scene)
        {
          // support for rc_visard < 1.5

          for (size_t i = 0; i < list.size() && val.size() == 0; i++)
          {
            if (list[i].compare(0, 1, "StaticHigh", 0, 1) == 0)
            {
              val = "StaticHigh";
            }
          }
        }

        for (size_t i = 0; i < list.size() && val.size() == 0; i++)
        {
          if (list[i].compare(0, 1, cfg.depth_quality, 0, 1) == 0)
          {
            val = list[i];
          }
        }

        if (val.size() > 0)
        {
          rcg::setEnum(nodemap, "DepthQuality", val.c_str(), true);
          ROS_DEBUG_STREAM("Set DepthQuality to " << val);
        }
      }

      if (lvl & 32)
      {
        lvl &= ~32;
        rcg::setBoolean(nodemap, "DepthDoubleShot", cfg.depth_double_shot, false);
        ROS_DEBUG_STREAM("Set DepthDoubleShot to " << cfg.depth_double_shot);
      }

      if (lvl & 2097152)
      {
        lvl &= ~2097152;
        ROS_DEBUG_STREAM("Set DepthStaticScene to " << cfg.depth_static_scene);
        if (!rcg::setBoolean(nodemap, "DepthStaticScene", cfg.depth_static_scene, false))
        {
          // support for rc_visard < 1.5

          std::string quality = cfg.depth_quality;

          if (cfg.depth_static_scene && quality == "High")
          {
            quality = "StaticHigh";
          }

          std::vector<std::string> list;
          rcg::getEnum(nodemap, "DepthQuality", list, true);

          std::string val;
          for (size_t i = 0; i < list.size() && val.size() == 0; i++)
          {
            if (list[i].compare(0, 1, quality, 0, 1) == 0)
            {
              val = list[i];
            }
          }

          if (val.size() > 0)
          {
            rcg::setEnum(nodemap, "DepthQuality", val.c_str(), true);
            ROS_DEBUG_STREAM("Set DepthQuality to " << val);
          }
        }
      }

      if (lvl & 64)
      {
        lvl &= ~64;
        rcg::setInteger(nodemap, "DepthSeg", cfg.depth_seg, true);
        ROS_DEBUG_STREAM("Set DepthSeg to " << cfg.depth_seg);
      }

      if (lvl & 4194304)
      {
        lvl &= ~4194304;
        rcg::setBoolean(nodemap, "DepthSmooth", cfg.depth_smooth, false);
        ROS_DEBUG_STREAM("Set DepthSmooth to " << cfg.depth_smooth);
      }

      if (lvl & 256)
      {
        lvl &= ~256;
        rcg::setInteger(nodemap, "DepthFill", cfg.depth_fill, true);
        ROS_DEBUG_STREAM("Set DepthFill to " << cfg.depth_fill);
      }

      if (lvl & 512)
      {
        lvl &= ~512;
        rcg::setFloat(nodemap, "DepthMinConf", cfg.depth_minconf, true);
        ROS_DEBUG_STREAM("Set DepthMinConf to " << cfg.depth_minconf);
      }

      if (lvl & 1024)
      {
        lvl &= ~1024;
        rcg::setFloat(nodemap, "DepthMinDepth", cfg.depth_mindepth, true);
        ROS_DEBUG_STREAM("Set DepthMinDepth to " << cfg.depth_mindepth);
      }

      if (lvl & 2048)
      {
        lvl &= ~2048;
        rcg::setFloat(nodemap, "DepthMaxDepth", cfg.depth_maxdepth, true);
        ROS_DEBUG_STREAM("Set DepthMaxDepth to " << cfg.depth_maxdepth);
      }

      if (lvl & 4096)
      {
        lvl &= ~4096;
        rcg::setFloat(nodemap, "DepthMaxDepthErr", cfg.depth_maxdeptherr, true);
        ROS_DEBUG_STREAM("Set DepthMaxDepthErr to " << cfg.depth_maxdeptherr);
      }

      if (lvl & 131072)
      {
        lvl &= ~131072;
        rcg::setBoolean(nodemap, "GevIEEE1588", cfg.ptp_enabled, true);
        ROS_DEBUG_STREAM("Set GevIEEE1588 to " << cfg.ptp_enabled);
      }

      // lvl 262144 (out1_mode) is immediately set in dynamic reconfigure callback

      // lvl 524288 (out2_mode) is immediately set in dynamic reconfigure callback
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("rc_visard_driver: " << ex.what());
    }
  }
}

/*
  Disable all available components.

  @param nodemap Feature nodemap.
*/

void disableAll(const std::shared_ptr<GenApi::CNodeMapRef>& nodemap)
{
  std::vector<std::string> component;

  rcg::getEnum(nodemap, "ComponentSelector", component, true);

  for (size_t i = 0; i < component.size(); i++)
  {
    rcg::setEnum(nodemap, "ComponentSelector", component[i].c_str(), true);
    rcg::setBoolean(nodemap, "ComponentEnable", false, true);
  }

  rcg::setEnum(nodemap, "ComponentSelector", "Intensity");
  rcg::setEnum(nodemap, "PixelFormat", "Mono8");

  // if multipart is available, still send single components per buffer
  rcg::setEnum(nodemap, "AcquisitionMultiPartMode", "SingleComponent");
}

/*
  Conditionally enables a component.

  @param nodemap Feature nodemap.
  @param en_curr Current selection status, which will be changed.
  @param en_new  New selection status.
  @return        1 if status changed, 0 otherwise.
*/

int enable(const std::shared_ptr<GenApi::CNodeMapRef>& nodemap, const char* component, bool& en_curr, bool en_new)
{
  if (en_new != en_curr)
  {
    if (en_new)
      ROS_INFO_STREAM("rc_visard_driver: Enabled image stream: " << component);
    else
      ROS_INFO_STREAM("rc_visard_driver: Disabled image stream: " << component);

    rcg::setEnum(nodemap, "ComponentSelector", component, true);
    rcg::setBoolean(nodemap, "ComponentEnable", en_new);
    en_curr = en_new;

    return 1;
  }

  return 0;
}

rc_common_msgs::CameraParam extractChunkData(const std::shared_ptr<GenApi::CNodeMapRef>& rcgnodemap)
{
  rc_common_msgs::CameraParam cam_param;

  // get list of all available IO lines and iterate over them to get their LineSource values
  std::vector<std::string> lines;
  rcg::getEnum(rcgnodemap, "ChunkLineSelector", lines, false);
  for (auto&& line : lines)
  {
    rc_common_msgs::KeyValue line_source;
    line_source.key = line;
    rcg::setEnum(rcgnodemap, "ChunkLineSelector", line.c_str(), false);       // set respective selector
    line_source.value = rcg::getEnum(rcgnodemap, "ChunkLineSource", false);   // get the actual source value
    cam_param.line_source.push_back(line_source);
  }

  // get LineStatusAll
  cam_param.line_status_all = rcg::getInteger(rcgnodemap, "ChunkLineStatusAll", 0, 0, false);

  // get camera's exposure time and gain value
  cam_param.gain = rcg::getFloat(rcgnodemap, "ChunkGain", 0, 0, false);
  cam_param.exposure_time = rcg::getFloat(rcgnodemap, "ChunkExposureTime", 0, 0, false) / 1000000l;

  // add extra data
  cam_param.extra_data.clear();
  try
  {
    rc_common_msgs::KeyValue kv;
    kv.key = "noise";
    kv.value = std::to_string(rcg::getFloat(rcgnodemap, "ChunkRcNoise", 0, 0, true));
    cam_param.extra_data.push_back(kv);
  }
  catch (std::invalid_argument& e)
  {
    // calculate camera's noise level
    static float NOISE_BASE = 2.0;
    rc_common_msgs::KeyValue kv;
    kv.key = "noise";
    kv.value = std::to_string(static_cast<float>(NOISE_BASE * std::exp(cam_param.gain * std::log(10)/20)));
    cam_param.extra_data.push_back(kv);
  }

  try
  {
    rc_common_msgs::KeyValue kv;
    kv.key = "test";
    kv.value = std::to_string(rcg::getBoolean(rcgnodemap, "ChunkRcTest", true));
    cam_param.extra_data.push_back(kv);
  }
  catch (std::invalid_argument& e)
  {
  }

  try
  {
    std::string out1_reduction;
    try
    {
      out1_reduction = std::to_string(rcg::getFloat(rcgnodemap, "ChunkRcOut1Reduction", 0, 0, true));
    }
    catch (const std::exception& e)
    {
      // try to fallback to older name in versions < 20.10.1
      out1_reduction = std::to_string(rcg::getFloat(rcgnodemap, "ChunkRcAdaptiveOut1Reduction", 0, 0, true));
    }

    rc_common_msgs::KeyValue kv;
    kv.key = "out1_reduction";
    kv.value = out1_reduction;
    cam_param.extra_data.push_back(kv);
  }
  catch (std::invalid_argument& e)
  {
  }

  try
  {
    rc_common_msgs::KeyValue kv;
    kv.key = "brightness";
    kv.value = std::to_string(rcg::getFloat(rcgnodemap, "ChunkRcBrightness", 0, 0, true));
    cam_param.extra_data.push_back(kv);
  }
  catch (std::invalid_argument& e)
  {
  }

  return cam_param;
}

} //anonymous namespace

void DeviceNodelet::grab(std::string device, rcg::Device::ACCESS access)
{
  unsigned int maxNumConsecutiveFails = 5;

  stopImageThread = false;

  imageRequested = true;
  imageSuccess = false;

  // at most 5 consecutive failures are permitted

  unsigned int cntConsecutiveFails = 0;
  while (!stopImageThread && cntConsecutiveFails < maxNumConsecutiveFails)
  {
    imageSuccess = false;
    try
    {
      // mark all dynamic parameters as changed

      level=0xffffffff;

      // initially switch off all components

      disableAll(rcgnodemap);

      bool ccolor = false;
      bool cintensity = false;
      bool cintensitycombined = false;
      bool cdisparity = false;
      bool cconfidence = false;
      bool cerror = false;

      bool firstTime = true;

      // get focal length factor of the camera

      double f = rcg::getFloat(rcgnodemap, "FocalLengthFactor", 0, 0, false);
      if (!f)  // backward compatibility: also check for old name 'FocalLength'
      {
        f = rcg::getFloat(rcgnodemap, "FocalLength", 0, 0, false);
        if (!f)
        {
          throw std::runtime_error("Focal length not found: Neither 'FocalLength' nor 'FocalLengthFactor'!");
        }
      }

      // get baseline of the camera (for backward capability check unit)

      double t = rcg::getFloat(rcgnodemap, "Baseline", 0, 0, true);
      GenApi::INode* node = rcgnodemap->_GetNode("Baseline");
      if (!node || !GenApi::IsReadable(node))
      {
        throw std::invalid_argument("Feature not found or not readable: Baseline");
      }
      GenApi::IFloat* val = dynamic_cast<GenApi::IFloat*>(node);
      if (!val)
      {
        throw std::invalid_argument("Feature not float: Baseline");
      }
      if (val->GetUnit() == "mm")  // in previous versions rc_visard reported as mm
      {
        t /= 1000;
      }

      // get disparity scale

      double scale = rcg::getFloat(rcgnodemap, "Scan3dCoordinateScale", 0, 0, true);

      // check certain values

      rcg::checkFeature(rcgnodemap, "Scan3dOutputMode", "DisparityC");
      rcg::checkFeature(rcgnodemap, "Scan3dCoordinateOffset", "0");
      rcg::checkFeature(rcgnodemap, "Scan3dInvalidDataFlag", "1");
      rcg::checkFeature(rcgnodemap, "Scan3dInvalidDataValue", "0");

      // get current configuration and start dynamic reconfigure server

      initConfiguration(rcgnodemap, config, access);

      float mindepth = config.depth_mindepth;
      float maxdepth = config.depth_maxdepth;
      bool is_depth_acquisition_continuous = (config.depth_acquisition_mode[0] == 'C');

      // prepare chunk adapter for getting chunk data

      std::shared_ptr<GenApi::CChunkAdapter> chunkadapter;
      if (dev_supports_chunk_data)
      {
        chunkadapter = rcg::getChunkAdapter(rcgnodemap, rcgdev->getTLType());
      }

      // initialize all publishers

      ros::NodeHandle nh(getNodeHandle(), "stereo");
      image_transport::ImageTransport it(nh);

      CameraInfoPublisher lcaminfo(nh, tfPrefix, f, t, true);
      CameraInfoPublisher rcaminfo(nh, tfPrefix, f, t, false);

      ImagePublisher limage(it, tfPrefix, true, false, true);
      ImagePublisher rimage(it, tfPrefix, false, false, true);

      DisparityPublisher disp(nh, tfPrefix, f, t, scale);
      DisparityColorPublisher cdisp(it, tfPrefix, f, t, scale);
      DepthPublisher depth(nh, tfPrefix, f, t, scale);

      ConfidencePublisher confidence(nh, tfPrefix);
      ErrorDisparityPublisher error_disp(nh, tfPrefix, scale);
      ErrorDepthPublisher error_depth(nh, tfPrefix, f, t, scale);

      Points2Publisher points2(nh, tfPrefix, f, t, scale);

      // add color image publishers if the camera supports color

      std::shared_ptr<ImagePublisher> limage_color;
      std::shared_ptr<ImagePublisher> rimage_color;
      if (dev_supports_color)
      {
        limage_color = std::make_shared<ImagePublisher>(it, tfPrefix, true, true, true);
        rimage_color = std::make_shared<ImagePublisher>(it, tfPrefix, false, true, true);
      }

      // add camera/image params publishers if the camera supports chunkdata

      std::shared_ptr<CameraParamPublisher> lcamparams;
      std::shared_ptr<CameraParamPublisher> rcamparams;
      if (dev_supports_chunk_data)
      {
        lcamparams = std::make_shared<CameraParamPublisher>(nh, tfPrefix, true);
        rcamparams = std::make_shared<CameraParamPublisher>(nh, tfPrefix, false);
      }

      // start streaming of first stream

      std::vector<std::shared_ptr<rcg::Stream> > stream = rcgdev->getStreams();

      gev_packet_size="";

      if (stream.size() > 0)
      {
        stream[0]->open();
        stream[0]->startStreaming();

        // enter grabbing loop
        ros::SteadyTime tlastimage = ros::SteadyTime::now();
        std::string out1_mode_on_sensor;

        while (!stopImageThread)
        {
          const rcg::Buffer* buffer = stream[0]->grab(40);

          if (buffer != 0 && !buffer->getIsIncomplete())
          {
            totalCompleteBuffers++;
            if (gev_packet_size.size() == 0)
            {
              gev_packet_size = rcg::getString(rcgnodemap, "GevSCPSPacketSize", true, true);
              ROS_INFO_STREAM("rc_visard_driver: Image streams ready (Packet size "
                              << gev_packet_size << ")");
            }

            // get camera/image parameters (GenICam chunk data) if possible
            // (e.g. out1_mode, line_status_all, current gain and exposure, etc.)

            bool out1 = false;
            if (chunkadapter && buffer->getContainsChunkdata())
            {
              chunkadapter->AttachBuffer(reinterpret_cast<std::uint8_t*>(buffer->getGlobalBase()),
                                                                         buffer->getSizeFilled());

              // get current out1 mode
              rcg::setEnum(rcgnodemap, "ChunkLineSelector", "Out1", false);
              std::string v = rcg::getEnum(rcgnodemap, "ChunkLineSource", false);

              if (v.size() > 0)
              {
                out1_mode_on_sensor = v;
              }
            }

            // if in alternate mode, then make publishers aware of it before
            // publishing

            bool out1_alternate = (out1_mode_on_sensor == "ExposureAlternateActive");
            points2.setOut1Alternate(out1_alternate);

            rc_common_msgs::CameraParam cam_param = extractChunkData(rcgnodemap);
            cam_param.is_color_camera = dev_supports_color;
            out1 = (cam_param.line_status_all & 0x1);

            uint32_t npart=buffer->getNumberOfParts();
            for (uint32_t part=0; part<npart; part++)
            {
              if (buffer->getImagePresent(part))
              {
                // reset counter of consecutive missing images and failures
                tlastimage = ros::SteadyTime::now();
                cntConsecutiveFails = 0;
                imageSuccess = true;

                // the buffer is offered to all publishers

                disp.setDepthRange(mindepth, maxdepth);
                cdisp.setDepthRange(mindepth, maxdepth);

                uint64_t pixelformat = buffer->getPixelFormat(part);

                lcaminfo.publish(buffer, part, pixelformat);
                rcaminfo.publish(buffer, part, pixelformat);

                if (lcamparams && rcamparams)
                {
                  lcamparams->publish(buffer, cam_param, pixelformat);
                  rcamparams->publish(buffer, cam_param, pixelformat);
                }

                limage.publish(buffer, part, pixelformat, out1);
                rimage.publish(buffer, part, pixelformat, out1);

                if (limage_color && rimage_color)
                {
                  limage_color->publish(buffer, part, pixelformat, out1);
                  rimage_color->publish(buffer, part, pixelformat, out1);
                }

                disp.publish(buffer, part, pixelformat);
                cdisp.publish(buffer, part, pixelformat);
                depth.publish(buffer, part, pixelformat);

                confidence.publish(buffer, part, pixelformat);
                error_disp.publish(buffer, part, pixelformat);
                error_depth.publish(buffer, part, pixelformat);

                points2.publish(buffer, part, pixelformat, out1);

                // use out1_mode for updating dynamic parameters (below) only
                // from buffers with intensity images

                if (pixelformat != Mono8 && pixelformat != YCbCr411_8)
                {
                  out1_mode_on_sensor = "";
                }
              }
            }

            // detach buffer from nodemap

            if (chunkadapter) chunkadapter->DetachBuffer();
          }
          else if (buffer != 0 && buffer->getIsIncomplete())
          {
            tlastimage = ros::SteadyTime::now();
            totalIncompleteBuffers++;
            ROS_WARN("rc_visard_driver: Received incomplete image buffer");
          }
          else if (buffer == 0)
          {
            // throw an expection if data from enabled components is expected,
            // but not comming for more than 3 seconds

            if (cintensity || cintensitycombined ||
                (is_depth_acquisition_continuous && (cdisparity || cconfidence || cerror)))
            {
              double t = (ros::SteadyTime::now() - tlastimage).toSec();

              if (t > 3)  // report error
              {
                totalImageReceiveTimeouts++;
                std::ostringstream out;
                out << "No images received for " << t << " seconds!";
                throw std::underflow_error(out.str());
              }
            }
            else
            {
              // if nothing is expected then store current time as last time
              // to avoid possible timeout after resubscription

              tlastimage = ros::SteadyTime::now();
            }

            // get out1 mode from sensor (this is also used to check if the
            // connection is still valid)

            rcg::setEnum(rcgnodemap, "LineSelector", "Out1", true);
            out1_mode_on_sensor = rcg::getString(rcgnodemap, "LineSource", true, true);
          }

          // trigger depth

          if (dev_supports_depth_acquisition_trigger && perform_depth_acquisition_trigger)
          {
            perform_depth_acquisition_trigger = false;
            rcg::callCommand(rcgnodemap, "DepthAcquisitionTrigger", true);
            ROS_INFO("rc_visard_driver: Depth image acquisition triggered");
          }

          // determine what should be streamed, according to subscriptions to
          // topics

          // switch color on or off

          if (limage_color && rimage_color && (limage_color->used() || rimage_color->used() || points2.used()))
          {
            if (!ccolor)
            {
              rcg::setEnum(rcgnodemap, "ComponentSelector", "Intensity", true);
              rcg::setEnum(rcgnodemap, "PixelFormat", "YCbCr411_8", true);
              ccolor = true;
            }
          }
          else
          {
            if (ccolor)
            {
              rcg::setEnum(rcgnodemap, "ComponentSelector", "Intensity", true);
              rcg::setEnum(rcgnodemap, "PixelFormat", "Mono8", true);
              ccolor = false;
            }
          }

          // enable or disable components

          int changed = enable(rcgnodemap, "IntensityCombined", cintensitycombined,
                               rimage.used() || (rimage_color && rimage_color->used()));

          changed += enable(rcgnodemap, "Intensity", cintensity,
                            !cintensitycombined && (lcaminfo.used() || rcaminfo.used() ||
                                                    (lcamparams && lcamparams->used()) ||
                                                    (rcamparams && rcamparams->used()) ||
                                                    limage.used() ||
                                                    (limage_color && limage_color->used()) || points2.used()));

          changed += enable(rcgnodemap, "Disparity", cdisparity,
                            disp.used() || cdisp.used() || depth.used() || error_depth.used() || points2.used());

          changed += enable(rcgnodemap, "Confidence", cconfidence, confidence.used());

          changed += enable(rcgnodemap, "Error", cerror, error_disp.used() || error_depth.used());

          // report activation of all components, everytime when a component is
          // enabled or disabled

          if (changed > 0 || firstTime)
          {
            if (cintensity || cintensitycombined || cdisparity || cconfidence || cerror)
              imageRequested = true;
            else
              imageRequested = false;
            firstTime = false;
          }

          // check if dynamic configuration has changed

          if (level != 0)
          {
            // set all changed values via genicam

            mtx.lock();
            rc_visard_driver::rc_visard_driverConfig cfg = config;
            uint32_t lvl = level;
            level = 0;
            mtx.unlock();

            setConfiguration(rcgnodemap, cfg, lvl, iocontrol_avail);

            mindepth = cfg.depth_mindepth;
            maxdepth = cfg.depth_maxdepth;

            // if depth acquisition changed to continuous mode, reset last
            // grabbing time to avoid triggering timout if only disparity,
            // confidence and/or error components are enabled

            is_depth_acquisition_continuous = (cfg.depth_acquisition_mode[0] == 'C');

            if (lvl & 1048576)
            {
              if (is_depth_acquisition_continuous)
              {
                tlastimage = ros::SteadyTime::now();
              }
            }
          }

          // update out1 mode, if it is different to current settings on sensor
          // (which is the only GEV parameter which could have changed outside this code,
          //  i.e. on the rc_visard by the stereomatching module)

          mtx.lock();
          if (out1_mode_on_sensor.size() == 0)
          {
            // use current settings if the value on the sensor cannot be determined
            out1_mode_on_sensor = config.out1_mode;
          }

          if (out1_mode_on_sensor != config.out1_mode)
          {
            config.out1_mode = out1_mode_on_sensor;
            reconfig->updateConfig(config);
          }
          mtx.unlock();
        }

        stream[0]->stopStreaming();
        stream[0]->close();
      }
      else
      {
        ROS_ERROR("rc_visard_driver: No stream");
        cntConsecutiveFails++;
      }
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("rc_visard_driver: " << ex.what());
      cntConsecutiveFails++;
    }
  }

  // report failures, if any

  if (cntConsecutiveFails > 0)
  {
    ROS_ERROR_STREAM("rc_visard_driver: Number of consecutive failures: " << cntConsecutiveFails);
  }
  if (cntConsecutiveFails >= maxNumConsecutiveFails)
  {
    ROS_ERROR_STREAM("rc_visard_driver: Image grabbing failed.");
    recoveryRequested = true;
    updater.force_update();
  }
}

bool DeviceNodelet::depthAcquisitionTrigger(rc_common_msgs::Trigger::Request& req,
                                            rc_common_msgs::Trigger::Response& resp)
{
  perform_depth_acquisition_trigger = true;

  resp.return_code.value = ReturnCodeConstants::SUCCESS;
  resp.return_code.message = "";

  return true;
}


void DeviceNodelet::produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("connection_loss_total", totalConnectionLosses);
  stat.add("complete_buffers_total", totalCompleteBuffers);
  stat.add("incomplete_buffers_total", totalIncompleteBuffers);
  stat.add("image_receive_timeouts_total", totalImageReceiveTimeouts);
  stat.add("current_reconnect_trial", cntConsecutiveRecoveryFails);

  // general connection status is supervised by the recoveryRequested variable

  if (recoveryRequested) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Disconnected");
    return;
  }

  // at least we are connected to gev server

  stat.add("ip_address", dev_ipaddr);
  stat.add("gev_packet_size", gev_packet_size);

  if (imageRequested) {
    if (imageSuccess) {
      // someone subscribed to images, and we actually receive data via GigE vision
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Streaming");
    } else {
      // someone subscribed to images, but we do not receive any data via GigE vision (yet)
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No data");
    }
  } else {
    // no one requested images -> node is ok but stale
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Idle");
  }

}

void DeviceNodelet::produce_device_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (dev_serialno.empty()) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Info");
    stat.add("serial", dev_serialno);
    stat.add("mac", dev_macaddr);
    stat.add("user_id", gev_userid);
    stat.add("image_version", dev_version);
  }
}

} // namespace rc

PLUGINLIB_EXPORT_CLASS(rc::DeviceNodelet, nodelet::Nodelet)
