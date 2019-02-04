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
#include "publishers/image_publisher.h"
#include "publishers/disparity_publisher.h"
#include "publishers/disparity_color_publisher.h"
#include "publishers/depth_publisher.h"
#include "publishers/confidence_publisher.h"
#include "publishers/error_disparity_publisher.h"
#include "publishers/error_depth_publisher.h"
#include "publishers/points2_publisher.h"

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

namespace {

  /// Returns true if vendor name and model name indicate that genicam device is an rc_visard
  bool isRcVisardDevice(const std::string& vendor, const std::string& model)
  {
    bool isKuka3DPerception = vendor.find("KUKA") != std::string::npos && model.find("3d_perception") != std::string::npos;
    bool isRcVisard = vendor.find("Roboception") != std::string::npos && model.find("rc_visard") != std::string::npos;
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

#define ROS_HAS_STEADYTIME (ROS_VERSION_MINIMUM(1, 13, 1) || ((ROS_VERSION_MINOR == 12) && ROS_VERSION_PATCH >= 8))

ThreadedStream::Ptr DeviceNodelet::CreateDynamicsStreamOfType(rcd::RemoteInterface::Ptr rcdIface,
                                                              const std::string& stream, ros::NodeHandle& nh,
                                                              const std::string& frame_id_prefix, bool tfEnabled)
{
  if (stream == "pose")
  {
    return ThreadedStream::Ptr(new PoseStream(rcdIface, stream, nh, frame_id_prefix, tfEnabled));
  }
  if (stream == "pose_ins" || stream == "pose_rt" || stream == "pose_rt_ins" || stream == "imu")
  {
    return ThreadedStream::Ptr(new Protobuf2RosStream(rcdIface, stream, nh, frame_id_prefix));
  }
  if (stream == "dynamics" || stream == "dynamics_ins")
  {
    return ThreadedStream::Ptr(new DynamicsStream(rcdIface, stream, nh, frame_id_prefix));
  }

  throw std::runtime_error(std::string("Not yet implemented! Stream type: ") + stream);
}

DeviceNodelet::DeviceNodelet()
{
  reconfig = 0;
  dev_supports_gain = false;
  dev_supports_wb = false;
  dev_supports_depth_acquisition_trigger = false;
  perform_depth_acquisition_trigger = false;
  iocontrol_avail = false;
  level = 0;

  stopImageThread = imageRequested = imageSuccess = false;

  dynamicsStreams = ThreadedStream::Manager::create();

  stopRecoverThread = false;
  recoveryRequested = true;
  cntConsecutiveRecoveryFails = -1;  // first time not giving any warnings
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

        // extract some diagnostics data from device
        dev_serialno = rcg::getString(rcgnodemap, "DeviceID", true);
        dev_macaddr = rcg::getString(rcgnodemap, "GevMACAddress", true);
        dev_ipaddr = rcg::getString(rcgnodemap, "GevCurrentIPAddress", true);
        dev_version = rcg::getString(rcgnodemap, "DeviceVersion", true);
        gev_userid = rcg::getString(rcgnodemap, "DeviceUserID", true);
        gev_packet_size = rcg::getString(rcgnodemap, "GevSCPSPacketSize", true);

        updater.setHardwareID(dev_serialno);
        updater.force_update();

        // instantiating dynamics interface and autostart dynamics on sensor if desired

        std::string currentIPAddress = rcg::getString(rcgnodemap, "GevCurrentIPAddress", true);
        dynamicsInterface = rcd::RemoteInterface::create(currentIPAddress);
        if (autostartDynamics || autostartSlam)
        {
          std_srvs::Trigger::Request dummyreq;
          std_srvs::Trigger::Response dummyresp;
          if (!((autostartSlam && this->dynamicsStartSlam(dummyreq, dummyresp)) ||
                (autostartDynamics && this->dynamicsStart(dummyreq, dummyresp))))
          {  // autostart failed!
            ROS_WARN("rc_visard_driver: Could not auto-start dynamics module!");
            cntConsecutiveRecoveryFails++;
            continue;  // to next trial!
          }
        }

        // add streaming thread for each available stream on rc_visard device

        auto availStreams = dynamicsInterface->getAvailableStreams();
        dynamicsStreams = ThreadedStream::Manager::create();
        for (const auto& streamName : availStreams)
        {
          try
          {
            auto newStream =
                CreateDynamicsStreamOfType(dynamicsInterface, streamName, getNodeHandle(), tfPrefix, tfEnabled);
            dynamicsStreams->add(newStream);
          }
          catch (const std::exception& e)
          {
            ROS_WARN_STREAM("rc_visard_driver: Unable to create dynamics stream of type " << streamName << ": "
                                                                                          << e.what());
          }
        }

        successfullyOpened = true;
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
    std_srvs::Trigger::Request dummyreq;
    std_srvs::Trigger::Response dummyresp;
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

  cfg.camera_exp_value = rcg::getFloat(nodemap, "ExposureTime", 0, 0, true) / 1000000;
  cfg.camera_exp_max = rcg::getFloat(nodemap, "ExposureTimeAutoMax", 0, 0, true) / 1000000;

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

  // get optional white balancing values (only for color camera)

  v = rcg::getEnum(nodemap, "BalanceWhiteAuto", false);
  if (v.size() > 0)
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
  cfg.depth_disprange = rcg::getInteger(nodemap, "DepthDispRange", 0, 0, true);
  cfg.depth_seg = rcg::getInteger(nodemap, "DepthSeg", 0, 0, true);
  cfg.depth_median = rcg::getInteger(nodemap, "DepthMedian", 0, 0, true);
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

    // enabling chunk data for getting the live line status

    rcg::setBoolean(nodemap, "ChunkModeActive", iocontrol_avail, false);
  }
  catch (const std::exception&)
  {
    ROS_WARN("rc_visard_driver: rc_visard has an older firmware, IO control functions are not available.");

    cfg.out1_mode = "ExposureActive";
    cfg.out2_mode = "Low";

    iocontrol_avail = false;
  }


  // try to get ROS parameters: if parameter is not set in parameter server, default to current sensor configuration

  pnh.param("camera_fps", cfg.camera_fps, cfg.camera_fps);
  pnh.param("camera_exp_auto", cfg.camera_exp_auto, cfg.camera_exp_auto);
  pnh.param("camera_exp_value", cfg.camera_exp_value, cfg.camera_exp_value);
  pnh.param("camera_gain_value", cfg.camera_gain_value, cfg.camera_gain_value);
  pnh.param("camera_exp_max", cfg.camera_exp_max, cfg.camera_exp_max);
  pnh.param("camera_wb_auto", cfg.camera_wb_auto, cfg.camera_wb_auto);
  pnh.param("camera_wb_ratio_red", cfg.camera_wb_ratio_red, cfg.camera_wb_ratio_red);
  pnh.param("camera_wb_ratio_blue", cfg.camera_wb_ratio_blue, cfg.camera_wb_ratio_blue);
  pnh.param("depth_acquisition_mode", cfg.depth_acquisition_mode, cfg.depth_acquisition_mode);
  pnh.param("depth_quality", cfg.depth_quality, cfg.depth_quality);
  pnh.param("depth_static_scene", cfg.depth_static_scene, cfg.depth_static_scene);
  pnh.param("depth_disprange", cfg.depth_disprange, cfg.depth_disprange);
  pnh.param("depth_seg", cfg.depth_seg, cfg.depth_seg);
  pnh.param("depth_smooth", cfg.depth_smooth, cfg.depth_smooth);
  pnh.param("depth_median", cfg.depth_median, cfg.depth_median);
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
  pnh.setParam("camera_exp_value", cfg.camera_exp_value);
  pnh.setParam("camera_gain_value", cfg.camera_gain_value);
  pnh.setParam("camera_exp_max", cfg.camera_exp_max);
  pnh.setParam("camera_wb_auto", cfg.camera_wb_auto);
  pnh.setParam("camera_wb_ratio_red", cfg.camera_wb_ratio_red);
  pnh.setParam("camera_wb_ratio_blue", cfg.camera_wb_ratio_blue);
  pnh.setParam("depth_acquisition_mode", cfg.depth_acquisition_mode);
  pnh.setParam("depth_quality", cfg.depth_quality);
  pnh.setParam("depth_static_scene", cfg.depth_static_scene);
  pnh.setParam("depth_disprange", cfg.depth_disprange);
  pnh.setParam("depth_seg", cfg.depth_seg);
  pnh.setParam("depth_smooth", cfg.depth_smooth);
  pnh.setParam("depth_median", cfg.depth_median);
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
    reconfig = new dynamic_reconfigure::Server<rc_visard_driver::rc_visard_driverConfig>(pnh);
  }
  // always set callback to (re)load configuration even after recovery
  dynamic_reconfigure::Server<rc_visard_driver::rc_visard_driverConfig>::CallbackType cb;
  cb = boost::bind(&DeviceNodelet::reconfigure, this, _1, _2);
  reconfig->setCallback(cb);
}

void DeviceNodelet::reconfigure(rc_visard_driver::rc_visard_driverConfig& c, uint32_t l)
{
  std::lock_guard<std::mutex> lock(mtx);

  // check and correct parameters

  if (!dev_supports_gain)
  {
    c.camera_gain_value = 0;
    l &= ~8192;
  }

  c.camera_gain_value = round(c.camera_gain_value / 6) * 6;

  if (!dev_supports_wb)
  {
    c.camera_wb_auto = true;
    c.camera_wb_ratio_red = 1;
    c.camera_wb_ratio_blue = 1;
    l &= ~(16384 | 32768 | 65536);
  }

  if (dev_supports_depth_acquisition_trigger)
  {
    c.depth_acquisition_mode = c.depth_acquisition_mode.substr(0, 1);

    if (c.depth_acquisition_mode[0] == 'S')
    {
      c.depth_acquisition_mode = "SingleFrame";
    }
    else
    {
      c.depth_acquisition_mode = "Continuous";
    }
  }
  else
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
      c.out1_mode = "ExposureActive";
    }

    if (c.out2_mode != "Low" && c.out2_mode != "High" && c.out2_mode != "ExposureActive" &&
        c.out2_mode != "ExposureAlternateActive")
    {
      c.out2_mode = "Low";
    }
  }
  else
  {
    c.out1_mode = "ExposureActive";
    c.out2_mode = "Low";
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
      }

      if (lvl & 2)
      {
        lvl &= ~2;

        if (cfg.camera_exp_auto)
        {
          rcg::setEnum(nodemap, "ExposureAuto", "Continuous", true);
        }
        else
        {
          rcg::setEnum(nodemap, "ExposureAuto", "Off", true);
        }
      }

      if (lvl & 4)
      {
        lvl &= ~4;
        rcg::setFloat(nodemap, "ExposureTime", 1000000 * cfg.camera_exp_value, true);
      }

      if (lvl & 8192)
      {
        lvl &= ~8192;
        rcg::setFloat(nodemap, "Gain", cfg.camera_gain_value, true);
      }

      if (lvl & 8)
      {
        lvl &= ~8;
        rcg::setFloat(nodemap, "ExposureTimeAutoMax", 1000000 * cfg.camera_exp_max, true);
      }

      if (lvl & 16384)
      {
        lvl &= ~16384;

        if (cfg.camera_wb_auto)
        {
          rcg::setEnum(nodemap, "BalanceWhiteAuto", "Continuous", false);
        }
        else
        {
          rcg::setEnum(nodemap, "BalanceWhiteAuto", "Off", false);
        }
      }

      if (lvl & 32768)
      {
        lvl &= ~32768;

        rcg::setEnum(nodemap, "BalanceRatioSelector", "Red", false);
        rcg::setFloat(nodemap, "BalanceRatio", cfg.camera_wb_ratio_red, false);
      }

      if (lvl & 65536)
      {
        lvl &= ~65536;

        rcg::setEnum(nodemap, "BalanceRatioSelector", "Blue", false);
        rcg::setFloat(nodemap, "BalanceRatio", cfg.camera_wb_ratio_blue, false);
      }

      if (lvl & 1048576)
      {
        lvl &= ~1048576;

        std::vector<std::string> list;
        rcg::getEnum(nodemap, "DepthAcquisitionMode", list, true);

        std::string val;
        for (size_t i = 0; i < list.size(); i++)
        {
          if (list[i].compare(0, 1, cfg.depth_acquisition_mode, 0, 1) == 0)
          {
            val = list[i];
          }
        }

        if (val.size() > 0)
        {
          rcg::setEnum(nodemap, "DepthAcquisitionMode", val.c_str(), true);
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
        }
      }

      if (lvl & 2097152)
      {
        lvl &= ~2097152;

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
          }
        }
      }

      if (lvl & 32)
      {
        lvl &= ~32;
        rcg::setInteger(nodemap, "DepthDispRange", cfg.depth_disprange, true);
      }

      if (lvl & 64)
      {
        lvl &= ~64;
        rcg::setInteger(nodemap, "DepthSeg", cfg.depth_seg, true);
      }

      if (lvl & 4194304)
      {
        lvl &= ~4194304;
        rcg::setBoolean(nodemap, "DepthSmooth", cfg.depth_smooth, false);
      }

      if (lvl & 128)
      {
        lvl &= ~128;
        rcg::setInteger(nodemap, "DepthMedian", cfg.depth_median, true);
      }

      if (lvl & 256)
      {
        lvl &= ~256;
        rcg::setInteger(nodemap, "DepthFill", cfg.depth_fill, true);
      }

      if (lvl & 512)
      {
        lvl &= ~512;
        rcg::setFloat(nodemap, "DepthMinConf", cfg.depth_minconf, true);
      }

      if (lvl & 1024)
      {
        lvl &= ~1024;
        rcg::setFloat(nodemap, "DepthMinDepth", cfg.depth_mindepth, true);
      }

      if (lvl & 2048)
      {
        lvl &= ~2048;
        rcg::setFloat(nodemap, "DepthMaxDepth", cfg.depth_maxdepth, true);
      }

      if (lvl & 4096)
      {
        lvl &= ~4096;
        rcg::setFloat(nodemap, "DepthMaxDepthErr", cfg.depth_maxdeptherr, true);
      }

      if (lvl & 131072)
      {
        lvl &= ~131072;
        rcg::setBoolean(nodemap, "GevIEEE1588", cfg.ptp_enabled, true);
      }

      if (lvl & 262144)
      {
        lvl &= ~262144;

        if (iocontrol_avail)
        {
          rcg::setEnum(nodemap, "LineSelector", "Out1", true);
          rcg::setEnum(nodemap, "LineSource", cfg.out1_mode.c_str(), true);
        }
      }

      if (lvl & 524288)
      {
        lvl &= ~524288;

        if (iocontrol_avail)
        {
          rcg::setEnum(nodemap, "LineSelector", "Out2", true);
          rcg::setEnum(nodemap, "LineSource", cfg.out2_mode.c_str(), true);
        }
      }
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
}

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

      int disprange = config.depth_disprange;
      bool is_depth_acquisition_continuous = (config.depth_acquisition_mode[0] == 'C');

      // prepare chunk adapter for getting chunk data, if iocontrol is available

      std::shared_ptr<GenApi::CChunkAdapter> chunkadapter;

      if (iocontrol_avail)
      {
        chunkadapter = rcg::getChunkAdapter(rcgnodemap, rcgdev->getTLType());
      }

      // initialize all publishers

      ros::NodeHandle nh(getNodeHandle(), "stereo");
      image_transport::ImageTransport it(nh);

      CameraInfoPublisher lcaminfo(nh, tfPrefix, f, t, true);
      CameraInfoPublisher rcaminfo(nh, tfPrefix, f, t, false);

      ImagePublisher limage(it, tfPrefix, true, false, iocontrol_avail);
      ImagePublisher rimage(it, tfPrefix, false, false, iocontrol_avail);

      DisparityPublisher disp(nh, tfPrefix, f, t, scale);
      DisparityColorPublisher cdisp(it, tfPrefix, scale);
      DepthPublisher depth(nh, tfPrefix, f, t, scale);

      ConfidencePublisher confidence(nh, tfPrefix);
      ErrorDisparityPublisher error_disp(nh, tfPrefix, scale);
      ErrorDepthPublisher error_depth(nh, tfPrefix, f, t, scale);

      Points2Publisher points2(nh, tfPrefix, f, t, scale);

      // add color image publishers if the camera supports color

      std::shared_ptr<ImagePublisher> limage_color;
      std::shared_ptr<ImagePublisher> rimage_color;

      {
        std::vector<std::string> format;
        rcg::setEnum(rcgnodemap, "ComponentSelector", "Intensity", true);
        rcg::getEnum(rcgnodemap, "PixelFormat", format, true);

        for (size_t i = 0; i < format.size(); i++)
        {
          if (format[i] == "YCbCr411_8")
          {
            limage_color = std::make_shared<ImagePublisher>(it, tfPrefix, true, true, iocontrol_avail);
            rimage_color = std::make_shared<ImagePublisher>(it, tfPrefix, false, true, iocontrol_avail);
            break;
          }
        }
      }

      // start streaming of first stream

      std::vector<std::shared_ptr<rcg::Stream> > stream = rcgdev->getStreams();

      gev_packet_size="";

      if (stream.size() > 0)
      {
        stream[0]->open();
        stream[0]->startStreaming();

        // enter grabbing loop
#if ROS_HAS_STEADYTIME
        ros::SteadyTime tlastimage = ros::SteadyTime::now();
#else
        ros::WallTime tlastimage = ros::WallTime::now();
#endif

        while (!stopImageThread)
        {
          const rcg::Buffer* buffer = stream[0]->grab(40);

          if (buffer != 0 && !buffer->getIsIncomplete())
          {
            if (gev_packet_size.size() == 0)
            {
              gev_packet_size = rcg::getString(rcgnodemap, "GevSCPSPacketSize", true, true);
              ROS_INFO_STREAM("rc_visard_driver: Image streams ready (Packet size "
                              << gev_packet_size << ")");
            }

            // get out1 line status from chunk data if possible

            bool out1 = false;
            if (iocontrol_avail && chunkadapter && buffer->getContainsChunkdata())
            {
              chunkadapter->AttachBuffer(reinterpret_cast<std::uint8_t*>(buffer->getGlobalBase()),
                                                                         buffer->getSizeFilled());
              out1 = (rcg::getInteger(rcgnodemap, "ChunkLineStatusAll", 0, 0, false) & 0x1);
            }

            uint32_t npart=buffer->getNumberOfParts();
            for (uint32_t part=0; part<npart; part++)
            {
              if (buffer->getImagePresent(part))
              {
                // reset counter of consecutive missing images and failures
#if ROS_HAS_STEADYTIME
                tlastimage = ros::SteadyTime::now();
#else
                tlastimage = ros::WallTime::now();
#endif
                cntConsecutiveFails = 0;
                imageSuccess = true;

                // the buffer is offered to all publishers

                disp.setDisprange(disprange);
                cdisp.setDisprange(disprange);

                uint64_t pixelformat = buffer->getPixelFormat(part);

                lcaminfo.publish(buffer, part, pixelformat);
                rcaminfo.publish(buffer, part, pixelformat);

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
              }
            }

            // detach buffer from nodemap

            if (chunkadapter) chunkadapter->DetachBuffer();
          }
          else if (buffer != 0 && buffer->getIsIncomplete())
          {
#if ROS_HAS_STEADYTIME
            tlastimage = ros::SteadyTime::now();
#else
            tlastimage = ros::WallTime::now();
#endif
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
#if ROS_HAS_STEADYTIME
              double t = (ros::SteadyTime::now() - tlastimage).toSec();
#else
              double t = (ros::WallTime::now() - tlastimage).toSec();
#endif

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

#if ROS_HAS_STEADYTIME
              tlastimage = ros::SteadyTime::now();
#else
              tlastimage = ros::WallTime::now();
#endif
            }
          }

          // trigger depth

          if (dev_supports_depth_acquisition_trigger && perform_depth_acquisition_trigger)
          {
            perform_depth_acquisition_trigger = false;
            rcg::callCommand(rcgnodemap, "DepthAcquisitionTrigger", true);
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
                            !cintensitycombined && (lcaminfo.used() || rcaminfo.used() || limage.used() ||
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

          // check if dynamic configuration hast changed

          if (level != 0)
          {
            // set all changed values via genicam

            mtx.lock();
            rc_visard_driver::rc_visard_driverConfig cfg = config;
            uint32_t lvl = level;
            level = 0;
            mtx.unlock();

            setConfiguration(rcgnodemap, cfg, lvl, iocontrol_avail);

            disprange = cfg.depth_disprange;

            // if in alternate mode, then make publishers aware of it

            if (lvl & 262144)
            {
              bool alternate = (cfg.out1_mode == "ExposureAlternateActive");

              limage.setOut1Alternate(alternate);
              rimage.setOut1Alternate(alternate);
              points2.setOut1Alternate(alternate);

              if (limage_color && rimage_color)
              {
                limage_color->setOut1Alternate(alternate);
                rimage_color->setOut1Alternate(alternate);
              }
            }

            // if depth acquisition changed to continuous mode, reset last
            // grabbing time to avoid triggering timout if only disparity,
            // confidence and/or error components are enabled

            is_depth_acquisition_continuous = (cfg.depth_acquisition_mode[0] == 'C');

            if (lvl & 1048576)
            {
              if (is_depth_acquisition_continuous)
              {
#if ROS_HAS_STEADYTIME
                tlastimage = ros::SteadyTime::now();
#else
                tlastimage = ros::WallTime::now();
#endif
              }
            }
          }
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

bool DeviceNodelet::depthAcquisitionTrigger(std_srvs::Trigger::Request& req,
                                            std_srvs::Trigger::Response& resp)
{
  perform_depth_acquisition_trigger = true;

  resp.success = true;
  resp.message = "";

  return true;
}

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

void DeviceNodelet::produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.add("connection_loss_total", totalConnectionLosses);
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

}

PLUGINLIB_EXPORT_CLASS(rc::DeviceNodelet, nodelet::Nodelet)
