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
#include "camera_info_publisher.h"
#include "image_publisher.h"
#include "disparity_publisher.h"
#include "disparity_color_publisher.h"
#include "depth_publisher.h"
#include "confidence_publisher.h"
#include "error_disparity_publisher.h"
#include "error_depth_publisher.h"
#include "points2_publisher.h"

#include <rc_genicam_api/device.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/buffer.h>
#include <rc_genicam_api/config.h>

#include <pluginlib/class_list_macros.h>
#include <exception>

#include <rc_genicam_api/pixel_formats.h>

#include <sstream>
#include <stdexcept>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>


namespace rc
{

namespace rcd = dynamics;


ThreadedStream::Ptr DeviceNodelet::CreateDynamicsStreamOfType(
        rcd::RemoteInterface::Ptr rcdIface,
        const std::string &stream, ros::NodeHandle& nh, bool tfEnabled)
{
  if (stream=="pose")
  {
    return ThreadedStream::Ptr(new PoseStream(rcdIface, stream, nh, tfEnabled));
  }
  if (stream=="pose_ins" || stream=="pose_rt" || stream=="pose_rt_ins" || stream=="imu")
  {
    return ThreadedStream::Ptr(new Protobuf2RosStream(rcdIface, stream, nh));
  }

  throw std::runtime_error(std::string("Not yet implemented! Stream type: ") + stream);
}


DeviceNodelet::DeviceNodelet()
{
  reconfig=0;
  level=0;

  stopImageThread = imageRequested = imageSuccess = false;

  dynamicsStreams = ThreadedStream::Manager::create();

  stopRecoverThread = false;
  recoveryRequested = true;
  cntConsecutiveRecoveryFails = -1; // first time not giving any warnings
}

DeviceNodelet::~DeviceNodelet()
{
  std::cout << "rc_visard_driver: Shutting down" << std::endl;

  // signal running threads and wait until they finish

  stopImageThread=true;
  dynamicsStreams->stop_all();
  stopRecoverThread=true;

  if (imageThread.joinable())        imageThread.join();
  dynamicsStreams->join_all();
  if (recoverThread.joinable()) recoverThread.join();

  delete reconfig;

  rcg::System::clearSystems();
}

void DeviceNodelet::onInit()
{
  // run initialization and recover routine in separate thread

  recoverThread = std::thread(&DeviceNodelet::keepAliveAndRecoverFromFails, this);
}

void DeviceNodelet::keepAliveAndRecoverFromFails()
{
  // get parameter configuration

  ros::NodeHandle pnh(getPrivateNodeHandle());

  std::string device;
  std::string access="control";

  tfEnabled = false;
  std::string ns = tf::strip_leading_slash(ros::this_node::getNamespace());
  std::string tfprefix = (ns != "") ? ns + "_" : "";
  tfChildFrame = tfprefix + "camera";

  pnh.param("device", device, device);
  pnh.param("gev_access", access, access);
  pnh.param("enable_tf", tfEnabled, tfEnabled);

  rcg::Device::ACCESS access_id;
  if (access == "exclusive")
  {
    access_id=rcg::Device::EXCLUSIVE;
  }
  else if (access == "control")
  {
    access_id=rcg::Device::CONTROL;
  }
  else if (access == "off")
  {
    access_id=rcg::Device::READONLY;
  }
  else
  {
    ROS_FATAL_STREAM("rc_visard_driver: Access must be 'control', 'exclusive' or 'off': " << access);
    return;
  }

  // setup services for starting and stopping rcdynamics module

  dynamicsStartService = pnh.advertiseService("startDynamics",
                                              &DeviceNodelet::startDynamics, this);
  dynamicsRestartService = pnh.advertiseService("restartDynamics",
                                                &DeviceNodelet::restartDynamics, this);
  dynamicsStopService = pnh.advertiseService("stopDynamics",
                                             &DeviceNodelet::stopDynamics, this);

  // run start-keep-alive-and-recover loop

  static int maxNumRecoveryTrials = 5;

  while (!stopRecoverThread &&
          cntConsecutiveRecoveryFails <= maxNumRecoveryTrials)
  {
    // check if everything is running smoothly

    recoveryRequested = recoveryRequested || dynamicsStreams->any_failed();

    if (!recoveryRequested)
    {
      bool allSucceeded = (!imageRequested || imageSuccess);
      allSucceeded = allSucceeded && dynamicsStreams->all_succeeded();
      if ( (cntConsecutiveRecoveryFails > 0) && allSucceeded )
      {
        cntConsecutiveRecoveryFails = 0;
        ROS_INFO("rc_visard_driver: Device successfully recovered from previous fail(s)!");
      }

      usleep(1000 * 100);
      continue;
    }
    cntConsecutiveRecoveryFails++;

    // stop image and dynamics threads

    stopImageThread = true;
    dynamicsStreams->stop_all();

    if (imageThread.joinable()) imageThread.join();
    dynamicsStreams->join_all();

    // try discover, open and bring up device

    bool successfullyOpened = false;
    while (!stopRecoverThread && !successfullyOpened
           && cntConsecutiveRecoveryFails <= maxNumRecoveryTrials)
    {
      // if we are recovering, put warning and wait before retrying

      if (cntConsecutiveRecoveryFails>0)
      {
        ROS_ERROR_STREAM("rc_visard_driver: Failed or lost connection. Trying to recover"
                                 " rc_visard_driver from failed state ("
                                 << cntConsecutiveRecoveryFails
                                 << "/" << maxNumRecoveryTrials << ")...");
        usleep(1000 * 500);
      }

      try
      {
        if (rcgdev)
        {
          rcgdev->close();
        }
        rcgdev=rcg::getDevice(device.c_str());
        if (!rcgdev)
        {
          throw std::invalid_argument("Unknown device '" + device + "'");
        }

        ROS_INFO_STREAM("rc_visard_driver: Opening connection to '" << rcgdev->getID() << "'");
        rcgdev->open(access_id);
        rcgnodemap=rcgdev->getRemoteNodeMap();

        std::string currentIPAddress = rcg::getString(rcgnodemap, "GevCurrentIPAddress", true);
        dynamicsInterface = rcd::RemoteInterface::create(currentIPAddress);

        // add streaming thread for each available stream on rc_visard device

        auto availStreams = dynamicsInterface->getAvailableStreams();
        dynamicsStreams = ThreadedStream::Manager::create();
        for (const auto &streamName : availStreams)
        {
          try
          {
            if (streamName != "dynamics" && streamName != "dynamics_ins")
            {
              auto newStream = CreateDynamicsStreamOfType(dynamicsInterface, streamName,
                                                          getNodeHandle(), tfEnabled);
              dynamicsStreams->add(newStream);
            }
            else
            {
              ROS_INFO_STREAM("Unsupported dynamics stream: " << streamName);
            }
          } catch(const std::exception &e)
          {
            ROS_WARN_STREAM("Unable to create dynamics stream of type "
                            << streamName << ": " << e.what());
          }
        }

        successfullyOpened = true;
      } catch (std::exception &ex)
      {
        cntConsecutiveRecoveryFails++;
        ROS_ERROR_STREAM("rc_visard_driver: " << ex.what());
      }
    }
    if (stopRecoverThread)  break;

    if (cntConsecutiveRecoveryFails > maxNumRecoveryTrials)
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

  std::cout << "rc_visard_driver: stopped." << std::endl;
}

void DeviceNodelet::initConfiguration(const std::shared_ptr<GenApi::CNodeMapRef> &nodemap,
  rc_visard_driver::rc_visard_driverConfig &cfg, rcg::Device::ACCESS access)
{
  ros::NodeHandle pnh(getPrivateNodeHandle());

  // get current camera configuration

  cfg.camera_fps=rcg::getFloat(nodemap, "AcquisitionFrameRate", 0, 0, true);

  std::string v=rcg::getEnum(nodemap, "ExposureAuto", true);
  cfg.camera_exp_auto=(v != "Off");

  cfg.camera_exp_value=rcg::getFloat(nodemap, "ExposureTime", 0, 0, true)/1000000;
  cfg.camera_exp_max=rcg::getFloat(nodemap, "ExposureTimeAutoMax", 0, 0, true)/1000000;

  v=rcg::getEnum(nodemap, "DepthQuality", true);
  cfg.depth_quality=v.substr(0, 1);

  cfg.depth_disprange=rcg::getInteger(nodemap, "DepthDispRange", 0, 0, true);
  cfg.depth_seg=rcg::getInteger(nodemap, "DepthSeg", 0, 0, true);
  cfg.depth_median=rcg::getInteger(nodemap, "DepthMedian", 0, 0, true);
  cfg.depth_fill=rcg::getInteger(nodemap, "DepthFill", 0, 0, true);
  cfg.depth_minconf=rcg::getFloat(nodemap, "DepthMinConf", 0, 0, true);
  cfg.depth_mindepth=rcg::getFloat(nodemap, "DepthMinDepth", 0, 0, true);
  cfg.depth_maxdepth=rcg::getFloat(nodemap, "DepthMaxDepth", 0, 0, true);
  cfg.depth_maxdeptherr=rcg::getFloat(nodemap, "DepthMaxDepthErr", 0, 0, true);

  // setup reconfigure server

  if (reconfig == 0)
  {
    // set ROS parameters according to current configuration

    pnh.setParam("camera_fps", cfg.camera_fps);
    pnh.setParam("camera_exp_auto", cfg.camera_exp_auto);
    pnh.setParam("camera_exp_value", cfg.camera_exp_value);
    pnh.setParam("camera_exp_max", cfg.camera_exp_max);
    pnh.setParam("depth_quality", cfg.depth_quality);
    pnh.setParam("depth_disprange", cfg.depth_disprange);
    pnh.setParam("depth_seg", cfg.depth_seg);
    pnh.setParam("depth_median", cfg.depth_median);
    pnh.setParam("depth_fill", cfg.depth_fill);
    pnh.setParam("depth_minconf", cfg.depth_minconf);
    pnh.setParam("depth_mindepth", cfg.depth_mindepth);
    pnh.setParam("depth_maxdepth", cfg.depth_maxdepth);
    pnh.setParam("depth_maxdeptherr", cfg.depth_maxdeptherr);

    // TODO: we need to dismangle initialization of dynreconfserver from not-READONLY-access-condition
    reconfig=new dynamic_reconfigure::Server<rc_visard_driver::rc_visard_driverConfig>(pnh);
    dynamic_reconfigure::Server<rc_visard_driver::rc_visard_driverConfig>::CallbackType cb;
    cb=boost::bind(&DeviceNodelet::reconfigure, this, _1, _2);
    reconfig->setCallback(cb);
  }
}

void DeviceNodelet::reconfigure(rc_visard_driver::rc_visard_driverConfig &c, uint32_t l)
{
  mtx.lock();

  c.depth_quality=c.depth_quality.substr(0, 1);

  if (c.depth_quality[0] != 'L' && c.depth_quality[0] != 'M' && c.depth_quality[0] != 'H' &&
      c.depth_quality[0] != 'S')
  {
    c.depth_quality="H";
  }

  config=c;
  level|=l;

  mtx.unlock();
}

namespace
{

/*
  Set changed values of the given configuration.
*/

void setConfiguration(const std::shared_ptr<GenApi::CNodeMapRef> &nodemap,
                      const rc_visard_driver::rc_visard_driverConfig &cfg, uint32_t lvl)
{
  uint32_t prev_lvl=0;

  while (lvl != 0 && prev_lvl != lvl)
  {
    prev_lvl=lvl; // used to avoid endless loops

    try
    {
      // NOTE: The flags used in lvl are defined in cfg/rc_visard_driver.cfg

      // set changed values via genicam

      if (lvl&1)
      {
        lvl&=~1;
        rcg::setFloat(nodemap, "AcquisitionFrameRate", cfg.camera_fps, true);
      }

      if (lvl&2)
      {
        lvl&=~2;

        if (cfg.camera_exp_auto)
        {
          rcg::setEnum(nodemap, "ExposureAuto", "Continuous", true);
        }
        else
        {
          rcg::setEnum(nodemap, "ExposureAuto", "Off", true);
        }
      }

      if (lvl&4)
      {
        lvl&=~4;
        rcg::setFloat(nodemap, "ExposureTime", 1000000*cfg.camera_exp_value, true);
      }

      if (lvl&8)
      {
        lvl&=~8;
        rcg::setFloat(nodemap, "ExposureTimeAutoMax", 1000000*cfg.camera_exp_max, true);
      }

      if (lvl&16)
      {
        lvl&=~16;

        std::vector<std::string> list;
        rcg::getEnum(nodemap, "DepthQuality", list, true);

        std::string val;
        for (size_t i=0; i<list.size(); i++)
        {
          if (list[i].compare(0, 1, cfg.depth_quality, 0, 1) == 0)
          {
            val=list[i];
          }
        }

        if (val.size() > 0)
        {
          rcg::setEnum(nodemap, "DepthQuality", val.c_str(), true);
        }
      }

      if (lvl&32)
      {
        lvl&=~32;
        rcg::setInteger(nodemap, "DepthDispRange", cfg.depth_disprange, true);
      }

      if (lvl&64)
      {
        lvl&=~64;
        rcg::setInteger(nodemap, "DepthSeg", cfg.depth_seg, true);
      }

      if (lvl&128)
      {
        lvl&=~128;
        rcg::setInteger(nodemap, "DepthMedian", cfg.depth_median, true);
      }

      if (lvl&256)
      {
        lvl&=~256;
        rcg::setInteger(nodemap, "DepthFill", cfg.depth_fill, true);
      }

      if (lvl&512)
      {
        lvl&=~512;
        rcg::setFloat(nodemap, "DepthMinConf", cfg.depth_minconf, true);
      }

      if (lvl&1024)
      {
        lvl&=~1024;
        rcg::setFloat(nodemap, "DepthMinDepth", cfg.depth_mindepth, true);
      }

      if (lvl&2048)
      {
        lvl&=~2048;
        rcg::setFloat(nodemap, "DepthMaxDepth", cfg.depth_maxdepth, true);
      }

      if (lvl&4096)
      {
        lvl&=~4096;
        rcg::setFloat(nodemap, "DepthMaxDepthErr", cfg.depth_maxdeptherr, true);
      }
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM("rc_visard_driver: " << ex.what());
    }
  }
}

/*
  Disable all available components.

  @param nodemap Feature nodemap.
*/

void disableAll(const std::shared_ptr<GenApi::CNodeMapRef> &nodemap)
{
  std::vector<std::string> component;

  rcg::getEnum(nodemap, "ComponentSelector", component, true);

  for (size_t i=0; i<component.size(); i++)
  {
    rcg::setEnum(nodemap, "ComponentSelector", component[i].c_str(), true);
    rcg::setBoolean(nodemap, "ComponentEnable", false, true);
  }

  rcg::setEnum(nodemap, "PixelFormat", "Mono8");
}

/*
  Conditionally enables a component.

  @param nodemap Feature nodemap.
  @param en_curr Current selection status, which will be changed.
  @param en_new  New selection status.
  @return        1 if status changed, 0 otherwise.
*/

int enable(const std::shared_ptr<GenApi::CNodeMapRef> &nodemap, const char *component,
           bool &en_curr, bool en_new)
{
  if (en_new != en_curr)
  {
    if (en_new)
      ROS_INFO_STREAM("rc_visard_driver: Enabled image stream: " << component);
    else
      ROS_INFO_STREAM("rc_visard_driver: Disabled image stream: " << component);

    rcg::setEnum(nodemap, "ComponentSelector", component, true);
    rcg::setBoolean(nodemap, "ComponentEnable", en_new);
    en_curr=en_new;

    return 1;
  }

  return 0;
}

}

void DeviceNodelet::grab(std::string device, rcg::Device::ACCESS access)
{
  unsigned int maxNumConsecutiveFails = 5;

  stopImageThread=false;

  imageRequested = true;
  imageSuccess = false;

  // at most 5 consecutive failures are permitted

  unsigned int cntConsecutiveFails = 0;
  while (!stopImageThread && cntConsecutiveFails < maxNumConsecutiveFails)
  {

    imageSuccess = false;
    try
    {
      // initially switch off all components

      disableAll(rcgnodemap);

      bool ccolor=false;
      bool cintensity=false;
      bool cintensitycombined=false;
      bool cdisparity=false;
      bool cconfidence=false;
      bool cerror=false;

      bool firstTime = true;

      // get focal length factor of the camera

      double f=rcg::getFloat(rcgnodemap, "FocalLengthFactor", 0, 0, false);
      if (!f) // backward compatibility: also check for old name 'FocalLength'
      {
        f=rcg::getFloat(rcgnodemap, "FocalLength", 0, 0, false);
        if (!f)
        {
          throw std::runtime_error("Focal length not found: Neither 'FocalLength' nor 'FocalLengthFactor'!");
        }
      }

      // get baseline of the camera (for backward capability check unit)

      double t=rcg::getFloat(rcgnodemap, "Baseline", 0, 0, true);
      GenApi::INode *node=rcgnodemap->_GetNode("Baseline");
      if (!node || !GenApi::IsReadable(node))
      {
        throw std::invalid_argument(
                "Feature not found or not readable: Baseline");
      }
      GenApi::IFloat *val=dynamic_cast<GenApi::IFloat *>(node);
      if (!val)
      {
        throw std::invalid_argument("Feature not float: Baseline");
      }
      if (val->GetUnit()=="mm") // in previous versions rc_visard reported as mm
      {
        t /= 1000;
      }

      // get disparity scale

      double scale=rcg::getFloat(rcgnodemap, "Scan3dCoordinateScale", 0, 0, true);

      // check certain values

      rcg::checkFeature(rcgnodemap, "Scan3dOutputMode", "DisparityC");
      rcg::checkFeature(rcgnodemap, "Scan3dCoordinateOffset", "0");
      rcg::checkFeature(rcgnodemap, "Scan3dInvalidDataFlag", "1");
      rcg::checkFeature(rcgnodemap, "Scan3dInvalidDataValue", "0");

      // get current configuration and start dynamic reconfigure server

      initConfiguration(rcgnodemap, config, access);

      int disprange=config.depth_disprange;

      // initialize all publishers

      ros::NodeHandle nh(getNodeHandle(), "stereo");
      image_transport::ImageTransport it(nh);

      CameraInfoPublisher lcaminfo(nh, tfChildFrame, f, t, true);
      CameraInfoPublisher rcaminfo(nh, tfChildFrame, f, t, false);

      ImagePublisher limage(it, tfChildFrame, true, false);
      ImagePublisher rimage(it, tfChildFrame, false, false);

      DisparityPublisher disp(nh, tfChildFrame, f, t, scale);
      DisparityColorPublisher cdisp(it, tfChildFrame, scale);
      DepthPublisher depth(nh, tfChildFrame, f, t, scale);

      ConfidencePublisher confidence(nh, tfChildFrame);
      ErrorDisparityPublisher error_disp(nh, tfChildFrame, scale);
      ErrorDepthPublisher error_depth(nh, tfChildFrame, f, t, scale);

      Points2Publisher points2(nh, tfChildFrame, f, t, scale);

      // add color image publishers if the camera supports color

      std::shared_ptr<Publisher> limage_color;
      std::shared_ptr<Publisher> rimage_color;

      {
        std::vector<std::string> format;
        rcg::getEnum(rcgnodemap, "PixelFormat", format, true);

        for (size_t i=0; i<format.size(); i++)
        {
          if (format[i] == "YCbCr411_8")
          {
            limage_color=std::shared_ptr<Publisher>(new ImagePublisher(it, tfChildFrame, true, true));
            rimage_color=std::shared_ptr<Publisher>(new ImagePublisher(it, tfChildFrame, false, true));
            break;
          }
        }
      }

      // start streaming of first stream

      std::vector<std::shared_ptr<rcg::Stream> > stream=rcgdev->getStreams();

      if (stream.size() > 0)
      {
        stream[0]->open();
        stream[0]->startStreaming();

        ROS_INFO("rc_visard_driver: Image streams ready");

        // enter grabbing loop

        int missing=0;
        ros::Time tlastimage=ros::Time::now();

        while (!stopImageThread)
        {
          const rcg::Buffer *buffer=stream[0]->grab(500);

          if (buffer != 0 && !buffer->getIsIncomplete() && buffer->getImagePresent())
          {
            // reset counter of consecutive missing images and failures

            missing=0;
            tlastimage=ros::Time::now();
            cntConsecutiveFails=0;
            imageSuccess = true;

            // the buffer is offered to all publishers

            disp.setDisprange(disprange);
            cdisp.setDisprange(disprange);

            uint64_t pixelformat=buffer->getPixelFormat();

            lcaminfo.publish(buffer, pixelformat);
            rcaminfo.publish(buffer, pixelformat);

            limage.publish(buffer, pixelformat);
            rimage.publish(buffer, pixelformat);

            if (limage_color && rimage_color)
            {
              limage_color->publish(buffer, pixelformat);
              rimage_color->publish(buffer, pixelformat);
            }

            disp.publish(buffer, pixelformat);
            cdisp.publish(buffer, pixelformat);
            depth.publish(buffer, pixelformat);

            confidence.publish(buffer, pixelformat);
            error_disp.publish(buffer, pixelformat);
            error_depth.publish(buffer, pixelformat);

            points2.publish(buffer, pixelformat);
          }
          else if (buffer != 0 && buffer->getIsIncomplete())
          {
            missing=0;
            ROS_WARN("rc_visard_driver: Received incomplete buffer");
          }
          else if (buffer == 0)
          {
            // throw an expection if components are enabled and there is no
            // data for 6*0.5 seconds

            if (cintensity || cintensitycombined || cdisparity || cconfidence || cerror)
            {
              missing++;
              if (missing >= 6) // report error
              {
                std::ostringstream out;

                out << "No images received for ";
                out << (ros::Time::now()-tlastimage).toSec();
                out << " seconds!";

                throw std::underflow_error(out.str());
              }
            }
          }

          // determine what should be streamed, according to subscriptions to
          // topics

          // switch color on or off

          if (limage_color && rimage_color && (limage_color->used() || rimage_color->used() ||
                                               points2.used()))
          {
            if (!ccolor)
            {
              rcg::setEnum(rcgnodemap, "PixelFormat", "YCbCr411_8", true);
              ccolor=true;
            }
          }
          else
          {
            if (ccolor)
            {
              rcg::setEnum(rcgnodemap, "PixelFormat", "Mono8", true);
              ccolor=false;
            }
          }

          // enable or disable components

          int changed=enable(rcgnodemap, "IntensityCombined", cintensitycombined,
                             rimage.used() || (rimage_color && rimage_color->used()));

          changed+=enable(rcgnodemap, "Intensity", cintensity, !cintensitycombined &&
                          (lcaminfo.used() || rcaminfo.used() || limage.used() ||
                          (limage_color && limage_color->used()) || points2.used()));

          changed+=enable(rcgnodemap, "Disparity", cdisparity, disp.used() || cdisp.used() ||
                          depth.used() || error_depth.used() || points2.used());

          changed+=enable(rcgnodemap, "Confidence", cconfidence, confidence.used());

          changed+=enable(rcgnodemap, "Error", cerror, error_disp.used() || error_depth.used());

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
            rc_visard_driver::rc_visard_driverConfig cfg=config;
            uint32_t lvl=level;
            level=0;
            mtx.unlock();

            setConfiguration(rcgnodemap, cfg, lvl);

            disprange=cfg.depth_disprange;
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
    catch (const std::exception &ex)
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
  }

}

void handleDynamicsStateChangeRequest(
        rcd::RemoteInterface::Ptr dynIF,
        int state, std_srvs::Trigger::Response &resp)
{
  resp.success = true;
  resp.message = "";

  if (dynIF)
  {
    try
    {
      switch (state) {
        case 0: // STOP
          dynIF->stop();
          break;
        case 1: // START
          dynIF->start(false);
          break;
        case 2: // RESTART
          dynIF->start(true);
          break;
        default:
          throw std::runtime_error("handleDynamicsStateChangeRequest: unrecognized state change request");
      }
    }
    catch (std::exception &e)
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

  if (!resp.success) ROS_ERROR_STREAM(resp.message);
}

bool DeviceNodelet::startDynamics(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &resp){
  handleDynamicsStateChangeRequest(dynamicsInterface, 1, resp);
  return true;
}

bool DeviceNodelet::restartDynamics(std_srvs::Trigger::Request &req,
                                    std_srvs::Trigger::Response &resp){
  handleDynamicsStateChangeRequest(dynamicsInterface, 2, resp);
  return true;
}

bool DeviceNodelet::stopDynamics(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &resp){
  handleDynamicsStateChangeRequest(dynamicsInterface, 0, resp);
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(rc::DeviceNodelet, nodelet::Nodelet)
