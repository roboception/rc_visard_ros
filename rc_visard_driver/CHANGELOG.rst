^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rc_visard_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.5 (2020-10-20)
------------------

* add noise, test, adaptive_out1_reduction and brightness to CameraParam extra_data if available

3.0.4 (2020-09-23)
------------------

* Removed forgotten debug log output

3.0.3 (2020-09-22)
------------------

* Removed parameter disparity range as it will be removed on rc_visard >= 20.10.0
  (Use dynamic parameters depth_mindepth and depth_maxdepth for controlling the
   depth range and coloring of disparity_color images.)

3.0.2 (2020-07-28)
------------------

* Fixed filtering out images with projection if stereo matching is used single shot and controls a random dot projector
* Removed parameter depth_median as it will be removed on rc_visard >= 20.10.0
* rc_pick_client: fix grasp markers
* Fixed dockerfiles
* Update readme

3.0.1 (2020-05-14)
------------------

3.0.0 (2020-05-13)
------------------
* rc_visard_driver depends on rc_common_msgs >= 0.5
* Added estimated noise level of image as extra_data in CameraParam message
* Added dynamic parameter camera_exp_auto_mode with backward compatibility to old rc_visards
* Only update dynamic out1_mode parameter from buffers with intensity images or by polling global nodemap
* Ensure that the alternate active mode is set before publishing images
* Used updating of out1_mode to check for valid connection, even if no buffers are expected.
* dynamic reconfigure parameter out1_mode is automatically updated if changed on sensor
* Disparity range that is used for disparity images and visualization now correctly considers quality setting
* warn if DepthAcquisitionMode is not supported by rc_visard firmware version
* add complete_buffers_total to diagnostics
* Publish rc_common_msgs/CameraParams with every image.
  This provides information that is missing in CameraInfo,
  like current exposure time and gain.
* add imu2cam as static tf which is published once the driver is ready
  - driver checks if cam2imu transform can be retrieved from sensor
  or if sensor image is too old
  - in first case static tf is published
  - in latter case old behaviour is implemented, i.e. tf is published
  when subscribed to dynamics stream
* fix empty error message
* change to warning for those logs that inform about missing left images for disp images
* Implemented depth acquisition mode SingleFrameOut1
* Contributors: Christian Emmerich, Felix Ruess, Heiko Hirschmueller

2.7.0 (2019-07-19)
------------------
* replaced std_srvs/Trigger with rc_common_msgs/Trigger
* add support for setting exposure region via dynamic_reconfigure

2.6.4 (2019-06-19)
------------------
* fix race condition when changing exposure mode from auto to manual
* require ROS version with SteadyTime
* use enums in dynamic reconfigure for easier usage

2.6.3 (2019-06-12)
------------------

2.6.2 (2019-06-11)
------------------

2.6.1 (2019-05-20)
------------------

2.6.0 (2019-05-20)
------------------
* auto-start dynamics only on the very first startup
* improve handling and error messages for dynamics streams
* update exposure and gain after switching from auto to manual
* add check if rc_visard ready (genicam feature: RcSystemReady)
* if multipart is available, still send single components per buffer
* refactoring/cleanup

2.5.0 (2019-02-05)
------------------
* add parameter for max number of reconnections
* fix: enable driver to try to recover even if the very first time no connection worked out
* add diagnostics
* fix reporting of package size
* Fixed hanging image streams after restart of sensor
* Support for rc_visard firmware v1.5.0 additions (require `StereoPlus` license)
  * quality full
  * advanced smoothing
* improved driver's auto-connect behavior
* also reapply dynamic_reconfigure params after recovery
* fix projection matrix in published right CameraInfo

2.4.2 (2018-10-29)
------------------

2.4.1 (2018-10-29)
------------------
* Fixed link error if rc_genicam_api is not installed in a standard directory
* docker images: upgrade packages first

2.4.0 (2018-10-16)
------------------
* added `depth_acquisition_mode` parameter
* added `depth_acquisition_trigger` service call
* Reduced latency for passing changes of dynamic parameters and topic discriptions to GenICam
* Fixed using wrong disparity range in disparity color publisher
* now depends on rc_genicam_api >= 2.0.0

2.3.0 (2018-08-21)
------------------

* read params from parameter server before falling back to current device params
* New image topics ...out1_low and ...out1_high are offered if iocontrol module is available

2.2.1 (2018-07-05)
------------------

* Changed to component intensity before changing pixel format for supporting color rc_visards with version >= 1.3.0

2.2.0 (2018-07-03)
------------------

* fix out1_mode/out2_mode description and default
* change/add service calls for onboard SLAM module:
  - rename `dynamics_reset_slam` to `slam_reset`
  - rename `get_trajectory` to `slam_get_trajectory`
  - add `slam_save_map`, `slam_load_map` and `slam_remove_map`
* install Rviz example config file

2.1.1 (2018-06-15)
------------------

* Adjusting disparity range to content of disparity image for colored disparity visualization
* Added debug message if left and disparity images cannot be synchronized for creating point clouds
* Implemented parameters for IO control and relaxed time synchronization in case of exposure alternate mode

2.1.0 (2018-04-23)
------------------

* add ptp_enabled dynamic_reconfigure parameter (to enable PrecisionTimeProtocol Slave on rc_visard)
* add reset service for SLAM
* README updates
* use 'rc_visard' as default device name (works with one rc_visard with factory settings connected)

2.0.0 (2018-02-27)
------------------
* rc_genicam_api and rc_dynamics_api as dependency instead of submodule
* don't reset if datastreams time out
* added get_trajectory service
* Use new statemachine interface
  Return codes are now strings.
* Add services start_slam, restart_slam and stop_slam
* Publishing dynamics as odometry message
* visualizing dynamics message
  - angular velocity, linear accelerarion published as marker
  for visualization
  - cam2imu-transform is published with re-created timestamp
* Contributors: Christian Emmerich, Felix Endres, Felix Ruess, Heiko Hirschmueller

1.2.1 (2018-02-26)
------------------
* use rc_genicam_api as dependency
  instead of including as submodule
  also remove launchfile, as the device is a required parameter anyway...
* Contributors: Felix Ruess

1.2.0 (2018-02-11)
------------------

* Setting default of median to 1 instead of 0, which also means off
* install rc_visard_driver node in package lib dir, so start it with `rosrun rc_visard_driver rc_visard_driver`

1.1.3 (2017-04-13)
------------------

* Added possibility to start as ROS node alternatively to nodelet
* Printing shutdown information to stdout, since ROS log messages just before exit disappear

1.1.2 (2017-04-11)
------------------

* The module reconnects to the GigE Vision server in case of errors
* Added reporting enabled componets and missing images

1.1.0 (2017-04-10)
------------------

* Implemented setting camera framerate via dynamic reconfigure
* Implementation of dynamic reconfigure parameters for controlling the depth image

1.0.1 (2017-03-16)
------------------

* Focal length of disparity image now relates to the size of the disparity image
* Use color for point cloud if color images are available

1.0.0 (2017-03-05)
------------------

* Initial release
