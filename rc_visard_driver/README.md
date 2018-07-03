
rc_visard_driver
----------------

This nodelet provides data from a Roboception rc_visard sensor on several ROS
topics.

Build/Installation
------------------

See the [main README](../README.md)


Configuration
-------------

#### Parameters

Parameters to be set to the ROS param server before run-time.

- `device`: The ID of the device, i.e. Roboception rc_visard sensor.
   This can be either the
   - lower case MAC address, but with ':' replaced by '_'.
   - serial number
   - user defined name (factory default: `rc_visard`), must be unique among all
     reachable sensors
   See https://github.com/roboception/rc_genicam_api#device-id for more details.
   By default this parameter is set to `rc_visard`, which works with one
   rc_visard with factory settings connected.

- `gev_access`:  The gev_access mode, i.e.:
  - 'control'   Configuration and streaming with the possibility of other
                clients to read GenICam parameters. This is the default.
  - 'exclusive' Exclusive access to the sensor. This prevents other clients to
                read GenICam parameters.
  - 'off'       Switches gev access completely off. The node only streams pose
                information if switched on.

- `enable_tf`: If true then the node subscribes to the rc_visard's
  dynamics-pose stream and publishes them on tf. Default: false

- `enable_visualization_markers`: If true, additional visualization markers are
  published that visualize the rc_visard's dynamics state (velocities and accelerations),
  see `/dynamics_visualization_markers` topic. Default: false

- `autostart_dynamics`: If true, the rc_visard's dynamics module is turned on
  with this ROS node's start up. Default: false

- `autostart_dynamics_with_slam`: If true, the rc_visard's dynamics module tries
  to turn on SLAM with this ROS node's start up.
  If SLAM is not available (no license) only a warning is printed. Default: false

- `autostop_dynamics`: If true, the rc_visard's dynamics module is turned off
  when this ROS node shuts down. Default: false

- `autopublish_trajectory`: If true, results of the `get_trajectory` service
  calls are automatically published to `/trajectory` topic.
  Default: false

#### Dynamic-reconfigure Parameters

  These parameters can be changed during runtime via dynamic reconfigure:

  - `ptp_enabled`: Enable PTP slave (PrecisionTimeProtocol, IEEE1588)

  - `camera_fps`: Frames per second that are published by this nodelet.
    Publishing frames will be slowed down depending on this setting. Setting
    it higher than the real framerate of the specific device has no effect.

  - `camera_exp_auto`: If true, then the exposure time is chosen automatically,
    up to exp_max as maximum. If false, then exp_value is used as exposure
    time in seconds.

  - `camera_exp_max`: Maximum exposure time in seconds if exp_auto is true.

  - `camera_exp_value`: Exposure time in seconds if exp_auto is false.

  - `camera_gain_value`: Gain factor in decibel if exp_auto is false.

  - `depth_quality`: Quality can be `Low`, `Medium`, `High` and `StaticHigh`.
    Only the first letter will be checked, thus specification of `L`, `M`,
    `H` or `S` is sufficient.

    + `StaticHigh` quality means computation with 640x480 pixel, limited to 3 Hz
      and accumulation input images. The scene must be static during image
      accumulation! The timestamp of the disparity image is taken from the first
      image that was used for accumulation.
    + `High` quality means computation with 640x480 pixel.
    + `Medium` quality means computation with 320x240 pixel.
    + `Low` quality means computation with 214x160 pixel.

    Default: `High`.

  - `depth_disprange`: Disparity range in pixel, related to the downscaled
    image at quality=H. The range is adapted to the quality.

  - `depth_fill`: Higher numbers fill gaps with measurments with potentielly
    higher errors.

  - `depth_seg`: Maximum size of isolated disparity regions that will be
    invalidated, related to full resolution.

  - `depth_median`: Performs median filtering with the given window size.

  - `depth_minconf`: Minimal confidence. All disparities with lower confidence
    will be set to invalid.

  - `depth_mindepth`: Minimum depth in meter. All disparities with lower depth
    will be set to invalid.

  - `depth_maxdepth`: Maximum depth in meter. All disparities with higher depth
    will be set to invalid.

  - `depth_maxdeptherr`: Maximum depth error in meter. All disparities with a
    higher depth error will be set to invalid.

For color sensors, the following dynamic-reconfigure parameters are additionally
available:

  - `camera_wb_auto`: If true, then white balancing is done automatically. If
    false, then the red and blue to green ratios can be chosen manually.

  - `camera_wb_ratio_red`: Red to green ratio for color balancing if
    `camera_wb_auto` is false.

  - `camera_wb_ratio_blue`: Blue to green ratio for color balancing if
    `camera_wb_auto` is false.

Provided Topics
---------------

The following topics are provided. The nodelet tries to request only
data (e.g., images, poses) from the sensor if there is subscriber
to the corresponding topic.

#### Images, Stereo Data, Point Clouds

- /stereo/left/camera_info (sensor_msgs::CameraInfo)
- /stereo/right/camera_info (sensor_msgs::CameraInfo)
- /stereo/left/image_rect (sensor_msgs::Image, MONO8)
- /stereo/right/image_rect (sensor_msgs::Image, MONO8)
- /stereo/disparity (stereo_msgs::DisparityImage)
- /stereo/disparity_color (sensor_msgs::Image, RGB8, visually pleasing)
- /stereo/depth (sensor_msgs::Image, TYPE_32FC1)
- /stereo/confidence (sensor_msgs::Image, TYPE_32FC1, values between 0 and 1)
- /stereo/error_disparity (sensor_msgs::Image, TYPE_32FC1)
- /stereo/error_depth (sensor_msgs::Image, TYPE_32FC1)
- /stereo/points2 (sensor_msgs::PointCloud2)

For color sensors, the following topics are additionally available:

- /stereo/left/image_rect_color (sensor_msgs::Image, format: RGB8)
- /stereo/right/image_rect_color (sensor_msgs::Image, format: RGB8)

#### Dynamic State (i.e. poses, IMU data, etc.)

These topics deliver the rc_visard's estimated dynamic state such as its
position, orientation, and velocity. For these topics to work properly,
the rc_visard's dynamics module must be turned on (see respective
service calls or startup-parameters).

- /pose (geometry_msgs/PoseStamped; same data as provided via tf if
  `enable_tf` is set to true)
- /pose_ins (geometry_msgs/PoseStamped)
- /pose_rt (geometry_msgs/PoseStamped)
- /pose_rt_ins (geometry_msgs/PoseStamped)
- /dynamics (nav_msgs/Odometry)
- /dynamics_ins (nav_msgs/Odometry)

This topic delivers raw measurements from the on-board IMU sensor:
- /imu (sensor_msgs/Imu)

#### TF

If the parameter `enable_tf` is set to true, the node subscribes to the
rc_visard's pose stream (same data published on `/pose` topic) and publishes them on tf.


Relevant Coordinate Frames
--------------------------

The following coordinate frames are relevant for interpreting the data
provided by the rc_visard:

- `camera`: The pupil's center of the rc_visard's left camera. All
  stereo-camera data such as images and point clouds are given in this frame.
- `world`: Relevant for navigation applications. The world frame’s origin is
  located at the origin of the rc_visard’s IMU coordinate frame at the instant
  when state estimation is switched on. Estimated poses of the rc_visard are
  given in this frame, i.e. as the rc_visard moves in the world and state
  estimation is running, the `camera` frame will change w.r.t. this frame.
- `imu`: The IMU coordinate frame is inside the rc_visard’s housing. The raw
  IMU measurements are given in this frame.

#### Running multiple rc_visard's in one ros environment

For operating multiple rc_visard's in one ros environment, each ros node must
be started in separate namespaces, e.g., `my_visard`. As a result, all
frame_ids in all ros messages will be prefixed, e.g., to `my_visard_world` or
`my_visard_camera`.


Services
--------

The following services are offered to start, stop, and restart the rc_visard's
dynamic module (which needs to be started for working dynamic-state estimates).

- `dynamics_start`
- `dynamics_restart`
- `dynamics_stop`
- `dynamics_start_slam`
- `dynamics_restart_slam`
- `dynamics_stop_slam`

The trajectory constructed and stored by the `rc_slam` node
can be retrieved by
- `slam_get_trajectory`

The onboard map of the `rc_slam` node can be saved on the rc_visard for loading it
after a SLAM restart or power cycle:
- `slam_save_map`
- `slam_load_map`
- `slam_remove_map`

The onboard `rc_slam` node can be "reset" (clears the internal state of the SLAM component,
including the trajectory) to free the memory with
- `slam_reset`


Launching
---------

- Using command line parameters:

      rosrun rc_visard_driver rc_visard_driver _device:=00_1e_06_32_03_40 _enable_tf:=True _autostart_dynamics:=True _autostop_dynamics:=True

- As a nodelet, and in a separate **namespace**:

      ROS_NAMESPACE=my_visard rosrun nodelet nodelet standalone rc_visard_driver _device:=00_1e_06_32_03_40

  Note that in this setup all frame_ids in all ros messages (including
  tf-messages) will be prefixed with `my_visard`, e.g., the frame_id of
  the published camera images will be `my_visard_camera`, the frame_id
  of the poses will be `my_visard_world`, and the frame_id of
  the Imu messages will be `my_visard_imu`.
