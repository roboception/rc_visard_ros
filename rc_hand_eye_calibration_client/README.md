ROS Hand-eye calibration client for Roboception's rc_visard
=========================================================

This node provides ROS service calls and topics to calibrate the rc_visard to a robot (aka hand-eye calibration).

It also provides the new or pre-existing calibration via /tf or /tf_static.
The default behavior is to request the existing calibration of the rc_visard
once on startup, broadcast it (latched) on /tf_static and only broadcast again
if the advertised ROS services `calibrate` or `get_calibration` are called.

The calibration routine consists of several steps:

1. Setting calibration parameters, i.e. grid size and mounting, via dynamic reconfigure.
2. For a user-defined number of robot calibration poses repeat

    1. Move the robot to the pose (calibration grid must be visible in the rc_visard's view).
    2. Send the robot pose to rc_visard (`set_pose`)

3. Trigger the calibration tranformation to be calculated (`calibrate`).

After the calibration transform is calculated and tested, it should be saved to the rc_visard (`save_calibration`).

For detailed instructions on the calibration routine consult the rc_visard manual: https://doc.rc-visard.com.

Installation
------------

On Debian/Ubuntu add the ROS sources and

```bash
sudo apt-get install ros-${ROS_DISTRO}-rc-hand-eye-calibration-client
```

### From Source

This package relies on git submodules for the cpr library which need to be initialized before building from source.

~~~bash
git submodule update --init --recursive
~~~

Configuration
-------------

### Parameters

* `host`: The IP address or hostname of the rc_visard that should be calibrated
* `rc_visard_frame_id`: Name of the frame on the rc_visard when calibrating. Default: "camera"
* `end_effector_frame_id`: Name of the frame calibrated to when using a `robot_mounted` (see below) rc_visard. Default: "end_effector".
* `base_frame_id`: Name of the frame calibrated to when using a statically (externally) mounted rc_visard (`robot_mounted == false`). Default: "base_link"
* `calibration_request_period`: Decimal number, controls the requesting of calibration from the rc_visard. Default: 0.0.
  * If positive: Interval in seconds for automated requests to get the latest calibration
  * If zero: Only request once on startup (default)
  * If negative, the calibration is never requested.
  * In any case, new results will be broadcast when the services `calibrate` or `get_calibration` (see further below) of this node are called.
* `calibration_publication_period`: Decimal number, controls broadcasting of the calibration on /tf or /tf_static. Default: 0.0
  * If positive: Interval in seconds.
  * If negative or zero, the calibration is broadcast on /tf_static only when changed in a manual or periodic calibration request (default)

### Dynamic reconfigure parameters

* `grid_width`: The width of the calibration pattern in meters
* `grid_height`: The height of the calibration pattern in meters
* `robot_mounted` Whether the camera is mounted on the robot or not

Services
--------

The following services are offered to follow the calibration routine:

* `reset_calibration`: Deletes all previously provided poses and corresponding images. The last saved calibration result is reloaded. This service might be used to (re-)start the hand-eye calibration from scratch.
* `set_pose`: Provides a robot pose as calibration pose to the hand-eye calibration routine.
* `calibrate`: Calculates and returns the hand-eye calibration transformation with the robot poses configured by the `set_pose` service. Broadcast the result via /tf or /tf_static.
* `get_calibration`: Returns the existing hand-eye calibration transformation. Broadcast the result via /tf or /tf_static.
* `save_calibration`: Persistently saves the result of hand-eye calibration to the rc_visard and overwrites the existing one. The stored result can be retrieved any time by the get_calibration service.
* `remove_calibration`: Removes the stored hand-eye calibration on the rc_visard. After this call the `get_calibration` service reports again that no hand-eye calibration is available. Periodic broadcasting via /tf will be stopped.


Launching
---------

Using command line parameters:

~~~bash
rosrun rc_hand_eye_calibration_client rc_hand_eye_calibration_client_node _host:="10.0.2.44"
~~~
