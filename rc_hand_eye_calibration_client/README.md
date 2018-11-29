ROS Hand-eye calibration client for Roboception's rc_visard
=========================================================

This node provides ROS service calls and topics to calibrate the rc_visard to a robot (aka hand-eye calibration). The calibration routine consists of several steps:

1. Setting calibration parameters, i.e. grid size and mounting, via dynamic reconfigure.
0. For a user-defined number of robot calibration poses repeat
    
    1. Move the robot to the pose (calibration grid must be visible in the rc_visard's view).
    0. Send the robot pose to rc_visard (`set_pose`)

0. Trigger the calibration tranformation to be calculated (`calibrate`).

After the calibration transform is calculated and tested, it should be saved to the rc_visard (`save_calibration`).

For detailed instructions on the calibration routine consult the rc_visard manual: https://doc.rc-visard.com.


Installation
------------

This package relies on git submodules for the cpr library which need to be initialized before building from source.
~~~
git submodule update --init --recursive
~~~

Configuration
-------------

### Parameters

* `host`: The IP address or hostname of the rc_visard that should be calibrated

### Dynamic reconfigure parameters

* `grid_width`: The width of the calibration pattern in meters
* `grid_height`: The height of the calibration pattern in meters
* `robot_mounted` Whether the camera is mounted on the robot or not

Services
--------

The following services are offered to follow the calibration routine:

* `reset_calibration`: Deletes all previously provided poses and corresponding images. The last saved calibration result is reloaded. This service might be used to (re-)start the hand-eye calibration from scratch.
* `set_pose`: Provides a robot pose as calibration pose to the hand-eye calibration routine.
* `calibrate`: Calculates and returns the hand-eye calibration transformation with the robot poses configured by the `set_pose` service.
* `get_calibration`
* `save_calibration`: Persistently saves the result of hand-eye calibration to the rc_visard and overwrites the existing one. The stored result can be retrieved any time by the get_calibration service.
* `remove_calibration`: Removes the stored hand-eye calibration on the rc_visard. After this call the `get_calibration` service reports again that no hand-eye calibration is available.


Launching
---------

Using command line parameters:

~~~
rosrun rc_hand_eye_calibration_client rc_hand_eye_calibration_client_node _host:="10.0.2.44"
~~~
