
rc_visard_description
----------------

Package with xacro and urdf files for rc_visard_65 and rc_visard_160 

Build/Installation
------------------

See the [main README](../README.md)

Usage
---------

When the position of the sensor depends on the /camera frame (position from IMU or SLAM) rc_visard_65_standalone.urdf or rc_visard_160_standalone.urdf can be used for visualization.

When the sensor is mounted on the robot or staticaly mounted in the scene rc_visard_65.xacro or rc_visard_160.xacro can be used to include the sensor in the urdf.xacro files.


Model accuracy
-------------

The position of the camera frame of the model can be different than the position of this frame in real sensor. The position can vary +/- 1.5cm in z axis. 


