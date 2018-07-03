
rc_visard_description
=====================

Package with xacro and urdf files for rc_visard_65 and rc_visard_160

Build/Installation
------------------

See the [main README](../README.md)

Usage
-----

When the position of the sensor depends on the /camera frame (position from IMU or SLAM) rc_visard_65_standalone.urdf or rc_visard_160_standalone.urdf can be used for visualization.

When the sensor is mounted on the robot or staticaly mounted in the scene rc_visard_65.xacro or rc_visard_160.xacro can be used to include the sensor in the urdf.xacro files.


Model accuracy
--------------

The position of the camera frame of the model can be different than the position of this frame in real sensor. The position can vary up to +/- 3 mm in z axis.
The inertia of the real rc_visard can be different up to +/- 5% compared to data in urdf file.

Simulation
----------

The bahaviour of gazebo model of the sensor and real sensor are slightly different.
Depth data provided by the Gazebo model is created with openni kinect plugin, therefore no disparity, confidence and error data is published.
Moreover Gazebo does not publish the dynamics state of the sensor except IMU data.
Also the /imu topic published by Gazebo plugin does not include gravity in the acceleration and has the orientation part which is not provided by the rc_visard.

The simulation model works with Gazebo 2.X.
