ROS interfaces for rc_visard
============================

This repositorty contains ROS interfaces for the [Roboception rc_visard][] 3D sensor.

Please also consult the manual for more details: https://doc.rc-visard.com

The `rc_visard` ROS package is a convenience metapackage which depends on all others.

Installation
------------

On Debian/Ubuntu add the ROS sources and

```bash
sudo apt-get install ros-${ROS_DISTRO}-rc-visard
```

rc_visard_driver
----------------

Nodelet/node providing a ROS interface to configure the rc_visard and receive
images/poses.

See the [rc_visard_driver README](rc_visard_driver/README.md) for more details.

rc_visard_description
---------------------

Package with xacro and urdf files for rc_visard_65 and rc_visard_160

See the [rc_visard_description README](rc_visard_description/README.md) for more details.

rc_hand_eye_calibration_client
------------------------------

Package for calibrating the rc_visard to a robot.
See the [rc_hand_eye_calibration_client README](rc_hand_eye_calibration_client/README.md) for more details.

rc_pick_client
--------------

ROS client for rc_visard's grasp generation modules.
See the [rc_pick_client README](rc_pick_client/README.md) for more details.

rc_tagdetect_client
--------------

ROS client for rc_visard's tag detection modules.
See the [rc_tagdetect_client README](rc_tagdetect_client/README.md) for more details

Acknowledgements
----------------

This FTP (Focused Technical Project) has received funding from the European Union’s Horizon 2020 research and innovation programme under the project ROSIN with the grant agreement No 732287.

ROSIN: ROS-Industrial Quality-Assured Robot Software Components: http://rosin-project.eu

![EU flag](rosin_eu_flag.jpg) ![ROSIN logo](rosin_ack_logo_wide.png)

[Roboception rc_visard]: http://roboception.com/rc_visard
