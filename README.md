ROS interfaces for rc_visard
============================

This repositorty contains ROS interfaces for the [Roboception rc_visard][] 3D sensor.

Please also consult the manual for more details: https://doc.rc-visard.com

**This stack is mostly in maintenance mode!** Please consider migrating to

* [rc_genicam_driver for ROS1](https://github.com/roboception/rc_genicam_driver_ros)
* [rc_genicam_driver for ROS2](https://github.com/roboception/rc_genicam_driver_ros2)
* [rc_reason_clients for ROS1](https://github.com/roboception/rc_reason_clients_ros)
* [rc_reason_clients for ROS2](https://github.com/roboception/rc_reason_clients_ros2)

Installation
------------

On Debian/Ubuntu add the ROS sources and

```bash
sudo apt-get install ros-${ROS_DISTRO}-rc-visard
```

rc_visard
---------

The `rc_visard` ROS package is a convenience metapackage which depends on all others.

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

**This package is not developed anymore**. A new client is available in the
[rc_reason_clients_ros repository](https://github.com/roboception/rc_reason_clients_ros/tree/master/rc_reason_clients#rc_hand_eye_calibration_client).

Package for calibrating the rc_visard to a robot.
See the [rc_hand_eye_calibration_client README](rc_hand_eye_calibration_client/README.md) for more details.

rc_pick_client
--------------

**This package is not developed anymore**. New clients are available in the
[rc_reason_clients_ros repository](https://github.com/roboception/rc_reason_clients_ros/tree/master/rc_reason_clients#rc_itempick_client).

ROS client for rc_visard's ItemPick and BoxPick modules.
See the [rc_pick_client README](rc_pick_client/README.md) for more details.

rc_tagdetect_client
-------------------

**This package is not developed anymore**. New clients are available in the
[rc_reason_clients_ros repository](https://github.com/roboception/rc_reason_clients_ros/tree/master/rc_reason_clients#rc_april_tag_detect_client-and-rc_qr_code_detect_client).

ROS client for rc_visard's tag detection modules.
See the [rc_tagdetect_client README](rc_tagdetect_client/README.md) for more details

rc_silhouettematch_client
-------------------------

**This package is not developed anymore**. A new client is available in the
[rc_reason_clients_ros repository](https://github.com/roboception/rc_reason_clients_ros/tree/master/rc_reason_clients#rc_silhouettematch_client).

ROS client for rc_visard's SilhouetteMatch module.
See the [rc_silhouettematch_client README](rc_silhouettematch_client/README.md) for more details

Acknowledgements
----------------

This FTP (Focused Technical Project) has received funding from the European Union’s Horizon 2020 research and innovation programme under the project ROSIN with the grant agreement No 732287.

ROSIN: ROS-Industrial Quality-Assured Robot Software Components: http://rosin-project.eu

![EU flag](rosin_eu_flag.jpg) ![ROSIN logo](rosin_ack_logo_wide.png)

[Roboception rc_visard]: http://roboception.com/rc_visard
