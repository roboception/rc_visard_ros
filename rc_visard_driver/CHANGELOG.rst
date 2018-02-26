^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rc_visard_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
