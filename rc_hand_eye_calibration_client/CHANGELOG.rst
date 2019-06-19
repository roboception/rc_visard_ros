^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rc_hand_eye_calibration_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.4 (2019-06-19)
------------------
* fix published tf transform

2.6.3 (2019-06-12)
------------------
* add debian package dependencies
* tf: invert transform so that the sensor is the child

2.6.2 (2019-06-11)
------------------

2.6.1 (2019-05-20)
------------------

2.6.0 (2019-05-20)
------------------
* Enforce bool for robot_mounted parameter.
  This should make it compatible to older rc_visard firmware versions.
* Publish hand-eye calibration via tf

2.5.0 (2019-02-05)
------------------
* rename `ip` parameter to `host`
* allow hostname as device parameter

2.4.2 (2018-10-29)
------------------
* depend on curl

2.4.1 (2018-10-29)
------------------

2.4.0 (2018-10-16)
------------------
* first release
