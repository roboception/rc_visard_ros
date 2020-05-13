^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rc_hand_eye_calibration_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2019-05-13)
------------------
* add set_calibration service
* refactor an cleanup
* added 4DOF calibration parameters
* update cpr for support of curl >= 7.62.0
  and set EXCLUDE_FROM_ALL to not install it
* update nlohmann json to 3.7.3
* Using own trigger message with integer status return field
* Contributors: Annett Stelzer, Christian Emmerich, Felix Ruess, Heiko Hirschmueller

2.7.0 (2019-07-19)
------------------
* add `device` parameter that can take serial number or GEV name
  (has precedence over old `host` parameter)

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
