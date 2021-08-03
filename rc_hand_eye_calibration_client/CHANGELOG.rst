^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rc_hand_eye_calibration_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.3 (2021-08-03)
------------------

3.2.2 (2021-08-02)
------------------

3.2.1 (2021-02-11)
------------------

3.2.0 (2021-01-28)
------------------
* update cmake files for better version handling
* add new error values to hand_eye_calibration_client

3.1.0 (2020-11-17)
------------------

3.0.5 (2020-10-20)
------------------

3.0.4 (2020-09-23)
------------------

3.0.3 (2020-09-22)
------------------

3.0.2 (2020-07-28)
------------------

3.0.1 (2020-05-14)
------------------

3.0.0 (2020-05-13)
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
