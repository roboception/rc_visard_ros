ROS Hand-eye calibration client for Roboception's rc_visard
=========================================================

This package provides ROS service calls and topics to calibrate the rc_visard to a robot.

Installation
------------

This repository relies on git submodules for the cpr library which need to be initialized before building from source.
~~~
git submodule update --init --recursive
~~~

Parameters
----------

* `ip`: IP address of the rc_visard that should be calibrated

Documentation
-------------

`make doc` will build the Doxygen documentation.
This README.md will be used as the main page.

If needed adjust the `doc/Doxyfile.in` to your specific needs (e.g. INPUT directories)
