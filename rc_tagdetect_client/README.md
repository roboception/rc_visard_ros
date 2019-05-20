ROS client for Roboception's tag detection modules
==================================================

This node provides ROS service calls and parameters for TagDetect node.
For detail description of the TagDetect check rc_visard manual: https://doc.rc-visard.com/latest/en/tagdetect.html

Installation
------------

On Debian/Ubuntu add the ROS sources and

```bash
sudo apt-get install ros-${ROS_DISTRO}-rc-tagdetect-client
```

### From Source

This package relies on git submodules for the cpr library which need to be initialized before building from source.

~~~bash
git submodule update --init --recursive
~~~

Configuration
-------------

### Parameters

* `host`: The IP address or hostname of the rc_visard that should be calibrated

### Dynamic reconfigure parameters

* `use_cached_images`: Use most recently received image pair instead of waiting for a new pair
* `forget_after_n_detections`: Number of detection runs after which to forget about a previous tag during tag re-identification
* `max_corner_distance`: Maximum distance of corresponding tag corners in meters during tag re-identification
* `quality`: Quality of tag detection (H, M or L).
* `detect_inverted_tags`: Detect tags with black and white exchanged
* `publish_visualization`: Whether or not the tf and markers should be published


Services
--------

The following services are offered by the node:

* `detect`: Trigger single detection
* `start_continuous_detection`: Starts continuous detection.
* `stop_continuous_detection`: Stops continuous detection.


Launching
---------

Using command line parameters:

~~~
rosrun rc_tagdetect_client rc_april_node _host:=sensor_ip
rosrun rc_tagdetect_client rc_qr_node _host:=sensor_ip
~~~
