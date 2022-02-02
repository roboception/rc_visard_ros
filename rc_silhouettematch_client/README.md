ROS client for Roboception's SilhouetteMatch module
===================================================

**This package is not developed anymore**. A new client is available in the
[rc_reason_clients_ros repository](https://github.com/roboception/rc_reason_clients_ros/tree/master/rc_reason_clients#rc_silhouettematch_client).

This node provides ROS services and parameters for Roboception's SilhouetteMatch node.
For a detailed description of the SilhouetteMatch module, check the rc_visard manual: https://doc.rc-visard.com/latest/en/silhouettematch.html.

Installation
------------

On Debian/Ubuntu add the ROS sources and

```bash
sudo apt-get install ros-${ROS_DISTRO}-rc-silhouettematch-client
```

### From Source

This package relies on git submodules for the cpr library which need to be initialized before building from source.

~~~bash
git submodule update --init --recursive
~~~

Configuration
-------------

### Parameters

* `device`: The ID of the device, i.e. Roboception rc_visard sensor. This can be either:
  * serial number, e.g. `02912345`.
    IMPORTANT: preceed with a colon (`:02912345`) when passing this on the command line or
    setting it via rosparam (see https://github.com/ros/ros_comm/issues/1339).
    This is not neccessary when specifying it as a string in a launch file.
  * user defined name (factory default is the name of the rc_visard's model), must be unique among all
    reachable sensors.
* `host`: If `device` is not used: The IP address or hostname of the rc_visard.

### Dynamic reconfigure parameters

* `max_number_of_detected_objects`: Maximum number of detected objects.
* `edge_sensitivity`: Sensitivity of the edge detector.
* `match_max_distance`: Maximum allowed distance in pixels between the template and the detected edges in the image.
* `match_percentile`: Percentage of template pixels that must be within the maximum distance to successfully match the template.
* `quality`: Detection quality (H(igh), M(edium), or L(ow)).
* `publish_vis`: Whether TF and markers should be published.


Services
--------

The following services are offered by the node:

* `detect_object`: Triggers detection of an object.
* `calibrate_base_plane`: Triggers base-plane calibration.
* `get_base_plane_calibration`: Returns the configured base-plane calibration.
* `delete_base_plane_calibration`: Deletes the configured base-plane calibration.
* `set_region_of_interest`: Sets a region of interest.
* `get_regions_of_interest`: Returns the configured regions of interest with the requested ids.
  If no ids are provided, all configured regions of interest are returned.
* `delete_regions_of_interest`: Deletes the configured regions of interest with the requested ids.


Launching
---------

Using command line parameters:

~~~
rosrun rc_silhouettematch_client rc_silhouettematch_client _device:=:<serial_number>
~~~
