ROS client for Roboception's grasp generation modules
=====================================================

This node provides ROS service calls and parameters for ItemPick node.
For detail description of the ItemPick module check rc_visard manual: https://doc.rc-visard.com/latest/en/itempick.html

Installation
------------

On Debian/Ubuntu add the ROS sources and

```bash
sudo apt-get install ros-${ROS_DISTRO}-rc-pick-client
```

### From Source

This package relies on git submodules for the cpr library which need to be initialized before building from source.

~~~bash
git submodule update --init --recursive
~~~

Configuration
-------------

### Parameters

* `host`: The IP address or hostname of the rc_visard that should be used

**Since version 2.7, the device ID can be used instead of the sensor's IP address:**

* `device`: The ID of the device, i.e. Roboception rc_visard sensor. This can be either:

  * serial number, e.g. `02912345`

    IMPORTANT: preceed with a colon (`:02912345`) when passing this on the commandline or
    setting it via rosparam (see https://github.com/ros/ros_comm/issues/1339).
    This is not neccessary when specifying it as a string in a launch file.

  * user defined name (factory default is the name of the rc_visard's model), must be unique among all
    reachable sensors

### Dynamic reconfigure parameters

* `load_carrier_crop_distance`: Safety margin in meters by which the load carrier inner dimensions are reduced to define the region of interest for grasp computation
* `load_carrier_model_tolerance`: Indicates how much the estimated load carrier dimensions are allowed to differ from the load carrier model dimensions in meters
* `cluster_max_dimension`: Indicates how much the estimated load carrier dimensions are allowed to differ from the load carrier model dimensions in meters
* `cluster_max_curvature`: Maximum curvature allowed within one cluster. The smaller this value, the more clusters will be split apart.
* `clustering_patch_size`: Size in pixels of the square patches the depth map is subdivided into during the first clustering step
* `clustering_max_surface_rmse`: Maximum root-mean-square error (RMSE) in meters of points belonging to a surface
* `clustering_discontinuity_factor`: Factor used to discriminate depth discontinuities within a patch. The smaller this value, the more clusters will be split apart.

Services
--------

The following services are offered by the node:

* `start`: Starts the component.
* `stop`: Stops the component.
* `set_region_of_interest`: Persistently stores a region of interest on the rc_visard.
* `get_region_of_interests`: Returns the configured regions of interest with the requested region_of_interest_ids. If no region_of_interest_ids are provided, all configured regions of interest are returned.
* `delete_regions_of_interest`: Deletes the configured regions of interest with the requested region_of_interest_ids
* `set_load_carrier`: Persistently stores a load carrier on the rc_visard.
* `get_load_carriers`: Returns the configured load carriers with the requested load_carrier_ids. If no load_carrier_ids are provided, all configured load carriers are returned.
* `delete_load_carriers`: Deletes the configured load carriers with the requested load_carrier_ids
* `detect_load_carrier`: Triggers a load carrier detection.
* `compute_grasps`: Triggers the computation of grasping poses for a suction device. All images used by the node are guaranteed to be newer than the service trigger time.

For the BoxPick node, an additional service is offered:
* `detect_items`: Triggers the detection of rectangles.


Launch
------

Using command line parameters:

**For the ItemPick module:**
~~~
rosrun rc_pick_client rc_itempick_client_node _host:=<sensor_ip>
~~~
Since version 2.7:
~~~
rosrun rc_pick_client rc_itempick_client_node _device:=:<serial_number>
~~~


**For the BoxPick module:**
~~~
rosrun rc_pick_client rc_boxpick_client_node _host:=<sensor_ip>
~~~

Since version 2.7:
~~~
rosrun rc_pick_client rc_boxpick_client_node _device:=:<serial_number>
~~~