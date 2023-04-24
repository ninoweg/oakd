# oakd
This package is used to fully integrate an DepthAI SpatialTracking pipeline into ROS (including IMU).

Pipeline: StereoCamera + IMU + YoloSpatialDetectionNetwork + ObjectTracker

The main node uses Luxonis depthai_bridge wherever possible to pbulish IMU, RGB, DEPTH and SPATIAL DETECTION as ROS topics. It adds a timer based custom ROS msg and publisher for Tracklets. 

```
roslaunch oakd oakd_node.launch
```

To publish and visualize Tracklets as visualization_msgs/Markers use the marker_publisher_node

```
roslaunch oakd marker_publisher.launch
```

costum msgs can be found here: https://github.com/ninoweg/oakd_msgs

## SLAM
Used with https://github.com/ninoweg/rc_car or other mobile platform which provides reliable odometry this package can be used for SLAM.
```
roslaunch oakd depthimage_to_laserscan.launch
roslaunch oakd gmapping.launch
```
without odometry try (didn't seem to work reliably):
```
roslaunch oakd laser_scan_match.launch
```

## useful links
* http://tools.luxonis.com/
* https://github.com/luxonis/depthai-ml-training/tree/master/colab-notebooks
* https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
* https://docs.luxonis.com/projects/api/en/latest/samples/ObjectTracker/spatial_object_tracker/
* https://github.com/ultralytics/ultralytics/issues/1429

