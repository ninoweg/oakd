# Spatial strawberry detection for fruit quality assessment
See paper: [Spatial strawberry detection for fruit quality assessment](../pdf/Spatial_strawberry_detection_for_fruit_quality_assessment.pdf)


### RECORD
```
cd scripts
python3 record.py
```
```
./convert.bash
```

### REPLAY
```
roslaunch oakd replay_publisher.launch 
```

### SLAM
```
roslaunch oakd depthimage_to_laserscan.launch
roslaunch oakd gmapping.launch
```
without odometry try (didn't seem to work reliably):
```
roslaunch oakd laser_scan_match.launch
```

### RTABMAP
http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping
```
roslaunch oakd replay_publisher.launch 
```
```
roslaunch rtabmap_launch rtabmap.launch args:="--delete_db_on_start" rgb_topic:=/replay_publisher/color/image depth_topic:=/replay_publisher/stereo/depth camera_info_topic:=/replay_publisher/color/camera_info frame_id:=oak-d_frame approx_sync:=true wait_imu_to_init:=false visual_odometry:=false odom_topic:=/odom
```
```
rosrun imu_filter_madgwick imu_filter_node    imu/data_raw:=/imu    imu/data:=/replay_publisher/imu/data     _use_mag:=false    _publish_tf:=false
```

### LINKS
* http://tools.luxonis.com/
* https://github.com/luxonis/depthai-ml-training/tree/master/colab-notebooks
* https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/
* https://docs.luxonis.com/projects/api/en/latest/samples/ObjectTracker/spatial_object_tracker/
* https://github.com/ultralytics/ultralytics/issues/1429

