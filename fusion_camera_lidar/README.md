depthimage_to_laserscan
=======================

Converts a depth image to a laser scan for use with navigation and localization.

ROS Wiki Page:
http://www.ros.org/wiki/depthimage_to_laserscan


fusion_camera_lidar

node name: depthimage_to_laserscan

Subscriptions:
/laser_scan 
/camera/depth/image_rect_raw 
/camera/depth/camera_info

Publications：
/scan       (cartographer订阅使用)
/depth_scan (发布深度图转换的虚拟激光图，方便在rviz中对比观察)



运行方式
将fusion_camera_lidar下载到src文件夹下catkin_make
```
roslaunch depthimage_to_laserscan zed2_depthimage_to_laserscan.launch
```


注意事项：
depthimage_to_laserscan使用了延迟订阅方式，即只有cartographer或rviz中订阅了/scan，在depthimage_to_laserscan节点中才会订阅/camera/depth/image_rect_raw和/laser_scan