# cartographer-SLAM-using-2d-lidar-and-RGBD-camera

>本次设计主要是自己的毕业设计相关知识的学习
>题目为无光环境下AGV小车的2维地图重建。
>总结题目也就是为了排除视觉传感器的缺陷，使用雷达作为主传感器，同时使用RGBD相机的深度图作为2d雷达的补充信息。以此获取更加完善的二维地图。

主要使用cartographer作为框架，将深度信息转化为2维雷达信息补充进原雷达信息。

这里是对cartographer源码的学习与注释。
注释部分主要学习自https://blog.csdn.net/learnmoreonce/category_6989560.html?spm=1001.2014.3001.5482

