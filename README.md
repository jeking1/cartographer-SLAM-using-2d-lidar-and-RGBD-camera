# cartographer-SLAM-using-2d-lidar-and-RGBD-camera

>本次设计主要是自己的毕业设计相关知识的学习
>题目为无光环境的AGV小车的SLAM建图。
>根据老师要求，要用深度图对二维雷达的激光信息进行补充

主要使用cartographer作为框架，将深度信息转化为2维雷达信息补充进原雷达信息。

这里是对cartographer源码的学习与注释。
注释部分主要学习自
https://blog.csdn.net/learnmoreonce/category_6989560.html?spm=1001.2014.3001.5482
https://www.zhihu.com/column/c_1040559544505704448


深度图转换为虚拟激光雷达信息：使用的是depthimage_to_laserscan包

融合部分的代码直接在depthimage_to_laserscan中进行的增添与修改。这部分代码是我自己写的，第一次写工程代码，有点乱。但是好在我的注释还是加的比较全的。

首先，对depthimage_to_laserscan的代码进行解读。在我写的这篇博客中：【待撰写】

其次，融合部分的代码思路，在这篇博客中：【待撰写】

然后，交代一下整体学习设计以及实现的过程：【待撰写】

