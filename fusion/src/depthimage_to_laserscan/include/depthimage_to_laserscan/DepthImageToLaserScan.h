/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey
 */

#ifndef DEPTH_IMAGE_TO_LASERSCAN
#define DEPTH_IMAGE_TO_LASERSCAN

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depthimage_to_laserscan/depth_traits.h>
#include <sstream>
#include <limits.h>
#include <math.h>

namespace depthimage_to_laserscan
{
  /**
   * -类DepthImageToLaserScan
   * --参数
   * ---image_geometry::PinholeCameraModel cam_model_; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.
   * ---float scan_time_; ///< Stores the time between scans. 存储扫描间隔时间
   * ---float range_min_; ///< Stores the current minimum range to use.存储要使用的最小范围
   * ---float range_max_; ///< Stores the current maximum range to use.存储要使用的最大范围
   * ---int scan_height_; ///< Number of pixel rows to use when producing a laserscan from an area.从区域内生成激光扫描时使用的像素行数
   * ---std::string output_frame_id_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.输出的帧的id是每个激光帧的id，但不一定是相机帧的id
   * --函数
   * ---DepthImageToLaserScan()
   * ---~DepthImageToLaserScan()
   * ---sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr& depth_msg,
	 *			   const sensor_msgs::CameraInfoConstPtr& info_msg)
   * ---void set_scan_time(const float scan_time)
   * ---void set_range_limits(const float range_min, const float range_max);
   * ---void set_scan_height(const int scan_height);
   *  ---void set_output_frame(const std::string& output_frame_id);
   *  ---double magnitude_of_ray(const cv::Point3d& ray) const;
   *  ---double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;
   *  ---bool use_point(const float new_value, const float old_value, const float range_min, const float range_max) const;
   *  ---void convert(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& cam_model,
   *     const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height)
   *
   * */
  class DepthImageToLaserScan
  {
  public:
    DepthImageToLaserScan();
    ~DepthImageToLaserScan();

    /**
     * Converts the information in a depth image (sensor_msgs::Image) to a sensor_msgs::LaserScan.
     *
     * This function converts the information in the depth encoded image (UInt16 or Float32 encoding) into
     * a sensor_msgs::LaserScan as accurately as possible.  To do this, it requires the synchornized Image/CameraInfo
     * pair associated with the image.
     * 这个函数用于将深度编码图像中的信息尽可能的转化为一个激光帧。为此，它需要与图像相联系的同步化的图像对
     * 
     * @param depth_msg UInt16 or Float32 encoded depth image.
     * @param info_msg CameraInfo associated with depth_msg
     * @return sensor_msgs::LaserScanPtr for the center row(s) of the depth image. 返回的是激光帧
     *
     */
    sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr& depth_msg,
					   const sensor_msgs::CameraInfoConstPtr& info_msg);
    sensor_msgs::LaserScanPtr fusion_msg(sensor_msgs::LaserScanPtr& laser_scan, sensor_msgs::LaserScanPtr& scan_msg);
  
    /**
     * Sets the scan time parameter. 
     * 设置扫描时间参数
     *
     * This function stores the desired value for scan_time.  In sensor_msgs::LaserScan, scan_time is defined as
     * "time between scans [seconds]".  This value is not easily calculated from consquetive messages, and is thus
     * left to the user to set correctly.
     *
     * 此函数用于存储扫描时间。在sensor_msgs::LaserScan中，扫描时间定义为“两帧之间的时间【s】”。这个值不容易从消耗性的信息中计算出来
     * 所以留给用户去设置正确值。
     * @param scan_time The value to use for outgoing sensor_msgs::LaserScan. 用于输出sensor_msgs::LaserScan
     *
     */
    void set_scan_time(const float scan_time);

    /**
     * Sets the minimum and maximum range for the sensor_msgs::LaserScan.
     *
     * range_min is used to determine how close of a value to allow through when multiple radii correspond to the same
     * angular increment.  range_max is used to set the output message.
     * 当多个半径对应相同的角度增量时，range_min用于确定允许通过的值的接近程度。range_max用于设置输出信息。
     *
     * @param range_min Minimum range to assign points to the laserscan, also minimum range to use points in the output scan.为激光扫描指定点的最小范围，以及在输出扫描中使用点的最小范围。
     * @param range_max Maximum range to use points in the output scan.输出扫描中使用点的最大范围。
     *
     */
    void set_range_limits(const float range_min, const float range_max);

    /**
     * Sets the number of image rows to use in the output LaserScan.
     *
     * scan_height is the number of rows (pixels) to use in the output.  This will provide scan_height number of radii for each
     * angular increment.  The output scan will output the closest radius that is still not smaller than range_min.  This function
     * can be used to vertically compress obstacles into a single LaserScan.
     * scan_height 是要在输出中使用的像素的行数。这将为每个角度增量提供扫描高度的半径数。输出帧将输出不小于range_min的最近半径。这个函数可以被用于将障碍物垂直压缩为单个激光扫描帧。
     * double magnitude_of_ray(const cv::Point3d& ray) const;
     * @param scan_height Number of pixels centered around the center of the image to compress into the LaserScan.
     *
     */
    void set_scan_height(const int scan_height);

    /**
     * Sets the frame_id for the output LaserScan.
     *
     * Output frame_id for the LaserScan.  Will probably NOT be the same frame_id as the depth image.
     * Example: For OpenNI cameras, this should be set to 'camera_depth_frame' while the camera uses 'camera_depth_optical_frame'.
     * 输出的虚拟激光帧id，不一定和深度图id相同。
     * 
     * @param output_frame_id Frame_id to use for the output sensor_msgs::LaserScan.
     *
     */
    void set_output_frame(const std::string& output_frame_id);

  private:
    /**
     * Computes euclidean length of a cv::Point3d (as a ray from origin)
     *
     * This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
     * 此函数用于计算cv:：Point3d的长度，假定它是从原点（0,0,0）开始的向量。
     * @param ray The ray for which the magnitude is desired.
     * @return Returns the magnitude of the ray.
     *
     */
    double magnitude_of_ray(const cv::Point3d& ray) const;

    /**
     * Computes the angle between two cv::Point3d
     *
     * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
     * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
     * 计算两个cv:：Point3d的角度，假定它们是从原点（0,0,0）开始的向量。
     * 
     * @param ray1 The first ray
     * @param ray2 The second ray
     * @return The angle between the two rays (in radians)
     *
     */
    double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;

    /**
     * Determines whether or not new_value should replace old_value in the LaserScan.
     *
     * Uses the values of range_min, and range_max to determine if new_value is a valid point.  Then it determines if
     * new_value is 'more ideal' (currently shorter range) than old_value.
     * 使用range_min和range_max确定新值是不是有效值。然后确定new_value是不是比old_value更理想。
     *
     * @param new_value The current calculated range.
     * @param old_value The current range in the output LaserScan.
     * @param range_min The minimum acceptable range for the output LaserScan.
     * @param range_max The maximum acceptable range for the output LaserScan.
     * @return If true, insert new_value into the output LaserScan.
     *
     */
    bool use_point(const float new_value, const float old_value, const float range_min, const float range_max) const;

    /**
    * Converts the depth image to a laserscan using the DepthTraits to assist.
    *
    * This uses a method to inverse project each pixel into a LaserScan angular increment.  This method first projects the pixel
    * forward into Cartesian coordinates, then calculates the range and angle for this point.  When multiple points coorespond to
    * a specific angular measurement, then the shortest range is used.
    * 这使用一种方法来将每个像素反向投影为Lsaer_scan角度增量。这种方法首先将像素投影到笛卡尔坐标系，然后计算该点的范围和角度。当多个点响应同一个角度
    * 使用最小的那个角度。
    *
    * @param depth_msg The UInt16 or Float32 encoded depth message.
    * @param cam_model The image_geometry camera model for this image. 此图像的相机模型
    * @param scan_msg The output LaserScan. 输出的Laser_scan
    * @param scan_height The number of vertical pixels to feed into each angular_measurement.
    *
    */




  //  image_geometry::PinholeCameraModel 针孔相机模型参数    http://docs.ros.org/en/diamondback/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html
  // 
   template<typename T>
    void convert(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& cam_model,
        const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height) const{
      // Use correct principal point from calibration
      const float center_x = cam_model.cx();
      const float center_y = cam_model.cy();

      // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
      const double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters( T(1) );
      const float constant_x = unit_scaling / cam_model.fx();

      const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
      const int row_step = depth_msg->step / sizeof(T);

      const int offset = (int)(center_y - scan_height/2);
      depth_row += offset*row_step; // Offset to center of image
        // depth_row += (int)center_y*row_step;
      for(int v = offset; v < offset+scan_height_; ++v, depth_row += row_step){
        // for(int v = (int)center_y; v < center_y+scan_height_; ++v, depth_row += row_step){
        //depth_msg->width = 1280
        for (int u = 0; u < (int)depth_msg->width; ++u) // Loop over each pixel in row
        {
          const T depth = depth_row[u];

          double r = depth; // Assign to pass through NaNs and Infs
          const double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
          const int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;

          if (depthimage_to_laserscan::DepthTraits<T>::valid(depth)){ // Not NaN or Inf
            // Calculate in XYZ
            double x = (u - center_x) * depth * constant_x;
            double z = depthimage_to_laserscan::DepthTraits<T>::toMeters(depth);

            // Calculate actual distance 计算激光的真实距离
            r = hypot(x, z);
          }

          // Determine if this point should be used. 判断激光距离是否超出预设的有效范围
          if(use_point(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max)){
            scan_msg->ranges[index] = r;
          }
        }
      }
      
    }

    /**
     * sensor_msgs::LaserScan
     * 
     * @param header Header 是一个结构体，包含seq、stamp、frame—id。seq扫描顺序增加的id序列，stamp激光数据的时间戳，frame-id是扫描数据的名字。
     * @param angle_min float32 开始扫描的角度[rad]
     * @param angle_max float32 结束扫描的角度[rad]
     * @param angle_increment float32 每次扫描增加的角度[rad]
     * @param time_increment float32 测量的时间间隔[seconds] //3d的才会使用
     * @param scan_time float32 扫描的时间间隔[seconds]
     * @param range_min float32 测距最小值[m]
     * @param range_max float32 测距最大值[m]
     * @param range float32[] 转一圈是360个测量数据[m]
     * @param intensities float32[] 强度数据 如果设备不提供强度数据，则数组为空
     * 
     * */
    /**
    * tanjx的修改
    * 激光和虚拟激光数据融合
    * 
    */

   /**
   *       data: [ 9.4739916886681130e-01, 2.2130862573608440e-02,
       -3.1928833325416595e-01, -1.7949002487082114e-01,
       3.1960787078675879e-01, -1.1809700273904222e-01,
       9.4016163869581981e-01, -5.5618504000562126e-02,
       -1.6900407143905755e-02, -9.9275541945929602e-01,
       -1.1895820010645553e-01, -5.9308159627838385e-02, 0., 0., 0., 1. ]

2021.5.7.11h51
data: [ 9.2815982406332631e-01, 1.6620644424378272e-01,
       -3.3300864686457243e-01, -1.8846368838351693e-01,
       3.4972254724462082e-01, -8.3364791391535631e-02,
       9.3313688787068072e-01, -1.0806183779318781e-01,
       1.2733216774825257e-01, -9.8256080190901351e-01,
       -1.3550199116045025e-01, 5.4403586555472061e-02, 0., 0., 0., 1. ]

2021.5.7.12h16
   data: [ 9.3016524337823114e-01, -1.5146811419525830e-01,
       -3.3443987560284760e-01, -1.8498225722409428e-01,
       2.9179133748856401e-01, -2.4786642655159263e-01,
       9.2380736625942683e-01, -8.8327301508334691e-02,
       -2.2282377650904073e-01, -9.5688016228296635e-01,
       -1.8635965135074972e-01, 1.0238927970398511e-01, 0., 0., 0., 1. ]

2021.5.7.14h7
   data: [ 9.4666707641078052e-01, 6.1529997776840734e-02,
       -3.1628390065485079e-01, -1.9404330939716322e-01,
       3.2119116331499598e-01, -2.5832894171072995e-01,
       9.1109955245460106e-01, -1.0009342076037800e-01,
       -2.5645331899296991e-02, -9.6409554363050387e-01,
       -2.6431439556630032e-01, 1.0820979329313304e-01, 0., 0., 0., 1. ]

2021.5.7.17h36
   data: [ 9.4398917910098268e-01, 1.0584976027731757e-02,
       -3.2980659184246952e-01, -1.9170186586126101e-01,
       3.2532664767222541e-01, -1.9705212104754133e-01,
       9.2484216702365851e-01, -1.1474548681100917e-01,
       -5.5199656290638444e-02, -9.8033587095094987e-01,
       -1.8945864475456875e-01, 1.2973188784604420e-01, 0., 0., 0., 1. ]
       B =

0.8405    0.5106   -0.1813    0.1273
   -0.3626    0.2815   -0.8884    0.0436
   -0.4026    0.8125    0.4217    0.0028
         0         0         0    1.0000
      **/
  double R[4][4] = {0,0,0,0,
                   0,9.4398917910098268e-01, 1.0584976027731757e-02,
       -3.2980659184246952e-01,
                   0,-1.9170186586126101e-01,
       3.2532664767222541e-01, -1.9705212104754133e-01,
                   0,9.2484216702365851e-01, -1.1474548681100917e-01,
       -5.5199656290638444e-02};
  
  double T[4][2] = {0,0,
                   0,-9.8033587095094987e-01,
                   0,-1.8945864475456875e-01, 
                   0, 1.2973188784604420e-01};

  double inv[4][4] = {0.8405 ,   0.5106 ,  -0.1813,    0.1273,
   -0.3626  ,  0.2815  , -0.8884   , 0.0436,
   -0.4026  ,  0.8125  ,  0.4217,    0.0028,
         0      ,   0     ,    0  ,  1.0000};
  struct kinect_Triplet{
    double x,y,z;
  };


   kinect_Triplet laser_to_kinect(double r,double a,const double l = 0.01){
// r,a是雷达测得的数据,r是距离，a是偏移角，而l是相机与雷达硬件在y轴(高度)上的距离，需要手动输入
    kinect_Triplet kinect_Triplet_temp;
    double sina = std::sin(a),cosa = std::cos(a);
    double rsina = r*sina, rcosa = r*cosa;
    kinect_Triplet_temp.x = R[1][1]*rsina+R[1][2]*l+R[1][3]*rcosa+T[1][1];
    kinect_Triplet_temp.y = R[2][1]*rsina+R[2][2]*l+R[2][3]*rcosa+T[2][1];
    kinect_Triplet_temp.z = R[3][1]*rsina+R[3][2]*l+R[3][3]*rcosa+T[3][1];
    return kinect_Triplet_temp;
  }

  std::pair<double,double> virtual_laser_scan(kinect_Triplet kinect_Triplet_temp){
    //因为转化后的三元组只代表一个点，因此只需要去掉y值，就是在x0z平面上的虚拟激光点。
    double d = std::sqrt(kinect_Triplet_temp.x*kinect_Triplet_temp.x+kinect_Triplet_temp.z*kinect_Triplet_temp.z);
    double a = 1.57064632675;
    if(kinect_Triplet_temp.z != 0) a = std::atan(kinect_Triplet_temp.x/kinect_Triplet_temp.z);
    return std::make_pair(d,a);
  }

  double kinect_to_laser_dis(double x,double z){
    double dsin = inv[0][0]*x + inv[0][2]*z + inv[0][3];
    double dcos = inv[2][0]*x + inv[2][2]*z + inv[2][3];
    return std::sqrt(dsin*dsin+dcos*dcos);
  }

  sensor_msgs::LaserScanPtr fusion(sensor_msgs::LaserScanPtr& laser_msg,sensor_msgs::LaserScanPtr& scan_msg){
    sensor_msgs::LaserScanPtr msg(new sensor_msgs::LaserScan());
    msg->angle_increment = laser_msg->angle_increment;
    msg->angle_max = laser_msg->angle_max;
    msg->angle_min = laser_msg->angle_min;
    msg->header = laser_msg->header;
    msg->intensities = laser_msg->intensities;
    msg->range_max = laser_msg->range_max;
    msg->range_min = laser_msg->range_min;
    msg->scan_time = laser_msg->scan_time;
    msg->time_increment = laser_msg->time_increment;
    msg->ranges=laser_msg->ranges;
    
    int scan_msg_num_direction = std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
    int laser_msg_num_direction = std::ceil((laser_msg->angle_max - laser_msg->angle_min) / laser_msg->angle_increment);
    ROS_INFO("<---------DEBUG--------->");
    ROS_INFO("scan_msg's angle_max is %f", scan_msg->angle_max);
    ROS_INFO("scan_msg's angle_min is %f", scan_msg->angle_min);
    ROS_INFO("laser_msg's angle_max is %f", laser_msg->angle_max);
    ROS_INFO("laser_msg's angle_min is %f", laser_msg->angle_min);
    // ROS_INFO("laser_msg's angle_increment is %f", laser_msg->angle_increment);
    // ROS_INFO("laser_msg's num_direction is %d", laser_msg->ranges.size());
    // ROS_INFO("scan_msg's num_direction is  %d", scan_msg->ranges.size());
    // ROS_INFO("scan_msg_num_direction = %d",scan_msg_num_direction);
    // ROS_INFO("laser_msg_num_direction = %d",laser_msg_num_direction); 
    ROS_INFO("<---------DEBUG--------->");
    for(int i = 0; i < laser_msg_num_direction; ++i){
      //循环遍历每个二元组(r,a)
      if(!std::isfinite(laser_msg->ranges[i])) continue;
      kinect_Triplet kinect_Triplet_temp = laser_to_kinect(laser_msg->ranges[i],(laser_msg->angle_min+laser_msg->angle_increment*i));//暂时缺少l具体值
      // ROS_INFO("<---------DEBUG--------->");
      // ROS_INFO("x = %f", kinect_Triplet_temp.x);
      // ROS_INFO("y = %f", kinect_Triplet_temp.y);
      // ROS_INFO("z = %f", kinect_Triplet_temp.z);
      // ROS_INFO("<---------DEBUG--------->");
      std::pair<double,double> virtual_laser_scan_temp = virtual_laser_scan(kinect_Triplet_temp); //temp.first = d,temp.second = a
      int j = (virtual_laser_scan_temp.second - scan_msg->angle_min) / scan_msg->angle_increment; //获取该点在scan_msg中是第几个点
      double range = 0.0;
      // ROS_INFO("<---------DEBUG--------->");
      // ROS_INFO("laser_msg->ranges[i] = %f",laser_msg->ranges[i]);
      // ROS_INFO("laser_msg->angle_min+laser_msg->angle_increment*i = %f",laser_msg->angle_min+laser_msg->angle_increment*i);
      // ROS_INFO("kinect_Triplet_temp.z = %f",kinect_Triplet_temp.x);
      // ROS_INFO("kinect_Triplet_temp.z = %f",kinect_Triplet_temp.z);
      // ROS_INFO("virtual_laser_scan_temp.first is %f",virtual_laser_scan_temp.first);
      // ROS_INFO("virtual_laser_scan_temp.second is %f",virtual_laser_scan_temp.second);
      // ROS_INFO("j is %d",j);
      // ROS_INFO("virtual_laser_scan_temp.first is finite?   %d",std::isfinite(virtual_laser_scan_temp.first));
      // ROS_INFO("virtual_laser_scan_temp.second is finite?   %d",std::isfinite(virtual_laser_scan_temp.second));
      // ROS_INFO("<---------DEBUG--------->\n");
      if(!std::isfinite(scan_msg->ranges[j])) continue;
      if(!std::isfinite(virtual_laser_scan_temp.first)) continue;
      // if(!std::isfinite(virtual_laser_scan_temp.first)){
      //   if(j < scan_msg_num_direction && j >= 0 && scan_msg->ranges[j] < msg->ranges[i]){
      //     msg->ranges[i] = scan_msg->ranges[j];
      //     continue;
      //   }
      // }
      // else {
      //   range = (scan_msg->ranges[j] < virtual_laser_scan_temp.first) ? scan_msg->ranges[j]:virtual_laser_scan_temp.first;
      //   if(j < scan_msg_num_direction && j >= 0 && range < msg->ranges[i] && fabs(range - msg->ranges[i])){
      //     msg->ranges[i] = range;
      //     continue;
      //   }
      // }

/**
 * 测试直接相机坐标系旋转-90°
 * 
 * */
    double l_angle = msg->angle_min+msg->angle_increment*i;
    // l_angle -= 2.0;
    // if(l_angle < msg->angle_min) l_angle += (3.1415926535)*2.0;
    if(l_angle > scan_msg->angle_min && l_angle < scan_msg->angle_max){
      // ROS_INFO("<---------DEBUG--------->");
      ROS_INFO("l_angle is %f",l_angle);
      ROS_INFO("scan_msg->angle_min is %f",scan_msg->angle_min);
      ROS_INFO("scan_msg->angle_max is %f",scan_msg->angle_max);
      // ROS_INFO("<---------DEBUG--------->");
      int k = (l_angle - scan_msg->angle_min )/scan_msg->angle_increment;
      if(scan_msg->ranges[k] < msg->ranges[i] && fabs(scan_msg->ranges[k] - msg->ranges[i]) > 0.5)
        msg->ranges[i] = scan_msg->ranges[k];
    }




      // if(j < scan_msg_num_direction && j >= 0 && scan_msg->ranges[j] < virtual_laser_scan_temp.first && fabs(scan_msg->ranges[j] - virtual_laser_scan_temp.first) > 0.1){
      //   msg->ranges[i] = scan_msg->ranges[j]; 
      //   // msg->ranges[i] = 1.0;
      //   // msg->ranges[i] = kinect_to_laser_dis(scan_msg->ranges[j]*std::sin(scan_msg->angle_min+scan_msg->angle_increment*j),scan_msg->ranges[j]*std::cos(scan_msg->angle_min+scan_msg->angle_increment*j));
      // }
    }

    return msg;
  }
  


// sensor_msgs/CameraInfo 参数  http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html

    image_geometry::PinholeCameraModel cam_model_; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.

    float scan_time_; ///< Stores the time between scans. 存储扫描间隔时间
    float range_min_; ///< Stores the current minimum range to use.存储要使用的最小范围
    float range_max_; ///< Stores the current maximum range to use.存储要使用的最大范围
    int scan_height_; ///< Number of pixel rows to use when producing a laserscan from an area.从区域内生成激光扫描时使用的像素行数
    std::string output_frame_id_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.输出的帧的id是每个激光帧的id，但不一定是相机帧的id
  };


}; // depthimage_to_laserscan

#endif
