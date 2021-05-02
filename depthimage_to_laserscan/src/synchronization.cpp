#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <iostream>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


//PointCloud2 syn_pointcloud;
//Imu syn_imu;
Image color;
Image depth;


void callback(const ImageConstPtr& ori_color, const ImageConstPtr& ori_depth)
{
  // Solve all of perception here...
  //syn_pointcloud = *ori_pointcloud;
  //syn_imu = *ori_imu;
  color = *ori_color;
  depth = *ori_depth;
  cout << "syn velodyne points‘ timestamp : " << color.header.stamp << endl;
  cout << "syn Imu‘s timestamp : " << depth.header.stamp << endl;
  ROS_INFO("pointcloud stamp value is: %d", color.header.stamp);
  ROS_INFO("imu stamp value is: %d", depth.header.stamp);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_synchronizer");
  ros::NodeHandle nh;

  
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth/image_rect_raw", 1);
  message_filters::Subscriber<L> color_sub(nh, "/camera/color/image_raw", 1);
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;    
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_sub, depth_sub); //queue size=10
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}