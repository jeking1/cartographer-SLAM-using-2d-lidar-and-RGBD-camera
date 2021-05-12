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

#include <depthimage_to_laserscan/DepthImageToLaserScanROS.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace depthimage_to_laserscan;

DepthImageToLaserScanROS::DepthImageToLaserScanROS(ros::NodeHandle& n, ros::NodeHandle& pnh):pnh_(pnh), it_(n), srv_(pnh) {
  boost::mutex::scoped_lock lock(connect_mutex_);

  // Dynamic Reconfigure
  // 动态的重新配置
  dynamic_reconfigure::Server<depthimage_to_laserscan::DepthConfig>::CallbackType f;
  f = boost::bind(&DepthImageToLaserScanROS::reconfigureCb, this, _1, _2);
  srv_.setCallback(f);

  // Lazy subscription to depth image topic
  // 延迟订阅深度图topic
  pub_ = n.advertise<sensor_msgs::LaserScan>("/scan", 10, boost::bind(&DepthImageToLaserScanROS::connectCb, this, _1), boost::bind(&DepthImageToLaserScanROS::disconnectCb, this, _1));
  pubs_ = n.advertise<sensor_msgs::LaserScan>("/depth_scan", 10);
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS(){
  sub_.shutdown();
}


/**
 * tanjx修改代码
 *
 * */
// sensor_msgs::LaserScanPtr scan_msg ;


// void DepthImageToLaserScanROS::laserCb(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
//   try{
    
//     sensor_msgs::LaserScanPtr laser_msg_(new sensor_msgs::LaserScan());
//     laser_msg_->angle_increment = laser_msg->angle_increment;
//     laser_msg_->angle_max = laser_msg->angle_max;
//     laser_msg_->angle_min = laser_msg->angle_min;
//     laser_msg_->header = laser_msg->header;
//     laser_msg_->intensities = laser_msg->intensities;
//     laser_msg_->range_max = laser_msg->range_max;
//     laser_msg_->range_min = laser_msg->range_min;
//     laser_msg_->scan_time = laser_msg->scan_time;
//     laser_msg_->time_increment = laser_msg->time_increment;
//     laser_msg_->ranges = laser_msg->ranges;
//     // pub_.publish(laser_msg_);
//     // ROS_INFO("laser_msg   's header stamp is : %d", laser_msg->header.stamp);
//     // ROS_INFO("laser_msg___'s header stamp is : %d", laser_msg_->header.stamp);
//     // ROS_INFO("laser_msg   's angle_increment is : %f", laser_msg->angle_increment);
//     // ROS_INFO("laser_msg___'s angle_increment is : %f \n", laser_msg_->angle_increment);
//     sensor_msgs::LaserScanPtr msg = dtl_.fusion_msg(laser_msg_,scan_msg);
//     pub_.publish(msg);
//   }
//   catch (std::runtime_error& e)
//   {
//     ROS_ERROR_THROTTLE(1.0, "Could not fuse laser scan and virtual laser scan: %s", e.what());
//   }
// }

// void DepthImageToLaserScanROS::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
//         const sensor_msgs::CameraInfoConstPtr& info_msg){
//   try
//   {
//     scan_msg = dtl_.convert_msg(depth_msg, info_msg);
//     // ROS_INFO("<----publish---->");
//     double d = 2.0*3.1415926535;
//     scan_msg->angle_min += 3.1415926;
//     scan_msg->angle_max += 3.1415926;
//     // if(scan_msg->angle_max > 3.1415926535) scan_msg->angle_min -= d,scan_msg->angle_max -= d;
//     pubs_.publish(scan_msg);
//     // scan_msg->angle_min -= 2.0;
//     // scan_msg->angle_max -= 2.0;
//   }
//   catch (std::runtime_error& e)
//   {
//     ROS_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
//   }
// }
void DepthImageToLaserScanROS::syncCb(const sensor_msgs::ImageConstPtr& ori_depth,const sensor_msgs::CameraInfoConstPtr& ori_camera,
              const sensor_msgs::LaserScanConstPtr& ori_laser){
  try{
    sensor_msgs::LaserScanPtr s_msg = dtl_.convert_msg(ori_depth, ori_camera);
    double d = 2.0*3.1415926535;
    s_msg->angle_min += 3.1415926;
    s_msg->angle_max += 3.1415926;
    // if(s_msg->angle_max > 3.1415926535) s_msg->angle_min -= d,s_msg->angle_max -= d;
    pubs_.publish(s_msg);

    sensor_msgs::LaserScanPtr laser_msg_(new sensor_msgs::LaserScan());
    laser_msg_->angle_increment = ori_laser->angle_increment;
    laser_msg_->angle_max = ori_laser->angle_max;
    laser_msg_->angle_min = ori_laser->angle_min;
    laser_msg_->header = ori_laser->header;
    laser_msg_->intensities = ori_laser->intensities;
    laser_msg_->range_max = ori_laser->range_max;
    laser_msg_->range_min = ori_laser->range_min;
    laser_msg_->scan_time = ori_laser->scan_time;
    laser_msg_->time_increment = ori_laser->time_increment;
    laser_msg_->ranges = ori_laser->ranges;
    sensor_msgs::LaserScanPtr msg = dtl_.fusion_msg(laser_msg_,s_msg);
    pub_.publish(msg);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not syncCb: %s", e.what());
  }
}

void DepthImageToLaserScanROS::connectCb(const ros::SingleSubscriberPublisher& pub) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (!sub_ && pub_.getNumSubscribers() > 0) {
    ROS_DEBUG("Connecting to depth topic.");


    //时间戳同步
    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(pnh_,"/camera/depth/image_rect_raw",1);
    camera_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(pnh_,"/camera/depth/camera_info",1);
    scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(pnh_,"/laser_scan",1);
    sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *image_sub_, *camera_sub_,*scan_sub_);
    sync_->registerCallback(boost::bind(&DepthImageToLaserScanROS::syncCb,this,_1,_2,_3));



    // image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    // sub_ = it_.subscribeCamera("image", 10, &DepthImageToLaserScanROS::depthCb, this, hints);

    /**
     * ros::Subscriber laser_sub_ = pnh_.subscribe("/laser_scan", 10, DepthImageToLaserScanROS::laserCb); 报错，需要用下面这句话
     * */
    // ros::Subscriber* laser_sub_ = new ros::Subscriber(pnh_.subscribe<sensor_msgs::LaserScan>("/laser_scan", 10, &DepthImageToLaserScanROS::laserCb,this));

    //同步时间
    // typedef sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::LaserScan> MySyncPolicy;
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),sub_,laser_sub_);
    // sync.registerCallback(boost::bind(&syncCb,_1,_2));

  }
}

void DepthImageToLaserScanROS::disconnectCb(const ros::SingleSubscriberPublisher& pub) {
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0) {
    ROS_DEBUG("Unsubscribing from depth topic.");
    sub_.shutdown();
  }
}

//重要
void DepthImageToLaserScanROS::reconfigureCb(depthimage_to_laserscan::DepthConfig& config, uint32_t level){
    dtl_.set_scan_time(config.scan_time);
    dtl_.set_range_limits(config.range_min, config.range_max);
    dtl_.set_scan_height(config.scan_height);
    dtl_.set_output_frame(config.output_frame_id);
}
