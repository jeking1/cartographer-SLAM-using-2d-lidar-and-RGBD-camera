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

#ifndef DEPTH_IMAGE_TO_LASERSCAN_ROS
#define DEPTH_IMAGE_TO_LASERSCAN_ROS

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <depthimage_to_laserscan/DepthConfig.h>

#include <depthimage_to_laserscan/DepthImageToLaserScan.h>

namespace depthimage_to_laserscan
{
  class DepthImageToLaserScanROS
  {
  public:
    DepthImageToLaserScanROS(ros::NodeHandle& n, ros::NodeHandle& pnh);

    ~DepthImageToLaserScanROS();

  private:
    /**
     * Callback for image_transport
     * 图像传输回调
     * Synchronized callback for depth image and camera info.  Publishes laserscan at the end of this callback.
     * 深度相机和图像信息的同步回调。！！！！在此回调结束时发布laserscan
     * @param depth_msg Image provided by image_transport.
     * @param info_msg CameraInfo provided by image_transport.
     *
     */
    void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
		  const sensor_msgs::CameraInfoConstPtr& info_msg);

    /**
     * tanjx的修改
     * laserscan的订阅回调
     * */
    void laserCb(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * Callback that is called when there is a new subscriber.
     * 当有新订阅时调用的回调函数
     * Will not subscribe to the image and camera_info until we have a subscriber for our LaserScan (lazy subscribing).
     * 在有laserscan的订阅之前，不会订阅图像和相机信息
     * 
     * 这里不确定会不会因为添加laserscan的订阅，导致这个函数失效
     */
    void connectCb(const ros::SingleSubscriberPublisher& pub);

    /**
     * Callback called when a subscriber unsubscribes.
     * 当订阅者取消订阅时的回调
     * If all current subscribers of our LaserScan stop listening, stop subscribing (lazy subscribing).
     * 如果laserscan的所有订阅者都停止监听，则停止订阅
     */
    void disconnectCb(const ros::SingleSubscriberPublisher& pub);

    /**
     * Dynamic reconfigure callback.
     * 动态重新配置回调。
     * Callback that is used to set each of the parameters insde the DepthImageToLaserScan object.
     * 用于设置DepthImageToLaserScan对象中的每个参数的回调。
     * @param config Dynamic Reconfigure object.
     * @param level Dynamic Reconfigure level.
     *
     */
    void reconfigureCb(depthimage_to_laserscan::DepthConfig& config, uint32_t level);

    ros::NodeHandle pnh_; ///< Private nodehandle used to generate the transport hints in the connectCb.用于在connectCb中生成传输提示的私有nodehandle。
    image_transport::ImageTransport it_; ///< Subscribes to synchronized Image CameraInfo pairs. 订阅同步图像camerainfo对。
    /**
     * image_transport::CameraSubscriber:
     *   https://docs.ros.org/en/api/image_transport/html/camera__subscriber_8h_source.html
     *image_transport::CameraSubscriber::CameraSubscriber	(	ImageTransport & 	image_it,
     *             ros::NodeHandle & 	info_nh,
     *             const std::string & 	base_topic,
     *             uint32_t 	queue_size,
     *             const Callback & 	callback,
     *             const ros::VoidPtr & 	tracked_object = ros::VoidPtr(),
     *             const TransportHints & 	transport_hints = TransportHints() 
     *             )	
     * */
    image_transport::CameraSubscriber sub_; ///< Subscriber for image_transport 图像传输订阅
    /**
     * tanjx的修改
     * laserscan的订阅者
     * */
    sensor_msgs::LaserScan laser_msg_;
    ros::Subscriber laser_sub_;
    ros::Publisher pub_; ///< Publisher for output LaserScan messages 输出激光扫描信息的发布者
    dynamic_reconfigure::Server<depthimage_to_laserscan::DepthConfig> srv_; ///< Dynamic reconfigure server 动态重新配置服务器

    depthimage_to_laserscan::DepthImageToLaserScan dtl_; ///< Instance of the DepthImageToLaserScan conversion class. deptimageTolaserscan转换类的实例。

    boost::mutex connect_mutex_; ///< Prevents the connectCb and disconnectCb from being called until everything is initialized. 防止在初始化所有内容之前调用connectCb和disconnectCb。
  };


}; // depthimage_to_laserscan

#endif
