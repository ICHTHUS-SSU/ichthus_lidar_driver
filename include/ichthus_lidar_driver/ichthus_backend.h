/*
 * Copyright 2019 Youngjoon Choi, Kyujin Choi, Chaewon Hong, Soorin Cho, Hannah Baek, and Kanghee Kim at Soongsil University. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __ICHTHUS_LIDAR_BACK_H_
#define __ICHTHUS_LIDAR_BACK_H_

#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <cmath>
#include <sys/timerfd.h>
#include <sys/mman.h>
#include <errno.h>
#include <iostream>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#ifdef USE_GPU
#include <gpu_transform/gpu_transform.cuh>
#endif

#define MAX_NSENSORS 8 

int quit(char *tag);

namespace ichthus_lidar_back
{

  boost::mutex mutex_;

  struct pose
  {
    pose() : roll_(0), pitch_(0), yaw_(0), x_(0), y_(0), z_(0) {}
    double roll_;
    double pitch_;
    double yaw_;
    double x_;
    double y_;
    double z_;
  }; // struct pose

  class InputCloud
  {
  public:

    InputCloud(const pose& p, const std::string& topic, ros::NodeHandle& nh);
    ~InputCloud() {}

    /* public functions */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
    Eigen::Affine3f makeLidarTF();

    pose ps_;
    std::string topic_name_;
    std::string frame_id_;
    ros::Subscriber sub_;
    Eigen::Affine3f transform_;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr tf_cloud_ptr_;

    boost::mutex tf_cloud_mutex_;
    boost::condition_variable condition_;
  }; // class InputCloud

  class OutputCloud
  {
  public:
    OutputCloud(const std::string& topic, const std::string& frame, const int rate, ros::NodeHandle& nh);
    ~OutputCloud() {}

    /* public functions */
    void publishCloud();

    pcl::PointCloud<pcl::PointXYZINormal> cloud_;
    ros::Publisher pub_;
    
    std::string topic_name_;
    std::string frame_id_;
    ros::Time ts_;

    int rate_;
		
  }; //class OutputCloud

  typedef std::shared_ptr< ::ichthus_lidar_back::InputCloud > InputCloudPtr;
  typedef std::shared_ptr< ::ichthus_lidar_back::OutputCloud > OutputCloudPtr;

  class IchthusLidarBack : public nodelet::Nodelet
  {
  public:
    IchthusLidarBack() {};
    ~IchthusLidarBack() { main_loop_.join(); }
    virtual void onInit();
    
  private:
    std::vector<InputCloudPtr> in_cloud_arr_;
    OutputCloudPtr out_cloud_ptr_;

    boost::thread main_loop_;
    
    /* private functions */
    void doLoop();
    void mergeNpub();
  };

} // namespace of IchthusLidarBack
PLUGINLIB_EXPORT_CLASS(ichthus_lidar_back::IchthusLidarBack, nodelet::Nodelet)

#endif
