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

#include <ichthus_lidar_driver/ichthus_backend.h>
#include <ichthus_lidar_driver/common/util.h>

namespace ichthus_lidar_back
{
  #define WAIT_TIME_NS 100000000 // 100 ms

  //InputCloud member function
  InputCloud::InputCloud(const pose &p, const std::string &topic, ros::NodeHandle &nh)
  {
    tf_cloud_ptr_.reset(new pcl::PointCloud<PointT>);
    ps_ = p;
    topic_name_ = topic;
    transform_ = makeLidarTF();
    sub_ = nh.subscribe(topic_name_, 10, &InputCloud::cloudCallback, this);
  }

  void InputCloud::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    pcl::PointCloud<PointT>::Ptr in_cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr tf_cloud_ptr(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *in_cloud_ptr);

#ifdef USE_GPU
      gpu::transformPointCloud(*in_cloud_ptr, *tf_cloud_ptr, transform_);
#else
      pcl::transformPointCloud(*in_cloud_ptr, *tf_cloud_ptr, transform_);
#endif

    {
      boost::mutex::scoped_lock lock(tf_cloud_mutex_);
      tf_cloud_ptr_ = tf_cloud_ptr;
    }
  }

  Eigen::Affine3f InputCloud::makeLidarTF()
  {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    Eigen::AngleAxisf roll_angle(ps_.roll_, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(ps_.pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(ps_.yaw_, Eigen::Vector3f::UnitZ());
    Eigen::Affine3f translation(Eigen::Translation3f(ps_.x_, ps_.y_, ps_.z_));

    transform = yaw_angle * pitch_angle * roll_angle;
    transform = translation * transform;
    
    std::cout << "Transform matrix: " << std::endl;
    std::cout << transform.matrix() << std::endl;

    return transform;
  }

  //InputCloud member function end

  //OutputCloud member function start
  OutputCloud::OutputCloud(const std::string &topic, const std::string &frame, const int rate, ros::NodeHandle &nh)
  {
    cloud_.clear();

    topic_name_ = topic;
    frame_id_ = frame;
    ts_ = ros::Time(0, 0);
    rate_ = rate;
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name_, 1);
  }

  void OutputCloud::publishCloud()
  {
    if (cloud_.empty() == true)
    {
      ROS_INFO("Output cloud is still empty. [backend]");
      return;
    }

    sensor_msgs::PointCloud2::Ptr cloud_msg_ptr(new sensor_msgs::PointCloud2);

    pcl::toROSMsg(cloud_, *cloud_msg_ptr);
    cloud_msg_ptr->header.frame_id = frame_id_;
    cloud_msg_ptr->header.stamp = ts_;
    pub_.publish(*cloud_msg_ptr);
  }

  //OutputCloud member function end

  void IchthusLidarBack::mergeNpub()
  {
    out_cloud_ptr_->cloud_.clear();
    ros::Time oldest_ts = ros::Time(0);
    for (int i = 0; i < in_cloud_arr_.size(); i++)
    {
      {
        boost::mutex::scoped_lock lock(in_cloud_arr_[i]->tf_cloud_mutex_);
        pcl::PointCloud<PointT>::Ptr &tf_cloud_ptr = in_cloud_arr_[i]->tf_cloud_ptr_;
        uint64_t curr_ts_ns = tf_cloud_ptr->header.stamp * 1e3; 
        
        if (oldest_ts.toNSec() > curr_ts_ns || oldest_ts.isZero() == true)
        {
          oldest_ts.fromNSec(curr_ts_ns);
        }

        out_cloud_ptr_->cloud_ += *tf_cloud_ptr;
      }
    }

    out_cloud_ptr_->ts_ = oldest_ts;
    out_cloud_ptr_->publishCloud();
  }

  void IchthusLidarBack::doLoop()
  {
    int period_us = 1000000.0 / out_cloud_ptr_->rate_;
    int timerfd = init_timer(period_us);

    while (ros::ok())
    {
      mergeNpub();
      wait_timer(timerfd);
    }
  }

  void IchthusLidarBack::onInit()
  {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    std::string s_key("in_cloud0");  //searching key (input)
    std::string so_key("out_cloud"); //searching key (output)
    std::string key;

    //get all parameters
    for (int i = 1; i <= MAX_NSENSORS; i++)
    {
      //Searching key must be Cloud[1] ~ Cloud[MAX_NSENSORS]
      s_key[s_key.find((i + '0') - 1)] = i + '0';

      if (private_nh.searchParam(s_key, key))
      {
        std::string topic_name(s_key); //set to default
        pose cloud_pose;               //set to default by default constructor

        if (!private_nh.getParam(key + "/topic_name", topic_name))
        {
          std::cout << "not found : " << key + "/topic_name" << std::endl;
        }
        if (!private_nh.getParam(key + "/pose_yaw", cloud_pose.yaw_))
        {
          std::cout << "not found : " << key + "/pose_yaw" << std::endl;
        }
        if (!private_nh.getParam(key + "/pose_pitch", cloud_pose.pitch_))
        {
          std::cout << "not found : " << key + "/pose_pitch" << std::endl;
        }
        if (!private_nh.getParam(key + "/pose_roll", cloud_pose.roll_))
        {
          std::cout << "not found : " << key + "/pose_roll" << std::endl;
        }
        if (!private_nh.getParam(key + "/pose_x", cloud_pose.x_))
        {
          std::cout << "not found : " << key + "/pose_x" << std::endl;
        }
        if (!private_nh.getParam(key + "/pose_y", cloud_pose.y_))
        {
          std::cout << "not found : " << key + "/pose_y" << std::endl;
        }
        if (!private_nh.getParam(key + "/pose_z", cloud_pose.z_))
        {
          std::cout << "not found : " << key + "/pose_z" << std::endl;
        }

        cloud_pose.yaw_ = M_PI * (cloud_pose.yaw_ / 180.0);     //degree to radian
        cloud_pose.pitch_ = M_PI * (cloud_pose.pitch_ / 180.0); //degree to radian
        cloud_pose.roll_ = M_PI * (cloud_pose.roll_ / 180.0);   //degree to radian

        std::cout << key << " info" << std::endl;
        std::cout << "(yaw,pitch,roll,x,y,z) = "
                  << cloud_pose.yaw_ << " "
                  << cloud_pose.pitch_ << " "
                  << cloud_pose.roll_ << " "
                  << cloud_pose.x_ << " "
                  << cloud_pose.y_ << " "
                  << cloud_pose.z_ << std::endl;

        //create inputCloud object
        in_cloud_arr_.emplace_back(new InputCloud(cloud_pose, topic_name, nh));
      }
    }
    if (in_cloud_arr_.size() == 0)
    {
      std::cout << "No input data found" << std::endl;
    }

    // OutputCloud object
    if (private_nh.searchParam(so_key, key))
    {
      std::string topic_name(so_key);           // set to default
      std::string frame_id("default_frame_id"); // set to default
      int rate = 10;                            // set to default

      if (!private_nh.getParam(key + "/topic_name", topic_name))
      {
        std::cout << "not found : " << key + "/topic_name" << std::endl;
      }
      if (!private_nh.getParam(key + "/frame_id", frame_id))
      {
        std::cout << "not found : " << key + "/frame_id" << std::endl;
      }
      if (!private_nh.getParam(key + "/rate", rate))
      {
        std::cout << "not found : " << key + "/rate" << std::endl;
      }

      out_cloud_ptr_.reset(new OutputCloud(topic_name, frame_id, rate, nh));
    }

    main_loop_ = boost::thread(&IchthusLidarBack::doLoop, this);
  }

} // namespace IchthusLidarBack
