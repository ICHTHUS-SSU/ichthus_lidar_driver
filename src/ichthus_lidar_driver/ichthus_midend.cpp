#include <ichthus_lidar_driver/ichthus_midend.h>

namespace ichthus_lidar_mid
{
  void IchthusLidarMid::VLP16MsgCallback(const ichthus_lidar_driver::VLP16ConstPtr& msg)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      VLP16_buf_[(buf_idx_ + 1) % NUM_BUFFER].push_back(msg);
    }
  }

  void IchthusLidarMid::OSI64MsgCallback(const ichthus_lidar_driver::OSI64ConstPtr& msg)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      OSI64_buf_[(buf_idx_ + 1) % NUM_BUFFER].push_back(msg);
    }
  }

  void IchthusLidarMid::doLoop()
  {
    int period_us = 1000000.0 / param_.rate_;
    int timerfd = init_timer(period_us);

    while (ros::ok())
    {
      pcl::PointCloud<pcl::PointXYZINormal> out_cloud;

      if (param_.mode_ == LIDAR_MODE::VLP16_MODE)
      {
        VLP16::add_VLP16_vector_msg_to_cloud(VLP16_buf_[buf_idx_], out_cloud, sin_rot_table_, cos_rot_table_, calibration_);
      }
      else if (param_.mode_ == LIDAR_MODE::OSI64_MODE)
      {
        
        OSI64::add_OSI64_vector_msg_to_cloud(OSI64_buf_[buf_idx_], out_cloud);
      }
      else
      {
        /* do nothing */
      }

      if (out_cloud.empty() == false)
      {
        sensor_msgs::PointCloud2Ptr cloud_msg_ptr(new sensor_msgs::PointCloud2);
        
        pcl::toROSMsg(out_cloud, *cloud_msg_ptr);

        cloud_msg_ptr->header.frame_id = param_.frame_;
        cloud_msg_ptr->header.stamp = ros::Time::now(); 

        pub_cloud_.publish(cloud_msg_ptr);

        if (param_.mode_ == LIDAR_MODE::VLP16_MODE)
        {
          VLP16_buf_[buf_idx_].clear();
        }
        else if (param_.mode_ == LIDAR_MODE::OSI64_MODE)
        {
          OSI64_buf_[buf_idx_].clear();
        }
      }
      else
      {
        ROS_INFO("Buffer is empty. [midend]");
      }

      {
        boost::mutex::scoped_lock lock(mutex_);
        buf_idx_ = (buf_idx_ + 1) % NUM_BUFFER;
      }

      wait_timer(timerfd);
    }
  }

  void IchthusLidarMid::onInit()
  {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    private_nh.getParam("calibration_path", param_.calibration_path_);
    private_nh.getParam("mode", param_.mode_);
    private_nh.getParam("frame", param_.frame_);
    private_nh.getParam("pub_topic", param_.pub_topic_);
    private_nh.getParam("sub_topic", param_.sub_topic_);
    private_nh.getParam("rate", param_.rate_);

    Calibration calibration(param_.calibration_path_, false);

    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index)
    {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }

    if (calibration.initialized == false)
    {
      ROS_ERROR("main() : cannot read VLP16 calibration file");
      exit(1);
    }

    calibration_ = calibration;
    pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>(param_.pub_topic_, 1);

    if (param_.mode_ == LIDAR_MODE::VLP16_MODE)
    {
      sub_lidar_packet_ = nh.subscribe(param_.sub_topic_, 1000, &IchthusLidarMid::VLP16MsgCallback, this);
    }
    else if (param_.mode_ == LIDAR_MODE::OSI64_MODE)
    {
      sub_lidar_packet_ = nh.subscribe(param_.sub_topic_, 1000, &IchthusLidarMid::OSI64MsgCallback, this);
    }
    else
    {
      ROS_ERROR("Invalid LiDAR");
      exit(1);
    }

    main_loop_ = boost::thread(&IchthusLidarMid::doLoop, this);
  }
} // namespace IchthusLidarMid