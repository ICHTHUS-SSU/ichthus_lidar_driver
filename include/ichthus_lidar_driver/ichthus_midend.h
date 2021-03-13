#ifndef __ICHTHUS_LIDAR_MID_H_
#define __ICHTHUS_LIDAR_MID_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <sys/timerfd.h>
#include <sys/mman.h>
#include <errno.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <ichthus_lidar_driver/common/lidar.h>

extern int init_timer(int usec);
extern int wait_timer(int timerfd);

namespace ichthus_lidar_mid
{
  #define PointT pcl::PointXYZICA
  
  const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
  const uint16_t ROTATION_MAX_UNITS    =    36000u;  // [deg/100]
  const int NUM_BUFFER                 =         2;  // dual buffer
  struct M_Param
  {
    int mode_;
    std::string calibration_path_;
    std::string pub_topic_;
    std::string sub_topic_;
    std::string frame_;
    int rate_;
  };

  class IchthusLidarMid : public nodelet::Nodelet
  {
  private:
    int buf_idx_;

    ros::Publisher pub_cloud_;
    ros::Subscriber sub_lidar_packet_;

    M_Param param_;
  
    virtual void onInit();
    void VLP16MsgCallback(const ichthus_lidar_driver::VLP16ConstPtr& msg);
    void OSI64MsgCallback(const ichthus_lidar_driver::OSI64ConstPtr& msg);

    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];

    std::vector<ichthus_lidar_driver::VLP16ConstPtr> VLP16_buf_[NUM_BUFFER];
    std::vector<ichthus_lidar_driver::OSI64ConstPtr> OSI64_buf_[NUM_BUFFER];

    Calibration calibration_;

    boost::thread main_loop_;
    boost::mutex mutex_;
    
    void doLoop();

  public:
    IchthusLidarMid()
    : buf_idx_(0)
    
    {}
    ~IchthusLidarMid() { main_loop_.join(); }
  };
}
PLUGINLIB_EXPORT_CLASS(ichthus_lidar_mid::IchthusLidarMid, nodelet::Nodelet)
#endif
