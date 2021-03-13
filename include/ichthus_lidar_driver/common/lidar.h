#ifndef __LIDAR_H_
#define __LIDAR_H_

#include <ichthus_lidar_driver/common/point_types.h>
#include <ichthus_lidar_driver/common/calibration.h>
#include <ichthus_lidar_driver/common/packet.h>
#include <ichthus_lidar_driver/VLP16.h>
#include <ichthus_lidar_driver/OSI64.h>

// #include <pthread.h> // for mutex

// extern Calibration calibration_;
enum LIDAR_MODE { VLP16_MODE, OSI64_MODE };
// static pthread_mutex_t mutex_lock;

namespace VLP16 
{
  
  const int SIZE_BLOCK = 100;
  const int RAW_SCAN_SIZE = 3;
  const int SCANS_PER_BLOCK = 32; 
  const int BLOCKS_PER_PACKET = 12;
  const int PACKET_TS_SIZE = 4;

  const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE); // 32 * 3

  // const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
  // const uint16_t ROTATION_MAX_UNITS    =    36000u;  // [deg/100]
  const float DISTANCE_RESOLUTION      =    0.002f; // [m] 

  /** @todo make this work for both big and little-endian machines */
  const uint16_t UPPER_BANK = 0xeeff;
  const uint16_t LOWER_BANK = 0xddff;

  /** Special Defines for VLP16 support **/
  const int    VLP16_FIRINGS_PER_BLOCK =   2;  
  const int    VLP16_SCANS_PER_FIRING  =  16; 
  const float  VLP16_DSR_TOFFSET       =   2.304f;   // [µs]
  const float  VLP16_FIRING_TOFFSET    =  55.296f;   // [µs]
  const float  VLP16_BLOCK_TDURATION   = 110.592f;   // [µs]
  
  struct VLP16_block 
  {
    uint16_t header;       ///< UPPER_BANK or LOWER_BANK
    uint16_t azimuth;      ///< 0-35999(== 359.99°), divided by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE]; //data[96];
  };

  struct VLP16_packet 
  {
    VLP16_block blocks[BLOCKS_PER_PACKET]; // 1200 Bytes
    uint8_t timestamp[PACKET_TS_SIZE]; // 4 Bytes
    uint16_t revolution;    // 2 Bytes
  };

  struct VLP16_msg 
  {
    struct timeval ts;
    uint8_t buf[VLP16_MESSAGE_BYTES];
    uint32_t size;
  };
  
  struct Config
  {
    std::string calibrationFile;     ///< calibration file name
    double max_range;                ///< maximum range to publish
    double min_range;                ///< minimum range to publish
    int min_angle;                   ///< minimum angle to publish
    int max_angle;                   ///< maximum angle to publish
     
    double tmp_min_angle;
    double tmp_max_angle;
  };
  
  static Config vlp_config; //???

  void get_VLP16_msg(bool socket_flag, ip_pkt& packet, ichthus_lidar_driver::VLP16& vlp16_msg);
  bool update_VLP16_packet(const ichthus_lidar_driver::VLP16& msg, int direction); // , int lidar_port);
  void print_VLP16_packet(const ichthus_lidar_driver::VLP16& msg); // , int lidar_port);
  bool pointInRange(float range);
  void setParameters(double min_range,double max_range,double view_direction,double view_width);

  void add_VLP16_vector_msg_to_cloud(const std::vector<ichthus_lidar_driver::VLP16::ConstPtr> &vector_msg, pcl::PointCloud<pcl::PointXYZINormal>& cloud
				     ,float* sin_rot_table, float* cos_rot_table, Calibration& calibration);

  void add_VLP16_vector_msg_to_cloud(const std::vector<ichthus_lidar_driver::VLP16::ConstPtr> &vector_msg, pcl::PointCloud<pcl::PointXYZICA> &cloud, float* sin_rot_table, float* cos_rot_table, Calibration& calibration);
  
  // void add_VLP16_msg_to_cloud(const ichthus_lidar_driver::VLP16 &msg, pcl::PointCloud<PointXYZIR>& cloud);
  void batch_VLP16_msg_to_cloud(const std::vector<ichthus_lidar_driver::VLP16::ConstPtr> &vector_msg, pcl::PointCloud<PointXYZIR>& cloud);
}

namespace OSI64 
{
  const int BLOCK_PER_AZIMUTH = 64;
  const int SCANS_PER_BLOCK = 3; // 3 words (96 bits)
  const int BLOCK_DATA_SIZE = (BLOCK_PER_AZIMUTH * SCANS_PER_BLOCK); //64 * 3 WORDS 
  const int BLOCKS_PER_PACKET = 16;
  const int PACKET_TS_SIZE = 2; 

  struct OSI64_block 
  {
    uint32_t timestamp[PACKET_TS_SIZE];
    uint16_t measurement_id;
    uint16_t frame_id;
    uint32_t encoder_count;
    uint32_t data[BLOCK_DATA_SIZE]; //워드단위(32bit) 임을 잊지말기!!벨로다인이랑 헷갈려하지말기!
    uint32_t packet_status;
  };

  struct OSI64_packet 
  {
    OSI64_block blocks[BLOCKS_PER_PACKET]; 
  };
  
  struct OSI64_msg 
  {
    struct timeval ts;
    uint8_t buf[OSI64_MESSAGE_BYTES];
    uint32_t size;
  };


  void get_OSI64_msg(bool socket_flag, ip_pkt& packet, ichthus_lidar_driver::OSI64& osi64_msg, int offset);
  void print_OSI64_packet(const ichthus_lidar_driver::OSI64& msg);//, int lidar_port);

  const int encoder_ticks_per_rev = 90112;
  const float beam_altitude_angles[BLOCK_PER_AZIMUTH] = { 
    16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
    12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
    8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
    3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
    -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
    -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
    -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
    -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
  };
  const float beam_azimuth_angles[BLOCK_PER_AZIMUTH] = { 
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
    3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  };
  struct trig_table_entry {
    float sin_beam_altitude_angles;
    float cos_beam_altitude_angles;
    float beam_azimuth_angles;
  };
  // table of vertical angle cos, sin, and horizontal offset of each pixel
  static trig_table_entry trig_table[BLOCK_PER_AZIMUTH];
  // static bool init_tables();
  static bool init_tables() 
  {
    for (int i = 0; i < BLOCK_PER_AZIMUTH; i++) 
    {
      trig_table[i] = {sinf(beam_altitude_angles[i] * 2 * M_PI / 360.0f),
		       cosf(beam_altitude_angles[i] * 2 * M_PI / 360.0f),
		       beam_azimuth_angles[i] * 2 * (float)M_PI / 360.0f};
    }   
    return true;
  }
  static bool tables_initialized = init_tables();

  void add_OSI64_vector_msg_to_cloud(const std::vector<ichthus_lidar_driver::OSI64ConstPtr> &msgs, pcl::PointCloud<pcl::PointXYZINormal>& out_cloud);
  void add_OSI64_vector_msg_to_cloud(const std::vector<ichthus_lidar_driver::OSI64ConstPtr> &msgs, pcl::PointCloud<pcl::PointXYZICA>& out_cloud);
}

#endif