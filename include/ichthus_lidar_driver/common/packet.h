#ifndef __PACKET_H_
#define __PACKET_H_

#include <iostream>
#include <chrono> //for ns

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sys/types.h>
#include <sys/socket.h> // recvfrom()
#include <arpa/inet.h>  // htons()
#include <pcap/pcap.h>  //pcap_open_offline()
#include <poll.h>       //pollfd
#include <time.h>
#include <math.h>
#include <angles/angles.h>
#include <sys/timerfd.h>
#include <sys/mman.h>
#include <errno.h>

struct lidar_param
{
  std::string pcap_file;
  std::string ip_addr[3];
  int ip_port[3];
};

struct
{
  std::string frame_id; ///< tf frame ID
  std::string model;    ///< device model name
  int npackets;         ///< number of packets to collect
  double rpm;           ///< device rotation rate (RPMs)
  int cut_angle;        ///< cutting angle in 1/100°
  double time_offset;   ///< time in seconds added to each velodyne time stamp
} config_;

struct EIGEN_ALIGN16 PointXYZIR
{
  PCL_ADD_POINT4D;                // quad-word XYZ
  PCL_ADD_INTENSITY;              ///< laser intensity reading // fixed by qjinchoi, 191001
  uint16_t ring;                  ///< laser ring number
  uint32_t ts_sec;                // added by qjinchoi, 191004
  uint32_t ts_usec;               // added by qjinchoi, 191004
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(uint32_t, ts_sec, ts_sec)(uint32_t, ts_usec, ts_usec))

////////////////////////////////////////////////////
// The UDP/IP header processing functions
////////////////////////////////////////////////////

#define ETH_HEADER_BYTES 14
#define ETH_PAYLOAD_BYTES 1500
#define IP_HEADER_BYTES 20
#define UDP_HEADER_BYTES 8
#define IP_PAYLOAD_BYTES (ETH_PAYLOAD_BYTES - IP_HEADER_BYTES)
#define VLP16_MESSAGE_BYTES 1206
#define OSI64_MESSAGE_BYTES 12608

#define SERVER_PORTNO 7502 // You should set all lidar to same portNo.
// #define SERVER_PORTNO 2368
const size_t VLP16_packet_bytes = 1248;
const size_t OSI64_packet_bytes = 1514; // OSI64 msg to be fragmented in unit of 1472 bytes

struct ip_pkt
{
  struct timeval ts;
  uint8_t hdr[IP_HEADER_BYTES];     //20
  uint8_t buf[OSI64_MESSAGE_BYTES]; //1500-20 == 1480 //vlp ->1214
  // uint8_t buf[IP_PAYLOAD_BYTES]; //1500-20 == 1480 //vlp ->1214
  uint32_t size;
  ip_pkt() : size(0) {}
};

struct ip_pkthdr
{
  uint16_t desc;
  uint16_t len;
  uint16_t id;
  uint16_t flag_n_frag;
  uint8_t ttl;
  uint8_t transport;
  uint16_t checksum;
  uint32_t src_addr;
  uint32_t dst_addr;
};

struct udp_pkthdr
{
  uint16_t src_port;
  uint16_t dst_port;
  uint16_t len;
  uint16_t checksum;
};

void get_ip_header(ip_pkt &packet, ip_pkthdr &header);
void get_udp_header(uint8_t *ip_datagram, udp_pkthdr &header);
////////////////////////////////////////////////////
// The socket I/O functions
////////////////////////////////////////////////////
int create_socket();
int read_from_socket(int sockfd, struct ip_pkt &packet, struct sockaddr_in &sender_address);
std::string ipaddr_from_sockaddr(struct sockaddr_in &sender_address);

////////////////////////////////////////////////////
// The PCAP I/O functions
////////////////////////////////////////////////////

bool open_pcap_file(pcap_t **pcap_fd, std::string pcap_file);
// bool open_pcap_file(pcap_t **pcap_fd, std::string pcap_file ,std::string ip_addr, uint16_t ip_port);
bool read_pcap_file(pcap_t *pcap_, struct ip_pkt &packet);
#endif