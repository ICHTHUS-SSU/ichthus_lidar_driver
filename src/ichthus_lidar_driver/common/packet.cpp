/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <ichthus_lidar_driver/common/packet.h>

void get_ip_header(ip_pkt &packet, ip_pkthdr &header)
{
  uint8_t *ip_packet = packet.hdr;
  header.desc = (ip_packet[0] << 8) | (ip_packet[1]);
  header.len = (ip_packet[2] << 8) | (ip_packet[3]);
  header.id = (ip_packet[4] << 8) | (ip_packet[5]);
  header.flag_n_frag = (ip_packet[6] << 8) | (ip_packet[7]); //df bit, mf bit, fragment offset
  header.ttl = ip_packet[8];
  header.transport = ip_packet[9];
  header.src_addr = (ip_packet[15] << 24) | (ip_packet[14] << 16) | (ip_packet[13] << 8) | (ip_packet[12]);
}

void get_udp_header(uint8_t *ip_datagram, udp_pkthdr &header)
{
  uint8_t *udp_packet = ip_datagram;
  header.src_port = (udp_packet[0] << 8) | (udp_packet[1]);
  header.dst_port = (udp_packet[2] << 8) | (udp_packet[3]);
  header.len = (udp_packet[4] << 8) | (udp_packet[5]);
  header.checksum = (udp_packet[6] << 8) | (udp_packet[7]);
}

////////////////////////////////////////////////////
// The socket I/O functions
////////////////////////////////////////////////////
int create_socket()
{
  int sockfd = -1;
  int option = 1; // SO_REUSEADDR 의 옵션 값을 TRUE 로
  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(int));
  if (sockfd == -1)
  {
    perror("socket");
    return -1;
  }

  sockaddr_in my_addr;                     // my address information
  memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
  my_addr.sin_family = AF_INET;            // host byte order
  my_addr.sin_port = htons(SERVER_PORTNO); // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

  if (bind(sockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind");
    return -1;
  }

  if (fcntl(sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  { // why FASYNC ???
    perror("non-block");
    return -1;
  }
  ROS_INFO("SERVER_PORTNO fd is %d\n", SERVER_PORTNO);
  ROS_INFO("lidar socket fd is %d\n", sockfd);
  return sockfd;
}

int read_from_socket(int sockfd, struct ip_pkt &packet, struct sockaddr_in &sender_address)
{
  struct pollfd fds[1];
  fds[0].fd = sockfd;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000; // one second (in msec)
  // sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true)
  {
    // Unfortunately, the Linux kernel recvfrom() implementation
    // uses a non-interruptible sleep() when waiting for data,
    // which would cause this method to hang if the device is not
    // providing data.  We poll() the device first to make sure
    // the recvfrom() will not block.
    //
    // Note, however, that there is a known Linux kernel bug:
    //
    //   Under Linux, select() may report a socket file descriptor
    //   as "ready for reading", while nevertheless a subsequent
    //   read blocks.  This could for example happen when data has
    //   arrived but upon examination has wrong checksum and is
    //   discarded.  There may be other circumstances in which a
    //   file descriptor is spuriously reported as ready.  Thus it
    //   may be safer to use O_NONBLOCK on sockets that should not
    //   block
    // poll() until input available
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0) // poll() error?
      {
        if (errno != EINTR)
          ROS_ERROR("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0) // poll() timeout?
      {
        ROS_WARN("LiDAR poll() timeout");
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))
      { // device error?
        ROS_ERROR("poll() reports error LiDAR");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    // Receive packets that should now be available from the
    // socket using a blocking read.
    ssize_t max_buf_size = 99999;
    uint8_t buf[max_buf_size];
    ssize_t nbytes = recvfrom(sockfd, buf, max_buf_size,
                              0, (sockaddr *)&sender_address, &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        ROS_INFO("recvfail");
        return 1;
      }
    }
    else if (nbytes == VLP16_MESSAGE_BYTES)
    {
      packet.size = nbytes;
      // packet.ts = ros::Time::now().toSec();
      memcpy(packet.buf, buf, packet.size);
      gettimeofday(&packet.ts, NULL);
      return 0;
    }
    else if (nbytes == OSI64_MESSAGE_BYTES)
    {
      packet.size = nbytes;
      // ROS_INFO("packet.size : %d", packet.size);
      // packet.ts = ros::Time::now().toSec();
      memcpy(packet.buf, buf, packet.size);
      gettimeofday(&packet.ts, NULL); // READ할때의 시간을 packet.ts로 함
      return 0;
    }
    else
    {
      ROS_INFO("read_from_socket() :  socket SIZE");
    }
  }
  return -1;
}

std::string ipaddr_from_sockaddr(struct sockaddr_in &sender_address)
{
  return inet_ntoa(sender_address.sin_addr);
}

////////////////////////////////////////////////////
// The PCAP I/O functions
////////////////////////////////////////////////////

bool open_pcap_file(pcap_t **pcap_fd, std::string pcap_file)
{ // ,std::string ip_addr, uint16_t ip_port) {
  pcap_t *pcap;
  char errbuf[PCAP_ERRBUF_SIZE];
  bpf_program pcap_packet_filter;
  std::stringstream filter;

  ROS_INFO("Opening PCAP file : %s", pcap_file.c_str());
  if ((pcap = pcap_open_offline(pcap_file.c_str(), errbuf)) == NULL)
  {
    ROS_ERROR("Error : pcap_open_offline() : %s", pcap_file.c_str());
    return false;
  }

  // filter << "src host " << ip_addr << " && " << "udp dst port " << ip_port;
  // pcap_compile(pcap, &pcap_packet_filter, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  *pcap_fd = pcap;
  return true;
}

bool read_pcap_file(pcap_t *pcap_, struct ip_pkt &packet)
{
  struct pcap_pkthdr *pcap_pkthdr;
  const u_char *p_data;
  int ret;
  for (int retry = 0; retry < 5; retry++)
  {
    if ((ret = pcap_next_ex(pcap_, &pcap_pkthdr, &p_data)) >= 0)
    {
      //if (read_fast_ == false) packet_rate_.sleep();
      p_data += ETH_HEADER_BYTES;
      memcpy(packet.hdr, p_data, IP_HEADER_BYTES);
      p_data += IP_HEADER_BYTES;
      packet.size = pcap_pkthdr->len - (ETH_HEADER_BYTES + IP_HEADER_BYTES); // pcap_pkthdr->len = 1514 or 810 (Ouster Lidar), 90 (Ouster IMU)
      //ROS_INFO("read_pcap_file() : packet.size : %u", packet.size);
      memcpy(packet.buf, p_data, packet.size);
      packet.ts = pcap_pkthdr->ts; // time synchronization considered here!!!
      return true;
    }
    ROS_INFO("Warning : pcap_next_ex() tries %d times", retry + 1);
  } // loop back and try again
  ROS_INFO("Error : pcap_next_ex() fails!");
  return false;
}
