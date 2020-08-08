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

#ifndef __ICHTHUS_LIDAR_FRONT_H_
#define __ICHTHUS_LIDAR_FRONT_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ichthus_lidar_driver/common/lidar.h>

std::string position[4] = { "front", "left", "right", "back" };
std::string parameter[3] = { "mode_", "ip_add_", "/lidar_msg" };

namespace ichthus_lidar_front
{
  struct F_Param
  {
    int lidar_num;
    int* mode;
    std::string pcap_file;
    std::string* ip_addr;
    float pcap_wait_factor;
  };

  class IchthusLidarFront : public nodelet::Nodelet
  {
  public:
    IchthusLidarFront() : socket_flag(false), queue_size(1000) {}
    ~IchthusLidarFront() { main_loop_.join(); delete param.mode; delete param.ip_addr; }

  private:
    ros::Publisher pub_lidar_data[4];
    bool socket_flag, ret;
    int sockfd, queue_size;
    sockaddr_in sender_address;
    pcap_t *pcap_fd;
    F_Param param;

    boost::thread main_loop_;

    void extMsg();
    virtual void onInit();
  };
}
PLUGINLIB_EXPORT_CLASS(ichthus_lidar_front::IchthusLidarFront, nodelet::Nodelet)
#endif
