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

#include <ichthus_lidar_driver/ichthus_frontend.h>
#include <ichthus_lidar_driver/common/util.h>

#define ROS_INFO(...) ((void)0) //strip out PRINT instructions from code
#define TIMERFD_ENABLED 1

namespace ichthus_lidar_front
{
	void IchthusLidarFront::extMsg()
	{
		ip_pkt ip_pkt;
		std::string dst_ipaddr;
		ip_pkthdr ip_hdr;
		udp_pkthdr udp_hdr;
		int frag_offset, mf_flag, dst_port, mode_index, i;
		unsigned int npub = 0, nskip = 0;
		int timerfd = -1;
		int64_t iat;
		struct itimerspec timeout;
		bool unknown_lidar;
		uint32_t pkt_ts1, pkt_ts2;

		if (!socket_flag && (ret = read_pcap_file(pcap_fd, ip_pkt)) == true)
		{
			pkt_ts1 = ip_pkt.ts.tv_sec * 1000000 + ip_pkt.ts.tv_usec; //packet 1's time stamp
#if TIMERFD_ENABLED
			if ((timerfd = timerfd_create(CLOCK_REALTIME, 0)) <= 0)
				std::cout << "timerfd_create" << std::endl; //init_timerfd
#endif
		}

		while (ros::ok())
		{
			if (socket_flag == true)
			{
				while (true)
				{
					ret = read_from_socket(sockfd, ip_pkt, sender_address);
					dst_ipaddr = ipaddr_from_sockaddr(sender_address);
					frag_offset = 0; // added by khkim on 20190828
					if (ret == 0)
						break;
				}
				get_ip_header(ip_pkt, ip_hdr);
			}
			else
			{
				if ((ret = read_pcap_file(pcap_fd, ip_pkt)) == false)
				{
					ROS_ERROR("cannot read pcap file");
					exit(1);
				}

				else
				{
					std::cout << ip_pkt.size << std::endl;

					pkt_ts2 = ip_pkt.ts.tv_sec * 1000000 + ip_pkt.ts.tv_usec;
					// ROS_WARN("pkt_ts1 : %u", pkt_ts1);
					// ROS_WARN("pkt_ts2 : %u", pkt_ts2);
					iat = pkt_ts2 - pkt_ts1;
					if (iat < 0)
						iat += 0x100000000;				 // complement the negative value!
					timeout.it_value.tv_sec = 0; // current value
					timeout.it_value.tv_nsec = iat * 1000 * param.pcap_wait_factor;
					timeout.it_interval.tv_sec = 1; // recurring - timer interval
					timeout.it_interval.tv_nsec = 0;

#if TIMERFD_ENABLED
					if ((ret = timerfd_settime(timerfd, 0, &timeout, NULL)) != 0)
						std::cout << "timerfd_settime" << std::endl;
#endif
					if (iat == 0)
						iat = 1;
					//ROS_ERROR("timetoarrival : %d", iat);
				}
				get_ip_header(ip_pkt, ip_hdr);
				sender_address.sin_addr.s_addr = ip_hdr.src_addr;
				dst_ipaddr = inet_ntoa(sender_address.sin_addr);

				if ((frag_offset = ip_hdr.flag_n_frag & 0x0fff) == 0)
				{
					get_udp_header(ip_pkt.buf, udp_hdr);
					//ROS_INFO("[0] ipaddr=%s", dst_ipaddr.c_str());
					dst_port = udp_hdr.dst_port;
				}
				ROS_INFO("ipaddr=%s", dst_ipaddr.c_str());
			}

			if (ip_pkt.size < 700)
				continue;

			for (i = 0; i < param.lidar_num; i++)
				if (dst_ipaddr == param.ip_addr[i])
				{
					mode_index = i;
					unknown_lidar = false;
					break;
				}

			if (i == param.lidar_num)
				unknown_lidar = true;

			if (!unknown_lidar && param.mode[mode_index] == LIDAR_MODE::VLP16_MODE)
			{
				//if (ip_pkt.size != 1206)
				//ROS_WARN("%s\t%d", param.ip_addr[i], ip_pkt.size); // by khkim on 20200527
				ichthus_lidar_driver::VLP16 VLP_msg;

				VLP16::get_VLP16_msg(socket_flag, ip_pkt, VLP_msg);
				pub_lidar_data[mode_index].publish(VLP_msg);
			}

			else if (!unknown_lidar && param.mode[mode_index] == LIDAR_MODE::OSI64_MODE)
			{
				ROS_INFO("%s\t%d", param.ip_addr[i].c_str(), ip_pkt.size);
				ichthus_lidar_driver::OSI64 OSI_msg;
				OSI64::get_OSI64_msg(socket_flag, ip_pkt, OSI_msg, frag_offset);

				if (OSI_msg.size == OSI64_MESSAGE_BYTES)
				{
					npub++;
					pub_lidar_data[mode_index].publish(OSI_msg);
					if (socket_flag == false)
						ROS_INFO("[%u] lidar msg published...", npub);
					nskip = 0;
				}
				else
				{
					nskip++;
					ROS_INFO("[%u] lidar msg skipped...(due to wrong size)", nskip);
					continue;
				}
			}
			else
			{
				nskip++;
				ROS_INFO("[%u] lidar msg skipped...(due to unknown lidar IP)", nskip);
				continue;
			}

			if (socket_flag == false)
			{
#if TIMERFD_ENABLED
				wait_timer(timerfd); // src/util.cpp
#endif
				ROS_INFO("timer wait");
				pkt_ts1 = pkt_ts2;
			}

			// else
			// 	ROS_WARN("Unknown LIDAR!!");
		}
	}
	void IchthusLidarFront::onInit()
	{
		ros::NodeHandle &nh = getNodeHandle();
		ros::NodeHandle &private_nh = getPrivateNodeHandle();

		private_nh.getParam("lidar_num", param.lidar_num);
		private_nh.getParam("pcap_file", param.pcap_file);
		private_nh.getParam("pcap_wait_factor", param.pcap_wait_factor); // added by khkim on 20191017

		param.mode = new int[param.lidar_num];
		param.ip_addr = new std::string[param.lidar_num];

		for (int i = 0; i < param.lidar_num; ++i)
		{
			private_nh.getParam(parameter[0] + position[i], param.mode[i]);
			private_nh.getParam(parameter[1] + position[i], param.ip_addr[i]);

			if (param.mode[i] == LIDAR_MODE::VLP16_MODE)
				pub_lidar_data[i] = nh.advertise<ichthus_lidar_driver::VLP16>(position[i] + parameter[2], queue_size);

			else if (param.mode[i] == LIDAR_MODE::OSI64_MODE)
				pub_lidar_data[i] = nh.advertise<ichthus_lidar_driver::OSI64>(position[i] + parameter[2], queue_size);

			else
			{
				ROS_ERROR("Unknown Mode");
				exit(1);
			}
		}

		if (param.pcap_file == "")
		{
			socket_flag = true;
			sockfd = create_socket();
		}
		else
		{
			socket_flag = false;
			if ((ret = open_pcap_file(&pcap_fd, param.pcap_file)) == false)
				exit(1);
		}

		main_loop_ = boost::thread(&IchthusLidarFront::extMsg, this);
	}
} // namespace IchthusLidarFront