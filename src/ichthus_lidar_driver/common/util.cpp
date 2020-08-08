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

#include<ichthus_lidar_driver/common/util.h>

void quit(const char *tag){
  printf("%s error: %s (errno=%d)\n", tag, strerror(errno), errno);
  exit(1);
}

int init_timer(int usec) {
  int ret;
  int fd = -1;
  struct itimerspec timeout;

  if (usec >= 1000000) {
    printf("init_timer() accepts 1~999999 us.\n");
    exit(1);
  }

  if ((fd = timerfd_create(CLOCK_REALTIME, 0)) <= 0)
    quit("timerfd_create");

  //if ((ret = fcntl(fd, F_SETFL, O_NONBLOCK)) != 0)
  //  quit("fcntl");

  timeout.it_value.tv_sec = 0;
  timeout.it_value.tv_nsec = usec * 1000;
  timeout.it_interval.tv_sec = 0; // recurring
  timeout.it_interval.tv_nsec = usec * 1000;
  if ((ret = timerfd_settime(fd, 0, &timeout, NULL)) != 0)
    quit("timerfd_settime");

  return fd;
}

int wait_timer(int timerfd) {
  int ret;
  unsigned long long missed;
  if ((ret = read(timerfd, &missed, sizeof(missed))) < 0)
    std::cout << "read error" << std::endl;

//  ROS_ERROR("%lu",missed);

  return ret;
}

