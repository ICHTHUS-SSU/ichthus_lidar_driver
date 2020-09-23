#include <ichthus_lidar_driver/common/lidar.h>

namespace VLP16
{
  ////////////////////////////////////////////////////////////////////////////////////////////////
  // Copied from http://docs.ros.org/melodic/api/velodyne_pointcloud/html/rawdata_8cc_source.html
  ////////////////////////////////////////////////////////////////////////////////////////////////
  
  void get_VLP16_msg(bool socket_flag, ip_pkt &packet, ichthus_lidar_driver::VLP16 &vlp16_msg)
  {
    // IP packet's payload(1214Bytes)
    //ROS_INFO("pakcet.buf : %lu", packet.buf.size());

    //ip---------------------
    if (socket_flag == false)
    {
      uint8_t *pkt_base;
      int pkt_size;
      pkt_base = (uint8_t *)packet.buf + UDP_HEADER_BYTES;
      pkt_size = packet.size - UDP_HEADER_BYTES;
      vlp16_msg.size = pkt_size;

      memcpy(&vlp16_msg.buf, pkt_base, pkt_size); // ???
    }
    //socket.ver------------
    else
    {
      memcpy(&vlp16_msg.buf, packet.buf, VLP16_MESSAGE_BYTES);
    }
    vlp16_msg.ts.sec = (uint32_t)packet.ts.tv_sec;
    vlp16_msg.ts.nsec = (uint32_t)packet.ts.tv_usec * 1000;
    // ROS_INFO("vlp16_msg.ts : %ld(s) %ld(us)",
    //                 vlp16_msg.ts.tv_sec, vlp16_msg.ts.tv_usec);
  }

  bool update_VLP16_packet(const ichthus_lidar_driver::VLP16 &msg, int direction)
  {
    VLP16_packet *p_data = (VLP16_packet *)&msg.buf;
    float azimuth = (float)(p_data->blocks[BLOCKS_PER_PACKET - 1].azimuth);

    if (direction == 0)
    {
      if (azimuth / 100.0 > 180.0)
        return true;
      else
        return false;
    }
    else if (direction == 1)
    {
      if (azimuth / 100.0 < 180.0)
        return true;
      else
        return false;
    }
    else
      return false;
  }

  void print_VLP16_packet(const ichthus_lidar_driver::VLP16 &msg)
  { //, int lidar_port) {
    // cf) msg : 1214Bytes = UDP(8) + data(1206)
    VLP16_packet *p_data = (VLP16_packet *)&msg.buf;
    float azimuth;
    float distance;
    uint8_t intensity;

    // ROS_INFO("[LIDAR_ID = %d]=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*", lidar_port);
    for (int block = 0; block < BLOCKS_PER_PACKET; block++)
    {
      if (UPPER_BANK != p_data->blocks[block].header)
      {
        ROS_WARN("skipping invalid VLP-16 packet : block %d header value is %u",
                 block, p_data->blocks[block].header);
        return;
      }
      azimuth = (float)(p_data->blocks[block].azimuth);

      ROS_INFO("block # : %d", block);
      ROS_INFO("azimuth : %f", azimuth / 100.0);

      for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
      {
        for (int channel_idx = 0; channel_idx < VLP16_SCANS_PER_FIRING; channel_idx++, k += RAW_SCAN_SIZE)
        {
          distance = DISTANCE_RESOLUTION *
                     ((p_data->blocks[block].data[k + 1] << 8) | (p_data->blocks[block].data[k]));
          intensity = p_data->blocks[block].data[k + 2];
          // ROS_INFO("channel : %d, distance : %2.2f(m), intensity : %u",
          // firing * VLP16_SCANS_PER_FIRING + channel_idx, distance, intensity);
        }
      }

      const uint8_t *p = p_data->timestamp;
      uint32_t timestamp = (p[3] << 24) | (p[2] << 16) | (p[1] << 8) | (p[0]);
      // ROS_INFO("timestamp : %u", timestamp);
      // ROS_INFO("==================================================");
    }
  }

  bool pointInRange(float range)
  {
    return (range >= vlp_config.min_range && range <= vlp_config.max_range);
  }

  /** Update parameters: conversions and update */
  void setParameters(double min_range, double max_range, double view_direction, double view_width)
  {
    vlp_config.min_range = min_range;
    vlp_config.max_range = max_range;

    //converting angle parameters into the velodyne reference (rad)
    vlp_config.tmp_min_angle = view_direction + view_width / 2;
    vlp_config.tmp_max_angle = view_direction - view_width / 2;

    //computing positive modulo to keep theses angles into [0;2*M_PI]
    vlp_config.tmp_min_angle = fmod(fmod(vlp_config.tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    vlp_config.tmp_max_angle = fmod(fmod(vlp_config.tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

    //converting into the hardware velodyne ref (negative yaml and degrees)
    //adding 0.5 perfomrs a centered double to int conversion
    vlp_config.min_angle = 100 * (2 * M_PI - vlp_config.tmp_min_angle) * 180 / M_PI + 0.5;
    vlp_config.max_angle = 100 * (2 * M_PI - vlp_config.tmp_max_angle) * 180 / M_PI + 0.5;
    if (vlp_config.min_angle == vlp_config.max_angle)
    {
      //avoid returning empty cloud if min_angle = max_angle
      vlp_config.min_angle = 0;
      vlp_config.max_angle = 36000;
    }
  }

  void add_VLP16_vector_msg_to_cloud(const std::vector<ichthus_lidar_driver::VLP16::ConstPtr> &vector_msg, pcl::PointCloud<pcl::PointXYZINormal> &cloud, float* sin_rot_table, float* cos_rot_table, Calibration& calibration)
  {
    // VLP16_packet *p_data = (VLP16_packet *) &msg.buf; ///???
    PointXYZIR point;
    float azimuth, azimuth_diff, azimuth_corrected_f, last_azimuth_diff = 0;
    int azimuth_corrected;
    float x, y, z;
    float intensity;
    float sec, usec;
    bool need_update = true;
    sec = 0; usec = 0;

#if 0
    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table[rot_index] = cosf(rotation);
      sin_rot_table[rot_index] = sinf(rotation);
    }
    //param from "velodyne_pointcloud/launch/velodyne_vlp16.launch
#endif

    setParameters(0.4, 130.0, 0, 2 * M_PI); // ???
    for (int i = 0; i < vector_msg.size(); i++)
    {
      VLP16_packet *p_data = (VLP16_packet *)&(*vector_msg[i]).buf;

      for (int block = 0; block < BLOCKS_PER_PACKET; block++)
      {
        // ignore packets with mangled or otherwise different contents
        if (UPPER_BANK != p_data->blocks[block].header)
        {
          // Do not flood the log with messages, only issue at most one
          // of these warnings per minute.
          ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLP-16 packet: block "
                                           << block << " header value is "
                                           << p_data->blocks[block].header);
          return; // bad packet: skip the rest
        }
        // *p_data = (VLP16_packet *) &msg.buf;
        // Calculate difference between current and next block's azimuth angle
        azimuth = (float)(p_data->blocks[block].azimuth);
        if (block < BLOCKS_PER_PACKET - 1)
        {
          azimuth_diff = (float)((36000 + p_data->blocks[block + 1].azimuth - p_data->blocks[block].azimuth) % 36000);
          last_azimuth_diff = azimuth_diff;
        }
        else
        {
          azimuth_diff = last_azimuth_diff;
        }

        for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
        {
          for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)
          {

            LaserCorrection &corrections = calibration.laser_corrections[dsr];
            // ROS_INFO("add : calibration : %p", &calibration);
            /** correct for the laser rotation as a function of timing during the firings **/
            azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) + (firing * VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
            azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

            /*condition added to avoid calculating points which are not
              in the interesting defined area (min_angle < area < max_angle)*/
            // ROS_INFO("azimuth_corrected : %d", azimuth_corrected);
            if ((azimuth_corrected >= vlp_config.min_angle && azimuth_corrected <= vlp_config.max_angle && vlp_config.min_angle < vlp_config.max_angle) || (vlp_config.min_angle > vlp_config.max_angle && (azimuth_corrected <= vlp_config.max_angle || azimuth_corrected >= vlp_config.min_angle)))
            {

              // convert polar coordinates to Euclidean XYZ
              float distance = DISTANCE_RESOLUTION * ((p_data->blocks[block].data[k + 1] << 8) | (p_data->blocks[block].data[k]));
              uint8_t intensity = p_data->blocks[block].data[k + 2];

              float cos_vert_angle = corrections.cos_vert_correction;
              float sin_vert_angle = corrections.sin_vert_correction;
              float cos_rot_correction = corrections.cos_rot_correction;
              float sin_rot_correction = corrections.sin_rot_correction;

              // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
              // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)

              float cos_rot_angle =
                  cos_rot_table[azimuth_corrected] * cos_rot_correction +
                  sin_rot_table[azimuth_corrected] * sin_rot_correction;
              float sin_rot_angle =
                  sin_rot_table[azimuth_corrected] * cos_rot_correction -
                  cos_rot_table[azimuth_corrected] * sin_rot_correction;
              // ROS_WARN("cos_rot_angle : %f, sin_rot_angle : %f",cos_rot_angle,sin_rot_angle);
              float horiz_offset = corrections.horiz_offset_correction;
              float vert_offset = corrections.vert_offset_correction;

              // Compute the distance in the xy plane (w/o accounting for rotation)
              /* the new term of 'vert_offset * sin_vert_angle'
               * was added to the expression due to the mathemathical
               * model we used.
               */
              float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

              // Calculate temporal X, use absolute value.
              float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

              // Calculate temporal Y, use absolute value
              float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
              if (xx < 0)
                xx = -xx;
              if (yy < 0)
                yy = -yy;

              // Get 2points calibration values,Linear interpolation to get distance
              // correction for X and Y, that means distance correction use
              // different value at different distance
              float distance_corr_x = 0;
              float distance_corr_y = 0;
              if (corrections.two_pt_correction_available)
              {
                distance_corr_x =
                    (corrections.dist_correction - corrections.dist_correction_x) * (xx - 2.4) / (25.04 - 2.4) + corrections.dist_correction_x;
                distance_corr_x -= corrections.dist_correction;
                distance_corr_y =
                    (corrections.dist_correction - corrections.dist_correction_y) * (yy - 1.93) / (25.04 - 1.93) + corrections.dist_correction_y;
                distance_corr_y -= corrections.dist_correction;
              }

              float distance_x = distance + distance_corr_x;
              /* the new term of 'vert_offset * sin_vert_angle'
               * was added to the expression due to the mathemathical
               * model we used.
               */
              xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
              x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

              float distance_y = distance + distance_corr_y;
              /* the new term of 'vert_offset * sin_vert_angle'
               * was added to the expression due to the mathemathical
               * model we used.
               */
              xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
              y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

              // Using distance_y is not symmetric, but the velodyne manual
              // does this.
              /* the new term of 'vert_offset * cos_vert_angle'
               * was added to the expression due to the mathemathical
               * model we used.
               */
              z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

              /** Use standard ROS coordinate system (right-hand rule) */
              float x_coord = y;
              float y_coord = -x;
              float z_coord = z;

              /** Intensity Calculation */
              float min_intensity = corrections.min_intensity;
              float max_intensity = corrections.max_intensity;

              //intensity = p_data->blocks[block].data[k+2];

              float focal_offset = 256 * (1 - corrections.focal_distance / 13100) * (1 - corrections.focal_distance / 13100);
              float focal_slope = corrections.focal_slope;
              //intensity += focal_slope * (std::abs(focal_offset - 256 *
              //  (1 - tmp.uint/65535)*(1 - tmp.uint/65535)));
              intensity = (intensity < min_intensity) ? min_intensity : intensity;
              intensity = (intensity > max_intensity) ? max_intensity : intensity;
              //ROS_INFO("(x,y,z)(m) = (%f, %f, %f)", x_coord, y_coord, z_coord);

              if (pointInRange(distance))
              { // from rawdata.h
                // append this point to the cloud
                pcl::PointXYZINormal point;
                point.normal_x = corrections.laser_ring;
                point.x = x_coord;
                point.y = y_coord;
                point.z = z_coord;
                point.intensity = intensity;
                // point.ts = msg.ts.tv_sec();
                if (need_update == true)
                {
                  sec = (float)((int)((*vector_msg[i]).ts.sec) % 1000000); // fixed by qjin, 1. long->float , 2. upper value not needed
                  usec = (float)((int)((*vector_msg[i]).ts.nsec) / 1000);  // fixed by qjin, 1. long->float
                  need_update = false;
                }
                point.normal_y = sec;
                point.normal_z = usec;
                cloud.push_back(point);
                //++cloud.width;
              }
            } //for 반복문
          }
        }
      }
    }
    return;
  }
} // namespace VLP16

namespace OSI64
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  // rewritten while referring to https://github.com/ouster-lidar/ouster_example/tree/master/ouster_ros/src
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  void get_OSI64_msg(bool socket_flag, ip_pkt &packet, ichthus_lidar_driver::OSI64 &osi64_msg, int offset)
  {
    if (socket_flag == true)
    {
      uint8_t *pkt_base;
      int pkt_size, msg_size;
      osi64_msg.size = packet.size;
      osi64_msg.ts.sec = (uint32_t)packet.ts.tv_sec;
      osi64_msg.ts.nsec = (uint32_t)packet.ts.tv_usec * 1000;
      memcpy(&osi64_msg.buf, packet.buf, OSI64_MESSAGE_BYTES);
    }
    else
    {
      bool tables_initialized = init_tables();
      uint8_t *pkt_base;
      int pkt_size, msg_size;

      if (offset == 0)
      { // 1st packet of the ouster msg (size = udp_header + 1472)
        pkt_base = (uint8_t *)packet.buf + UDP_HEADER_BYTES;
        pkt_size = packet.size - UDP_HEADER_BYTES;
        //osi64_msg.buf.resize(pkt_size);
        memcpy(&osi64_msg.buf, pkt_base, pkt_size);
        osi64_msg.size = pkt_size;
        osi64_msg.ts.sec = (uint32_t)packet.ts.tv_sec;
        osi64_msg.ts.nsec = (uint32_t)packet.ts.tv_usec * 1000;
        ROS_INFO("get_OSI64_msg() : (pkt_size, offset) = (%u, %d)", pkt_size, offset * 8);
        ROS_INFO("get_OSI64_msg() : ts=%u.%09u sec", osi64_msg.ts.sec, osi64_msg.ts.nsec);
      }
      else
      { // subsequent packet of the ouster msg (size = 1480, 1480, ..., 776)
        pkt_base = (uint8_t *)packet.buf;
        pkt_size = packet.size;
        //msg_size = osi64_msg.buf.size();
        //osi64_msg.buf.resize(msg_size + pkt_size);
        memcpy(&osi64_msg.buf[osi64_msg.size], pkt_base, pkt_size);
        osi64_msg.size += pkt_size;
        ROS_INFO("get_OSI64_msg() : (pkt_size, offset) = (%u, %d)", pkt_size, offset * 8);

#if 0 // removed by khkim on 20190828
      if (offset*8 == 11840) { // last packet of msg
        ROS_INFO("get_OSI64_msg() : pkt ts=(%ld(s),%ld(us))", packet.ts.tv_sec, packet.ts.tv_usec);
        double temp = ((packet.ts.tv_sec + packet.ts.tv_usec * 0.000001) + (osi64_msg.ts.sec + osi64_msg.ts.nsec * 0.000001)) / 2;
        //ROS_INFO("get_OSI64_msg() : average : %f(s)",temp);
        osi64_msg.ts.sec = temp;
        osi64_msg.ts.nsec = (temp - osi64_msg.ts.sec) * 1000000;
        ROS_INFO("get_OSI64_msg() : msg final ts=(%u(s),%u(ns))", osi64_msg.ts.sec, osi64_msg.ts.nsec);
      }
#endif
      }
      //ROS_INFO("get_OSI64_msg() : message size = %lu", osi64_msg.buf.size());
    }
  }

  void print_OSI64_packet(const ichthus_lidar_driver::OSI64 &msg)
  { //, int lidar_port) {
    OSI64_packet *p_data = (OSI64_packet *)&msg.buf;
    if (msg.size != OSI64_MESSAGE_BYTES)
    {
      ROS_INFO("print_OSI64_packet() : wrong msg size!");
      return;
    }

    for (int block = 0; block < BLOCKS_PER_PACKET; block++)
    {
      uint32_t packet_status;
      if ((packet_status = p_data->blocks[block].packet_status) == 0)
      {
        ROS_INFO("Bad Packet");
        return;
      }
      // ROS_INFO("block idx# : %d", block);
      // ROS_INFO("block size : %lu", sizeof(p_data->blocks[block]));
      uint32_t a = p_data->blocks[block].timestamp[0];
      uint32_t b = p_data->blocks[block].timestamp[1];
      // ROS_INFO("timestamp a: %u, b : %u(ns)", a, b);
      uint32_t encoder_count = p_data->blocks[block].encoder_count;
      uint16_t measurement_id = p_data->blocks[block].measurement_id;
      // ROS_INFO("measurement_id : %u", measurement_id);
      for (int channel_idx = 0, k = 0; channel_idx < BLOCK_PER_AZIMUTH; channel_idx++, k += SCANS_PER_BLOCK /*word*/)
      {
        uint32_t range = (p_data->blocks[block].data[k]) & 0x000fffff;
        uint32_t reflectivity = (p_data->blocks[block].data[k + 1]) & 0x0000ffff;
        // ROS_INFO("range : %f (m), reflectivity : %u", range *0.001, reflectivity);
      }
    }
  }

  void add_OSI64_vector_msg_to_cloud(const std::vector<ichthus_lidar_driver::OSI64ConstPtr> &msgs, pcl::PointCloud<pcl::PointXYZINormal> &out_cloud)
  {
    float sec, usec;
    bool need_update = true;
    sec = 0; usec = 0;
    for (int i = 0; i < msgs.size(); i++)
    {
      const ichthus_lidar_driver::OSI64 &msg = *msgs[i];

      OSI64_packet *p_data = (OSI64_packet *)&msg.buf;
      //PointXYZIR point;
      for (int block = 0; block < BLOCKS_PER_PACKET; block++)
      {
        for (int channel_idx = 0, k = 0; channel_idx < BLOCK_PER_AZIMUTH; channel_idx++, k += SCANS_PER_BLOCK /*word*/)
        {
          pcl::PointXYZINormal point;
          uint32_t encoder_count = p_data->blocks[block].encoder_count;
          float h_angle_0 = 2.0 * M_PI * (float)encoder_count / encoder_ticks_per_rev;
          trig_table_entry tte = trig_table[channel_idx];
          float h_angle = tte.beam_azimuth_angles + h_angle_0;

          uint32_t range = (p_data->blocks[block].data[k]) & 0x000fffff;
          float r = range / 1000.0; // change unit (mm) to (m)
          point.x = -r * tte.cos_beam_altitude_angles * cosf(h_angle);
          point.y = r * tte.cos_beam_altitude_angles * sinf(h_angle);
          point.z = r * tte.sin_beam_altitude_angles;
          // ROS_INFO("(x,y,z)(m) = (%f, %f, %f)", point.x, point.y, point.z);
          uint32_t reflectivity = (p_data->blocks[block].data[k + 1]) & 0x0000ffff;
          point.intensity = reflectivity;
          point.normal_x = channel_idx;

          if (need_update == true)
          {
            sec = (float)((int)(msg.ts.sec) % 1000000); // fixed by qjin, 1. long->float , 2. upper value not needed
            usec = (float)((int)(msg.ts.nsec) / 1000); // fixed by qjin, 1. long->float
            need_update = false;
          }
          point.normal_y = sec;
          point.normal_z = usec;

          out_cloud.push_back(point);
        }
      }
    }

    return;
  }
} //namespace OSI64
