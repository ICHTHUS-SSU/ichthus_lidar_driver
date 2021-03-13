// #define PCL_NO_PRECOMPILE
// #include <pcl/memory.h>
// #include <pcl/pcl_macros.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>

// namespace pcl
// {
//   struct PointXYZI
//   {
//     PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
//     float test;
//     PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
//   } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

//   POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
//                                     (float, x, x)
//                                     (float, y, y)
//                                     (float, z, z)
//                                     (float, test, test)
// )



#ifndef __ICHHTUS_POINT_TYPES_H
#define __ICHHTUS_POINT_TYPES_H

#include <pcl/point_types.h>

namespace pcl
{
  struct PointXYZICA
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 
    uint16_t channel;
    uint32_t azimuth;                            
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; 


POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZICA,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, channel, channel)
                                  (uint32_t, azimuth, azimuth))

#endif // __ICHHTUS_POINT_TYPES_H
