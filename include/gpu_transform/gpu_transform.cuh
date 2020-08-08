#ifndef __GPU_TRANSFORM_H_
#define __GPU_TRANSFORM_H_

//  This is the temploary patch for CUDA9 build problem
#if (__CUDACC_VER_MAJOR__ >= 9)
#undef __CUDACC_VER__
#define __CUDACC_VER__ 90000
#endif

#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <pcl/common/transforms.h>

#define BLOCK_SIZE_X 1024
#define BLOCK_SIZE_X2 512
#define BLOCK_SIZE_X3 256

#define BLOCK_X 16
#define BLOCK_Y 16
#define BLOCK_Z 4

#define SHARED_MEM_SIZE 3072

inline void gassert(cudaError_t err_code, const char *file, int line)
{
  if (err_code != cudaSuccess)
  {
    fprintf(stderr, "Error: %s %s %d\n", cudaGetErrorString(err_code), file, line);
    cudaDeviceReset();
    exit(EXIT_FAILURE);
  }
}

#define checkCudaErrors(err_code) gassert(err_code, __FILE__, __LINE__)

namespace gpu
{
  void transformPointCloud(pcl::PointCloud<pcl::PointXYZINormal> &cloud_in, pcl::PointCloud<pcl::PointXYZINormal> &cloud_out, const Eigen::Affine3f &transform);

} // namespace gpu

#endif