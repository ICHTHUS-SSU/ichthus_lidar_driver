#include <gpu_transform/gpu_transform.cuh>

namespace gpu
{
  __global__ void gpuTransformPointCloud(pcl::PointXYZINormal* in_points, pcl::PointXYZINormal* out_points, int num_points, float* transform)
  {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    float x, y, z;

    for (int i = idx; i < num_points; i += stride) 
    {
      x = in_points[i].x;
      y = in_points[i].y;
      z = in_points[i].z;
    
      out_points[i].x = transform[0 * 4 + 0] * x + transform[0 * 4 + 1] * y + transform[0 * 4 + 2] * z + transform[0 * 4 + 3];
      out_points[i].y = transform[1 * 4 + 0] * x + transform[1 * 4 + 1] * y + transform[1 * 4 + 2] * z + transform[1 * 4 + 3];
      out_points[i].z = transform[2 * 4 + 0] * x + transform[2 * 4 + 1] * y + transform[2 * 4 + 2] * z + transform[2 * 4 + 3];
      out_points[i].intensity = in_points[i].intensity;
      out_points[i].curvature = in_points[i].curvature;
      out_points[i].normal_x = in_points[i].normal_x;
      out_points[i].normal_y = in_points[i].normal_y;
      out_points[i].normal_z = in_points[i].normal_z;
      out_points[i].data[3] = in_points[i].data[3];
      out_points[i].data_n[3] = in_points[i].data_n[3];
    }
  }

  void transformPointCloud(pcl::PointCloud<pcl::PointXYZINormal> &cloud_in, pcl::PointCloud<pcl::PointXYZINormal> &cloud_out, const Eigen::Affine3f &transform)
  {
    int num_points;
    pcl::PointXYZINormal *d_points, *h_points;
    pcl::PointXYZINormal *d_tf_points, *h_tf_points;
    float *d_tf, *h_tf;

    Eigen::Transform<float, 3, Eigen::Affine> mat(transform.matrix());
    num_points = cloud_in.size();
    h_points = cloud_in.points.data();
    
    h_tf = (float *)malloc(sizeof(float) * 4 * 4); // 4x4 matrix
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        h_tf[4 * i + j] = mat(i, j);
      }
    }

    checkCudaErrors(cudaMalloc(&d_tf, sizeof(float) * 4 * 4)); // 4x4 matrix
    checkCudaErrors(cudaMalloc(&d_points, sizeof(pcl::PointXYZINormal) * num_points));
    checkCudaErrors(cudaMalloc(&d_tf_points, sizeof(pcl::PointXYZINormal) * num_points));
    
    checkCudaErrors(cudaMemcpy(d_tf, h_tf, sizeof(float) * 4 * 4, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_points, h_points, sizeof(pcl::PointXYZINormal) * num_points, cudaMemcpyHostToDevice));

    if (num_points > 0) 
    {
      int block_x = (num_points <= BLOCK_SIZE_X) ? num_points : BLOCK_SIZE_X;
      int grid_x = (num_points - 1) / block_x + 1;

      gpuTransformPointCloud<<<grid_x, block_x>>>(d_points, d_tf_points, num_points, d_tf);
      checkCudaErrors(cudaGetLastError());
      checkCudaErrors(cudaDeviceSynchronize());
    }

    cloud_out.clear();
    cloud_out.resize(num_points);
    cloud_out.header = cloud_in.header;
    h_tf_points = cloud_out.points.data();
    checkCudaErrors(cudaMemcpy(h_tf_points, d_tf_points, sizeof(pcl::PointXYZINormal) * num_points, cudaMemcpyDeviceToHost));
    
    free(h_tf);
    checkCudaErrors(cudaFree(d_points));
    checkCudaErrors(cudaFree(d_tf_points));
    checkCudaErrors(cudaFree(d_tf));
  }
}
