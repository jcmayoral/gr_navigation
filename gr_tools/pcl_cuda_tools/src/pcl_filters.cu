#include <cuda_runtime.h>
#include <stdio.h>
//#include <pcl/point_cloud.h>
#include <pcl_cuda_tools/pcl_cuda_filter_functions.cuh>

extern "C"
{
  __device__
  int getGlobalIdx_1D_1D_2(){
    return blockIdx.x *blockDim.x + threadIdx.x;
  }

    __global__
    void pcl_filter_passthrough_kernel(pcl::PointXYZ *d_point_cloud, bool* b, float min_limit, float max_limit, float filter_value, int size){
      int index = getGlobalIdx_1D_1D_2();
      if (index >= size){
        return;
      }
      float vSrcVector[3] = {d_point_cloud[index].x, d_point_cloud[index].y, d_point_cloud[index].z};

      if (max_limit < vSrcVector[2] || vSrcVector[2] < min_limit){
        //d_point_cloud[index].x = filter_value;
        //d_point_cloud[index].y = filter_value;
        //d_point_cloud[index].z = filter_value;
        b[index] = false;
      }

    }

    int pcl_apply_cuda_filter(pcl::PointCloud<pcl::PointXYZ> &point_cloud, bool* o_b,float min_limit, float max_limit, float filter_value, int size){
      // initialize x array on the host
      bool *b;
      pcl::PointXYZ * d_point_cloud;

      cudaMallocManaged(&b, size*sizeof(bool));
      cudaMalloc((void**)&d_point_cloud, point_cloud.points.size()*sizeof(pcl::PointXYZ) );
      cudaMemcpy(d_point_cloud, point_cloud.points.data(), point_cloud.points.size()*sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);
      cudaMemcpy(b, o_b, size*sizeof(bool), cudaMemcpyHostToDevice);

      int ngrid;      // The launch configurator returned block size
      int nblocks;    // The minimum grid size needed to achieve the maximum occupancy for a full device launch
      ngrid = 256;
      dim3 grid (ngrid);
      nblocks = ceil((size+ngrid -1)/ngrid);

      dim3 blocks(nblocks);

      pcl_filter_passthrough_kernel<<<blocks,grid>>>(d_point_cloud,b, min_limit, max_limit, filter_value, size);
      cudaDeviceSynchronize(); // to print results

      //cudaMemcpy(point_cloud.points.data(), d_point_cloud, point_cloud.points.size()*sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);

      //mask = (bool *)malloc(point_cloud.points.size()*sizeof(bool));

      //cudaMemcpy(mask, b, size*sizeof(bool), cudaMemcpyDeviceToHost);


      pcl::PointCloud<pcl::PointXYZ> new_point_cloud;
      for(size_t i = 0; i < point_cloud.points.size(); i++){
        if(b[i]){
          new_point_cloud.push_back(point_cloud[i]);
        }
      }

      printf("DONE\n");

      point_cloud = new_point_cloud;
      cudaFree(d_point_cloud);
      cudaFree(b);
      return 1;
    }
}
