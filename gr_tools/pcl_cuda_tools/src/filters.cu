#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
  __device__
  int getGlobalIdx_1D_1D(){
    return blockIdx.x *blockDim.x + threadIdx.x;
  }

  __device__
  int getGlobalIdx_2D_2D(){
    int blockId = blockIdx.x + blockIdx.y * gridDim.x;
    int threadId = blockId * (blockDim.x * blockDim.y) + (threadIdx.y * blockDim.x) + threadIdx.x;
    return threadId;
  }
    __global__
    void filter_passthrough_kernel(float *z, bool *b, float min_limit, float max_limit, float filter_value, int size){
      //__shared__ int s[256];
      //int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //int index = blockIdx.x * blockDim.x + threadIdx.x;
      //int stride = blockDim.x * gridDim.x;
      int index = getGlobalIdx_1D_1D();
      if (index >= size){
        return;
      }


      /*
      if (max_limit < x[index] || x[index] < min_limit){
        //x[index] = filter_value;
        //y[index] = filter_value;
        b[index] = true;
      }
      if (max_limit < y[index] || y[index] < min_limit){
        //x[index] = filter_value;
        //y[index] = filter_value;
        b[index] = true;
      }
      */
      if (max_limit < z[index] || z[index] < min_limit){
        //x[index] = filter_value;
        //y[index] = filter_value;
        b[index] = true;
      }

      /*
      if (isnan(z[index])){
        b[index] = true;
      }

      if (isnan(x[index])){
        b[index] = true;
      }

      if (isnan(y[index])){
        b[index] = true;
      }
      */
    }

    int apply_cuda_filter(float *o_z, bool *o_b, float min_limit, float max_limit, float filter_value, int size){
      // initialize x array on the host
      float *z;
      bool *b;

      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&z, size*sizeof(float));
      cudaMallocManaged(&b, size*sizeof(bool));

      cudaMemcpy(z, o_z, size*sizeof(float), cudaMemcpyHostToDevice);
      cudaMemcpy(b, o_b, size*sizeof(bool), cudaMemcpyHostToDevice);

      int ngrid;      // The launch configurator returned block size
      int nblocks;    // The minimum grid size needed to achieve the maximum occupancy for a full device launch

      ngrid = 1024;
      dim3 grid (ngrid);
      nblocks = ceil((size+ngrid -1)/ngrid);

      dim3 blocks(nblocks);

      // First param blocks
      // Second param number of threads
      //  blocks, threads each
      filter_passthrough_kernel<<<blocks,grid>>>(z,b,min_limit, max_limit, filter_value, size);
      cudaDeviceSynchronize(); // to print results
      cudaMemcpy(o_b, b, size*sizeof(bool), cudaMemcpyDeviceToHost);

      //free_memory(x);
      //free_memory(y);
      cudaFree(z);
      cudaFree(b);

      return 1;
    }
}
