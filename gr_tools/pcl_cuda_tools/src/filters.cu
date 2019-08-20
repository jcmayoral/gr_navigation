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
    void filter_passthrough_kernel(float *x, float *y, float *z, float min_limit, float max_limit){
      //__shared__ int s[256];
      //int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //int index = blockIdx.x * blockDim.x + threadIdx.x;
      //int stride = blockDim.x * gridDim.x;
      int index = getGlobalIdx_2D_2D();

      if (index > 1024*1024)
      printf("%d \n", index  );

      if (max_limit > x[index] > min_limit){
        x[index] = -1;
      }
      if (max_limit > y[index] > min_limit){
        y[index] = -1;
      }
      if (max_limit > z[index] > min_limit){
        z[index] = -1;
      }
    }

    void free_memory(float *x){
      cudaFree(x);
    }

    int apply_cuda_filter(float *o_x, float *o_y, float *o_z, float min_limit, float max_limit){
      // initialize x array on the host
      float * x, * y, *z;
      int size = 1024*1024;//sizeof(o_x)/sizeof(float);
      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&x, size*sizeof(float));
      cudaMallocManaged(&y, size*sizeof(float));
      cudaMallocManaged(&z, size*sizeof(float));
      printf("A\n");
      cudaMemcpy(x, o_x, size*sizeof(float), cudaMemcpyHostToDevice);
      cudaMemcpy(y, o_x, size*sizeof(float), cudaMemcpyHostToDevice);
      cudaMemcpy(z, o_x, size*sizeof(float), cudaMemcpyHostToDevice);
      printf("b\n");

      int nthreads = 32;
      dim3 threads (nthreads,nthreads);
      int nblocks =  16;//ceil(size / nthreads)/8;//size/ nthreads -1;
      //memset(t, 0x00, nthreads);
      dim3 blocks(nblocks,nblocks);
      // First param blocks
      // Second param number of threads
      //  blocks, threads each
      printf("C %d\n", nblocks);

      filter_passthrough_kernel<<<blocks,threads>>>(x,y,z, min_limit, max_limit);
      cudaDeviceSynchronize(); // to print results
      //cudaMemcpy(tr, t, sizeof(x), cudaMemcpyDeviceToHost);
      cudaMemcpy(o_x, x, size*sizeof(float), cudaMemcpyDeviceToHost);
      cudaMemcpy(o_y, y, size*sizeof(float), cudaMemcpyDeviceToHost);
      cudaMemcpy(o_z, z, size*sizeof(float), cudaMemcpyDeviceToHost);

      free_memory(x);
      free_memory(y);
      free_memory(z);
      printf("size %d\n",size );

      return 1;
    }
}
