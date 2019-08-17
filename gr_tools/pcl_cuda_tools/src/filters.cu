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
    void filter_passthrough_kernel(float *x, float *y, float *z, float limit){
      //__shared__ int s[256];
      //int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //int index = blockIdx.x * blockDim.x + threadIdx.x;
      //int stride = blockDim.x * gridDim.x;
      int index = getGlobalIdx_2D_2D();

      if (index > 1024*1024)
      printf("%d \n", index  );

      if (x[index] > limit){
        x[index] = -1;
      }
      if (y[index] > limit){
        y[index] = -1;
      }
      if (z[index] > limit){
        z[index] = -1;
      }
    }

    void free_memory(float *x){
      cudaFree(x);
    }

    int apply_cuda_filter(float *o_x, float *o_y, float *o_z, float limit){
      // initialize x array on the host
      float * x, * y, *z;
      int size = 1024*1024;//sizeof(o_x)/sizeof(float);
      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&x, size*sizeof(float));
      cudaMallocManaged(&y, size*sizeof(float));
      cudaMallocManaged(&z, size*sizeof(float));
      printf("A\n");
      cudaMemcpy(x, o_x, sizeof(o_x), cudaMemcpyHostToDevice);
      cudaMemcpy(y, o_x, sizeof(o_y), cudaMemcpyHostToDevice);
      cudaMemcpy(z, o_x, sizeof(o_z), cudaMemcpyHostToDevice);
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

      filter_passthrough_kernel<<<blocks,threads>>>(x,y,z,limit);
      cudaDeviceSynchronize(); // to print results
      //cudaMemcpy(tr, t, sizeof(x), cudaMemcpyDeviceToHost);
      free_memory(x);
      free_memory(y);
      free_memory(z);
      printf("size %d\n",size );

      return 1;
    }
}
