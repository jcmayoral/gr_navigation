#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
  __device__
  int getGlobalIdx_1D_1D(){
    return blockIdx.x *blockDim.x + threadIdx.x;
  }
    __global__
    void filter_passthrough_kernel(float *x, float *y, float *z, float limit){
      //__shared__ int s[256];
      //int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //int index = blockIdx.x * blockDim.x + threadIdx.x;
      //int stride = blockDim.x * gridDim.x;
      //printf("%d %d \n", gridDim.x, blockDim.x  );
      int index = getGlobalIdx_1D_1D();
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

      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&x, sizeof(o_x));
      cudaMallocManaged(&y, sizeof(x));
      cudaMallocManaged(&z, sizeof(x));
      cudaMemcpy(x, o_x, sizeof(o_x), cudaMemcpyHostToDevice);
      cudaMemcpy(y, o_x, sizeof(o_y), cudaMemcpyHostToDevice);
      cudaMemcpy(z, o_x, sizeof(o_z), cudaMemcpyHostToDevice);

      int nthreads = 512;
      dim3 threads (nthreads);
      int nblocks =  ceil(sizeof(x)/sizeof(float) / nthreads);//size/ nthreads -1;
      //memset(t, 0x00, nthreads);
      dim3 blocks(nblocks);
      // First param blocks
      // Second param number of threads
      //  blocks, threads each
      filter_passthrough_kernel<<<blocks,threads>>>(x,x,x,limit);
      cudaDeviceSynchronize(); // to print results
      //cudaMemcpy(tr, t, sizeof(x), cudaMemcpyDeviceToHost);
      free_memory(x);
      free_memory(y);
      free_memory(z);
      return 1;
    }
}
