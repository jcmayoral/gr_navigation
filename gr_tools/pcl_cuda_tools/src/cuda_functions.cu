#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
    __global__
    void do_cuda_stuff_kernel(int n, int *x, int *hist, float delta){
      int idx = blockIdx.x * blockDim.x + threadIdx.x;
      printf("Hello from thread %d!\n", idx);

      int index = blockIdx.x * blockDim.x + threadIdx.x;
      int stride = blockDim.x * gridDim.x;
      //int index = threadIdx.x;
      //int stride = blockDim.x;

      for(auto i =index; i<n; i+= stride){
        index = x[i] / delta;
        hist[index] = hist[index] + 1;
      }

    }

    void stop(int *x, int *hist){
      cudaFree(x);
      cudaFree(hist);
    }

    void do_cuda_stuff(int n, int *x, int *hist, float delta){
      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&x, n*sizeof(int));
      cudaMallocManaged(&hist, 1000*sizeof(int));
      //  blocks, threads each
      do_cuda_stuff_kernel<<<128, 128>>>(n,x, hist, delta);
      cudaDeviceSynchronize(); // to print results
      stop(x,hist);
    }
}
