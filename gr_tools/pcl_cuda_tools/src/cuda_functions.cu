#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
    __global__
    void do_cuda_stuff_kernel(int n, unsigned char *x, int *hist, float delta){
      //int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //printf("Hello from thread %f!\n", delta);

      int index = blockIdx.x * blockDim.x + threadIdx.x;
      int stride = blockDim.x * gridDim.x;
      //int index = threadIdx.x;
      //int stride = blockDim.x;
      //printf("delta %d", delta);
      int hist_index;
      for(auto i =index; i<n; i+= stride){
        hist_index = x[i] / delta;
        //printf("%d", x[i]);
        hist[hist_index] = hist[hist_index] + 1;
      }

    }

    void stop_cuda_stuff(unsigned char *x, int *hist, float delta, int n){
      cudaFree(x);
      cudaFree(hist);
    }

    void do_cuda_stuff(int n, unsigned char *x, int *hist, float delta){
      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&x, n*sizeof(int));
      cudaMallocManaged(&hist, 1000*sizeof(unsigned char));
      //cudaMalloc(&n, sizeof(int));
      //cudaMallocManaged(&delta, sizeof(float));

      //  blocks, threads each
      do_cuda_stuff_kernel<<<128, 128>>>(n,x, hist, delta);
      cudaDeviceSynchronize(); // to print results
      //stop(x,hist);
    }
}
