#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
    __global__
    void do_cuda_stuff_kernel()
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        printf("Hello from thread %d!\n", idx);
    }

    void stop(){
      //cudaFree();
      //cudaFree(y);
    }

    void do_cuda_stuff()
    {
        // 2 blocks, 3 threads each
        do_cuda_stuff_kernel<<<2, 3>>>();
        cudaDeviceSynchronize(); // to print results
        stop();
    }
}


/*
__global__ void add(int n, float *x, float *y){
  //threadIdx.x contains the index of the current thread within its block
  //blockDim.x contains the number of threads in the block.
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  //int index = threadIdx.x;
  //int stride = blockDim.x;
  for (int i = index; i < N; i += stride)
    y[i] = x[i] + y[i];
*/
