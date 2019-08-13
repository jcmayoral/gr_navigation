#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
    __global__
    void do_cuda_stuff_kernel(int n, float *x, float *y)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        printf("Hello from thread %d!\n", idx);

        int index = blockIdx.x * blockDim.x + threadIdx.x;
        int stride = blockDim.x * gridDim.x;
        //int index = threadIdx.x;
        //int stride = blockDim.x;
        for (int i = index; i < n; i += stride)
          y[i] = x[i] + y[i];
    }

    void stop(float *x, float *y){
      cudaFree(x);
      cudaFree(y);
    }

    void do_cuda_stuff(int n, float *x, float *y)
    {
        // Allocate Unified Memory – accessible from CPU or GPU
        cudaMallocManaged(&x, n*sizeof(float));
        cudaMallocManaged(&y, n*sizeof(float));
        // 2 blocks, 3 threads each
        do_cuda_stuff_kernel<<<128, 128>>>(n,x,y);
        cudaDeviceSynchronize(); // to print results
        stop(x,y);
    }
}
