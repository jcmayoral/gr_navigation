#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
    __global__
    void do_cuda_stuff_kernel(int n, int *x, int *hist){
      //int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //int index = blockIdx.x * blockDim.x + threadIdx.x;
      //int stride = blockDim.x * gridDim.x;

      int index = blockIdx.x * blockDim.x + threadIdx.x;
      int hist_index = static_cast<int>(x[index]);
      hist[hist_index] = hist[hist_index] + 1;

    }

    void stop_cuda_stuff(int *x,  int *hist){
      cudaFree(x);
      cudaFree(hist);
    }

    double do_cuda_stuff(int o_x[], int n){
      int bin_number = 255;
      // initialize x array on the host
      int * hist;
      int * result_hist;
      int * x;

      int h[bin_number] = {0};

      float delta = 0.1;

      hist = reinterpret_cast<int*>(malloc(bin_number*sizeof(int)));
      //x = o_x;
      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&x, n*sizeof(int));
      cudaMallocManaged(&hist, bin_number*sizeof(int));
      cudaMemcpy(x, o_x, n*sizeof(int), cudaMemcpyHostToDevice);
      cudaMemcpy(hist, h, bin_number*sizeof(int), cudaMemcpyHostToDevice);

      int threads = 128;
      int numBlocks = n/threads;
      // First param blocks
      // Second param number of threads
      //  blocks, threads each
      //int blockSize = 256;
      //numBlocks = (n + blockSize - 1) / blockSize;
      do_cuda_stuff_kernel<<<numBlocks,threads>>>(n,x, hist);
      cudaDeviceSynchronize(); // to print results

      result_hist = reinterpret_cast<int*>(malloc(bin_number*sizeof(int)));
      cudaMemcpy(result_hist, hist, bin_number*sizeof(int),  cudaMemcpyDeviceToHost);

      stop_cuda_stuff(x,hist);

      result_hist[0] = 0;
      result_hist[999] = 0;

      int max_index = -1;

      for (int w = 0; w < bin_number; w++){
        if (result_hist[w] > max_index){
          max_index = w;
        }
      }
      printf("Max Index %d", max_index);

      return double(max_index * delta) *0.001;
    }
}
