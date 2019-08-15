#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
    __global__
    void do_cuda_stuff_kernel(int n, int *x, int *hist){
      int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //int index = blockIdx.x * blockDim.x + threadIdx.x;
      //int stride = blockDim.x * gridDim.x;

      int bin_number = 1000;
      float max_value = 65535;
      float delta = max_value/bin_number;
      int hist_index = 0;

      int index = threadIdx.x;
      int stride = blockDim.x;


       for (int i = index; i < n; i += stride){
         hist_index = (int) x[i]/delta;
         hist[hist_index] = hist[hist_index] + 1;
       }

    }

    void stop_cuda_stuff(int *x,  int *hist){
      cudaFree(x);
      cudaFree(hist);
    }

    double do_cuda_stuff(int o_x[], int n){
      if (sizeof((o_x)) / sizeof((o_x)[0]) < 1){
        return -1;
      }
      int bin_number = 1000;
      // initialize x array on the host
      int * new_hist;
      int * result_hist;
      int * x;

      float max_value = 65535;
      float delta = max_value/bin_number;

      //x = o_x;
      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&x, n*sizeof(int));
      cudaMallocManaged(&new_hist, bin_number*sizeof(int));

      cudaMemcpy(x, o_x, n*sizeof(int), cudaMemcpyHostToDevice);
      /*
      printf(" \n");
      for (int h=0; h < bin_number; h++){
        x[h] = o_x[h];
        //printf("%d: %d ", h, x[h]);
      }*/

      int threads = 32;
      auto numBlocks = n/threads;
      // First param blocks
      // Second param number of threads
      //  blocks, threads each
      //int blockSize = 256;
      //numBlocks = (n + blockSize - 1) / blockSize;
      do_cuda_stuff_kernel<<<numBlocks,threads>>>(n,x, new_hist);
      cudaDeviceSynchronize(); // to print results

      result_hist = reinterpret_cast<int*>(malloc(bin_number*sizeof(int)));
      cudaMemcpy(result_hist, new_hist, bin_number*sizeof(int),  cudaMemcpyDeviceToHost);

      //printf(" \n");
      //for (int h=0; h < bin_number; h++){
      //  printf("%d: %d ", h, new_hist[h]);
      //}


      new_hist[0] = 0;

      int max_index = -1;

      for (int w = 0; w < bin_number; w++){
        if (new_hist[w] > max_index){
          max_index = w;
        }
      }
      printf("Max Index %d", max_index);
      stop_cuda_stuff(x,new_hist);

      return double(max_index * delta) *0.001;
    }
}
