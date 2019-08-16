#include <cuda_runtime.h>
#include <stdio.h>

extern "C"
{
    __device__
    int getGlobalIdx_1D_1D(){
      return blockIdx.x *blockDim.x + threadIdx.x;
    }

    __global__
    void do_cuda_stuff_kernel(int *x, int* t){
      //__shared__ int s[256];
      //int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //int index = blockIdx.x * blockDim.x + threadIdx.x;
      //int stride = blockDim.x * gridDim.x;
      //printf("%d %d \n", gridDim.x, blockDim.x  );
      int index = getGlobalIdx_1D_1D();
      t[x[index]] += 1;
    }

    void stop_cuda_stuff(int *x, int *t){
      cudaFree(x);
      cudaFree(t);
    }

    int do_cuda_stuff(int* o_x, int size){
      // initialize x array on the host
      int * x, *t, *tr;

      // Allocate Unified Memory – accessible from CPU or GPU
      cudaMallocManaged(&x, size*sizeof(int));
      cudaMemcpy(x, o_x, size*sizeof(int), cudaMemcpyHostToDevice);

      int nthreads = 512;
      dim3 threads (nthreads);
      int nblocks =  ceil(size / nthreads);//size/ nthreads -1;
      //printf("blocks....%d %d\n", nblocks, nblocks*nthreads);
      cudaMallocManaged(&t, nthreads*sizeof(int));
      t = static_cast<int*>(malloc(sizeof(int) * nthreads));
      tr = static_cast<int*>(malloc(sizeof(int) * nthreads));

      memset(t, 0x00, nthreads);
      dim3 blocks(nblocks);
      // First param blocks
      // Second param number of threads
      //  blocks, threads each
      do_cuda_stuff_kernel<<<blocks,threads>>>(x,t);
      cudaDeviceSynchronize(); // to print results
      cudaMemcpy(tr, t, sizeof(x), cudaMemcpyDeviceToHost);
      stop_cuda_stuff(x, t);


      int max_value = -1;
      int max_index = -1;
      tr[0] = -2;

      for (int i =0; i< nthreads; i++){
        if (tr[i] > max_value){
          max_value = tr[i];
          max_index = i;
        }
      }
      printf("result....%d %d\n", max_value, max_index);

      return max_index;
    }
}
