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
    void filter_passthrough_kernel(float *z, bool *b, float min_limit, float max_limit, float filter_value, int size){
      //__shared__ int s[256];
      //int idx = blockIdx.x * blockDim.x + threadIdx.x;
      //int index = blockIdx.x * blockDim.x + threadIdx.x;
      //int stride = blockDim.x * gridDim.x;
      int index = getGlobalIdx_1D_1D();
      if (index >= size){
        //printf("%d\n", index );
        return;
      }


      /*
      if (max_limit < x[index] || x[index] < min_limit){
        //x[index] = filter_value;
        //y[index] = filter_value;
        b[index] = true;
      }
      if (max_limit < y[index] || y[index] < min_limit){
        //x[index] = filter_value;
        //y[index] = filter_value;
        b[index] = true;
      }
      */
      if (max_limit < z[index] || z[index] < min_limit){
        //x[index] = filter_value;
        //y[index] = filter_value;
        b[index] = true;
      }

      /*
      if (isnan(z[index])){
        b[index] = true;
      }

      if (isnan(x[index])){
        b[index] = true;
      }

      if (isnan(y[index])){
        b[index] = true;
      }
      */
    }

    int apply_cuda_filter(float *o_z, bool *o_b, float min_limit, float max_limit, float filter_value, int size){
      // initialize x array on the host
      float *z;
      //float *y;
      //float *x;
      bool *b;
      // Allocate Unified Memory – accessible from CPU or GPU
      //cudaMallocManaged(&x, size*sizeof(float));
      //cudaMallocManaged(&y, size*sizeof(float));
      cudaMallocManaged(&z, size*sizeof(float));
      //cudaMallocManaged(&y, size*sizeof(float));
      //cudaMallocManaged(&x, size*sizeof(float));
      cudaMallocManaged(&b, size*sizeof(bool));

      printf("min limit %f", min_limit);
      printf("max limit %f", max_limit);
      //cudaMemcpy(x, o_x, size*sizeof(float), cudaMemcpyHostToDevice);
      //cudaMemcpy(y, o_y, size*sizeof(float), cudaMemcpyHostToDevice);
      cudaMemcpy(z, o_z, size*sizeof(float), cudaMemcpyHostToDevice);
      cudaMemcpy(b, o_b, size*sizeof(bool), cudaMemcpyHostToDevice);


      int ngrid;      // The launch configurator returned block size
      int nblocks;    // The minimum grid size needed to achieve the maximum occupancy for a full device launch
      //cudaOccupancyMaxPotentialBlockSize(&nblocks, &nthreads, filter_passthrough_kernel, 0, 0);
      ngrid = 1024;
      dim3 grid (ngrid);
      nblocks = ceil((size+ngrid -1)/ngrid);

      //nblocks = (size + nblocks -1)/nblocks;
      printf("\n A %d %d %d\n", size, nblocks, ngrid);

      //int nthreads = 1024;
      //int nblocks = ceil(size/nthreads);//7ceil(size / nthreads);//size/ nthreads -1;
      //memset(t, 0x00, nthreads);
      dim3 blocks(nblocks);
      // First param blocks
      // Second param number of threads
      //  blocks, threads each
      printf("C %d %d\n", nblocks, ngrid);
      printf("Deeee %d\n", size);

      //printf("OK %d", max_thread - size);
      //filter_passthrough_kernel<<<blocks,threads>>>(z,b,min_limit, max_limit, filter_value, size);
      filter_passthrough_kernel<<<blocks,grid>>>(z,b,min_limit, max_limit, filter_value, size);
      cudaDeviceSynchronize(); // to print results
      //cudaMemcpy(o_x, x, size*sizeof(float), cudaMemcpyDeviceToHost);
      //cudaMemcpy(o_y, y, size*sizeof(float), cudaMemcpyDeviceToHost);
      //cudaMemcpy(o_z, z, size*sizeof(float), cudaMemcpyDeviceToHost);
      cudaMemcpy(o_b, b, size*sizeof(bool), cudaMemcpyDeviceToHost);

      //free_memory(x);
      //free_memory(y);
      cudaFree(z);
      cudaFree(b);
      printf("DONE!!!!");


      return 1;
    }
}
