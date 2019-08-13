#include <iostream>
#include <math.h>
#include <pcl_cuda_tools/depth_registration.h>
#include <pcl_cuda_tools/cuda_functions.cuh>

DepthRegistration::DepthRegistration(){
  N = 100;

  // Allocate Unified Memory – accessible from CPU or GPU
  x = (float*)malloc(getN()*sizeof(float));
  y = (float*)malloc(getN()*sizeof(float));

  // initialize x and y arrays on the host
  for (int i = 0; i < getN(); i++) {
    x[i] = 1.0f;
    y[i] = 2.0f;
  }
};

void DepthRegistration::run(){
  // Run kernel on 1M elements on the GPU
  // First param blocks
  // Second param number of threads
  do_cuda_stuff(getN(),x,y);
}

DepthRegistration::~DepthRegistration(){
  // Free memory
}
