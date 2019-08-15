#include <iostream>
#include <math.h>
#include <pcl_cuda_tools/depth_registration.h>
#include <pcl_cuda_tools/cuda_functions.cuh>

DepthRegistration::DepthRegistration(cv::Mat image){
  std::cout << "Constructor cuda object"<<std::endl;
  N  = image.cols * image.rows;

  // Allocate Unified Memory – accessible from CPU or GPU
  x = (int*)malloc(getN()*sizeof(int));

  int bin_number = 1000;
  float max_value = 65535;

  // initialize x array on the host
  cv::MatIterator_<uchar> it;
  it = image.begin<uchar>();

  for (int i = 0; i < N; i++) {
    //uchar to int
    x[i] = static_cast<int>(*it);
    it = it+1;
  }

  hist = (int*)malloc(bin_number*sizeof(int));

  for (int h=0; h < bin_number; h++){
    hist[h] = 0;
  }

  delta = max_value/bin_number;
  std::cout << "End Constructor cuda object"<<std::endl;

};

double DepthRegistration::run(){
  // Run kernel on 1M elements on the GPU
  // First param blocks
  // Second param number of threads
  do_cuda_stuff(getN(),x, hist, delta);

  //ignoring 0
  hist[0] = 0;
  int auxiliar = sizeof(hist) / sizeof(hist[0]);
  int median = std::distance(hist, std::max_element(hist, hist+ auxiliar));

  return double(median * delta) *0.001;

}

DepthRegistration::~DepthRegistration(){
  delete[] x;
  delete[] hist;
}
