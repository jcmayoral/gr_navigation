#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <gr_pointcloud_filter/helloWorld.h>

#include <stdio.h>


namespace gr_pointcloud_filter{

__global__ void  my_test() {

}

void test(){
  printf("Hello!\n");

	my_test<<<1,1>>>();
}

}