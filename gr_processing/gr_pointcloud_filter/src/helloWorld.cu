#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <gr_pointcloud_filter/helloWorld.h>

#include <stdio.h>

__global__ void  my_test(gr_pointcloud_filter::CustomCUDAManager manager, sensor_msgs::PointCloud2 pc) {
	printf("%s", pc.header.frame_id);
}

namespace gr_pointcloud_filter{
	CustomCUDAManager::CustomCUDAManager(): id_("I cannot believe it"){

	}

	CustomCUDAManager::~CustomCUDAManager(){

	}

	void CustomCUDAManager::reconfigure(gr_pointcloud_filter::FiltersConfig &config){
		std::cout << "Reconfiguring" << std::endl;

	}
	
	void CustomCUDAManager::test(sensor_msgs::PointCloud2 pc){
			printf("Hello!\n");
			my_test<<<1,1>>>(*this, pc);
			std::cout <<getID());

	}
}