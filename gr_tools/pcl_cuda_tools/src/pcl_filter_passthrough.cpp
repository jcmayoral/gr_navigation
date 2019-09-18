#include <pcl_cuda_tools/filters/pcl_filter_passthrough.h>
#include <pcl_cuda_tools/pcl_cuda_filter_functions.cuh>

using namespace pcl_gpu;

double PCLFilterPassThrough::do_stuff (pcl::PointCloud<pcl::PointXYZ>  &input_cloud){

  int number_points = input_cloud.points.size();
  bool b[number_points];

  std::cout << "IN DOSTUFF" << number_points << std::endl;

  for(int i=0; i<number_points; i++)
  {
    b[i] = true;
  }
  //FAILING NOT SO SURE WHY
  //std::memset(b, true, sizeof(bool) * number_points);
  bool result = false;
  std::cout << "IN DOSTUFF" << std::endl;

  result = pcl_apply_cuda_filter(input_cloud, b, minimum_value_, maximum_value_, filter_value_,  number_points);
  return result;
}
