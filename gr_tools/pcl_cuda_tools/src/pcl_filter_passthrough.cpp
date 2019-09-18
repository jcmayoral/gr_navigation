#include <pcl_cuda_tools/filters/pcl_filter_passthrough.h>
#include <pcl_cuda_tools/pcl_cuda_filter_functions.cuh>

using namespace pcl_gpu;

double PCLFilterPassThrough::do_stuff (pcl::PointCloud<pcl::PointXYZ>  &input_cloud){
  bool *b;
  int number_points = input_cloud.points.size();

  memset(b, false, number_points);
  bool result = false;

  result = pcl_apply_cuda_filter(input_cloud, b, minimum_value_, maximum_value_, filter_value_,  number_points);
  return result;
}
