#include <pcl_cuda_tools/filters/filter_passthrough.h>
#include <pcl_cuda_tools/cuda_filter_functions.cuh>

//using namespace pcl::gpu;
//using namespace pcl;
using namespace pcl_gpu;

double FilterPassThrough::do_stuff (std::string channel, pcl::PointCloud<pcl::PointXYZ>  &input_cloud){
  float * x, *y, *z;
  bool *b;
  int number_points = input_cloud.points.size();
  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_pointer;
  //pc_pointer = input_cloud.makeShared();//boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(input_cloud);

  x = static_cast<float*>(malloc(sizeof(float) * number_points));
  y = static_cast<float*>(malloc(sizeof(float) * number_points));
  z = static_cast<float*>(malloc(sizeof(float) * number_points));
  b = static_cast<bool*>(malloc(sizeof(bool) * number_points));

  int removed_points = 0;

  for (int i=0; i< number_points; i++){
    x[i] = static_cast<float>(input_cloud.points[i].x);
    y[i] = static_cast<float>(input_cloud.points[i].y);
    z[i] = static_cast<float>(input_cloud.points[i].z);
  }
  memset(b, false, number_points);
  bool result = false;

  if (channel.find("z")!= -1){
    result = apply_cuda_filter(z,b, minimum_value_, maximum_value_, filter_value_,  number_points);
  }

  if (channel.find("x")!= -1){
    result = apply_cuda_filter(x,b, minimum_value_/3, maximum_value_/3, filter_value_,  number_points);
  }

  if (channel.find("y")!= -1){
    //TODO REVIEW
    result = apply_cuda_filter(y,b, minimum_value_, maximum_value_, filter_value_,  number_points);
  }

  input_cloud.points.clear();

  //TODO I SHOULD FIND A FASTER WAY
  for (int j=0; j< number_points; j++){
    if (b[j]){
      removed_points++;
      continue;
    }

    pcl::PointXYZ point;
    point.x = static_cast<float>(x[j]);
    point.y = static_cast<float>(y[j]);
    point.z = static_cast<float>(z[j]);

    input_cloud.points.push_back(point);
  }


  input_cloud.is_dense = false;

  //std::cout << "REMOVING "<< removed_points << std::endl;
  input_cloud.width = input_cloud.points.size();
  //input_cloud = *pc_pointer;

  return result;
}
