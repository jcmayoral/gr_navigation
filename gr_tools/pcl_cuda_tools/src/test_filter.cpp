#include <pcl_cuda_tools/filters/filter_passthrough.h>

int main (int argc, char **argv){
  pcl_gpu::FilterPassThrough passthrough_filter_;
  //boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl_gpu::FilterPassThrough::PointType>::Ptr input(new pcl::PointCloud<pcl_gpu::FilterPassThrough::PointType>);

  for (int i=0; i< 1024*1024; i++){
    pcl_gpu::FilterPassThrough::PointType p;
    p.x = 0.1;
    p.y = 0.1;
    p.z = 10.1;
    input->points.push_back(p);
  }
  passthrough_filter_.setHostCloud(input);
  std::cout <<  "RESULT: " << passthrough_filter_.do_stuff("z", *input);

  for (int i=0; i< 10; i++){
    std::cout << "this should be a ridiculous big value " << input->points[i].z << std::endl;
  }

  std::cout << "DOING NOTHING...NOT FINISHED"<< std::endl;
  return 0;
}
