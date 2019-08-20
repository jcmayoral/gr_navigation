#include <pcl_cuda_tools/filters/filter_passthrough.h>

int main (int argc, char **argv){
  pcl_gpu::FilterPassThrough passthrough_filter_;
  boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> input(new pcl::PointCloud<pcl::PointXYZ>);


  for (auto i=0; i< 1024*1024; i++){
    pcl::PointXYZ p;
    p.x = 10;
    p.y = 10;
    p.z = 10;
    input->points.emplace_back(p);
  }
  passthrough_filter_.setHostCloud(input);
  std::cout <<  "RESULT: " << passthrough_filter_.do_stuff(*input);
  std::cout << "DOING NOTHING...NOT FINISHED"<< std::endl;
  return 0;
}
