#include <pcl_cuda_tools/filters/filter_passthrough.h>

int main (int argc, char **argv){
  FilterPassThrough passthrough_filter_;
  boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> input(new pcl::PointCloud<pcl::PointXYZ>);


  for (auto i=0; i< 1024*1024; i++){
    pcl::PointXYZ p;
    input->points.emplace_back(p);
  }
  passthrough_filter_.setHostCloud(input);
  passthrough_filter_.do_stuff();
  std::cout << "DOING NOTHING...NOT FINISHED"<< std::endl;
  return 0;
}
