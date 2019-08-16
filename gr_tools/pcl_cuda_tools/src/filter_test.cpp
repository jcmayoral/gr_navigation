#include <pcl_cuda_tools/filters/filter_passthrough.h>

int main (int argc, char **argv){
  pcl::gpu::FilterPassThrough passthrough_filter_;
  boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> input;
  passthrough_filter_.setHostCloud(input);
  passthrough_filter_.do_stuff();
  std::cout << "DOING NOTHING...NOT FINISHED"<< std::endl;
  return 0;
}
