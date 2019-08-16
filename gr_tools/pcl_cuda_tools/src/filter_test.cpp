#include <pcl_cuda_tools/filters/passthrough.h>

int main (int argc, char **argv){
  pcl::cuda::PassThrough<pcl::PointXYZRGB> passthrough_filter_();
  return 0;
}
