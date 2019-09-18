#ifndef PCL_FILTER_TOOLS_CUH
#define PCL_FILTER_TOOLS_CUH
#include <pcl/point_cloud.h>
#include<pcl/point_types.h>


extern "C"
{
  int pcl_apply_cuda_filter(pcl::PointCloud<pcl::PointXYZ> &point_cloud, bool* o_b,float min_limit, float max_limit, float filter_value, int size);
}

#endif
