#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cassert>
#include <string>

using namespace pcl;

namespace pcl_gpu{

  class FilterPassThrough
  {
    public:
      typedef pcl::PointXYZI PointType;
      //using PointCloudHost = pcl::PointCloud<PointType>;
      //using PointCloudHostPtr = PointCloudHost::Ptr;
      //using PointCloudHostConstPtr = PointCloudHost::ConstPtr;

      FilterPassThrough () : minimum_value_ (-1.0), maximum_value_(1.0),
                             filter_value_ (std::numeric_limits<float>::max ()),
                             xy_scaler_(1), scale_axis_('w')
      {
        //printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAA");
      };

      inline void setScaleAxis(char axis) {
        scale_axis_ = axis;
      }

      inline char getScaleAxis() {
        return scale_axis_;
      }

      inline int getXYScaler() {
        return xy_scaler_;
      }

      inline void setXYScaler(int scale) {
        xy_scaler_ = scale;
      }

      inline void setMinimumValue(float min_limit) {
        minimum_value_ = min_limit;
      }

      inline float getMimumumValue() {
        return minimum_value_;
      }

      inline void setMaximumValue(float max_limit) {
        maximum_value_ = max_limit;
      }

      inline float getMaxClusterSize() {
        return (maximum_value_);
      }

      inline void setHostCloud ( pcl::PointCloud<PointType>::Ptr host_cloud) {
        host_cloud_ = host_cloud;
      }

      double do_stuff(std::string channel, pcl::PointCloud<PointType>  &input_cloud);

    protected:
       pcl::PointCloud<PointType>::Ptr host_cloud_;

      /** \brief The spatial cluster tolerance as a measure in the L2 Euclidean space. */
      double cluster_tolerance_;

      float minimum_value_;
      float maximum_value_;
      float filter_value_;
      int xy_scaler_;
      char scale_axis_;

      virtual std::string getClassName () const { return ("gpu::PassThroughFilter"); }
  };
}
