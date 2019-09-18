#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cassert>
#include <string>

using namespace pcl;

namespace pcl_gpu{

  class PCLFilterPassThrough
  {
    public:
      //using PointType = pcl::PointXYZ;
      //using PointCloudHost = pcl::PointCloud<PointType>;
      //using PointCloudHostPtr = PointCloudHost::Ptr;
      //using PointCloudHostConstPtr = PointCloudHost::ConstPtr;

      PCLFilterPassThrough () : minimum_value_ (-1.0), maximum_value_(1.0), filter_value_ (std::numeric_limits<float>::max ())
      {
        printf("PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP");
      };

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

      inline void setHostCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr host_cloud) {
        host_cloud_ = host_cloud;
      }

      double do_stuff(pcl::PointCloud<pcl::PointXYZ>  &input_cloud);

    protected:
      pcl::PointCloud<pcl::PointXYZ>::Ptr host_cloud_;

      /** \brief The spatial cluster tolerance as a measure in the L2 Euclidean space. */
      double cluster_tolerance_;

      float minimum_value_;
      float maximum_value_;
      float filter_value_;

      virtual std::string getClassName () const { return ("gpu::PCLPassThroughFilter"); }
  };
}
