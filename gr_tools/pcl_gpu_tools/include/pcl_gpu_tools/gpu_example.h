#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>


// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>


#include <functional>
#include <iostream>
#include <mutex>

//#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

//ROS stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/filters/passthrough.h>
#include <pcl_gpu_tools/GPUFilterConfig.h>
#include <dynamic_reconfigure/server.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl_gpu_tools/math_functions.hpp>


#include <pcl_cuda_tools/filters/filter_passthrough.h>

//using namespace pcl::cuda;
//using pcl::cuda::PointCloudAOS;
//using pcl::cuda::Device;


class GPUExample
{
private:
  //pcl::cuda::DisparityToCloud d2c;
  //pcl::visualization::CloudViewer viewer;
  boost::mutex mutex_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
  ros::Publisher cluster_pub_;
  ros::Publisher bb_pub_;
  pcl::gpu::Octree::PointCloud cloud_device;
  pcl::gpu::EuclideanClusterExtraction gec;
  pcl::ExtractIndices<pcl::PointXYZ> extraction_filter_;
  pcl::SACSegmentation<pcl::PointXYZ> segmentation_filter_;
  //pcl::ConditionAnd<pcl::PointXYZ>::Ptr conditional_filter_;
  pcl::PassThrough<pcl::PointXYZ> pass_through_filter_;
  pcl::ConditionalRemoval<pcl::PointXYZ> condition_removal_;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outliers_filter_;
  pcl::PointCloud<pcl::PointXYZ> main_cloud_;
  ros::Timer timer_;


  //Testing
  pcl_gpu::FilterPassThrough cuda_pass_;

  //Dynamic Reconfigure
  dynamic_reconfigure::Server<pcl_gpu_tools::GPUFilterConfig> dyn_server_;
  dynamic_reconfigure::Server<pcl_gpu_tools::GPUFilterConfig>::CallbackType dyn_server_cb_;
  double dynamic_std_;
  double dynamic_std_z_;
  bool output_publish_;
  bool remove_ground_;
  bool passthrough_enable_;
  bool is_processing_;
  bool is_timer_enable_;
  jsk_recognition_msgs::BoundingBoxArray bb;

public:
    GPUExample ();
    ~GPUExample(){};
    void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg);
    int run_filter(const boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> cloud_filtered);
    template <class T> void publishPointCloud(T);
    void timer_cb(const ros::TimerEvent&);
    void cluster();
    void publishBoundingBoxes(const geometry_msgs::PoseArray& cluster_array);
    void addBoundingBox(const geometry_msgs::Pose center, double v_x, double v_y, double v_z);
    void dyn_reconfigureCB(pcl_gpu_tools::GPUFilterConfig &config, uint32_t level);
    void removeGround(boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> pc);
};
