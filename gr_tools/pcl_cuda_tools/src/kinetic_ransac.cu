#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
//#include <pcl/cuda/io/cloud_to_pcl.h>
//#include <pcl/cuda/io/extract_indices.h>
//#include <pcl/cuda/io/disparity_to_cloud.h>
///#include <pcl/cuda/io/host_device.h>
//#include <pcl/cuda/sample_consensus/sac_model_plane.h>
#include <pcl/cuda/sample_consensus/sac_model_1point_plane.h>
#include <pcl/cuda/sample_consensus/ransac.h>
#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/point_cloud.h>


#include <boost/shared_ptr.hpp>

#include <functional>
#include <iostream>
#include <mutex>

//ROS stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace pcl::cuda;
using pcl::cuda::PointCloudAOS;
using pcl::cuda::Device;


template <template <typename> class Storage>

class SimpleKinectTool
{
private:
  //pcl::cuda::DisparityToCloud d2c;
  //pcl::visualization::CloudViewer viewer;
  std::mutex mutex_;
  bool go_on;
  ros::Subscriber pc_sub_;

public:
    SimpleKinectTool ()  {
        ros::NodeHandle nh;
        pc_sub_ = nh.subscribe("/camera/depth_registered/points", 10, &SimpleKinectTool::pointcloud_cb, this);
    }

    void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    //run_filter(*msg);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *output);
    ROS_INFO("PointCloud conversion succeded");

    //std::function<void (const pcl::PointCloud<pcl::PointXYZRGB> cloud)>  f;
    //f = [this](const pcl::PointCloud<pcl::PointXYZRGB> cloud){ run_filter<pcl::cuda::Device>(cloud);};
    //f = [this](const pcl::PointCloud<pcl::PointXYZRGB> cloud){ run_filter(cloud);};

    //std::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = std::bind (&SimpleKinectTool::file_cloud_cb<pcl_cuda::Host>, this, _1);

                                                                           //f = std::bind (&SimpleKinectTool::run_filter<pcl::cuda::Device>, this, _1);
    //f = [this](const pcl::PointCloud<pcl::PointXYZRGB::ConstPtr&> cloud){ run_filter<pcl::PointXYZRGB::ConstPtr&>(cloud)};
    //run_filter(*output);
    //run_filter(*output);
    }

    //template <template <typename> class Storage> void
    int run_filter(const boost::shared_ptr <pcl::PointCloud<pcl::PointXYZRGB>> cloud){
          ROS_INFO("WORKING");


    pcl::ScopeTime ttt ("all");
    pcl::cuda::PointCloudAOS<pcl::cuda::Host> data_host;

    data_host.points.resize (cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        PointXYZRGB pt;
        pt.x = cloud->points[i].x;
        pt.y = cloud->points[i].y;
        pt.z = cloud->points[i].z;
        // Pack RGB into a float
        pt.rgb = *(float*)(&cloud->points[i].rgb);
        data_host.points[i] = pt;
    }
    data_host.width = cloud->width;
    data_host.height = cloud->height;
    data_host.is_dense = cloud->is_dense;

    typename pcl::cuda::PointCloudAOS<pcl::cuda::Device>::Ptr data;// = pcl::cuda::toStorage<pcl::cuda::Host, pcl::cuda::Device> (data_host);


    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::cuda::toPCL (data_host, *output);

    //typename SampleConsensusModelPlane<Storage>::Ptr sac_model (new SampleConsensusModelPlane<Storage> (data_host));
    //RandomSampleConsensus<Storage> sac (sac_model);
    //sac.setMaxIterations (10000);
    //sac.setDistanceThreshold (0.05);
    /*

    {
        pcl::ScopeTime timer ("computeModel: ");
        if (!sac.computeModel (0))
        {
        std::cerr << "Failed to compute model" << std::endl;
        }
        else
        {
        typename SampleConsensusModel1PointPlane<Storage>::IndicesPtr inliers_stencil;
        inliers_stencil = sac.getInliersStencil ();

    //    OpenNIRGB color;
    //    color.r = 253; color.g = 0; color.b = 0;
    //    std::cerr << data->points.size() << " =?= " << inliers_stencil->size () << std::endl;
    //    colorIndices<Storage> (data, inliers_stencil, color);
        }
    }
    */


      typename SampleConsensusModel1PointPlane<pcl::cuda::Device>::Ptr sac_model (new SampleConsensusModel1PointPlane<pcl::cuda::Device> (data));
      RandomSampleConsensus<pcl::cuda::Device> sac (sac_model);
      sac.setMaxIterations (10000);
      sac.setDistanceThreshold (0.05);

      {
        pcl::ScopeTime timer ("computeModel: ");
        if (!sac.computeModel (0))
        {
          std::cerr << "Failed to compute model" << std::endl;
        }
        else
        {
          typename SampleConsensusModel1PointPlane<pcl::cuda::Device>::IndicesPtr inliers_stencil;
          inliers_stencil = sac.getInliersStencil ();

      //    OpenNIRGB color;
      //    color.r = 253; color.g = 0; color.b = 0;
      //    std::cerr << data->points.size() << " =?= " << inliers_stencil->size () << std::endl;
      //    colorIndices<Storage> (data, inliers_stencil, color);
        }
      }

    go_on = false;
    //std::cerr << "got here" << std::endl;
    //pcl_cuda::toPCL (*data, *output);
    //std::cerr << "not here" << std::endl;
    //viewer.showCloud (output);
    return 1;
    }

};
