#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

namespace gr_pointcloud_filter
{

    class MyNodeletClass : public nodelet::Nodelet
    {
    	private:
    		ros::Subscriber pointcloud_sub_;
    		ros::Publisher pointcloud_pub_;
        	sensor_msgs::PointCloud2 output_pointcloud_;
        	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
        	pcl::SACSegmentation<pcl::PointXYZ> segmentation_filter_;
        	pcl::ExtractIndices<pcl::PointXYZ> extraction_filter_;

    	public:
            virtual void onInit();
            void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg);
            void applyFilters(const sensor_msgs::PointCloud2 msg);
            void setFiltersParams();
    };

}
