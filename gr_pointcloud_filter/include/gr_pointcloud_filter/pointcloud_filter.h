//ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
//dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <gr_pointcloud_filter/FiltersConfig.h>

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

        	//Dynamic Reconfigure
        	dynamic_reconfigure::Server<gr_pointcloud_filter::FiltersConfig> dyn_server_;
        	dynamic_reconfigure::Server<gr_pointcloud_filter::FiltersConfig>::CallbackType dyn_server_cb_;

    	public:
            virtual void onInit();
            void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg);
            void applyFilters(const sensor_msgs::PointCloud2 msg);
            void setFiltersParams(gr_pointcloud_filter::FiltersConfig &config);
            void dyn_reconfigureCB(gr_pointcloud_filter::FiltersConfig &config, uint32_t level);

    };

}
