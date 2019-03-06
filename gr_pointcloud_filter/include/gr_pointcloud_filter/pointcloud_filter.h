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
    	public:
            virtual void onInit();
            void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg);
            void applyFilters(const sensor_msgs::PointCloud2 msg);

    };

}
