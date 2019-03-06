// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include <gr_pointcloud_filter/pointcloud_filter.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(gr_pointcloud_filter::MyNodeletClass, nodelet::Nodelet)

namespace gr_pointcloud_filter
{

    void MyNodeletClass::applyFilters(const sensor_msgs::PointCloud2 msg){
    	//Convering sensor_msg to pcl message
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::fromROSMsg (msg, *cloud);


    	//voxeling
    	voxel_filter_.setInputCloud (cloud);
    	voxel_filter_.filter (*cloud);

    	//segmentation of a plane
    	pcl::ModelCoefficients::Ptr filter_coefficients(new pcl::ModelCoefficients);
    	pcl::PointIndices::Ptr filter_inliers(new pcl::PointIndices);

    	segmentation_filter_.setInputCloud(cloud);
   	    segmentation_filter_.segment (*filter_inliers, *filter_coefficients);

   	    if (filter_inliers->indices.size () == 0)
   	    {
   	    	return;
		}

    	//extracting inliers (removing ground)
   	    extraction_filter_.setInputCloud (cloud);
		extraction_filter_.setIndices (filter_inliers);
		extraction_filter_.filter(*cloud);

    	// Convert to ROS data type
     	pcl::toROSMsg(*cloud, output_pointcloud_);
    	// Publish the data
    	pointcloud_pub_.publish (output_pointcloud_);
    }

    void MyNodeletClass::setFiltersParams(){
    	//voxeling
    	voxel_filter_.setLeafSize (1, 1, 1);
    	//segmentating
    	segmentation_filter_.setModelType(pcl::SACMODEL_PLANE);
    	segmentation_filter_.setMethodType(pcl::SAC_RANSAC);
    	segmentation_filter_.setMaxIterations (1000);
    	segmentation_filter_.setDistanceThreshold (0.01);
    	segmentation_filter_.setOptimizeCoefficients (true);
    	//extraction
		extraction_filter_.setNegative (false);

    }


    void MyNodeletClass::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    	ROS_DEBUG("Callback working");
    	applyFilters(*msg);
    }

    void MyNodeletClass::onInit()
    {

    	ROS_INFO("My NodeletClass constructor");
		ros::NodeHandle nh;

    	setFiltersParams();

		pointcloud_sub_ = nh.subscribe("/velodyne_points", 10, &MyNodeletClass::pointcloud_cb, this);
		pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filtered", 10);
		NODELET_DEBUG("Initializing nodelet...");
		NODELET_INFO("Here");
		//ros::spin();
    }
}
