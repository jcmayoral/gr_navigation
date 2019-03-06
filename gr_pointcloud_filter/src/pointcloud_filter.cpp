// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include <gr_pointcloud_filter/pointcloud_filter.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(gr_pointcloud_filter::MyNodeletClass, nodelet::Nodelet)

namespace gr_pointcloud_filter
{
	/*
	MyNodeletClass::MyNodeletClass(){
		ROS_INFO("My NodeletClass constructor");
		ros::NodeHandle nh;
		nh.subscribe("velodyne_points", 10, &MyNodeletClass::pointcloud_cb, this);
		ROS_INFO("Nodelet has been initialized");
	}
	*/

    void MyNodeletClass::applyFilters(const sensor_msgs::PointCloud2 msg){
    	ROS_INFO("applying filters");

    	// Convert the sensor_msgs/PointCloud2(cloud2) object to pcl/pointcloud(cloud1) object with fromROSMsg()
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::fromROSMsg (msg, *cloud);



    	// Downsample not used (Velodyne is already sparsed
    	/*
    	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    	sor.setInputCloud (cloudPtr);
    	sor.setLeafSize (0.1, 0.1, 0.1);
    	sor.filter (cloud_filtered);
		*/

    	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    	// Create the segmentation object

    	pcl::SACSegmentation<pcl::PointXYZ> seg;
    	// Optional
    	seg.setOptimizeCoefficients (true);
    	// Mandatory
    	seg.setModelType (pcl::SACMODEL_PLANE);
    	seg.setMethodType (pcl::SAC_RANSAC);
    	seg.setMaxIterations (1000);
    	seg.setDistanceThreshold (0.01);
    	seg.setInputCloud(cloud);
   	    seg.segment (*inliers, *coefficients);

   	    if (inliers->indices.size () == 0)
   	    {
   	    	return;
		}

    	// Create the filtering object
    	pcl::ExtractIndices<pcl::PointXYZ> extract;
		// Extract the inliers
   	    extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (true);
		extract.filter(*cloud);



    	// Convert to ROS data type
    	sensor_msgs::PointCloud2 output;
    	//pcl_conversions::fromPCL(cloud, output);


    	//Convert pcl/pointcloud object(cloud1) to sensor_msgs/PointCloud2(cloud2) with toROSMsg()
    	pcl::toROSMsg(*cloud, output);
    	// Publish the data
    	pointcloud_pub_.publish (output);
    }


    void MyNodeletClass::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    	ROS_DEBUG("Callback working");
    	applyFilters(*msg);
    }

    void MyNodeletClass::onInit()
    {

    	ROS_INFO("My NodeletClass constructor");
		ros::NodeHandle nh;
		pointcloud_sub_ = nh.subscribe("/velodyne_points", 10, &MyNodeletClass::pointcloud_cb, this);
		pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filtered", 10);
		NODELET_DEBUG("Initializing nodelet...");
		NODELET_INFO("Here");
		//ros::spin();
    }
}
