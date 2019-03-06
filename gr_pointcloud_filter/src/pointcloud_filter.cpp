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

   	  if (filter_inliers->indices.size () == 0){
        return;
		  }

    	//extracting inliers (removing ground)
   	  extraction_filter_.setInputCloud (cloud);
      extraction_filter_.setIndices (filter_inliers);
      extraction_filter_.filter(*cloud);
      //outliers removal filter
      outliers_filter_.setInputCloud (cloud);
      outliers_filter_.filter (*cloud);

    	// Convert to ROS data type
     	pcl::toROSMsg(*cloud, output_pointcloud_);
    	// Publish the data
    	pointcloud_pub_.publish (output_pointcloud_);
    }

    void MyNodeletClass::setFiltersParams(gr_pointcloud_filter::FiltersConfig &config){
      boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    	//voxeling
    	voxel_filter_.setLeafSize (config.leaf_size, config.leaf_size, config.leaf_size);
    	//segmentating
    	segmentation_filter_.setModelType(pcl::SACMODEL_PLANE);
    	segmentation_filter_.setMethodType(pcl::SAC_RANSAC);
    	segmentation_filter_.setMaxIterations (config.max_iterations);
    	segmentation_filter_.setDistanceThreshold (config.distance_threshold);
    	segmentation_filter_.setOptimizeCoefficients (config.optimize_coefficients);
    	//extraction
      extraction_filter_.setNegative (config.set_negative);
      //ouliers
      outliers_filter_.setMeanK (config.mean_k);
      outliers_filter_.setStddevMulThresh (config.std_threshold);
    }


    void MyNodeletClass::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    	ROS_DEBUG("Callback working");
    	applyFilters(*msg);
    }

    void MyNodeletClass::dyn_reconfigureCB(gr_pointcloud_filter::FiltersConfig &config, uint32_t level){
    	setFiltersParams(config);
    }

    void MyNodeletClass::onInit(){
      ROS_INFO("My NodeletClass constructor");
      ros::NodeHandle nh;
      dyn_server_cb_ = boost::bind(&MyNodeletClass::dyn_reconfigureCB, this, _1, _2);
      dyn_server_.setCallback(dyn_server_cb_);
      pointcloud_sub_ = nh.subscribe("/velodyne_points", 10, &MyNodeletClass::pointcloud_cb, this);
      pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filtered", 10);
      NODELET_DEBUG("Initializing nodelet...");
    }
}
