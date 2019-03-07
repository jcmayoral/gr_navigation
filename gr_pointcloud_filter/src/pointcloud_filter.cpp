// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>


//testing
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

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
    	//voxel_filter_.setInputCloud (cloud);
    	//voxel_filter_.filter (*cloud);

    	//segmentation of a plane
    	//pcl::ModelCoefficients::Ptr filter_coefficients(new pcl::ModelCoefficients);
    	//pcl::PointIndices::Ptr filter_inliers(new pcl::PointIndices);


    	//On testing
    	pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    	normal_estimator.setSearchMethod (tree);
    	normal_estimator.setInputCloud (cloud);
    	normal_estimator.setKSearch (50);
    	normal_estimator.compute (*normals);

    	pcl::IndicesPtr indices (new std::vector <int>);
    	pcl::PassThrough<pcl::PointXYZ> pass;
    	pass.setInputCloud (cloud);
    	pass.setFilterFieldName ("z");
    	pass.setFilterLimits (0.0, 1.0);
    	pass.filter (*indices);

    	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    	reg.setMinClusterSize (50);
    	reg.setMaxClusterSize (1000000);
    	reg.setSearchMethod (tree);
    	reg.setNumberOfNeighbours (100);
    	reg.setInputCloud (cloud);
    	//reg.setIndices (indices);
    	reg.setInputNormals (normals);
    	reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    	reg.setCurvatureThreshold (1.0);


    	std::vector <pcl::PointIndices> clusters;
    	reg.extract (clusters);



		/*
    	Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
    	segmentation_filter_.setAxis(axis);
    	segmentation_filter_.setEpsAngle(eps_angle_ * (M_PI/180.0f) ); // plane can be within 30 degrees of X-Z plane

    	segmentation_filter_.setInputCloud(cloud);
    	segmentation_filter_.segment (*filter_inliers, *filter_coefficients);

    	if (filter_inliers->indices.size () == 0){
    		return;
      }
    	ROS_INFO_STREAM_THROTTLE(1,"Plane height" << std::to_string(filter_coefficients->values[3]/filter_coefficients->values[2]));
    	//extracting inliers (removing ground)
    	extraction_filter_.setInputCloud (cloud);
    	extraction_filter_.setIndices (filter_inliers);
    	extraction_filter_.filter(*cloud);
*/
    	//outliers removal filter
    	//outliers_filter_.setInputCloud (cloud);
    	//outliers_filter_.filter (*cloud);

    	// Convert to ROS data type
    	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    	pcl::toROSMsg(*colored_cloud, output_pointcloud_);
    	output_pointcloud_.header.frame_id = "laser_frame";
    	output_pointcloud_.header.stamp = ros::Time::now();
    	// Publish the data
    	pointcloud_pub_.publish (output_pointcloud_);
    }

    void MyNodeletClass::setFiltersParams(gr_pointcloud_filter::FiltersConfig &config){
    	boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    	//voxeling
    	voxel_filter_.setLeafSize (config.leaf_size, config.leaf_size, config.leaf_size);

    	//segmentating
    	//segmentation_filter_.setModelType(pcl::SACMODEL_PLANE);
    	segmentation_filter_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    	eps_angle_ = config.eps_angle;
    	//segmentation_filter_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);

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
