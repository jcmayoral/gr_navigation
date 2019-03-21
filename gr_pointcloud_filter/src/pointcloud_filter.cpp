// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include <gr_pointcloud_filter/pointcloud_filter.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(gr_pointcloud_filter::MyNodeletClass, nodelet::Nodelet)

namespace gr_pointcloud_filter
{
    void MyNodeletClass::applyFilters(const sensor_msgs::PointCloud2 msg){
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::fromROSMsg(msg, *cloud);
		
		//Convering sensor_msg to pcl message
		if (!filters_enablers_[0]){
			// Convert to ROS data type
			pcl::toROSMsg(*cloud, output_pointcloud_);
    		// Publish the data
    		pointcloud_pub_.publish(output_pointcloud_);
    		return;
		}

    	//voxeling
		if (filters_enablers_[1]){
			voxel_filter_.setInputCloud(cloud);
			voxel_filter_.filter(*cloud);
		}

		//conditional_filter
		if (filters_enablers_[2]){
			condition_removal_.setInputCloud (cloud);
			condition_removal_.filter (*cloud);
		}

    	//segmentation of a plane
		if (filters_enablers_[3]){
			pcl::ModelCoefficients::Ptr filter_coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr filter_inliers(new pcl::PointIndices);
			segmentation_filter_.setInputCloud(cloud);
			segmentation_filter_.segment(*filter_inliers, *filter_coefficients);
			if (filter_inliers->indices.size () != 0){
		    	ROS_INFO_STREAM_THROTTLE(1,"Plane height" << std::to_string(filter_coefficients->values[3]/filter_coefficients->values[2]));
		    	//extracting inliers (removing ground)
				extraction_filter_.setInputCloud(cloud);
				extraction_filter_.setIndices(filter_inliers);
				extraction_filter_.filter(*cloud);
			}
		}

		if(filters_enablers_[4]){
			//outliers removal filter
			outliers_filter_.setInputCloud(cloud);
			outliers_filter_.filter(*cloud);
		}

		if(filters_enablers_[5]){
			//radius outliers On progress
			// build the filter
			radius_outliers_filter_.setInputCloud(cloud);
			// apply filter
			radius_outliers_filter_.filter(*cloud);
		}

    	// Convert to ROS data type
    	pcl::toROSMsg(*cloud, output_pointcloud_);
    	// Publish the data
    	pointcloud_pub_.publish(output_pointcloud_);
    }

    void MyNodeletClass::setFiltersParams(gr_pointcloud_filter::FiltersConfig &config){
    	boost::recursive_mutex::scoped_lock scoped_lock(mutex);
		//Enable
		filters_enablers_[0] = config.enable_filters;
		filters_enablers_[1] = config.voxel_filter;
		filters_enablers_[2] = config.conditional_filter;
		filters_enablers_[3] = config.ground_removal;
		filters_enablers_[4] = config.outlier_removal;
		filters_enablers_[5] = config.radius_outlier_removal;

    	//voxeling
    	voxel_filter_.setLeafSize(config.leaf_size, config.leaf_size, config.leaf_size);

		//condition
		conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, config.min_height)));
    	conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, config.max_height)));
		condition_removal_.setCondition (conditional_filter_);
		condition_removal_.setKeepOrganized(false);

    	//segmentating
    	//segmentation_filter_.setModelType(pcl::SACMODEL_PLANE);
    	segmentation_filter_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    	eps_angle_ = config.eps_angle;
		Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
		segmentation_filter_.setAxis(axis);
		segmentation_filter_.setEpsAngle(eps_angle_ * (M_PI/180.0f) ); // plane can be within 30 degrees of X-Z plane
			
    	//segmentation_filter_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    	segmentation_filter_.setMethodType(pcl::SAC_RANSAC);
    	segmentation_filter_.setMaxIterations(config.max_iterations);
    	segmentation_filter_.setDistanceThreshold(config.distance_threshold);
    	segmentation_filter_.setOptimizeCoefficients(config.optimize_coefficients);
    	//segmentation_filter_.setSamplesMaxDist(config.samples_max_dist);

    	//extraction
    	extraction_filter_.setNegative(config.set_negative);

    	//ouliers
    	outliers_filter_.setMeanK(config.mean_k);
    	outliers_filter_.setStddevMulThresh(config.std_threshold);

      	//radius_outliers
      	min_radius_ = config.min_radius_removal;
      	min_neighbours_ = config.min_neighbours;
		radius_outliers_filter_.setRadiusSearch(min_radius_);
      	radius_outliers_filter_.setMinNeighborsInRadius(min_neighbours_);
      	
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
      	segmentation_filter_ = pcl::SACSegmentation<pcl::PointXYZ> (true);
		conditional_filter_ = pcl::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());

      	dyn_server_cb_ = boost::bind(&MyNodeletClass::dyn_reconfigureCB, this, _1, _2);
      	dyn_server_.setCallback(dyn_server_cb_);
      	pointcloud_sub_ = nh.subscribe("/velodyne_points", 10, &MyNodeletClass::pointcloud_cb, this);
      	pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filtered", 10);
      	NODELET_DEBUG("Initializing nodelet...");
    }
}
