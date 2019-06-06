// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

// Include your header
#include <gr_pointcloud_filter/pointcloud_filter.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(gr_pointcloud_filter::MyNodeletClass, nodelet::Nodelet)

namespace gr_pointcloud_filter{

     template <class T> void MyNodeletClass::publishPointCloud(T t){
       ROS_INFO("Template Function");
       if (!enable_visualization_){
        return;
      }

      pcl::toROSMsg(t, output_pointcloud_);
      output_pointcloud_.header.frame_id = "velodyne";
    	output_pointcloud_.header.stamp = ros::Time::now();
      // Publish the data
      pointcloud_pub_.publish(output_pointcloud_);
      last_processing_time_ = ros::Time::now();
     }

      
    void MyNodeletClass::applyFilters(const sensor_msgs::PointCloud2 msg){
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::fromROSMsg(msg, *cloud);

      //voxeling
      if (filters_enablers_[1]){
        voxel_filter_.setInputCloud(cloud);
        voxel_filter_.filter(*cloud);
      }

      //radius outliers
      if(filters_enablers_[2]){
        // build the filter
        radius_outliers_filter_.setInputCloud(cloud);
        // apply filter
        radius_outliers_filter_.filter(*cloud);
      }

      //conditional_filter
      if (filters_enablers_[3]){
        //conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -config.conditional_distance)));
        //conditional_filter_ = pcl::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());
        //conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, last_ground_height_)));
        condition_removal_.setCondition (conditional_filter_);
        condition_removal_.setKeepOrganized(false);
        condition_removal_.setInputCloud (cloud);
        condition_removal_.filter (*cloud);
      }

    	//segmentation of a plane
      if (filters_enablers_[4]){
        pcl::ModelCoefficients::Ptr filter_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr filter_inliers(new pcl::PointIndices);
        segmentation_filter_.setInputCloud(cloud);
        segmentation_filter_.segment(*filter_inliers, *filter_coefficients);
        if (filter_inliers->indices.size () != 0){
		    	ROS_INFO_STREAM_THROTTLE(1,"Plane height" << std::to_string(filter_coefficients->values[3]/filter_coefficients->values[2]));
          last_ground_height_ = filter_coefficients->values[3]/filter_coefficients->values[2];
          //extracting inliers (removing ground)
          extraction_filter_.setInputCloud(cloud);
          extraction_filter_.setIndices(filter_inliers);
          extraction_filter_.filter(*cloud);
			  }
      }

      if(filters_enablers_[5]){
        //outliers removal filter
        outliers_filter_.setInputCloud(cloud);
        outliers_filter_.filter(*cloud);
      }


      //Start testing
      //TODO remove from here ****
     	pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    	normal_estimator.setSearchMethod (tree);
    	normal_estimator.setInputCloud (cloud);
    	normal_estimator.setKSearch (1);
    	normal_estimator.compute (*normals);


    	pcl::IndicesPtr indices (new std::vector <int>);
    	pcl::PassThrough<pcl::PointXYZ> pass;
    	pass.setInputCloud (cloud);
    	pass.setFilterFieldName ("z");
      pass.setFilterLimits (-0.1,1.0);
    	pass.filter (*indices);


      if (indices->size() <1){
        ROS_WARN("Not enough indices");
        publishPointCloud<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
        return;
      }
      pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    	reg.setMinClusterSize (1);
    	reg.setMaxClusterSize (100);
    	reg.setSearchMethod (tree);
    	reg.setNumberOfNeighbours(cluster_neighbours_number_);
    	reg.setInputCloud (cloud);
    	reg.setIndices (indices);
    	reg.setInputNormals (normals);
    	reg.setSmoothnessThreshold ( smoothness_threshold_/ 180.0 * M_PI);
    	//reg.setCurvatureThreshold (10.0);


      std::vector <pcl::PointIndices> clusters;
    	reg.extract (clusters);
      ROS_DEBUG_STREAM("Clusters size" << clusters.size());
      if (clusters.size()== 0){
        ROS_WARN("Not clusters found");
        publishPointCloud<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
        return;
      }

      int j = 0;
      geometry_msgs::PoseArray clusters_msg;
      clusters_msg.header.frame_id = "velodyne";
      clusters_msg.header.stamp = ros::Time::now();

      for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        geometry_msgs::Pose cluster_center;
        cluster_center.orientation.w = 1.0;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          cloud_cluster->points.push_back (cloud->points[*pit]);
          cluster_center.position.x += cloud->points[*pit].x/it->indices.size();
          cluster_center.position.y += cloud->points[*pit].y/it->indices.size();
          cluster_center.position.z += cloud->points[*pit].z/it->indices.size();
        }
        std::cout << std::endl;
        clusters_msg.poses.push_back(cluster_center);
      }
      obstacle_pub_.publish(clusters_msg);
      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    	publishPointCloud<pcl::PointCloud <pcl::PointXYZRGB>>(*colored_cloud);

    }

    void MyNodeletClass::setFiltersParams(gr_pointcloud_filter::FiltersConfig &config){
      //boost::recursive_mutex::scoped_lock scoped_lock(mutex); //if params are bad the PC process takes too long and no way to change them
		  enable_visualization_ = config.visualize_pointcloud;
      //Enable
      filters_enablers_[0] = config.enable_filters;
      filters_enablers_[1] = config.voxel_filter;
      filters_enablers_[2] = config.radius_outlier_removal;
      filters_enablers_[3] = config.conditional_filter;
      filters_enablers_[4] = config.ground_removal;
      filters_enablers_[5] = config.outlier_removal;
      //voxeling
      voxel_filter_.setLeafSize(config.leaf_size, config.leaf_size, config.leaf_size/2);
      voxel_filter_.setMinimumPointsNumberPerVoxel(config.min_points_per_voxel);
      //condition

      conditional_filter_ = pcl::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());
      //Sphere
      conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -config.conditional_distance)));
      conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, config.conditional_distance)));
      conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -config.conditional_distance)));
      conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, config.conditional_distance)));
      conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -config.conditional_distance)));
      conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, config.conditional_distance)));
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

      //RegionGrowing
      smoothness_threshold_ = config.smoothness_threshold;
      cluster_neighbours_number_ = config.cluster_neighbours_number;
    }

    void MyNodeletClass::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg){
      //Convering sensor_msg to pcl message
      if (!filters_enablers_[0]){
        pointcloud_pub_.publish(msg);
        return;
		  }

        if((last_processing_time_-msg->header.stamp).toSec() > 0.05){
          ROS_ERROR("Too old PointCloud msg skipping");
          return;
      }

      applyFilters(*msg);
    }

    void MyNodeletClass::dyn_reconfigureCB(gr_pointcloud_filter::FiltersConfig &config, uint32_t level){
    	setFiltersParams(config);
    }

    void MyNodeletClass::onInit(){
      ros::NodeHandle nh;
      segmentation_filter_ = pcl::SACSegmentation<pcl::PointXYZ> (true);
      conditional_filter_ = pcl::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());

    	dyn_server_cb_ = boost::bind(&MyNodeletClass::dyn_reconfigureCB, this, _1, _2);
    	dyn_server_.setCallback(dyn_server_cb_);
    	pointcloud_sub_ = nh.subscribe("/velodyne_points", 10, &MyNodeletClass::pointcloud_cb, this);
    	pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filtered", 1);
      obstacle_pub_ = nh.advertise<geometry_msgs::PoseArray>("detected_objects",1);
      last_ground_height_ = 0;
      last_processing_time_ = ros::Time::now();
    	NODELET_DEBUG("Initializing nodelet...");
    }
}
