#include <pcl_gpu_tools/gpu_example.h>

GPUExample::GPUExample (): dynamic_std_(0.1), output_publish_(false)  {
    ros::NodeHandle nh("~");
    //gec.setMaxClusterSize (0);

    //conditional_filter_ = pc0l::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());
    //cilinder ROI
    double limit = 15.0;
    double time_window = 0.2;
    nh.getParam("roi", limit);
    nh.getParam("time_window", time_window);
    ROS_INFO_STREAM("ROI Radius [m] "<< limit );
    ROS_INFO_STREAM("Time Window [s] "<< time_window );

    pass_through_filter_.setFilterFieldName ("z");
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr conditional_filter (new pcl::ConditionAnd<pcl::PointXYZ> ());
    //conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -0.8)));
    //range_condAND->      addComparison (     FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE, newOriginX)));
    //conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -limit)));
    //conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2.0)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -limit)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, limit)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -limit)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, limit)));
    condition_removal_.setCondition (conditional_filter);
    condition_removal_.setKeepOrganized(true);

    segmentation_filter_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
    segmentation_filter_.setAxis(axis);
    //segmentation_filter_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    segmentation_filter_.setMethodType(pcl::SAC_RANSAC);

    timer_ = nh.createTimer(ros::Duration(time_window), &GPUExample::timer_cb, this);
	dyn_server_cb_ = boost::bind(&GPUExample::dyn_reconfigureCB, this, _1, _2);
    dyn_server_.setCallback(dyn_server_cb_);

    pc_sub_ = nh.subscribe("/velodyne_points", 1, &GPUExample::pointcloud_cb, this);
   	pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filtered", 1);
    cluster_pub_ = nh.advertise<geometry_msgs::PoseArray>("detected_objects",1);
    bb_pub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detection/bounding_boxes", 1);

};

void GPUExample::dyn_reconfigureCB(pcl_gpu_tools::GPUFilterConfig &config, uint32_t level){
    ROS_ERROR("RECONFIGURING");
    pass_through_filter_.setFilterLimits (config.min_passthrough_z, config.max_passthrough_z);
    segmentation_filter_.setEpsAngle(config.eps_angle* (M_PI/180.0f) ); // plane can be within n degrees of X-Z plane
    segmentation_filter_.setMaxIterations(config.max_iterations);
    segmentation_filter_.setDistanceThreshold(config.distance_threshold);
    segmentation_filter_.setOptimizeCoefficients(config.optimize_coefficients);
    extraction_filter_.setNegative(config.set_negative);
    outliers_filter_.setMeanK(config.outlier_meank);
    outliers_filter_.setStddevMulThresh(config.outlier_std);
    gec.setClusterTolerance (config.cluster_tolerance);
    gec.setMinClusterSize (config.min_cluster_size);
    dynamic_std_ = config.dynamic_classifier;
    dynamic_std_z_ = config.dynamic_classifier_z;
    output_publish_ = config.publish_output;

};

void GPUExample::timer_cb(const ros::TimerEvent&){
    //boost::mutex::scoped_lock lock(mutex_);
    //ROS_ERROR("TIMER CB");
    cluster();
    main_cloud_.points.clear();
}


template <class T> void GPUExample::publishPointCloud(T t){
    sensor_msgs::PointCloud2 output_pointcloud_;
    pcl::toROSMsg(t, output_pointcloud_);
    output_pointcloud_.header.frame_id = "velodyne";
    output_pointcloud_.header.stamp = ros::Time::now();
    // Publish the data
    pc_pub_.publish(output_pointcloud_);
}


void GPUExample::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    //run_filter(*msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *output);
    //ROS_INFO("PointCloud conversion succeded");
    auto result = run_filter(output);
};

//template <template <typename> class Storage> void
int GPUExample::run_filter(const boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> cloud_filtered){
    boost::mutex::scoped_lock lock(mutex_);
    bb.boxes.clear();
    //ROS_INFO("WORKING");
    //pcl::PCDWriter writer;

    condition_removal_.setInputCloud (cloud_filtered);
    condition_removal_.filter (*cloud_filtered);

    int original_size = (int) cloud_filtered->points.size ();
    pcl::ModelCoefficients::Ptr filter_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr filter_inliers(new pcl::PointIndices);

    auto init_size = cloud_filtered->points.size();
    int number_of_surfaces = 0;
    bool start = true;

    do{
        segmentation_filter_.setInputCloud(cloud_filtered);
        segmentation_filter_.segment(*filter_inliers, *filter_coefficients);

        if (filter_inliers->indices.size () != 0){
            //ROS_INFO_STREAM("Plane height" << std::to_string(filter_coefficients->values[3]/filter_coefficients->values[2]));
            //last_ground_height_ = filter_coefficients->values[3]/filter_coefficients->values[2];
            //extracting inliers (removing ground)
            extraction_filter_.setInputCloud(cloud_filtered);
            extraction_filter_.setIndices(filter_inliers);
            extraction_filter_.filter(*cloud_filtered);
            number_of_surfaces++;
        }
        //else{
           // ROS_INFO("ERROR");
            //publishPointCloud<pcl::PointCloud <pcl::PointXYZ>>(*cloud_filtered);
            //break;//return false;
        //}

        //ROS_INFO_STREAM(filter_inliers->indices.size () );
    }
    while (filter_inliers->indices.size () != 0 && cloud_filtered->points.size()> init_size*0.3);

    //removing points out of borders (potential false positives)
    pass_through_filter_.setInputCloud (cloud_filtered);
    pass_through_filter_.filter (*cloud_filtered);

    //removing outliers
    auto original_ponts_number = cloud_filtered->points.size();
    outliers_filter_.setInputCloud(cloud_filtered);
    outliers_filter_.filter(*cloud_filtered);

    ROS_WARN_STREAM("INFO: starting with the GPU version: PC outliers original size " << original_ponts_number << "actual "<< cloud_filtered->points.size());

    main_cloud_ += *cloud_filtered;
    return 1;
}

void GPUExample::addBoundingBox(const geometry_msgs::Pose center, double v_x, double v_y, double v_z){
  jsk_recognition_msgs::BoundingBox cluster_bb;
  cluster_bb.header.stamp = ros::Time::now();
  cluster_bb.header.frame_id = "velodyne"; //this should be a param
  cluster_bb.pose.position.x = center.position.x;
  cluster_bb.pose.position.y = center.position.y;
  cluster_bb.pose.position.z = center.position.z;
  cluster_bb.dimensions.x = v_x;
  cluster_bb.dimensions.y = v_y;
  cluster_bb.dimensions.z = v_z;
  bb.boxes.push_back(cluster_bb);
}

void GPUExample::publishBoundingBoxes(const geometry_msgs::PoseArray& cluster_array){
  const std::vector<geometry_msgs::Pose>& ps = cluster_array.poses;
  bb.header.stamp = ros::Time::now();
  bb.header.frame_id = cluster_array.header.frame_id;
  bb_pub_.publish(bb);
}

void GPUExample::cluster(){
    boost::mutex::scoped_lock lock(mutex_);
    boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> concatenated_pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(main_cloud_);
    if (concatenated_pc->points.size() == 0){
        return;
    }
    clock_t tStart = clock();

    //START TEMPORAL
    /*
    pcl::ModelCoefficients::Ptr filter_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr filter_inliers(new pcl::PointIndices);

    segmentation_filter_.setInputCloud(concatenated_pc);
    segmentation_filter_.segment(*filter_inliers, *filter_coefficients);

    if (filter_inliers->indices.size () != 0){
        ROS_ERROR_STREAM("Plane height" << std::to_string(filter_coefficients->values[3]/filter_coefficients->values[2]));
        //extracting inliers (removing ground)
        extraction_filter_.setInputCloud(concatenated_pc);
        extraction_filter_.setIndices(filter_inliers);
        extraction_filter_.filter(*concatenated_pc);
    }
    */
    //END TEMPORAL


    cloud_device.upload(concatenated_pc->points);
    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();

    std::vector<pcl::PointIndices> cluster_indices_gpu;
    gec.setSearchMethod (octree_device);

    gec.setHostCloud(concatenated_pc);
    gec.extract (cluster_indices_gpu);
    //octree_device->clear();
    //std::cout << "INFO: stopped with the GPU version" << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster_gpu->points.push_back (main_cloud_.points[*pit]); //*
        cloud_cluster_gpu->width = cloud_cluster_gpu->points.size ();
        cloud_cluster_gpu->height = 1;
        cloud_cluster_gpu->is_dense = true;

        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size () << " data points." << std::endl;
        //std::stringstream ss;
        //ss << "gpu_cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster_gpu, false); //*
        //j++;
    }

    geometry_msgs::PoseArray clusters_msg;
    clusters_msg.header.frame_id = "velodyne";
    clusters_msg.header.stamp = ros::Time::now();

    std::vector<double> x_vector;
    std::vector<double> y_vector;
    std::vector<double> z_vector;

    double cluster_std;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it){
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        x_vector.clear();
        y_vector.clear();
        z_vector.clear();
        geometry_msgs::Pose cluster_center;
        cluster_center.orientation.w = 1.0;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            //cloud_cluster->points.push_back (main_cloud_.points[*pit]);
            cluster_center.position.x += main_cloud_.points[*pit].x/it->indices.size();
            cluster_center.position.y += main_cloud_.points[*pit].y/it->indices.size();
            cluster_center.position.z += main_cloud_.points[*pit].z/it->indices.size();
            x_vector.push_back(main_cloud_.points[*pit].x);
            y_vector.push_back(main_cloud_.points[*pit].y);
            z_vector.push_back(main_cloud_.points[*pit].z);
        }

        //cluster_std = calculateStd<double>(x_vector)*calculateStd<double>(y_vector);
        double var_x = calculateVariance<double>(x_vector);
        double var_y = calculateVariance<double>(y_vector);
        double var_z = calculateVariance<double>(z_vector);

        cluster_std = var_x * var_y;// * calculateStd<double>(z_vector);
        //if (cluster_std> dynamic_std_ && dynamic_std_z_/2 < z_std  < dynamic_std_z_){
        if (cluster_std< dynamic_std_ && var_z  > dynamic_std_z_){
          std::cout << "VAR " << cluster_std << std::endl;
          clusters_msg.poses.push_back(cluster_center);
          addBoundingBox(cluster_center, var_x, var_y, var_z);
        }
    }

    cluster_pub_.publish(clusters_msg);
    publishBoundingBoxes(clusters_msg);

    if (output_publish_){
        publishPointCloud<pcl::PointCloud <pcl::PointXYZ>>(main_cloud_);
    }
    ROS_INFO_STREAM ("Clustering Time: " << (double)(clock() - tStart)/CLOCKS_PER_SEC);

};
