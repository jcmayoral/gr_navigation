#include <pcl_gpu_tools/gpu_example.h>

GPUExample::GPUExample ()  {
    ros::NodeHandle nh;

    gec.setClusterTolerance (1.0);
    gec.setMinClusterSize (1);
    //gec.setMaxClusterSize (0);

    //conditional_filter_ = pc0l::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());
    //Sphere
    double limit = 15.0;
    pass_through_filter_.setFilterFieldName ("z");
    pass_through_filter_.setFilterLimits (0.20,2.0);
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr conditional_filter (new pcl::ConditionAnd<pcl::PointXYZ> ());
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -0.75)));
    //range_condAND->      addComparison (     FieldComparisonConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GE, newOriginX)));
    //conditional_filter_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -limit)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2.0)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -limit)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, limit)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -limit)));
    conditional_filter->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, limit)));
    condition_removal_.setCondition (conditional_filter);
    condition_removal_.setKeepOrganized(false);


    segmentation_filter_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
    segmentation_filter_.setAxis(axis);
    segmentation_filter_.setEpsAngle(30.0* (M_PI/180.0f) ); // plane can be within n degrees of X-Z plane
    //segmentation_filter_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    segmentation_filter_.setMethodType(pcl::SAC_RANSAC);
    segmentation_filter_.setMaxIterations(200);
    segmentation_filter_.setDistanceThreshold(0.65);
    segmentation_filter_.setOptimizeCoefficients(true);

    extraction_filter_.setNegative(true);


    outliers_filter_.setMeanK(20);
    outliers_filter_.setStddevMulThresh(2.0);
    timer_ = nh.createTimer(ros::Duration(0.2), &GPUExample::timer_cb, this);

    pc_sub_ = nh.subscribe("/velodyne_points", 1, &GPUExample::pointcloud_cb, this);

   	pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/filtered", 1);
    cluster_pub_ = nh.advertise<geometry_msgs::PoseArray>("detected_objects",1);


};

void GPUExample::timer_cb(const ros::TimerEvent&){
    boost::mutex::scoped_lock lock(mutex_);
    ROS_ERROR("TIMER CB");
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
    ROS_INFO("PointCloud conversion succeded");
    auto result = run_filter(output);
};

//template <template <typename> class Storage> void
int GPUExample::run_filter(const boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> cloud_filtered){
    boost::mutex::scoped_lock lock(mutex_);

    ROS_INFO("WORKING");
    //pcl::PCDWriter writer;

    condition_removal_.setInputCloud (cloud_filtered);
    condition_removal_.filter (*cloud_filtered);
   
    int original_size = (int) cloud_filtered->points.size ();
    pcl::ModelCoefficients::Ptr filter_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr filter_inliers(new pcl::PointIndices);

    auto init_size = cloud_filtered->points.size();
    int number_of_surfaces = 0;
    
    while (cloud_filtered->points.size()> init_size*0.2){
        segmentation_filter_.setInputCloud(cloud_filtered);
        segmentation_filter_.segment(*filter_inliers, *filter_coefficients);
        
        if (filter_inliers->indices.size () != 0){
            ROS_INFO_STREAM("Plane height" << std::to_string(filter_coefficients->values[3]/filter_coefficients->values[2]));
            //last_ground_height_ = filter_coefficients->values[3]/filter_coefficients->values[2];
            }
        else{
            ROS_INFO("ERROR");
            publishPointCloud<pcl::PointCloud <pcl::PointXYZ>>(*cloud_filtered);
            break;//return false;
        }

        //extracting inliers (removing ground)
        extraction_filter_.setInputCloud(cloud_filtered);
        extraction_filter_.setIndices(filter_inliers);
        extraction_filter_.filter(*cloud_filtered);
        number_of_surfaces++;
    }
    ROS_INFO_STREAM("Removed surfaces  "<< number_of_surfaces);


    //removing points out of borders (potential false positives)
    //pcl::IndicesPtr indices (new std::vector <int>);
    //pass_through_filter_.setInputCloud (cloud_filtered);
    //pass_through_filter_.filter (*indices);


    //removing outliers
    auto original_ponts_number = cloud_filtered->points.size();
    outliers_filter_.setInputCloud(cloud_filtered);
    outliers_filter_.filter(*cloud_filtered);

    ROS_WARN_STREAM("INFO: starting with the GPU version: PC outliers original size " << original_ponts_number << "actual "<< cloud_filtered->points.size());

    main_cloud_ += *cloud_filtered;
    return 1;
}

void GPUExample::cluster(){
    clock_t tStart = clock();

    cloud_device.upload(main_cloud_.points);
    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();
   
    std::vector<pcl::PointIndices> cluster_indices_gpu;
    gec.setSearchMethod (octree_device);

    boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> concatenated_pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(main_cloud_);
    gec.setHostCloud(concatenated_pc);
    gec.extract (cluster_indices_gpu);
    //  octree_device.clear();
    ROS_INFO_STREAM ("GPU Time taken: " << (double)(clock() - tStart)/CLOCKS_PER_SEC);
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

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        geometry_msgs::Pose cluster_center;
        cluster_center.orientation.w = 1.0;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->points.push_back (main_cloud_.points[*pit]);
            cluster_center.position.x += main_cloud_.points[*pit].x/it->indices.size();
            cluster_center.position.y += main_cloud_.points[*pit].y/it->indices.size();
            cluster_center.position.z += main_cloud_.points[*pit].z/it->indices.size();
        }
        clusters_msg.poses.push_back(cluster_center);
    }

    cluster_pub_.publish(clusters_msg);
    publishPointCloud<pcl::PointCloud <pcl::PointXYZ>>(main_cloud_);
};
