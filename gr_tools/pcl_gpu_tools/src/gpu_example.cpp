#include <pcl_gpu_tools/gpu_example.h>

GPUExample::GPUExample (): dynamic_std_(0.1), output_publish_(false),
                           remove_ground_(true), passthrough_enable_(true),
                           is_processing_(false), is_timer_enable_(true),
                           tf2_listener_(tf_buffer_), last_detection_(ros::Time(0)),
                           sensor_frame_("velodyne"), global_frame_("odom"),
                           intensity_classifier_(0.01){
    ros::NodeHandle nh("~");
    //gec.setMaxClusterSize (0);

    //conditional_filter_ = pc0l::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());
    //cilinder ROI
    tStart = clock();
    double limit = 15.0;
    double time_window = 0.2;
    std::string scale_axis = "y";
    int xy_scale = 2;

    nh.getParam("roi", limit);
    nh.getParam("time_window", time_window);
    nh.getParam("global_frame", global_frame_);
    nh.getParam("sensor_frame", sensor_frame_);
    nh.getParam("xy_scale", xy_scale);
    //Error passing char as param
    nh.getParam("scale_axis", scale_axis);

    ROS_INFO_STREAM("ROI Radius [m] "<< limit );
    ROS_INFO_STREAM("Time Window [s] "<< time_window );
    ROS_INFO_STREAM("Global Frame "<< global_frame_ );
    ROS_INFO_STREAM("XY Scaler "<< xy_scale );
    ROS_INFO_STREAM("Scale Axis "<< scale_axis);

    pass_through_filter_.setFilterFieldName ("z");
    radius_cuda_pass_.setMinimumValue(-limit);
    radius_cuda_pass_.setMaximumValue(limit);
    radius_cuda_pass_.setScaleAxis(scale_axis[0]);
    radius_cuda_pass_.setXYScaler(xy_scale);

    /*
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
    condition_removal_.setKeepOrganized(false);
    */

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
    ROS_INFO("Setup Done ready to work");

};

void GPUExample::dyn_reconfigureCB(pcl_gpu_tools::GPUFilterConfig &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    main_cloud_.points.clear();
    ROS_ERROR("RECONFIGURING");
    timer_.stop();
    timer_.setPeriod(ros::Duration(config.cummulative_time), true);
    pass_through_filter_.setFilterLimits (config.min_passthrough_z, config.max_passthrough_z);
    //cuda_pass_.setMinimumValue(config.min_passthrough_z);
    //cuda_pass_.setMaximumValue(config.max_passthrough_z);
    pcl_cuda_pass_.setMinimumValue(config.min_passthrough_z);
    pcl_cuda_pass_.setMaximumValue(config.max_passthrough_z);

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
    intensity_classifier_ = config.intensity_classifier;

    if (config.mode == 1){
      remove_ground_ = false;//config.remove_ground = false;
      passthrough_enable_ = true;//config.passthrough_filter = true;
    }

    if (config.mode == 0){
      remove_ground_ = true;//config.remove_ground = true;
      passthrough_enable_ = false;//config.passthrough_filter = true;
    }

    is_timer_enable_ = config.timer_enable;

    if (config.timer_enable){
      timer_.start();
    }
    ROS_ERROR("END reconfigure");

};

void GPUExample::timer_cb(const ros::TimerEvent&){
    //boost::mutex::scoped_lock lock(mutex_);
    //  ROS_ERROR("timer ");
    tStart = clock();
    cluster();
    main_cloud_.points.clear();
}


template <class T> void GPUExample::publishPointCloud(T t){

    if(t.points.size() ==0 ){
      return;
    } 
    sensor_msgs::PointCloud2 output_pointcloud_;
    pcl::toROSMsg(t, output_pointcloud_);
    output_pointcloud_.header.frame_id = sensor_frame_;
    output_pointcloud_.header.stamp = ros::Time::now();
    // Publish the data
    pc_pub_.publish(output_pointcloud_);
}


void GPUExample::pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr msg){
    //run_filter(*msg);
    //ROS_ERROR("pointcloud cb");
    sensor_frame_ = msg->header.frame_id;
    pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *output);
    //ROS_INFO("PointCloud conversion succeded");
    auto result = run_filter(output);
};


void GPUExample::removeGround(boost::shared_ptr <pcl::PointCloud<pcl::PointXYZI>> pc){
  //ROS_ERROR("Remove ground");
  int original_size = (int) pc->points.size ();
  pcl::ModelCoefficients::Ptr filter_coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr filter_inliers(new pcl::PointIndices);

  auto init_size = pc->points.size();
  int number_of_surfaces = 0;
  bool start = true;

  do{
    segmentation_filter_.setInputCloud(pc);
    segmentation_filter_.segment(*filter_inliers, *filter_coefficients);
    ROS_INFO_STREAM("Distance to floor " << filter_coefficients->values[3]/filter_coefficients->values[2]);
      if (filter_inliers->indices.size () != 0){
          extraction_filter_.setInputCloud(pc);
          extraction_filter_.setIndices(filter_inliers);
          extraction_filter_.filter(*pc);
          number_of_surfaces++;
      }
  }
  while (filter_inliers->indices.size () != 0 && pc->points.size()> init_size*0.8);

  ROS_INFO_STREAM("Surface remove " << number_of_surfaces);
}

//template <template <typename> class Storage> void
int GPUExample::run_filter(const boost::shared_ptr <pcl::PointCloud<pcl::PointXYZI>> cloud_filtered){
    //boost::mutex::scoped_lock lock(mutex_);
    ROS_INFO("filter");
    bb.boxes.clear();

    //Reducing x,y
    radius_cuda_pass_.setHostCloud(cloud_filtered);
    auto res = radius_cuda_pass_.do_stuff("xy", *cloud_filtered);

    //radius_cuda_pass_.setHostCloud(cloud_filtered);
    //auto res2 = radius_cuda_pass_.do_stuff('y', *cloud_filtered);
    //condition_removal_.setInputCloud (cloud_filtered);
    //condition_removal_.filter (*cloud_filtered);

    //Remove Ground or passthrough on z
    if (remove_ground_){
      removeGround(cloud_filtered);
      //removing outliers
      outliers_filter_.setInputCloud(cloud_filtered);
      outliers_filter_.filter(*cloud_filtered);

    }
    if (passthrough_enable_){
      pcl_cuda_pass_.setHostCloud(cloud_filtered);
      pcl_cuda_pass_.do_stuff(*cloud_filtered);
      //cuda_pass_.setHostCloud(cloud_filtered);
      //auto res = cuda_pass_.do_stuff("z", *cloud_filtered);
      //pass_through_filter_.setInputCloud (cloud_filtered);
      //pass_through_filter_.filter (*cloud_filtered);
    }
    main_cloud_ += *cloud_filtered;


    if(!is_timer_enable_){
      timer_cb(ros::TimerEvent());
    }
    return 1;
}

void GPUExample::addBoundingBox(const geometry_msgs::Pose center, double v_x, double v_y, double v_z, double var_i){
  jsk_recognition_msgs::BoundingBox cluster_bb;
  //cluster_bb.header.stamp = ros::Time::now();
  geometry_msgs::Pose out;
  tf2::doTransform(center, out, to_odom_transform);

  cluster_bb.header.frame_id = global_frame_; //this should be a param
  cluster_bb.header.stamp = last_detection_;
  cluster_bb.pose.position.x = out.position.x;
  cluster_bb.pose.position.y = out.position.y;
  cluster_bb.pose.position.z = out.position.z;
  //TODO add orientation
  cluster_bb.pose.orientation.w = 1.0;
  cluster_bb.dimensions.x = v_x;
  cluster_bb.dimensions.y = v_y;
  cluster_bb.dimensions.z = v_z;

  cluster_bb.label = var_i*1000;
  cluster_bb.value = var_i;

  bb.boxes.push_back(cluster_bb);
}

void GPUExample::publishBoundingBoxes(const geometry_msgs::PoseArray& cluster_array){
  bb.header.stamp = ros::Time::now();
  bb.header.frame_id = global_frame_;//cluster_array.header.frame_id;
  bb_pub_.publish(bb);
  //ROS_INFO_STREAM("BoundingBoxes " << bb.boxes.size());
}

void GPUExample::cluster(){
    //ROS_ERROR("cluster");
    boost::mutex::scoped_lock lock(mutex_);
    //Cluster implementation requires XYZ ... If you have a lot of time maybe worth it to modifyied it
    boost::shared_ptr <pcl::PointCloud<pcl::PointXYZ>> pointcloud_xyz = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud(main_cloud_,*pointcloud_xyz.get());


    to_odom_transform = tf_buffer_.lookupTransform(global_frame_, sensor_frame_, last_detection_, ros::Duration(0.5) );

    if (pointcloud_xyz->points.size() == 0 ){
      ROS_ERROR("Cluster empty");
      return;
    }
    //ROS_INFO_STREAM("points "<< pointcloud_xyz->points.size());
    //concatenated_pc->width  = pointcloud_xyz->points.size();
    cloud_device.upload(pointcloud_xyz->points);
    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();
    std::vector<pcl::PointIndices> cluster_indices_gpu;
    gec.setSearchMethod (octree_device);

    gec.setHostCloud(pointcloud_xyz);
    gec.extract (cluster_indices_gpu);
    //octree_device->clear();

    geometry_msgs::PoseArray clusters_msg;

    std::vector<double> x_vector;
    std::vector<double> y_vector;
    std::vector<double> z_vector;
    std::vector<double> i_vector;

    //TODO Test
    pcl::PointCloud<PointXYZI> pointcloud_xyzi;//(new pcl::PointCloud<PointXYZI>);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr concatenated_xyzi;
    pcl::copyPointCloud(*pointcloud_xyz.get(),pointcloud_xyzi);


    double cluster_std;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it){
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        x_vector.clear();
        y_vector.clear();
        z_vector.clear();
        geometry_msgs::Pose cluster_center;
        cluster_center.orientation.w = 1.0;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cluster_center.position.x += main_cloud_.points[*pit].x/it->indices.size();
            cluster_center.position.y += main_cloud_.points[*pit].y/it->indices.size();
            cluster_center.position.z += main_cloud_.points[*pit].z/it->indices.size();
            //concatenated is already filtered
            pointcloud_xyzi.points[*pit].intensity = main_cloud_.points[*pit].intensity;
            x_vector.push_back(main_cloud_.points[*pit].x);
            y_vector.push_back(main_cloud_.points[*pit].y);
            z_vector.push_back(main_cloud_.points[*pit].z);
            i_vector.push_back(main_cloud_.points[*pit].intensity);
        }

        //cluster_std = calculateStd<double>(x_vector)*calculateStd<double>(y_vector);
        double var_x = calculateVariance<double>(x_vector);
        double var_y = calculateVariance<double>(y_vector);
        double var_z = calculateVariance<double>(z_vector);
        double var_i = calculateVariance<double>(z_vector);

        cluster_std = var_x * var_y;// * calculateStd<double>(z_vector);

        if (cluster_std< dynamic_std_ && var_z  > dynamic_std_z_ && var_i < intensity_classifier_){
        //if (cluster_std< dynamic_std_ && range_z  > dynamic_std_z_){
          clusters_msg.poses.push_back(cluster_center);
          auto range_x = getAbsoluteRange<double>(x_vector);
          auto range_y = getAbsoluteRange<double>(y_vector);
          auto range_z = getAbsoluteRange<double>(z_vector);
          //auto range_i = getAbsoluteRange<double>(i_vector);

          addBoundingBox(cluster_center, range_x, range_y, range_z, var_i);
        }
    }

    clusters_msg.header.frame_id = "velodyne";
    clusters_msg.header.stamp = ros::Time::now();
    cluster_pub_.publish(clusters_msg);
    publishBoundingBoxes(clusters_msg);

    if (output_publish_){
        publishPointCloud<pcl::PointCloud <pcl::PointXYZI>>(pointcloud_xyzi);
    }
    ROS_ERROR_STREAM ("Clustering Time: " << (double)(clock() - tStart)/CLOCKS_PER_SEC);

};
