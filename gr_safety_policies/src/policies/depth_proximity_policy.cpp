#include "gr_safety_policies/depth_proximity_policy.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_DECLARE_CLASS(gr_safety_policies, DepthProximityPolicy,
                        gr_safety_policies::DepthProximityPolicy,
                        safety_core::SafePolicy)

using namespace safety_core;
namespace gr_safety_policies
{

  void DepthProximityPolicy::instantiateServices(ros::NodeHandle nh){
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("proximity_visualization", 1);
    pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("proximity_pointcloud", 1);
    pointcloud_sub_ = nh.subscribe("/velodyne_points/filtered", 2, &DepthProximityPolicy::pointcloud_CB, this);
  }

  void DepthProximityPolicy::pointcloud_CB(const sensor_msgs::PointCloud2::ConstPtr& pointcloud){
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZRGBA> rgb_cloud;
    sensor_msgs::PointCloud2 output_pointcloud;

    pcl::fromROSMsg(*pointcloud, cloud);
    //pcl2 to pclxyzrgba
    pcl::copyPointCloud(cloud,rgb_cloud);

    //color
    bool warning_zone = false;

    for (int i = 0; i < rgb_cloud.points.size(); i++) {
      if (getRing(rgb_cloud.points[i].x, rgb_cloud.points[i].y) == 0){//FIRST POINT IN DANGER ZONE.. Return
        rgb_cloud.points[i].r = 255;
        fault_region_id_ = 0;
        last_detection_time_ = ros::Time::now();
        is_obstacle_detected_ = true;
        return;
      }

      if (getRing(rgb_cloud.points[i].x, rgb_cloud.points[i].y) == 1){
        rgb_cloud.points[i].b = 255;
        fault_region_id_ = 1;
        warning_zone = true;
        //last_detection_time_ = ros::Time::now();

        is_obstacle_detected_ = true;
      }

      /*
      if (getRing(rgb_cloud.points[i].x, rgb_cloud.points[i].y) > 1){
        rgb_cloud.points[i].g = 255;
      }
      */

    }

    if (!warning_zone){
        fault_region_id_ = 2;
        is_obstacle_detected_ = false;
    }

    // Convert to ROS data type
    pcl::toROSMsg(rgb_cloud, output_pointcloud);
    // Publish the data
    pointcloud_pub_.publish(output_pointcloud);
  }

  int DepthProximityPolicy::getRing(float x, float y){
    float distance = 0.0;
    distance = sqrt(pow(x,2) + pow(y,2));
    return int(distance/region_radius_);
  }

  DepthProximityPolicy::DepthProximityPolicy(): is_obstacle_detected_(false), region_radius_(2.5),
                                        regions_number_(3), action_executer_(NULL),
                                        fault_region_id_(0)
  {
    ros::NodeHandle nh;
    timer_publisher_ = nh.createTimer(ros::Duration(5), &DepthProximityPolicy::timer_cb, this);
    last_detection_time_ = ros::Time::now();
    policy_.id_ = "PROXIMITY_POLICY";
    policy_.action_ =  -1;
    policy_.state_ = PolicyDescription::UNKNOWN;
    policy_.type_ = PolicyDescription::ACTIVE;

    dyn_server_cb_ = boost::bind(&DepthProximityPolicy::dyn_reconfigureCB, this, _1, _2);
    dyn_server_.setCallback(dyn_server_cb_);
    ROS_INFO("Constructor ProximityPolicy");
  }

  void DepthProximityPolicy::timer_cb(const ros::TimerEvent& event){
    if (ros::Time::now().toSec() - last_detection_time_.toSec() > 5.0){
      if (action_executer_!= NULL){
        if (action_executer_->getSafetyID()==0){
          ROS_ERROR("Enable navigation... PAY ATTENTION");
          action_executer_->stop();
          action_executer_ = NULL;
        }
      }
    }

  }

  DepthProximityPolicy::~DepthProximityPolicy()
  {
    delete action_executer_;
  }

  void DepthProximityPolicy::dyn_reconfigureCB(gr_safety_policies::ProximityPolicyConfig &config, uint32_t level){
    region_radius_ = config.region_radius;

    visualization_msgs::Marker marker;

    marker_array_.markers.clear();
    for (auto i =1 ; i<=regions_number_; i++){
      createRingMarker(marker, i);
      marker_array_.markers.push_back(marker);
    }
  }

  void DepthProximityPolicy::createRingMarker(visualization_msgs::Marker& marker, int level){
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "proximity_regions"+std::to_string(level);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(1.0);
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;//region_radius_ * level;
    //marker.scale.y = region_radius_ * level;
    //marker.scale.z = 0.05;
    marker.color.r = 1.0f /level;
    marker.color.g = 1.0f /level;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point circumference;
    float r_2 = pow(region_radius_*level,2);

    for (float y= -region_radius_*level; y<=region_radius_*level; y+=0.1 ){
          if (pow(y,2) > r_2)
            continue;
          circumference.x = sqrt(r_2 - pow(y,2));
          circumference.y = y;
          marker.points.push_back(circumference);
    }


    //For some value 0 is never reached
    circumference.y = region_radius_*level;
    circumference.x = 0.0;
    marker.points.push_back(circumference);

    //mirror
    for (float y= region_radius_*level; y>=-region_radius_*level; y-= 0.1 ){
          if (pow(y,2) > r_2)
            continue;
          circumference.x = -sqrt(r_2 - pow(y,2));
          circumference.y = y;
          marker.points.push_back(circumference);
    }

    //For some value 0 is never reached
    circumference.y = -region_radius_*level;
    circumference.x = 0.0;
    marker.points.push_back(circumference);

  }

  void DepthProximityPolicy::publishTopics()
  {
    for (auto i=0; i< marker_array_.markers.size(); i++){
      marker_array_.markers[i].header.stamp = ros::Time::now();
    }
     marker_pub_.publish(marker_array_);
  }

  bool DepthProximityPolicy::checkPolicy()
  {
    //The next condition is true when is obstacle not detected and
    // and executer is initialized (obstacle not anymore on danger region)
    publishTopics();
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);

    if(action_executer_!= NULL && !is_obstacle_detected_){
      //ROS_WARN("Stopping executer due no obstacle detected");
      //action_executer_->stop();
      //action_executer_ = NULL;
      policy_.action_ = -1;
      policy_.state_ = PolicyDescription::SAFE;
    }
    return is_obstacle_detected_;
  }

  void DepthProximityPolicy::suggestAction(){

    //action_executer_ = new PublisherSafeAction();
    ROS_INFO_STREAM_THROTTLE(5, "Obstacle detected on region with ID "<< fault_region_id_ );
    switch (fault_region_id_){
      case 0: // if DANGER REGION... stop whatever is it and
      //if executer not initialized initialized it just if lock was not locked before
        if (action_executer_ != NULL){
          if (action_executer_->getSafetyID()!=0){ //overwrite other actions if initialize ignore
            action_executer_->stop();
            action_executer_ = new PublisherSafeAction();
            action_executer_->execute();
          }
        }
        else{//initialize if not
          action_executer_ = new PublisherSafeAction();
          action_executer_->execute();
        }
        break;
      case 1:
        if (action_executer_ != NULL){
          if (action_executer_->getSafetyID()==0){
            ROS_DEBUG("Obstacle too close avoiding reconfiguration");
            return;
          }
          if(action_executer_->getSafetyID()!=1){
            //Stop any action which is not DANGER AND WARNING
            ROS_INFO_STREAM("Stopping previous action " << action_executer_->getSafetyID());
            action_executer_->stop();
            action_executer_ = new DynamicReconfigureSafeAction();
            action_executer_->execute();
          }
        }
        else{//init if NULL
          action_executer_ = new DynamicReconfigureSafeAction();
          action_executer_->execute();
        }
        break;
      default:
        ROS_ERROR("Error detecting obstacles");
        break;
    }
    policy_.state_ = PolicyDescription::UNSAFE;
    policy_.action_ = action_executer_->getSafetyID();
  }
}