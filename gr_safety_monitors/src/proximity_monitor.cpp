#include "gr_safety_monitors/proximity_monitor.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(gr_safety_monitors, ProximityMonitor,
                        gr_safety_monitors::ProximityMonitor,
                        fault_core::FaultDetector)

using namespace fault_core;
//using namespace message_filters;
namespace gr_safety_monitors
{

  void ProximityMonitor::instantiateServices(ros::NodeHandle nh){
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("proximity_visualization", 1);
    pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("proximity_pointcloud", 1);
    pointcloud_sub_ = nh.subscribe("/velodyne_points/filtered", 2, &ProximityMonitor::pointcloud_CB, this);
  }

  void ProximityMonitor::pointcloud_CB(const sensor_msgs::PointCloud2::ConstPtr& pointcloud){
    pcl::PointCloud<pcl::PointXYZ> cloud;
 	  pcl::PointCloud<pcl::PointXYZRGBA> rgb_cloud;
    sensor_msgs::PointCloud2 output_pointcloud;

    pcl::fromROSMsg(*pointcloud, cloud);
	  //pcl2 to pclxyzrgba
    pcl::copyPointCloud(cloud,rgb_cloud);

    bool is_detected = false;
    //color
    for (int i = 0; i < rgb_cloud.points.size(); i++) {
      if (getRing(rgb_cloud.points[i].x, rgb_cloud.points[i].y) == 0){
        rgb_cloud.points[i].r = 255;
        ROS_ERROR_THROTTLE(5, "Obstacle Detected on first safety ring");
        is_detected = true;
      }

      if (getRing(rgb_cloud.points[i].x, rgb_cloud.points[i].y) == 1){
        rgb_cloud.points[i].b = 255;
      }

      if (getRing(rgb_cloud.points[i].x, rgb_cloud.points[i].y) == 2){
        rgb_cloud.points[i].g = 255;
      }

    }

    is_obstacle_detected_ = is_detected;

    // Convert to ROS data type
    pcl::toROSMsg(rgb_cloud, output_pointcloud);
    // Publish the data
    pointcloud_pub_.publish(output_pointcloud);
  }

  int ProximityMonitor::getRing(float x, float y){
    float distance = 0.0;
    distance = sqrt(pow(x,2) + pow(y,2));
    return int(distance/region_radius_);
  }

  ProximityMonitor::ProximityMonitor(): is_obstacle_detected_(false), region_radius_(2.5), regions_number_(3), action_executer_(NULL)
  {
    //Define Fault Type as Unknown
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    //Define Fault Cause as Unknown
    fault_.cause_ = FaultTopology::UNKNOWN;
    ROS_INFO("Constructor ProximityMonitor");
  }


  ProximityMonitor::~ProximityMonitor()
  {
    delete action_executer_;
  }

  fault_core::FaultTopology ProximityMonitor::getFault()
  {
     return fault_;
  }

  //This function is called on the navigation_manager, register n number of subscribers
  void ProximityMonitor::initialize(int sensor_number)
  {
    ROS_WARN("Function initialized deprecated for proximity_monitor");
  }

  void ProximityMonitor::createRingMarker(visualization_msgs::Marker& marker, int level){
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

    for (float y= -region_radius_*level; y<=region_radius_*level; y+=0.5 ){
          circumference.x = sqrt(r_2 - pow(y,2));
          circumference.y = y;
          marker.points.push_back(circumference);
    }
    //mirror
    for (float y= region_radius_*level; y>=-region_radius_*level; y-=0.5 ){
          circumference.x = -sqrt(r_2 - pow(y,2));
          circumference.y = y;
          marker.points.push_back(circumference);
    }

  }

  void ProximityMonitor::publishTopics()
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    for (auto i =1 ; i<=regions_number_; i++){
      createRingMarker(marker, i);
      marker_array.markers.push_back(marker);
    }

     marker_pub_.publish(marker_array);
  }

  bool ProximityMonitor::detectFault()
  {
    //The next condition is true when is obstacle not detected and
    // and executer is initialized (obstacle not anymore on danger region)
    publishTopics();

    if(action_executer_!= NULL && !is_obstacle_detected_){
      action_executer_->stop();
      action_executer_ = NULL;
    }
    return is_obstacle_detected_;
  }

  void ProximityMonitor::isolateFault(){
   	boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    diagnoseFault();
  }

  void ProximityMonitor::diagnoseFault(){
    fault_.cause_ = FaultTopology::DYNAMIC_OBSTACLE;
    fault_.type_ = FaultTopology::SENSORFAULT; // TODO Include fault definition of fault_core

    if (action_executer_ == NULL){ //if executer not initialized initialized it
      action_executer_ = new PublisherSafeAction(); 
    }
    action_executer_->execute();
  }
}
