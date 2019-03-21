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
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::fromROSMsg(pointcloud, *cloud);
    //pcl::fromPCLPointCloud2(*pointcloud, *cloud);
    pcl::PCLPointCloud2 pcl_pc;
	  pcl_conversions::toPCL(*pointcloud, pcl_pc);
  	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	  pcl::fromPCLPointCloud2(pcl_pc, cloud);
	  //pcl2 to pclxyzrgba
	  //pcl::PointCloud<PointXYZRGBA> input_cloud;
	  //pcl::fromPCLPointCloud2(pcl_pc, input_cloud);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_pointcloud;
    pcl::toROSMsg(cloud, output_pointcloud);
    // Publish the data
    pointcloud_pub_.publish(output_pointcloud);
  }

  ProximityMonitor::ProximityMonitor(): is_obstacle_detected_(false), region_radius_(25), regions_number_(3)
  {
    //Define Fault Type as Unknown
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    //Define Fault Cause as Unknown
    fault_.cause_ = FaultTopology::UNKNOWN;
    ROS_INFO("Constructor ProximityMonitor");
  }


  ProximityMonitor::~ProximityMonitor()
  {

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

  void ProximityMonitor::updateMarker(visualization_msgs::Marker& marker, int level){
    marker.header.frame_id = "velodyne";
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
      updateMarker(marker, i);
      marker_array.markers.push_back(marker);
    }
    
     marker_pub_.publish(marker_array);
  }

  bool ProximityMonitor::detectFault()
  {
    if (is_obstacle_detected_){
      isolateFault();
    }
    return is_obstacle_detected_;
  }

  void ProximityMonitor::isolateFault(){
    ROS_INFO("Isolating Platform Collision");
    diagnoseFault();
  }

  void ProximityMonitor::diagnoseFault(){
    fault_.cause_ = FaultTopology::MISLOCALIZATION; // By default run MisLocalization Recovery Strategy
    fault_.type_ = FaultTopology::COLLISION; // Classify the fault as a Collision
    ROS_ERROR_ONCE("Collision FOUND");
  }
}
