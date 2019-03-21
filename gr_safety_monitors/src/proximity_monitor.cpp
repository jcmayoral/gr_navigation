#include "gr_safety_monitors/proximity_monitor.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(gr_safety_monitors, ProximityMonitor,
                        gr_safety_monitors::ProximityMonitor,
                        fault_core::FaultDetector)

using namespace fault_core;
using namespace message_filters;
namespace gr_safety_monitors
{

  void ProximityMonitor::instantiateServices(ros::NodeHandle nh){
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("proximity_visualization", 1);


  }

  ProximityMonitor::ProximityMonitor(): is_obstacle_detected_(false)
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

  void ProximityMonitor::publishTopics()
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "proximity_regions";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.lifetime = ros::Duration(0.0);
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 10.0;
    marker.scale.y = 10.0;
    marker.scale.z = 10;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker_pub_.publish(marker);
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
