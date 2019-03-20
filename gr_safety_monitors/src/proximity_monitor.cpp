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


  }

  ProximityMonitor::ProximityMonitor(): is_obstacle_detected_(false)
  {
    //Define Fault Type as Unknown
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    //Define Fault Cause as Unknown
    fault_.cause_ = FaultTopology::UNKNOWN;
    ROS_INFO("Constructor SimpleCollisionDetector");
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
    ros::NodeHandle nh;
    ROS_INFO_STREAM("initializing " << sensor_number << " sensors");
  }

  bool ProximityMonitor::detectFault()
  {
    ROS_DEBUG("SimpleCollisionDetector Detect Fault");
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
