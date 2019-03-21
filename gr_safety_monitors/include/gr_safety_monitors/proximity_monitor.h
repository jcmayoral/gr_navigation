#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/AccelStamped.h>
#include <fusion_msgs/sensorFusionMsg.h>
#include <visualization_msgs/Marker.h>

namespace gr_safety_monitors
{

  class ProximityMonitor : public fault_core::FaultDetector
  {
    public:

      ProximityMonitor();
      ~ProximityMonitor();

      void instantiateServices(ros::NodeHandle nh);
      void initialize(int sensor_number);
      bool detectFault();
      void isolateFault();
      void diagnoseFault();
      void publishTopics();
      fault_core::FaultTopology getFault();

    private:
      std::vector<ros::Subscriber> array_subscribers_;
      bool is_obstacle_detected_;
      ros::Publisher marker_pub_;
  };

}