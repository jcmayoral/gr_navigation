#include <ros/ros.h>
#include <string>
#include <fault_core/fault_detector.h>
//message_filters maybe useful if syncronization is needed
/*
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
*/
//#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/AccelStamped.h>
#include <fusion_msgs/sensorFusionMsg.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gr_safety_monitors/safe_actions/publisher_safe_action.h>

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

      void createRingMarker(visualization_msgs::Marker& marker, int level);
      void pointcloud_CB(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
      int getRing(float x, float y);
    private:
      std::vector<ros::Subscriber> array_subscribers_;
      bool is_obstacle_detected_;
      ros::Publisher marker_pub_;
      ros::Publisher pointcloud_pub_;
      ros::Subscriber pointcloud_sub_;
      double region_radius_;
      int regions_number_;
      SafeAction* action_executer_;
  };

}