#ifndef PROXIMITY_POLICY_H
#define PROXIMITY_POLICY_H

#include <ros/ros.h>
#include <string>
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
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gr_safety_policies/safe_actions/publisher_safe_action.h>
#include <gr_safety_policies/safe_actions/dynamic_reconfigure_safe_action.h>
#include <safety_core/safe_action.h>
#include <safety_core/safe_policy.h>

#include <boost/thread/recursive_mutex.hpp>

#include <dynamic_reconfigure/server.h>
#include <gr_safety_policies/ProximityPolicyConfig.h>

namespace gr_safety_policies
{

  class HRIPolicy : public safety_core::SafePolicy
  {
    public:

      HRIPolicy();
      ~HRIPolicy();

      void instantiateServices(ros::NodeHandle nh);
      bool checkPolicy();
      void suggestAction();
      void commands_CB(const std_msgs::String::ConstPtr& command);

    private:
      bool is_command_received;
      ros::Timer timer_publisher_;
      ros::Publisher marker_pub_;
      ros::Subscriber pointcloud_sub_;
      visualization_msgs::MarkerArray marker_array_;

      ros::Time last_detection_time_;
      double region_radius_;
      double enabling_after_timeout_;
      int regions_number_;
      int fault_region_id_;
      SafeAction* action_executer_;
      boost::recursive_mutex mutex;
      dynamic_reconfigure::Server<gr_safety_policies::ProximityPolicyConfig> dyn_server_;
      dynamic_reconfigure::Server<gr_safety_policies::ProximityPolicyConfig>::CallbackType dyn_server_cb_;
  };

};

#endif
