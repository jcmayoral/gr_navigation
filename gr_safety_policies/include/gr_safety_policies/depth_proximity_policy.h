#ifndef DEPTH_POLICY_H
#define DEPTH_POLICY_H

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/AccelStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <gr_safety_policies/safe_actions/publisher_safe_action.h>
#include <gr_safety_policies/safe_actions/dynamic_reconfigure_safe_action.h>
#include <safety_core/safe_action.h>
#include <safety_core/safe_policy.h>

#include <boost/thread/recursive_mutex.hpp>

#include <dynamic_reconfigure/server.h>
#include <gr_safety_policies/ProximityPolicyConfig.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace gr_safety_policies
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImagesSyncPolicy;

  class DepthProximityPolicy : public safety_core::SafePolicy
  {
    public:

      DepthProximityPolicy();
      ~DepthProximityPolicy();

      void instantiateServices(ros::NodeHandle nh);
      bool checkPolicy();
      void suggestAction();
      void publishTopics();

      void createRingMarker(visualization_msgs::Marker& marker, int level);
      void images_CB(const sensor_msgs::ImageConstPtr& color_image, 
                     const sensor_msgs::ImageConstPtr& depth_image);
      int getRing(float x, float y);
      void dyn_reconfigureCB(gr_safety_policies::ProximityPolicyConfig &config, uint32_t level);
      void timer_cb(const ros::TimerEvent& event);

    protected:
      bool convertDepth2Mat(cv::Mat& frame,  const sensor_msgs::ImageConstPtr& depth_image);
      void publishOutput(cv::Mat frame);

    private:
      message_filters::Synchronizer<ImagesSyncPolicy>*images_syncronizer_;
      bool is_obstacle_detected_;
      ros::Timer timer_publisher_;
      ros::Publisher marker_pub_;
      ros::Publisher depth_image_pub_;
      ros::Subscriber depth_image_sub_;
      visualization_msgs::MarkerArray marker_array_;


      message_filters::Subscriber<sensor_msgs::Image>* sub_1;
      message_filters::Subscriber<sensor_msgs::Image>* sub_2;

      ros::Time last_detection_time_;
      double region_radius_;
      int regions_number_;
      int fault_region_id_;
      SafeAction* action_executer_;
      boost::recursive_mutex mutex;
      dynamic_reconfigure::Server<gr_safety_policies::ProximityPolicyConfig> dyn_server_;
      dynamic_reconfigure::Server<gr_safety_policies::ProximityPolicyConfig>::CallbackType dyn_server_cb_;
  };

};

#endif
