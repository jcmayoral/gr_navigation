#ifndef DEPTH_POLICY_H
#define DEPTH_POLICY_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/CameraInfo.h>

#include <boost/thread/recursive_mutex.hpp>

//#include <dynamic_reconfigure/server.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>


namespace gr_depth_processing
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImagesSyncPolicy;


  class MyNodeletClass : public nodelet::Nodelet
  {
    public:
      virtual void onInit();
      void images_CB(const sensor_msgs::ImageConstPtr& color_image,
                     const sensor_msgs::ImageConstPtr& depth_image);

      cv::Mat detectPeople(cv::Mat frame_gray);

    protected:
      bool convertROSImage2Mat(cv::Mat& frame,  const sensor_msgs::ImageConstPtr& ros_image);
      void publishOutput(cv::Mat frame);

    private:
      message_filters::Synchronizer<ImagesSyncPolicy>*images_syncronizer_;
      ros::Publisher obstacle_pub_;
      ros::Publisher depth_image_pub_;
      ros::Subscriber depth_image_sub_;
      visualization_msgs::MarkerArray marker_array_;

      cv::Ptr<cv::BackgroundSubtractorMOG2> background_substractor_;

      //intrinsic params
      sensor_msgs::CameraInfo camera_color_info_;
      sensor_msgs::CameraInfo camera_depth_info_;

      message_filters::Subscriber<sensor_msgs::Image>* sub_1;
      message_filters::Subscriber<sensor_msgs::Image>* sub_2;

      boost::recursive_mutex mutex;
  };

};

#endif
