#include "gr_depth_processing/depth_processing.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(gr_depth_processing::MyNodeletClass, nodelet::Nodelet)

namespace gr_depth_processing
{

  void MyNodeletClass::onInit(){

    filterImage = &cv_filter;

    ros::NodeHandle nh;
    ROS_INFO("Waiting for rgb camera info");
    boost::shared_ptr<sensor_msgs::CameraInfo const> camera_info;
    camera_info =  ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info");
    camera_color_info_ = *camera_info;

    ROS_INFO("Waiting for depth camera info");
    camera_info =  ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info");
    camera_depth_info_ = *camera_info;

    obstacle_pub_ = nh.advertise<geometry_msgs::PoseArray>("detected_objects",1);
    depth_image_pub_ = nh.advertise<sensor_msgs::Image>("depth_image_processed", 1);
    sub_1 = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/color/image_raw", 2);
    sub_2 = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/depth/image_rect_raw", 2);
    background_substractor_ = cv::createBackgroundSubtractorMOG2();
    //background_substractor_->setNMixtures(3);
    //background_substractor_->setHistory(3);
    //background_substractor_ =  new cv::BackgroundSubtractorMOG2(1, 16, true); //MOG2 approach
    images_syncronizer_ = new message_filters::Synchronizer<ImagesSyncPolicy>(ImagesSyncPolicy(2), *sub_1,*sub_2);
    images_syncronizer_->registerCallback(boost::bind(&MyNodeletClass::images_CB,this,_1,_2));
    ROS_INFO("Depth Processing initialized");
  }


  bool MyNodeletClass::convertROSImage2Mat(cv::Mat& frame, const sensor_msgs::ImageConstPtr& ros_image){
    try{
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      //input_frame = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
      //frame = cv_bridge::toCvShare(ros_image, sensor_msgs::image_encodings::TYPE_16UC1)->image; \\this to convert from Depth
      frame = cv_bridge::toCvShare(ros_image, sensor_msgs::image_encodings::RGB8)->image; //this to convert from color
      return true;
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }
  }

  void MyNodeletClass::publishOutput(cv::Mat frame){
    sensor_msgs::Image out_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header;

    try{
      //these lines are just for testing rotating image
      cv::Mat rot=cv::getRotationMatrix2D(cv::Point2f(0,0), 3.1416, 1.0);
      //cv::warpAffine(frame,frame, rot, frame.size());
      cv::rotate(frame,frame,1);


      //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame);//COLOR
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, frame);//GRAY

      //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, frame);//DEPTH
      img_bridge.toImageMsg(out_msg); // from cv_bridge to sensor_msgs::Image
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    depth_image_pub_.publish(out_msg);
  }


  void MyNodeletClass::images_CB(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image){
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    cv::Mat process_frame;

    if (!convertROSImage2Mat(process_frame, depth_image)){//DEPTH
    //if (!convertROSImage2Mat(process_frame, color_image)){//COLOR
      return;
    }
    
    filterImage(process_frame);
    publishOutput(process_frame);

  }

 
}
