#include "gr_depth_processing/depth_processing.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(gr_depth_processing::MyNodeletClass, nodelet::Nodelet)

namespace gr_depth_processing
{

  void MyNodeletClass::onInit(){

    filterImage = &cv_filter;
    registerImage = &register_pointclouds;

    ros::NodeHandle nh;
    ROS_INFO("Waiting for rgb and depth camera info");
    camera_color_info_ = getOneMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info");
    camera_depth_info_ = getOneMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info");;
    ROS_INFO("Camera info received");

    obstacle_pub_ = nh.advertise<geometry_msgs::PoseArray>("detected_objects",1);
    depth_image_pub_ = nh.advertise<sensor_msgs::Image>("depth_image_processed", 1);

    color_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/color/image_raw", 2);
    depth_image_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera/depth/image_rect_raw", 2);
    bounding_boxes_sub_ = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(nh, "/darknet_ros/bounding_boxes", 2);


    if(false){
      images_syncronizer_ = new message_filters::Synchronizer<ImagesSyncPolicy>(ImagesSyncPolicy(2), *color_image_sub_,*depth_image_sub_);
      images_syncronizer_->registerCallback(boost::bind(&MyNodeletClass::images_CB,this,_1,_2));
    }
    else{
      registered_syncronizer_ = new message_filters::Synchronizer<RegisteredSyncPolicy>(RegisteredSyncPolicy(2), *depth_image_sub_,*bounding_boxes_sub_);
      registered_syncronizer_->registerCallback(boost::bind(&MyNodeletClass::register_CB,this,_1,_2));
    }
    ROS_INFO("Depth Processing initialized");
  }


  bool MyNodeletClass::convertROSImage2Mat(cv::Mat& frame, const sensor_msgs::ImageConstPtr& ros_image){
    try{
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      //input_frame = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image;
      frame = cv_bridge::toCvShare(ros_image, sensor_msgs::image_encodings::TYPE_16UC1)->image; //realsense
      //frame = cv_bridge::toCvShare(ros_image, sensor_msgs::image_encodings::TYPE_32FC1)->image; //zed
      return true;
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }
  }

  void MyNodeletClass::publishOutput(cv::Mat frame, bool rotate){
    sensor_msgs::Image out_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header;

    try{
      //these lines are just for testing rotating image
      cv::Mat rot=cv::getRotationMatrix2D(cv::Point2f(0,0), 3.1416, 1.0);
      //cv::warpAffine(frame,frame, rot, frame.size());
      if (rotate){
        cv::rotate(frame,frame,1);
      }

      //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame);//COLOR
      //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, frame);//zed
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, frame);//realsense
      img_bridge.toImageMsg(out_msg); // from cv_bridge to sensor_msgs::Image
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    depth_image_pub_.publish(out_msg);
  }


  void MyNodeletClass::images_CB(const sensor_msgs::ImageConstPtr color_image, const sensor_msgs::ImageConstPtr depth_image){
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    cv::Mat process_frame;

    if (!convertROSImage2Mat(process_frame, depth_image)){//DEPTH
    //if (!convertROSImage2Mat(process_frame, color_image)){//COLOR
      return;
    }

    filterImage(process_frame);
    publishOutput(process_frame);

  }


  void MyNodeletClass::register_CB(const sensor_msgs::ImageConstPtr depth_image, const darknet_ros_msgs::BoundingBoxesConstPtr bounding_boxes){
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    cv::Mat process_frame;
    std::vector<std::string> distance_to_objects;
    std::vector<std::pair<int,int>> objects_center;
    std::vector<cv::Rect> boundRect;


    if (!convertROSImage2Mat(process_frame, depth_image)){//DEPTH
    //if (!convertROSImage2Mat(process_frame, color_image)){//COLOR
      return;
    }

    for (auto it = bounding_boxes->bounding_boxes.begin(); it != bounding_boxes->bounding_boxes.end(); ++it){
      int center_row = it->xmin + (it->xmax - it->xmin)/2;
      int center_col = it->ymin + (it->ymax - it->ymin)/2;
      objects_center.push_back(std::make_pair(center_row, center_col));
      distance_to_objects.push_back(it->Class + std::to_string(registerImage(*it, process_frame, camera_depth_info_)));
      boundRect.push_back(cv::Rect(it->xmin, it->ymin, it->xmax - it->xmin, it->ymax - it->ymin));
      //boundRect.push_back(cv::Rect(100,100,50,50));

    }

    auto it = distance_to_objects.begin();
    auto it2 = objects_center.begin();
    auto it3 = boundRect.begin();

    for (; it!= distance_to_objects.end(); ++it, ++it2, ++it3){
      cv::putText(process_frame, *it, cv::Point(it2->first, it2->second), cv::FONT_HERSHEY_PLAIN,
                  1,   0xffff , 2, 8);
      cv::rectangle(process_frame, *it3, 0xffff);

    }

    publishOutput(process_frame, false);
  }


}
