#include "gr_depth_processing/depth_processing.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(gr_depth_processing::MyNodeletClass, nodelet::Nodelet)

namespace gr_depth_processing
{

  void MyNodeletClass::onInit(){
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
    cv::Mat input_frame, output_frame;

    //if (!convertROSImage2Mat(input_frame, depth_image)){//DEPTH
    if (!convertROSImage2Mat(input_frame, color_image)){//COLOR
      return;
    }

    try{
      ROS_INFO_STREAM_THROTTLE(2, "TO DO");
      cv::GaussianBlur(input_frame, input_frame, cv::Size(5,5), 1, 0, cv::BORDER_DEFAULT);
      cv::cvtColor(input_frame, input_frame, cv::COLOR_BGR2GRAY );
      //background_substractor_->apply(input_frame, input_frame);
      //cv::circle(input_frame, cv::Point(50, 50), 10,cv::Scalar(0xffff));
      int erosion_size = 2.0;
      cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 3*erosion_size + 1, 3*erosion_size+1 ),
                                       cv::Point( erosion_size, erosion_size ) );
      cv::erode(input_frame, input_frame, element);
      cv::dilate(input_frame, input_frame, element);
      //output_frame = detectPeople(input_frame);
      cv::Canny(input_frame, output_frame,120, 200 );

    }
    catch( cv::Exception& e ){
      ROS_ERROR("cv exception: %s", e.what());
      return;
    }

    publishOutput(output_frame);

  }

  cv::Mat MyNodeletClass::detectPeople(cv::Mat frame_gray){

    //cv::equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
    cv::CascadeClassifier face_cascade;
    //face_cascade.load("/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_lowerbody.xml");
    face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_upperbody.xml");
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 4 );
    for ( size_t i = 0; i < faces.size(); i++ )
    {
        cv::Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        cv::ellipse( frame_gray, center, cv::Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, cv::Scalar( 0, 0, 0 ), 4 );
        cv::Mat faceROI = frame_gray( faces[i] );
        //-- In each face, detect eyes
        /*
        std::vector<cv::Rect> eyes;
        eyes_cascade.detectMultiScale( faceROI, eyes );
        for ( size_t j = 0; j < eyes.size(); j++ )
        {
            cv::Point eye_center( faces[i].x + eyes[j].x + eyes[j].width/2, faces[i].y + eyes[j].y + eyes[j].height/2 );
            int radius = cv::cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            cv::circle( frame, eye_center, radius, Scalar( 255, 0, 0 ), 4 );
        }
        */
    }
    //-- Show what you got
    //cv::imshow( "Capture - Face detection", frame_gray );
    return frame_gray;
    }
}
