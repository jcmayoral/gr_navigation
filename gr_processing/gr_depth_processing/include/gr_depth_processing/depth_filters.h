#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <sensor_msgs/CameraInfo.h>

void cv_filter(cv::Mat& frame){
    try{
        cv::GaussianBlur(frame, frame, cv::Size(5,5), 1, 0, cv::BORDER_DEFAULT);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY );

        //cv::Ptr<cv::BackgroundSubtractorMOG2> background_substractor_;
        //background_substractor_ = cv::createBackgroundSubtractorMOG2();
        //background_substractor_->setNMixtures(3);
        //background_substractor_->setHistory(3);
        //background_substractor_ =  new cv::BackgroundSubtractorMOG2(1, 16, true); //MOG2 approach
        //background_substractor_->apply(frame, frame);
        //cv::circle(frame, cv::Point(50, 50), 10,cv::Scalar(0xffff));
        int erosion_size = 2.0;
        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                        cv::Size( 3*erosion_size + 1, 3*erosion_size+1 ),
                                        cv::Point( erosion_size, erosion_size ) );
        cv::erode(frame, frame, element);
        cv::dilate(frame, frame, element);
        //output_frame = detectPeople(input_frame);
        cv::Canny(frame, frame,120, 200 );

      }
      catch( cv::Exception& e ){
        return;
      }
}

void cv_detectPeople(cv::Mat& frame_gray){
    //cv::equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
    cv::CascadeClassifier face_cascade;
    //face_cascade.load("/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_lowerbody.xml");
    face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_upperbody.xml");
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 4 );
    
    for ( size_t i = 0; i < faces.size(); i++ ){
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
    //return frame_gray;
}

double register_pointclouds(std::vector<cv::Rect> boundRect, cv::Mat depth_image, sensor_msgs::CameraInfo camera_info){
    return 0.0;
}