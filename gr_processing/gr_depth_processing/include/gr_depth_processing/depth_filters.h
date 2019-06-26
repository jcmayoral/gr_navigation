#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <darknet_ros_msgs/BoundingBox.h>
#include <sensor_msgs/CameraInfo.h>

#include <math.h>

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

double register_pointclouds(darknet_ros_msgs::BoundingBox bounding_box, cv::Mat& depth_image, sensor_msgs::CameraInfo camera_info, std::string label){
    uint16_t depth = 0.0;
    int pixel_number = 0;//(bounding_box.xmax - bounding_box.xmin) * (bounding_box.ymax - bounding_box.ymin);
    double mean_depth = 0.0;
    int mean_index = (bounding_box.xmax - bounding_box.xmin)/2 + depth_image.rows* (bounding_box.ymax - bounding_box.ymin)/2;

    for (auto i = bounding_box.xmin; i<=bounding_box.xmax; ++i){
        for (auto j = bounding_box.ymin; j<=bounding_box.ymax; ++j){
            //Assume rgb and depth are same size
            //TODO conversion between depth and rgb
            depth= depth_image.at<uint16_t>(i+j*depth_image.rows); //realsense
            //depth= depth_image.at<float>(i+j*depth_image.rows); //ZED

            //depth_image.at<float>(cv::Point(i, j)) = 255;
            //cv::circle(depth_image,cv::Point(i, j),1,cv::Scalar(255,255,255));

            if (std::isfinite(depth)){
                mean_depth+= depth;
                pixel_number++;
            }

        }
    }
    if (pixel_number == 0){
        return -1;
    }

    int center_row = bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin)/2;
    int center_col = bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin)/2;

    //cv::circle(depth_image,cv::Point(center_row, center_col),25,cv::Scalar(255,255,255));


    // then put the text itself
    cv::putText(depth_image, label +  std::to_string(mean_depth/pixel_number), cv::Point(center_row, center_col), cv::FONT_HERSHEY_PLAIN,
                1,   0xffff , 2, 8);

    // Use correct principal point from calibration
    float center_x = camera_info.K[2];
    float center_y = camera_info.K[5];

    float constant_x = 1.0 / camera_info.K[0];
    float constant_y = 1.0 / camera_info.K[4];

    return mean_depth;
}

/*
// Handles float or uint16 depths
template<typename T>
void convert(
    const sensor_msgs::ImageConstPtr& depth_msg,
    PointCloud::Ptr& cloud_msg,
    const image_geometry::PinholeCameraModel& model,
    double range_max = 0.0)
{
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth))
      {
        if (range_max != 0.0)
        {
          depth = DepthTraits<T>::fromMeters(range_max);
        }
        else
        {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      *iter_x = (u - center_x) * depth * constant_x;
      *iter_y = (v - center_y) * depth * constant_y;
      *iter_z = DepthTraits<T>::toMeters(depth);
    }
  }
}

} // namespace depth_image_proc

#en
*/
