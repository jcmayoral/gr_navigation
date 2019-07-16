#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <darknet_ros_msgs/BoundingBox.h>
#include <sensor_msgs/CameraInfo.h>

#include <math.h>
#include <random>

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

double register_pointclouds(darknet_ros_msgs::BoundingBox bounding_box, cv::Mat& depth_image, sensor_msgs::CameraInfo camera_info){
    uint16_t depth = 0.0;
    int pixel_number = 0;//(bounding_box.xmax - bounding_box.xmin) * (bounding_box.ymax - bounding_box.ymin);
    float mean_depth = 0;
    int mean_index = (bounding_box.xmax - bounding_box.xmin)/2 + depth_image.rows* (bounding_box.ymax - bounding_box.ymin)/2;

    for (auto i = bounding_box.xmin; i<=bounding_box.xmax; ++i){
        for (auto j = bounding_box.ymin; j<=bounding_box.ymax; ++j){
            //Assume rgb and depth are same size
            //TODO conversion between depth and rgb
            depth= depth_image.at<uint16_t>(i+j*depth_image.rows); //realsense
            //depth= depth_image.at<float>(i+j*depth_image.rows); //ZED

            //depth_image.at<float>(cv::Point(i, j)) = 255;
            //cv::circle(depth_image,cv::Point(i, j),1,cv::Scalar(255,255,255));
            //depth_image.at<uint16_t>(j,i) = 0xffff;

            if (std::isfinite(depth)){
                mean_depth+= depth;
                pixel_number++;
            }

        }
    }
    if (pixel_number == 0){
        return -1;
    }

    //cv::circle(depth_image,cv::Point(center_row, center_col),25,cv::Scalar(255,255,255))
    // Use correct principal point from calibration
    float center_x = camera_info.K[2];
    float center_y = camera_info.K[5];

    float constant_x = 1.0 / camera_info.K[0];
    float constant_y = 1.0 / camera_info.K[4];

    return (mean_depth/pixel_number)*0.001; //ROS_DEPTH_SCALE
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




double get_variance(float mean, uint16_t* points, int points_number, double* score_array){
    double var = 0;
    double scoring;

    for(int n = 0; n< points_number; ++n){
        scoring = pow(points[n] - mean,2);
        score_array[n] = scoring;
        var += scoring;
    }
    var /= points_number;
    return sqrt(var);
}


//On development
double register_ransac_pointclouds(darknet_ros_msgs::BoundingBox bounding_box, cv::Mat& depth_image, sensor_msgs::CameraInfo camera_info){
    int pixel_number = 0;//(bounding_box.xmax - bounding_box.xmin) * (bounding_box.ymax - bounding_box.ymin);
    float mean_depth = 0;

    double best_result = 100000;
    double current_result = 0;
    float best_variance = 100000;
    float current_std = 1000000;
    int maximum_iterations = 400;
    int number_inliers = 100;//0.025 * (bounding_box.xmax - bounding_box.xmin) * (bounding_box.ymax - bounding_box.ymin);
    uint16_t depth_array [number_inliers];
    std::pair<int,int> best_indices[number_inliers];
    double scores_indices[number_inliers];

    float threshold = 1500.0;
    int cell_x, cell_y = 0;

    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_int_distribution<> x_distr(bounding_box.xmin, bounding_box.xmax); // define the range
    std::uniform_int_distribution<> y_distr(bounding_box.ymin, bounding_box.ymax); // define the range


    std::pair<int,int> current_indices[number_inliers];
    bool is_initialize = false;

    for (auto iteration = 0; iteration < maximum_iterations; ++ iteration ){
        mean_depth = 0.0;
        pixel_number = 0;

        if (!is_initialize){
            ROS_INFO("NOT INITIALIZED");
            for(int n=0; n<number_inliers; ++n){
                cell_x = x_distr(eng);
                cell_y = y_distr(eng);
                depth_array[n] = depth_image.at<uint16_t>(cell_x+cell_y*depth_image.rows); //realsense
                if (std::isfinite(depth_array[n])){
                    mean_depth+= depth_array[n];
                    pixel_number++;
                    current_indices[n] = std::make_pair(cell_x,cell_y);
                    //depth_image.at<uint16_t>(cell_y,cell_x) = 0xffff;
                }
                else{
                    n = n-1;
                }
            }
            is_initialize = true;
            ROS_INFO("NOT INITIALIZED");
        }
        else{
            ROS_ERROR("INITIALIZED");
            std::cout << "HERE" << std::endl;
            int auxiliar = sizeof(scores_indices) / sizeof(scores_indices[0]); 

            int worst_inlier = std::distance(scores_indices, std::max_element(scores_indices, scores_indices+ auxiliar));
            std::cout << "WORST INDEX"<< worst_inlier << std::endl; 
            bool new_inlier_accepted = false;

            while(!new_inlier_accepted){
                cell_x = x_distr(eng);
                cell_y = y_distr(eng);
                depth_array[worst_inlier] = depth_image.at<uint16_t>(cell_x+cell_y*depth_image.rows); //realsense
                if (std::isfinite(depth_array[worst_inlier])){
                    //pixel_number++;
                    for(int n=0; n<number_inliers; ++n){
                        mean_depth+= depth_array[n];
                        pixel_number++;
                    }
                    //depth_image.at<uint16_t>(cell_y,cell_x) = 0xffff;
                    current_indices[worst_inlier] = std::make_pair(cell_x,cell_y);
                    new_inlier_accepted = true;
                    //depth_image.at<uint16_t>(cell_y,cell_x) = 0xffff;
                }
            }
        }

        current_result = (mean_depth/pixel_number)*0.001;
        current_std = get_variance(current_result, depth_array, number_inliers, scores_indices);
        std::cout << "STD DEV " << current_std << " MAX " << best_variance << std::endl;

        /*
        for (int n = 0; n < number_inliers; ++n){
            std::cout << "scores indices "<< scores_indices[n] << std::endl;
        }
        */

        if(1 < current_std < best_variance){
            ROS_WARN("HERE");
            best_result = current_result;
            best_variance = current_std;
            for (int z = 0; z < number_inliers; ++z){
                best_indices[z] = current_indices[z];
            }
        }


        if (best_variance < threshold){
           // break;
        }

    }

    ROS_INFO_STREAM("Finished"<< best_variance);

    for (int n = 0; n < number_inliers; ++n){
        //std::cout << "best indices "<< best_indices[n].first <<  ","<<best_indices[n].second << std::endl;
        depth_image.at<uint16_t>(best_indices[n].second,best_indices[n].first) = 0xffff;
    }

    return best_result; //ROS_DEPTH_SCALE

}
