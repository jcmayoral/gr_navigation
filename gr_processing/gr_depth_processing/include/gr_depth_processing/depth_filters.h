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
        //cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY );

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
        cv::Laplacian(frame, frame, CV_16U);
        //cv::Canny(frame, frame,120, 200 );

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
        scoring = points[n];//pow(points[n] - mean,2);
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
    int maximum_iterations = 1000;
    int number_changes = 25;
    int number_inliers = 400;//0.01 * (bounding_box.xmax - bounding_box.xmin) * (bounding_box.ymax - bounding_box.ymin);
    uint16_t depth_array [number_inliers];
    std::pair<int,int> best_indices[number_inliers];
    double scores_indices[number_inliers];

    float threshold = 40.0;
    int cell_x, cell_y = 0;

    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator


    std::pair<int,int> current_indices[number_inliers];
    bool is_initialize = false;

    for (auto iteration = 0; iteration < maximum_iterations; ++ iteration ){
        mean_depth = 0.0;

        if (!is_initialize){
            int dx = (bounding_box.xmax - bounding_box.xmin)/2 + bounding_box.xmin;
            int dy = (bounding_box.ymax - bounding_box.ymin)/2 + bounding_box.ymin;
            //std::uniform_int_distribution<> x_distr(dx -200, dx + 200); // define the range
            //std::uniform_int_distribution<> y_distr(dy -200, dy + 200); // define the range

            std::uniform_int_distribution<> x_distr( bounding_box.xmin,  bounding_box.xmax); // define the range
            std::uniform_int_distribution<> y_distr( bounding_box.ymin,  bounding_box.ymax); // define the range

            for(int n=0; n<number_inliers; ++n){
                cell_x = x_distr(eng);
                cell_y = y_distr(eng);
                depth_array[n] = depth_image.at<uint16_t>(cell_x+cell_y*depth_image.rows); //realsense
                if (std::isfinite(depth_array[n])){
                    //mean_depth+= depth_array[n];
                    current_indices[n] = std::make_pair(cell_x,cell_y);
                }
            }
            is_initialize = true;
        }
        else{
            int auxiliar = sizeof(scores_indices) / sizeof(scores_indices[0]); 
            int worst_inlier = std::distance(scores_indices, std::max_element(scores_indices, scores_indices+ auxiliar));
            int best_inlier = std::distance(scores_indices, std::min_element(scores_indices, scores_indices+ auxiliar));

            int best_x = current_indices[best_inlier].first;
            int best_y = current_indices[best_inlier].second;

            std::uniform_int_distribution<>local_x_distr(best_x -50,best_x + 50); // define the range
            std::uniform_int_distribution<>local_y_distr(best_y -50,best_y + 50); // define the range

            for(int n=0; n<number_inliers-number_changes; ++n){
                cell_x = local_x_distr(eng);
                cell_y = local_y_distr(eng);
                depth_array[n] = depth_image.at<uint16_t>(cell_x+cell_y*depth_image.rows); //realsense

                if (cell_x <= bounding_box.xmin || cell_x >= bounding_box.xmax || cell_x < 0){
                    n = n-1;
                    continue;
                }
                if (cell_y <= bounding_box.ymin || cell_y >= bounding_box.ymax || cell_y < 0){
                    n = n-1;
                    continue;
                }
                if (std::isfinite(depth_array[n])){
                    //mean_depth+= depth_array[n];
                    current_indices[n] = std::make_pair(cell_x,cell_y);
                }
            }

            for(int w =number_inliers-number_changes ; w < number_inliers ; w++){
                bool new_inlier_accepted = false;


                int best_x = current_indices[worst_inlier].first;
                int best_y = current_indices[worst_inlier].second;

                std::uniform_int_distribution<>local_x_distr(best_x -5,best_x + 5); // define the range
                std::uniform_int_distribution<>local_y_distr(best_y -5,best_y + 5); // define the range
                //std::uniform_int_distribution<> local_x_distr( bounding_box.xmin,  bounding_box.xmax); // define the range
                //std::uniform_int_distribution<> local_y_distr( bounding_box.ymin,  bounding_box.ymax); // define the range

                int tries = 0;

                while(!new_inlier_accepted && tries <5 ){
                    tries++;
                    //std::cout << "resample " << w << "of " << number_changes << std::endl;
                    cell_x = local_x_distr(eng);
                    cell_y = local_y_distr(eng);
                    //cell_x = current_indices[best_inlier].first +1;
                    //cell_y = current_indices[best_inlier].second +1;
                    if (cell_x <= bounding_box.xmin || cell_x >= bounding_box.xmax || cell_x < 0)
                        continue;
                    if (cell_y <= bounding_box.ymin || cell_y >= bounding_box.ymax || cell_y < 0)
                        continue;

                    depth_array[w] = depth_image.at<uint16_t>(cell_x+cell_y*depth_image.rows); //realsense
                    if (std::isfinite(depth_array[w])){
                        //pixel_number++;
                        /*
                        for(int n=0; n<number_inliers; ++n){
                            mean_depth+= depth_array[n];
                        }
                        */
                        current_indices[worst_inlier] = std::make_pair(cell_x,cell_y);
                        new_inlier_accepted = true;       
                        //depth_image.at<uint16_t>(cell_y,cell_x) = 0xffff;
                    }
                }

            }
        }

        current_result = (std::accumulate(depth_array,depth_array+number_inliers,0)/number_inliers) * 0.001;
        current_std = get_variance(current_result, depth_array, number_inliers, scores_indices);
        //std::cout << "STD DEV " << current_std << " MAX " << best_variance << std::endl;

        /*
        for (int n = 0; n < number_inliers; ++n){
            std::cout << "scores indices "<< scores_indices[n] << std::endl;
        }
        */

        if(current_std < best_variance){
            best_result = current_result;
            best_variance = current_std;
            for (int z = 0; z < number_inliers; ++z){
                best_indices[z] = current_indices[z];
            }
        }


        if (best_variance < threshold){
            ROS_ERROR("Threshold Reached");
           break;
        }

    }

    ROS_INFO_STREAM("Finished"<< best_variance);

    for (int n = 0; n < number_inliers; ++n){
        //std::cout << "best indices "<< best_indices[n].first <<  ","<<best_indices[n].second << std::endl;

        if (best_indices[n].first <= bounding_box.xmin || best_indices[n].first >= bounding_box.xmax || best_indices[n].first < 0)
                continue;
        if (best_indices[n].second <= bounding_box.ymin || best_indices[n].second >= bounding_box.ymax || best_indices[n].second < 0)
                continue;

        depth_image.at<uint16_t>(best_indices[n].second,best_indices[n].first) = 0xffff;
    }
    ROS_ERROR("FINISH");

    return best_result; //ROS_DEPTH_SCALE

}







double register_histogram_pointclouds(darknet_ros_msgs::BoundingBox bounding_box, cv::Mat& depth_image, sensor_msgs::CameraInfo camera_info){
    try{

        // Setup a rectangle to define your region of interest
        cv::Rect myROI(bounding_box.xmin,bounding_box.ymin,bounding_box.xmax -bounding_box.xmin , bounding_box.ymax-bounding_box.ymin);

        // Crop the full image to that image contained by the rectangle myROI
        // Note that this doesn't copy the data
        cv::Mat croppedImage = depth_image(myROI);



        //cv::imshow("cropped ", croppedImage);
         
        std::vector<cv::Mat> bgr_planes;
        //cv::split( croppedImage, bgr_planes );
        int histSize = 65535;
        float range[] = { 0, 65535 }; //the upper boundary is exclusive
        const float* histRange = { range };
        bool uniform = true, accumulate = false;
        cv::Mat b_hist, g_hist, r_hist;
        calcHist( &croppedImage, 2, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
        //calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
        //calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
        int hist_w = 512, hist_h = 400;
        int bin_w = cvRound( (double) hist_w/histSize );
        cv::Mat histImage( hist_h, hist_w,  CV_8UC1, cv::Scalar(0,0,0) );
        //cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
        //normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
        //normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    

        for( int i = 1; i < histSize; i++ )
        {
            cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ),
                cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                cv::Scalar( 0, 255, 0), 2, 8, 0  );
            //std::cout << cvRound(b_hist.at<float>(i)) << std::endl;
            /*line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ),
                Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                Scalar( 0, 255, 0), 2, 8, 0  );
            line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ),
                Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                Scalar( 0, 0, 255), 2, 8, 0  );*/
        }
        
        imshow("Source image", histImage );
        cv::waitKey(2000);
        
        //Initialize m
        double minVal; 
        double maxVal; 
        int minindex;
        int maxindex;
        //cv::Point minLoc; 
        //cv::Point maxLoc;

        minMaxIdx( b_hist, &minVal, &maxVal, &minindex, &maxindex );

        std::cout << "min val : " << minVal << " ID " << minindex <<std::endl;
        float fval = static_cast<float>(b_hist.at<float>(maxindex));
        std::cout << "max val: " << fval << " ID " << maxindex <<std::endl;

        return bin_w*maxindex;




      }
      catch( cv::Exception& e ){
        return 0.0;
      }
}
