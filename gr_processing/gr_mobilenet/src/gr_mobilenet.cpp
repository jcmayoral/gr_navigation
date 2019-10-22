#include <gr_mobilenet/gr_mobilenet_wrapper.h>

using namespace gr_mobilenet;

MobileNetWrapper::MobileNetWrapper():classes_(){

    ros::NodeHandle private_nh("~");

    std::string classes_file;
    if (private_nh.getParam("classes_file",classes_file)){
        std::cout << "param" << std::endl;
        readClassesFile(classes_file);
        ROS_INFO_STREAM(classes_file);
    }
    else{
        ROS_ERROR("CLASSES FILE NOT SET");
        classes_.push_back("UNKNOWN");
    }


    if (private_nh.getParam("model_bin",model_bin_)){
        ROS_INFO("Model bin file found");
    }

    if (private_nh.getParam("model_txt",model_txt_)){
        ROS_INFO("Model txt file found");
    }


    net_ = dnn::readNetFromCaffe(model_txt_, model_bin_);
    image_pub_ = private_nh.advertise<sensor_msgs::Image>("detection", 1);

    if (net_.empty())
    {
        std::cerr << "Can't load network by using the following files: " << std::endl;
        std::cerr << "prototxt:   " << model_txt_ << std::endl;
        std::cerr << "caffemodel: " << model_bin_ << std::endl;
        exit(-1);
    }

    std::string topic("image");
    if (private_nh.getParam("image_topic",topic)){
        ROS_INFO_STREAM("Subscribing to topic: " << topic);
        image_sub_ = private_nh.subscribe(topic, 1, &MobileNetWrapper::image_CB, this);
    }

};

MobileNetWrapper::~MobileNetWrapper(){
}

void MobileNetWrapper::image_CB(const sensor_msgs::ImageConstPtr image){
    ROS_INFO("RECEIVED");
    cv::Mat resized_image;

    try{
      //cv::Mat frame = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8)->image;
      cv_ptr_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      int w = cv_ptr_->image.cols;
      int h = cv_ptr_->image.rows;

      //if (rotate_flag >= 0) {
      //   cv::rotate(cv_ptr->image, rotated_image, rotate_flag);
      //   rotated_image.copyTo(cv_ptr->image);
      //}

      cv::resize(cv_ptr_->image, resized_image, cvSize(300, 300));
      cv::Mat blob = cv::dnn::blobFromImage(resized_image, 0.007843f,
                cvSize(300, 300), 100, false);

      process_image(blob, w, h);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
      //return false;
  }
}

void MobileNetWrapper::process_image(cv::Mat frame, int w , int h){
    net_.setInput(frame, "data");
    Mat detection = net_.forward("detection_out");
    Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    std::unique_lock<std::mutex> lock(mutx);
    std::ostringstream ss;

    float confidenceThreshold = 0.2;

    std::cout << detectionMat.rows << std::endl;

    for (int i = 0; i < detectionMat.rows; i++)
    {
        float confidence = detectionMat.at<float>(i, 2);
        std::cout << confidence << std::endl;
        if (confidence > confidenceThreshold)
        {
            int idx = static_cast<int>(detectionMat.at<float>(i, 1));
            int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * w);
            int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * h);
            int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * w);
            int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * h);

            std::cout << "!"<<detectionMat.at<float>(i, 3)<<","<< detectionMat.at<float>(i, 4) << " , "
                     << detectionMat.at<float>(i, 5) << "," << detectionMat.at<float>(i, 6)<< std::endl;

            std::cout << frame.cols << " , " << frame.rows << std::endl;
            Rect object((int)xLeftBottom, (int)yLeftBottom,
                        (int)(xRightTop - xLeftBottom),
                        (int)(yRightTop - yLeftBottom));

            cv::rectangle(cv_ptr_->image, object, Scalar(255, 255, 0), 2);
            std::cout << "!"<< idx << std::endl;

            std::cout << getClassName(idx) << ": " << confidence << std::endl;
            std::cout << "!"<< std::endl;
            ss.str("");
            ss << confidence;
            String conf(ss.str());
            String label = getClassName(idx) + ": " + conf;
            int baseLine = 0;
            Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.75, 2, &baseLine);
            putText(cv_ptr_->image, label, Point(xLeftBottom, yLeftBottom),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
        }
    }
    //publishDetection(frame);
    image_pub_.publish(cv_ptr_->toImageMsg());
}


void MobileNetWrapper::readClassesFile(const std::string filename){
    classes_.clear();
    std::ifstream file(filename);
    std::string s;
    while (std::getline(file, s)){
        classes_.push_back(s);
    }
    std::cout << "CLASSES " << classes_.size() << std::endl;
}

std::string MobileNetWrapper::getClassName(int index){
    std::cout << "SIZE classes " << classes_.size();
    if (index < classes_.size()){
        return classes_[index];
    }  
    return "UNKNOWN";
}


std::string MobileNetWrapper::getModelTxtFile(){
    return model_txt_;
}

std::string MobileNetWrapper::getModelBinFile(){
    return model_bin_;
}