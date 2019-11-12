#include <gr_mobilenet/gr_mobilenet_wrapper.h>

using namespace gr_mobilenet;

MobileNetWrapper::MobileNetWrapper():classes_(){

    ros::NodeHandle private_nh("~");

    std::string classes_file;
    if (private_nh.getParam("classes_file",classes_file)){
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
    bb_pub_ = private_nh.advertise<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1);

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
    //googlenet
    //Mat detection = net_.forward("loss3/classifier");
    Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    std::unique_lock<std::mutex> lock(mutx);
    std::ostringstream ss;

    float confidenceThreshold = 0.3;

    darknet_ros_msgs::BoundingBoxes bbs;
    bbs.header.frame_id = "camera_dummy";
    bbs.header.stamp = ros::Time::now();
    bbs.image_header = bbs.header;

    for (int i = 0; i < detectionMat.rows; i++)
    {
        darknet_ros_msgs::BoundingBox bb;

        float confidence = detectionMat.at<float>(i, 2);
        if (confidence > confidenceThreshold)
        {
            int idx = static_cast<int>(detectionMat.at<float>(i, 1));
            int xmin = static_cast<int>(detectionMat.at<float>(i, 3) * w);
            int ymin = static_cast<int>(detectionMat.at<float>(i, 4) * h);
            int xmax = static_cast<int>(detectionMat.at<float>(i, 5) * w);
            int ymax = static_cast<int>(detectionMat.at<float>(i, 6) * h);

            Rect object((int)xmin, (int)ymin,
                        (int)(xmax - xmin),
                        (int)(ymax - ymin));

            cv::rectangle(cv_ptr_->image, object, Scalar(255, 255, 0), 2);

            std::cout << getClassName(idx) << ": " << confidence << std::endl;
            ss.str("");
            ss << confidence;
            String conf(ss.str());
            String label = getClassName(idx) + ": " + conf;
            int baseLine = 0;
            Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.75, 2, &baseLine);
            putText(cv_ptr_->image, label, Point(xmin + (xmax - xmin)/2, ymin + (ymax - ymin)/2),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
            //Fill Bounding Box message
            bb.Class = getClassName(idx);
            bb.probability = confidence;
            bb.xmin = xmin;
            bb.xmax = xmax;
            bb.ymin = ymin;
            bb.ymax = ymax;
            bbs.bounding_boxes.push_back(bb);
        }
    }
    //publishDetection(frame);
    image_pub_.publish(cv_ptr_->toImageMsg());
    bb_pub_.publish(bbs);
}


void MobileNetWrapper::readClassesFile(const std::string filename){
    classes_.clear();
    std::ifstream file(filename);
    std::string s;
    while (std::getline(file, s)){
        classes_.push_back(s);
    }
}

std::string MobileNetWrapper::getClassName(int index){
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