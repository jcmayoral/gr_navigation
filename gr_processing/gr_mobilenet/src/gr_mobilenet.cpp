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

    std::string topic("image");
    if (private_nh.getParam("image_topic",topic)){
        ROS_INFO_STREAM("Subscribing to topic: " << topic);
        image_sub_ = private_nh.subscribe(topic, 1, &MobileNetWrapper::image_CB, this);
    }

    net_ = dnn::readNetFromCaffe(model_txt_, model_bin_);
    if (net_.empty())
    {
        std::cerr << "Can't load network by using the following files: " << std::endl;
        std::cerr << "prototxt:   " << model_txt_ << std::endl;
        std::cerr << "caffemodel: " << model_bin_ << std::endl;
        exit(-1);
    }


};

MobileNetWrapper::~MobileNetWrapper(){
}

void MobileNetWrapper::image_CB(const sensor_msgs::ImageConstPtr image){
    ROS_INFO("RECEIVED");
    try{
      cv::Mat frame = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::RGB8)->image;
      process_image(frame);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
      //return false;
  }
}

void MobileNetWrapper::process_image(cv::Mat frame){
    Mat img2;
    resize(frame, img2, Size(300,300));
    Mat inputBlob = blobFromImage(img2, 0.007843, Size(300,300), Scalar(127.5, 127.5, 127.5), false);

    net_.setInput(inputBlob, "data");
    Mat detection = net_.forward("detection_out");
    Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    std::ostringstream ss;
    float confidenceThreshold = 0.2;
    for (int i = 0; i < detectionMat.rows; i++)
    {
        float confidence = detectionMat.at<float>(i, 2);

        if (confidence > confidenceThreshold)
        {
            int idx = static_cast<int>(detectionMat.at<float>(i, 1));
            int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
            int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
            int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
            int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);

            Rect object((int)xLeftBottom, (int)yLeftBottom,
                        (int)(xRightTop - xLeftBottom),
                        (int)(yRightTop - yLeftBottom));

            rectangle(frame, object, Scalar(0, 255, 0), 2);

            std::cout << getClassName(idx) << ": " << confidence << std::endl;

            ss.str("");
            ss << confidence;
            String conf(ss.str());
            String label = getClassName(idx) + ": " + conf;
            int baseLine = 0;
            Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
            putText(frame, label, Point(xLeftBottom, yLeftBottom),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
        }
    }
    imshow("detections", frame);
    waitKey();


}


void MobileNetWrapper::readClassesFile(const std::string filename){
    classes_.clear();
    std::ifstream file(filename);
    std::string s;
    while (std::getline(file, s))
        std::cout << s << std::endl;
        classes_.push_back(s);
}

std::string MobileNetWrapper::getClassName(int index){
    return classes_[index];
}


std::string MobileNetWrapper::getModelTxtFile(){
    return model_txt_;
}

std::string MobileNetWrapper::getModelBinFile(){
    return model_bin_;
}