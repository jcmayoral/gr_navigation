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

};

MobileNetWrapper::~MobileNetWrapper(){
}

void MobileNetWrapper::image_CB(const sensor_msgs::ImageConstPtr image){
    ROS_INFO("RECEIVED");
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