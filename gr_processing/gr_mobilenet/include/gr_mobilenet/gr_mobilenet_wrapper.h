#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace cv::dnn;

namespace gr_mobilenet{
    class MobileNetWrapper{
        public:
            MobileNetWrapper();
            ~MobileNetWrapper();
            void readClassesFile(const std::string filename);
            void image_CB(const sensor_msgs::ImageConstPtr image);
            void process_image(cv::Mat frame);
            std::string getClassName(int index);
            std::string getModelTxtFile();
            std::string getModelBinFile();
            Net net_;
        private:
            std::vector<std::string> classes_;
            std::string model_txt_ ;
            std::string model_bin_;
            ros::Subscriber image_sub_;
    };
}