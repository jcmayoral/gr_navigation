#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>
using namespace cv;
using namespace cv::dnn;
#include <fstream>
#include <iostream>
#include <cstdlib>
using namespace std;

#include <gr_mobilenet/gr_mobilenet_wrapper.h>
using namespace gr_mobilenet;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gr_mobilenet");
    CV_TRACE_FUNCTION();
  
    MobileNetWrapper mobile_net;

    ros::spin();

    String modelTxt = mobile_net.getModelTxtFile();
    String modelBin = mobile_net.getModelBinFile();

    std::cout << modelTxt << modelBin << std::endl;

    return 0;
}