#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class DepthRegistration{
  public:
    DepthRegistration(cv::Mat image);

    ~DepthRegistration();

    cv::Mat getFrame(){
      return frame_;
    }

    double getN(){
      return frame_.cols * frame_.rows;
    }

    double  run();

  private:
    cv::Mat frame_;
    //std::mutex mtx;

};
