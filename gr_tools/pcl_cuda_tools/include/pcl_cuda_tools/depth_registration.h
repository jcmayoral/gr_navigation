#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class DepthRegistration{
  public:
    DepthRegistration(cv::Mat image);
    ~DepthRegistration();

    int getN(){
      return N;
    };

    double run();

    int *x;
    int *hist;
    float delta;

  private:
    int N;

};
