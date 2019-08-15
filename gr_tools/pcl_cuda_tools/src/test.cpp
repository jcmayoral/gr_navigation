#include <pcl_cuda_tools/depth_registration.h>

int main (int argc, char **argv)
{
  cv::Mat sample_img(1000,1000,CV_8UC1);

  std::cout << "Mat will be fill"<<std::endl;

  for(int r = 0; r < sample_img.rows; r++) {
        // We obtain a pointer to the beginning of row r
        for(int c = 0; c < sample_img.cols; c++) {
            // We invert the blue and red values of the pixel
            uchar v =1.0;
            sample_img.at<uchar>(r,c) = v;
        }
  }
  std::cout << "Mat has been filled"<<std::endl;

  DepthRegistration depth_registrator_(sample_img);
  auto result = depth_registrator_.run();

  float maxError = 0.0f;

  for (int i = 0; i < depth_registrator_.getN(); i++)
  // Check for errors (all values should be 3.0f)
    maxError = fmax(maxError, fabs(depth_registrator_.x[i]));
  std::cout << "Max error: " << maxError << std::endl;

  return 0;
}
