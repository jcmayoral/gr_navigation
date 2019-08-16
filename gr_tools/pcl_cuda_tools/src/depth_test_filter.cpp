#include <pcl_cuda_tools/depth_registration.h>

int main (int argc, char **argv)
{
  cv::Mat sample_img(1000,1000,CV_8UC1);

  std::cout << "Mat will be fill " << sample_img.rows << sample_img.cols <<std::endl;
  uchar v =0;
  for(int r = 0; r < sample_img.rows; r++) {
        for(int c = 0; c < sample_img.cols; c++) {
            sample_img.at<unsigned char>(r,c) = v;

            if (v < 255){
              v++;
            }

            else{
              v=0;
            }
        }
  }
  std::cout << "Mat has been filled"<<std::endl;

  DepthRegistration depth_registrator_(sample_img);
  auto result = depth_registrator_.run();
  /*
  for (int i = 0; i < depth_registrator_.getN(); i++)
  // Check for errors (all values should be 3.0f)
    maxError = fmax(maxError, fabs(depth_registrator_.x[i]));
  std::cout << "Max error: " << maxError << std::endl;
  */
  return 0;
}
