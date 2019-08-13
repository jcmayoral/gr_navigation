#include <pcl_cuda_tools/depth_registration.h>

int main (int argc, char **argv)
{
  std::cout << "Main";
  DepthRegistration depth_registrator_;
  depth_registrator_.run();

  // Check for errors (all values should be 3.0f)
  float maxError = 0.0f;
  for (int i = 0; i < depth_registrator_.getN(); i++)
    maxError = fmax(maxError, fabs(depth_registrator_.y[i]-3.0f));
  std::cout << "Max error: " << maxError << std::endl;

  return 0;
}
